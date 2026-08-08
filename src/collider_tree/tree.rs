use bevy::{
    ecs::{entity::Entity, resource::Resource},
    reflect::prelude::*,
};
use obvhs::{
    aabb::Aabb,
    bvh2::{Bvh2, reinsertion::ReinsertionOptimizer},
    ploc::{PlocBuilder, PlocSearchDistance, SortPrecision, rebuild::compute_rebuild_path_flags},
};

use crate::{
    collider_tree::{Bvh2Ext, ProxyId},
    data_structures::stable_vec::StableVec,
    prelude::{ActiveCollisionHooks, CollisionLayers},
};

/// A [Bounding Volume Hierarchy (BVH)][BVH] for accelerating queries on a set of colliders.
///
/// See the [`collider_tree`](crate::collider_tree) module for more information.
///
/// [BVH]: https://en.wikipedia.org/wiki/Bounding_volume_hierarchy
#[derive(Clone, Default)]
pub struct ColliderTree {
    /// The underlying BVH structure.
    pub bvh: Bvh2,
    /// The proxies stored in the tree.
    pub proxies: StableVec<ColliderTreeProxy>,
    /// A list of moved proxies since the last update.
    ///
    /// This is used during tree optimization to determine
    /// which proxies need to be reinserted or rebuilt.
    pub moved_proxies: MovedProxyList,
    /// A workspace for reusing allocations across tree operations.
    pub workspace: ColliderTreeWorkspace,
}

/// A set of [`ProxyId`]s that have moved since the last update, in insertion order.
///
/// Supports constant-time insertion, removal, and membership tests by keeping
/// a sparse index from [`ProxyId`] to the position of the ID in the dense list.
#[derive(Clone, Default, Debug)]
pub struct MovedProxyList {
    /// The moved proxy IDs, in insertion order.
    proxies: Vec<ProxyId>,
    /// Maps a [`ProxyId`] to its index in `proxies`, or [`PROXY_NOT_PRESENT`] if it is not in the list.
    indices: Vec<u32>,
}

/// The sentinel stored in [`MovedProxyList::indices`] for proxies that are not in the list.
const PROXY_NOT_PRESENT: u32 = u32::MAX;

impl MovedProxyList {
    /// Adds a proxy to the list if it is not already present.
    ///
    /// Returns `true` if the proxy was added.
    #[inline]
    pub fn push(&mut self, proxy_id: ProxyId) -> bool {
        if self.indices.len() <= proxy_id.index() {
            self.indices.resize(proxy_id.index() + 1, PROXY_NOT_PRESENT);
        } else if self.indices[proxy_id.index()] != PROXY_NOT_PRESENT {
            return false;
        }

        self.indices[proxy_id.index()] = self.proxies.len() as u32;
        self.proxies.push(proxy_id);

        true
    }

    /// Removes a proxy from the list. This may change the order of the remaining proxies.
    ///
    /// If the proxy is not present, nothing happens.
    #[inline]
    pub fn remove(&mut self, proxy_id: ProxyId) {
        let Some(index) = self.indices.get_mut(proxy_id.index()) else {
            return;
        };
        if *index == PROXY_NOT_PRESENT {
            return;
        }

        let index = core::mem::replace(index, PROXY_NOT_PRESENT) as usize;
        self.proxies.swap_remove(index);

        // The proxy that was swapped into the removed slot needs its index updated.
        if let Some(&swapped) = self.proxies.get(index) {
            self.indices[swapped.index()] = index as u32;
        }
    }

    /// Removes all proxies from the list.
    #[inline]
    pub fn clear(&mut self) {
        for proxy_id in self.proxies.drain(..) {
            self.indices[proxy_id.index()] = PROXY_NOT_PRESENT;
        }
    }
}

impl core::ops::Deref for MovedProxyList {
    type Target = [ProxyId];

    #[inline]
    fn deref(&self) -> &[ProxyId] {
        &self.proxies
    }
}

/// A proxy representing a collider in the [`ColliderTree`].
#[derive(Clone, Debug)]
pub struct ColliderTreeProxy {
    /// The collider entity this proxy represents.
    pub collider: Entity,
    /// The body this collider is attached to.
    pub body: Option<Entity>,
    /// The collision layers of the collider.
    pub layers: CollisionLayers,
    /// Flags for the proxy.
    pub flags: ColliderTreeProxyFlags,
}

/// Flags for a [`ColliderTreeProxy`].
#[repr(transparent)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct ColliderTreeProxyFlags(u32);

bitflags::bitflags! {
    impl ColliderTreeProxyFlags: u32 {
        /// Set if the collider is a sensor.
        const SENSOR = 1 << 0;
        /// Set if the body this collider is attached to has [`RigidBodyDisabled`](crate::dynamics::rigid_body::RigidBodyDisabled).
        const BODY_DISABLED = 1 << 1;
        /// Set if the custom filtering hook is active for this collider.
        const CUSTOM_FILTER = 1 << 2;
        /// Set if the contact modification hook is active for this collider.
        const MODIFY_CONTACTS = 1 << 3;
        /// Set if contact events are enabled for this collider.
        const CONTACT_EVENTS = 1 << 4;
    }
}

impl ColliderTreeProxyFlags {
    /// Creates [`ColliderTreeProxyFlags`] from the given sensor status and active collision hooks.
    #[inline]
    pub fn new(
        is_sensor: bool,
        is_body_disabled: bool,
        events_enabled: bool,
        active_hooks: ActiveCollisionHooks,
    ) -> Self {
        let mut flags = ColliderTreeProxyFlags::empty();
        if is_sensor {
            flags |= ColliderTreeProxyFlags::SENSOR;
        }
        if is_body_disabled {
            flags |= ColliderTreeProxyFlags::BODY_DISABLED;
        }
        if active_hooks.contains(ActiveCollisionHooks::FILTER_PAIRS) {
            flags |= ColliderTreeProxyFlags::CUSTOM_FILTER;
        }
        if active_hooks.contains(ActiveCollisionHooks::MODIFY_CONTACTS) {
            flags |= ColliderTreeProxyFlags::MODIFY_CONTACTS;
        }
        if events_enabled {
            flags |= ColliderTreeProxyFlags::CONTACT_EVENTS;
        }
        flags
    }
}

impl ColliderTreeProxy {
    /// Returns `true` if the collider is a sensor.
    #[inline]
    pub fn is_sensor(&self) -> bool {
        self.flags.contains(ColliderTreeProxyFlags::SENSOR)
    }

    /// Returns `true` if the custom filtering hook is active.
    #[inline]
    pub fn has_custom_filter(&self) -> bool {
        self.flags.contains(ColliderTreeProxyFlags::CUSTOM_FILTER)
    }

    /// Returns `true` if the contact modification hook is active.
    #[inline]
    pub fn has_contact_modification(&self) -> bool {
        self.flags.contains(ColliderTreeProxyFlags::MODIFY_CONTACTS)
    }
}

/// A workspace for performing operations on a [`ColliderTree`].
///
/// This stores temporary data structures and working memory used when modifying the tree.
/// It is recommended to reuse a single instance of this struct for all operations on a tree
/// to avoid unnecessary allocations.
#[derive(Resource, Default)]
pub struct ColliderTreeWorkspace {
    /// Builds the tree using PLOC (*Parallel, Locally Ordered Clustering*).
    pub ploc_builder: PlocBuilder,
    /// Restructures the BVH, optimizing node locations within the BVH hierarchy per SAH cost.
    pub reinsertion_optimizer: ReinsertionOptimizer,
    /// Temporary flagged nodes for partial rebuilds.
    pub temp_flags: Vec<bool>,
}

impl Clone for ColliderTreeWorkspace {
    fn clone(&self) -> Self {
        Self {
            ploc_builder: self.ploc_builder.clone(),
            reinsertion_optimizer: ReinsertionOptimizer::default(),
            temp_flags: Vec::new(),
        }
    }
}

impl ColliderTree {
    /// Adds a proxy to the tree, returning its index.
    #[inline]
    pub fn add_proxy(&mut self, aabb: Aabb, proxy: ColliderTreeProxy) -> ProxyId {
        let id = self.proxies.push(proxy) as u32;

        // Insert the proxy into the BVH.
        self.bvh.insert_primitive_greedy(aabb, id);

        // Add to moved proxies.
        self.moved_proxies.push(ProxyId::new(id));

        ProxyId::new(id)
    }

    /// Removes a proxy from the tree.
    ///
    /// Returns `true` if the proxy was successfully removed, or `false` if the proxy ID was invalid.
    #[inline]
    pub fn remove_proxy(&mut self, proxy_id: ProxyId) -> Option<ColliderTreeProxy> {
        if let Some(proxy) = self.proxies.try_remove(proxy_id.index()) {
            // Remove from the BVH.
            self.bvh.remove_primitive(proxy_id.id());

            // Remove from moved proxies.
            self.moved_proxies.remove(proxy_id);

            Some(proxy)
        } else {
            None
        }
    }

    /// Gets a proxy from the tree by its ID.
    ///
    /// Returns `None` if the proxy ID is invalid.
    #[inline]
    pub fn get_proxy(&self, proxy_id: ProxyId) -> Option<&ColliderTreeProxy> {
        self.proxies.get(proxy_id.index())
    }

    /// Gets a mutable reference to a proxy from the tree by its ID.
    ///
    /// Returns `None` if the proxy ID is invalid.
    #[inline]
    pub fn get_proxy_mut(&mut self, proxy_id: ProxyId) -> Option<&mut ColliderTreeProxy> {
        self.proxies.get_mut(proxy_id.index())
    }

    /// Gets a proxy from the tree by its ID without bounds checking.
    ///
    /// # Safety
    ///
    /// The caller must ensure that the `proxy_id` is valid.
    #[inline]
    pub unsafe fn get_proxy_unchecked(&self, proxy_id: ProxyId) -> &ColliderTreeProxy {
        unsafe { self.proxies.get_unchecked(proxy_id.index()) }
    }

    /// Gets a mutable reference to a proxy from the tree by its ID without bounds checking.
    ///
    /// # Safety
    ///
    /// The caller must ensure that the `proxy_id` is valid.
    #[inline]
    pub unsafe fn get_proxy_unchecked_mut(&mut self, proxy_id: ProxyId) -> &mut ColliderTreeProxy {
        unsafe { self.proxies.get_unchecked_mut(proxy_id.index()) }
    }

    /// Gets the AABB of a proxy in the tree.
    ///
    /// Returns `None` if the proxy ID is invalid.
    #[inline]
    pub fn get_proxy_aabb(&self, proxy_id: ProxyId) -> Option<Aabb> {
        let node_id = self.bvh.primitives_to_nodes.get(proxy_id.index())?;
        self.bvh.nodes.get(*node_id as usize).map(|node| node.aabb)
    }

    /// Gets the AABB of a proxy in the tree without bounds checking.
    ///
    /// # Safety
    ///
    /// The caller must ensure that the `proxy_id` is valid.
    #[inline]
    pub unsafe fn get_proxy_aabb_unchecked(&self, proxy_id: ProxyId) -> Aabb {
        unsafe {
            let node_id = *self.bvh.primitives_to_nodes.get_unchecked(proxy_id.index()) as usize;
            self.bvh.nodes.get_unchecked(node_id).aabb
        }
    }

    /// Updates the AABB of a proxy in the tree.
    ///
    /// If the BVH should be refitted at the same time, consider using
    /// [`resize_proxy_aabb`](Self::resize_proxy_aabb) instead.
    ///
    /// If resizing a large number of proxies, consider calling this method
    /// for each proxy and then calling [`refit_all`](Self::refit_all) once at the end.
    #[inline]
    pub fn set_proxy_aabb(&mut self, proxy_id: ProxyId, aabb: Aabb) {
        // Get the node index for the proxy.
        let node_index = self.bvh.primitives_to_nodes[proxy_id.index()] as usize;

        // Update the proxy's AABB in the BVH.
        self.bvh.nodes[node_index].set_aabb(aabb);
    }

    /// Resizes the AABB of a proxy in the tree.
    ///
    /// This is equivalent to calling [`set_proxy_aabb`](Self::set_proxy_aabb)
    /// and then refitting the BVH working up from the resized node.
    ///
    /// For resizing a large number of proxies, consider calling [`set_proxy_aabb`](Self::set_proxy_aabb)
    /// for each proxy and then calling [`refit_all`](Self::refit_all) once at the end.
    #[inline]
    pub fn resize_proxy_aabb(&mut self, proxy_id: ProxyId, aabb: Aabb) {
        let node_index = self.bvh.primitives_to_nodes[proxy_id.index()] as usize;
        self.bvh.resize_node(node_index, aabb);
    }

    /// Updates the AABB of a proxy and reinserts it at an optimal place in the tree.
    #[inline]
    pub fn reinsert_proxy(&mut self, proxy_id: ProxyId, aabb: Aabb) {
        // Reinsert the node into the BVH.
        let node_id = self.bvh.primitives_to_nodes[proxy_id.index()];
        self.bvh.resize_node(node_id as usize, aabb);
        self.bvh.reinsert_node(node_id as usize);
    }

    /// Refits the entire tree from the leaves up.
    #[inline]
    pub fn refit_all(&mut self) {
        self.bvh.refit_all();
    }

    /// Fully rebuilds the tree from the given list of AABBs.
    #[inline]
    pub fn rebuild_full(&mut self) {
        self.workspace.ploc_builder.full_rebuild(
            &mut self.bvh,
            PlocSearchDistance::Minimum,
            SortPrecision::U64,
            0,
        );
    }

    /// Rebuilds parts of the tree corresponding to the given list of leaf node indices.
    #[inline]
    pub fn rebuild_partial(&mut self, leaves: &[u32]) {
        self.bvh.init_parents_if_uninit();

        // TODO: We could maybe get flagged nodes while refitting the tree.
        compute_rebuild_path_flags(&self.bvh, leaves, &mut self.workspace.temp_flags);

        self.workspace.ploc_builder.partial_rebuild(
            &mut self.bvh,
            |node_id| self.workspace.temp_flags[node_id],
            PlocSearchDistance::Minimum,
            SortPrecision::U64,
            0,
        );
    }

    /// Restructures the tree using parallel reinsertion, optimizing node locations based on SAH cost.
    ///
    /// This can be used to improve query performance after the tree quality has degraded,
    /// for example after many proxy insertions and removals.
    #[inline]
    pub fn optimize(&mut self, batch_size_ratio: f32) {
        self.workspace
            .reinsertion_optimizer
            .run(&mut self.bvh, batch_size_ratio, None);
    }

    /// Restructures the tree using parallel reinsertion, optimizing node locations based on SAH cost.
    ///
    /// Only the specified candidate proxies are considered for reinsertion.
    #[inline]
    pub fn optimize_candidates(&mut self, candidates: &[u32], iterations: u32) {
        self.workspace.reinsertion_optimizer.run_with_candidates(
            &mut self.bvh,
            candidates,
            iterations,
        );
    }
}
