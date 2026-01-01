use bevy::ecs::{entity::Entity, resource::Resource};
use obvhs::{
    aabb::Aabb,
    bvh2::{Bvh2, insertion_removal::SiblingInsertionCandidate, reinsertion::ReinsertionOptimizer},
    faststack::HeapStack,
    ploc::{PlocBuilder, PlocSearchDistance, SortPrecision},
};
use slab::Slab;

use crate::prelude::CollisionLayers;

/// A Bounding Volume Hierarchy for accelerating queries on a set of colliders.
///
/// Add the [`BroadPhasePairs`] component to use this tree for broad phase collision detection.
#[derive(Clone, Default)]
pub struct ColliderTree {
    /// The underlying BVH structure.
    pub bvh: Bvh2,
    /// The proxies stored in the tree.
    pub proxies: Slab<BvhProxy>,
    /// A workspace for reusing allocations across tree operations.
    pub workspace: ColliderTreeWorkspace,
}
/// A proxy representing a collider in the [`ColliderTree`].
#[derive(Clone, Debug)]
pub struct BvhProxy {
    /// The entity this proxy represents.
    pub entity: Entity,
    /// The collision layers of the collider.
    pub layers: CollisionLayers,
    /// The tight AABB of the collider.
    pub aabb: Aabb,
    /// Flags for the proxy.
    pub flags: u32,
}

/// A workspace for performing operations on a [`ColliderTree`].
///
/// This stores temporary data structures and working memory used when modifying the tree.
/// It is recommended to reuse a single instance of this struct for all operations on a tree
/// to avoid unnecessary allocations.
#[derive(Resource)]
pub struct ColliderTreeWorkspace {
    /// Builds the tree using PLOC (*Parallel, Locally Ordered Clustering*).
    pub ploc_builder: PlocBuilder,
    /// Restructures the BVH, optimizing node locations within the BVH hierarchy per SAH cost.
    pub reinsertion_optimizer: ReinsertionOptimizer,
    /// A stack for tracking insertion candidates during proxy insertions.
    pub insertion_stack: HeapStack<SiblingInsertionCandidate>,
}

impl Clone for ColliderTreeWorkspace {
    fn clone(&self) -> Self {
        Self {
            ploc_builder: self.ploc_builder.clone(),
            reinsertion_optimizer: ReinsertionOptimizer::default(),
            insertion_stack: self.insertion_stack.clone(),
        }
    }
}

impl Default for ColliderTreeWorkspace {
    fn default() -> Self {
        Self {
            ploc_builder: PlocBuilder::default(),
            reinsertion_optimizer: ReinsertionOptimizer::default(),
            insertion_stack: HeapStack::new_with_capacity(2000),
        }
    }
}

impl ColliderTree {
    /// Adds a proxy to the tree, returning its index.
    #[inline]
    pub fn add_proxy(&mut self, aabb: Aabb, proxy: BvhProxy) -> u32 {
        let id = self.proxies.insert(proxy) as u32;
        self.bvh
            .insert_primitive(aabb, id, &mut self.workspace.insertion_stack);
        id
    }

    /// Removes a proxy from the tree.
    #[inline]
    pub fn remove_proxy(&mut self, proxy_index: u32) {
        if self.proxies.try_remove(proxy_index as usize).is_none() {
            return;
        }
        self.bvh.remove_primitive(proxy_index);
    }

    /// Fully rebuilds the tree from the given list of AABBs.
    #[inline]
    pub fn rebuild_full(&mut self) {
        let mut aabbs: Vec<Aabb> = Vec::with_capacity(self.proxies.len());
        let mut indices: Vec<u32> = Vec::with_capacity(self.proxies.len());

        for (i, proxy) in self.proxies.iter() {
            aabbs.push(proxy.aabb);
            indices.push(i as u32);
        }

        self.workspace.ploc_builder.build_with_bvh(
            &mut self.bvh,
            PlocSearchDistance::Minimum,
            &aabbs,
            indices,
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

    /// Updates the AABB of a proxy and reinserts it at an optimal place in the tree.
    #[inline]
    pub fn reinsert_proxy(&mut self, proxy_index: u32, aabb: Aabb) {
        // Update the proxy's AABB.
        self.proxies[proxy_index as usize].aabb = aabb;

        // Reinsert the node into the BVH.
        let node_id = self.bvh.primitives_to_nodes[proxy_index as usize];
        println!("Reinserting proxy {} at node {}", proxy_index, node_id);
        println!("Proxies: {:?}", self.proxies.iter().collect::<Vec<_>>());
        self.bvh.resize_node(node_id as usize, aabb);
        self.bvh.reinsert_node(node_id as usize);
    }

    /// Updates the AABB of a proxy in the tree and refits the BVH, resizing parent nodes as necessary.
    ///
    /// Unlike [`reinsert_proxy`](Self::reinsert_proxy), this does not change the position of the proxy in the tree.
    /// Enlarging proxies withoujt optimizing the tree can degrade query performance over time,
    /// so it is recommended to periodically call [`rebuild_full`](Self::rebuild_full) or [`optimize`](Self::optimize).
    #[inline]
    pub fn enlarge_proxy(&mut self, proxy_index: u32, aabb: Aabb) {
        // Get the node index for the proxy.
        let node_index = self.bvh.primitives_to_nodes[proxy_index as usize] as usize;

        // Update the proxy's AABB in the BVH.
        self.bvh.nodes[node_index].aabb = aabb;

        // Compute the parent indices if they haven't been initialized yet.
        self.bvh.init_parents_if_uninit();

        let mut index = self.bvh.parents[node_index] as usize;

        // Refit the BVH working up the tree.
        // Stop when a parent node's AABB is not changed.
        loop {
            let parent_node = &mut self.bvh.nodes[index];
            let new_aabb = parent_node.aabb.union(&aabb);

            if parent_node.aabb == new_aabb {
                break;
            }

            parent_node.aabb = new_aabb;

            if index == 0 {
                break;
            }

            index = self.bvh.parents[index] as usize;
        }
    }
}
