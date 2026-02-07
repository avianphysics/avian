//! Tree acceleration structures for spatial queries on [colliders](crate::collision::collider::Collider).
//!
//! To speed up [broad phase](crate::collision::broad_phase) collision detection and [spatial queries](crate::spatial_query),
//! Avian maintains a [`ColliderTree`] structure for all colliders in the physics world. This is implemented as
//! a [Bounding Volume Hierarchy (BVH)][BVH], which accelerates querying for [AABB](crate::collision::collider::ColliderAabb)
//! overlaps, ray intersections, and more.
//!
//! Colliders of dynamic, kinematic, and static bodies are all stored in a separate [`ColliderTree`]
//! to allow efficiently querying for specific subsets of colliders and to optimize tree updates based on body type.
//! Trees for dynamic and kinematic bodies are rebuilt every physics step, while the static tree is incrementally updated
//! when static colliders are added, removed, or modified. The trees are stored in the [`ColliderTrees`] resource.
//!
//! [BVH]: https://en.wikipedia.org/wiki/Bounding_volume_hierarchy
//!
//! # Usage
//!
//! The collider trees are fairly low-level, and not intended to be used directly.
//! For spatial queries, consider using the higher-level [`SpatialQuery`] API instead,
//! and for broad phase collision detection, consider using the [`BvhBroadPhasePlugin`].
//!
//! [`SpatialQuery`]: crate::spatial_query::SpatialQuery
//! [`BvhBroadPhasePlugin`]: crate::collision::broad_phase::BvhBroadPhasePlugin

mod diagnostics;
mod obvhs_ext;
mod optimization;
mod proxy_key;
mod traverse;
mod tree;
mod update;

pub use diagnostics::ColliderTreeDiagnostics;
pub use obvhs_ext::Bvh2Ext;
pub(crate) use obvhs_ext::obvhs_ray;
pub use optimization::{ColliderTreeOptimization, TreeOptimizationMode};
pub use proxy_key::{ColliderTreeProxyKey, ColliderTreeType, ProxyId};
pub use tree::{ColliderTree, ColliderTreeProxy, ColliderTreeProxyFlags, ColliderTreeWorkspace};
pub use update::{MovedProxies, update_moved_collider_aabbs};

use optimization::ColliderTreeOptimizationPlugin;
use update::ColliderTreeUpdatePlugin;

use core::marker::PhantomData;

use crate::prelude::*;
use bevy::prelude::*;

/// A plugin that manages [collider trees](crate::collider_tree) for a collider type `C`.
pub struct ColliderTreePlugin<C: AnyCollider>(PhantomData<C>);

impl<C: AnyCollider> Default for ColliderTreePlugin<C> {
    fn default() -> Self {
        Self(PhantomData)
    }
}

impl<C: AnyCollider> Plugin for ColliderTreePlugin<C> {
    fn build(&self, app: &mut App) {
        // Register required components.
        let _ = app.try_register_required_components_with::<C, ColliderTreeProxyKey>(|| {
            // Use a default proxy key. This will be overwritten when the proxy is actually created.
            ColliderTreeProxyKey::PLACEHOLDER
        });

        // Add plugin for updating trees as colliders move.
        app.add_plugins(ColliderTreeUpdatePlugin::<C>::default());

        // Add plugin for optimizing trees tp maintain good query performance.
        if !app.is_plugin_added::<ColliderTreeOptimizationPlugin>() {
            app.add_plugins(ColliderTreeOptimizationPlugin);
        }

        // Initialize resources.
        app.init_resource::<ColliderTrees>()
            .init_resource::<MovedProxies>();

        // Configure system sets.
        app.configure_sets(
            PhysicsSchedule,
            ColliderTreeSystems::UpdateAabbs
                .in_set(PhysicsStepSystems::BroadPhase)
                .after(BroadPhaseSystems::First)
                .before(BroadPhaseSystems::CollectCollisions),
        );
        app.configure_sets(
            PhysicsSchedule,
            ColliderTreeSystems::BeginOptimize.in_set(BroadPhaseSystems::Last),
        );
        app.configure_sets(
            PhysicsSchedule,
            ColliderTreeSystems::EndOptimize.in_set(PhysicsStepSystems::Finalize),
        );
    }

    fn finish(&self, app: &mut App) {
        // Register timer diagnostics for collider trees.
        app.register_physics_diagnostics::<ColliderTreeDiagnostics>();
    }
}

/// System sets for managing [`ColliderTrees`].
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ColliderTreeSystems {
    /// Updates the AABBs of colliders.
    UpdateAabbs,
    /// Begins optimizing acceleration structures to keep their query performance good.
    ///
    /// This runs concurrently with the simulation step as an async task.
    BeginOptimize,
    /// Completes the optimization of acceleration structures started in [`ColliderTreeSystems::BeginOptimize`].
    ///
    /// This runs at the end of the simulation step.
    EndOptimize,
}

/// Trees for accelerating queries on a set of colliders.
///
/// See the [`collider_tree`](crate::collider_tree) module for more information.
#[derive(Resource, Default)]
pub struct ColliderTrees {
    /// A tree for the colliders of dynamic bodies.
    pub dynamic_tree: ColliderTree,
    /// A tree for the colliders of kinematic bodies.
    pub kinematic_tree: ColliderTree,
    /// A tree for the colliders of static bodies.
    pub static_tree: ColliderTree,
    /// A tree for standalone colliders with no associated rigid body.
    pub standalone_tree: ColliderTree,
}

impl ColliderTrees {
    /// Returns the tree for the given [`ColliderTreeType`].
    #[inline]
    pub const fn tree_for_type(&self, tree_type: ColliderTreeType) -> &ColliderTree {
        match tree_type {
            ColliderTreeType::Dynamic => &self.dynamic_tree,
            ColliderTreeType::Kinematic => &self.kinematic_tree,
            ColliderTreeType::Static => &self.static_tree,
            ColliderTreeType::Standalone => &self.standalone_tree,
        }
    }

    /// Returns a mutable reference to the tree for the given [`ColliderTreeType`].
    #[inline]
    pub const fn tree_for_type_mut(&mut self, tree_type: ColliderTreeType) -> &mut ColliderTree {
        match tree_type {
            ColliderTreeType::Dynamic => &mut self.dynamic_tree,
            ColliderTreeType::Kinematic => &mut self.kinematic_tree,
            ColliderTreeType::Static => &mut self.static_tree,
            ColliderTreeType::Standalone => &mut self.standalone_tree,
        }
    }

    /// Returns an iterator over all collider trees.
    #[inline]
    pub fn iter_trees(&self) -> impl Iterator<Item = &ColliderTree> {
        [
            &self.dynamic_tree,
            &self.kinematic_tree,
            &self.static_tree,
            &self.standalone_tree,
        ]
        .into_iter()
    }

    /// Returns a mutable iterator over all collider trees.
    #[inline]
    pub fn iter_trees_mut(&mut self) -> impl Iterator<Item = &mut ColliderTree> {
        [
            &mut self.dynamic_tree,
            &mut self.kinematic_tree,
            &mut self.static_tree,
            &mut self.standalone_tree,
        ]
        .into_iter()
    }

    /// Returns the proxy with the given [`ColliderTreeProxyKey`], if it exists.
    #[inline]
    pub fn get_proxy(&self, key: ColliderTreeProxyKey) -> Option<&ColliderTreeProxy> {
        self.tree_for_type(key.tree_type())
            .proxies
            .get(key.id().index())
    }

    /// Returns a mutable reference to the proxy with the given [`ColliderTreeProxyKey`], if it exists.
    #[inline]
    pub fn get_proxy_mut(&mut self, key: ColliderTreeProxyKey) -> Option<&mut ColliderTreeProxy> {
        self.tree_for_type_mut(key.tree_type())
            .proxies
            .get_mut(key.id().index())
    }
}
