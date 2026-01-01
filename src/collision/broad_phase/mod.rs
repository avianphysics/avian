//! Finds pairs of entities with overlapping [`ColliderAabb`] and creates contacts
//! for the [narrow phase].
//!
//! [narrow phase]: crate::collision::narrow_phase
//!
//! # Overview
//!
//! To speed up collision detection, the broad phase quickly identifies pairs of entities
//! whose [`ColliderAabb`]s overlap. These contacts are then passed to the [narrow phase]
//! for more detailed collision checks.
//!
//! In Avian, the broad phase is implemented with two plugins:
//!
//! - [`BroadPhaseCorePlugin`]: Sets up resources, system sets, and diagnostics required for broad phase collision detection.
//! - [`BvhBroadPhasePlugin`]: Implements a broad phase using a [Bounding Volume Hierarchy (BVH)][BVH] to efficiently find overlapping AABBs.
//!
//! The former is required for all broad phase implementations, while the latter is an optional plugin
//! that can be replaced with another broad phase strategy if desired. See the following section for details.
//!
//! [BVH]: https://en.wikipedia.org/wiki/Bounding_volume_hierarchy
//!
//! # Custom Broad Phase Implementations
//!
//! By default, Avian uses the [`BvhBroadPhasePlugin`] for broad phase collision detection.
//! However, it is possible to replace it with a custom broad phase strategy, such as
//! sweep and prune (SAP) or some kind of spatial grid.
//!
//! For simplicity's sake, we will demonstrate how to create a simple brute-force O(n^2)
//! broad phase plugin that checks all pairs of colliders for AABB overlaps.
//!
//! In short, all we need to do is add a system to [`BroadPhaseSystems::CollectCollisions`]
//! that finds overlapping AABBs and creates contacts for them in the [`ContactGraph`] resource.
//! However, we are responsible for handling any pair filtering that we might need. This includes:
//!
//! - [`CollisionLayers`]
//! - [`CollisionHooks`]
//! - [`JointCollisionDisabled`]
//! - Skip collisions with parent rigid body
//! - Skip non-dynamic vs non-dynamic pairs
//!
//! and so on. We will only implement a subset of these for demonstration purposes,
//! but you can take a look at the source code of the [`BvhBroadPhasePlugin`] for a complete reference.
//!
//! First, we define our brute-force broad phase plugin:
//!
//! ```
#![cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#![cfg_attr(not(feature = "2d"), doc = "use avian3d::prelude::*;")]
//! use bevy::prelude::*;
//!
//! pub struct BruteForceBroadPhasePlugin;
//!
//! impl Plugin for BruteForceBroadPhasePlugin {
//!     fn build(&self, app: &mut App) {
//!         app.add_systems(
//!             PhysicsSchedule,
//!             collect_collision_pairs.in_set(BroadPhaseSystems::CollectCollisions),
//!         );
//!     }
//! }
//!
//! # fn collect_collision_pairs() {}
//! ```
//!
//! In `collect_collision_pairs`, we query all combinations of colliders,
//! check for AABB overlaps, and create contacts for overlapping colliders:
//!
//! ```
#![cfg_attr(
    feature = "2d",
    doc = "# use avian2d::{dynamics::solver::joint_graph::JointGraph, prelude::*};"
)]
#![cfg_attr(
    not(feature = "2d"),
    doc = "# use avian3d::{dynamics::solver::joint_graph::JointGraph, prelude::*};"
)]
//! # use bevy::prelude::*;
//! #
//! fn collect_collision_pairs(
//!     colliders: Query<(Entity, &ColliderAabb, &CollisionLayers, &ColliderOf)>,
//!     bodies: Query<&RigidBody>,
//!     mut contact_graph: ResMut<ContactGraph>,
//!     joint_graph: Res<JointGraph>,
//! ) {
//!     // Loop through all entity combinations and create contact pairs for overlapping AABBs.
//!     for [
//!         (collider1, aabb1, layers1, collider_of1),
//!         (collider2, aabb2, layers2, collider_of2),
//!     ] in colliders.iter_combinations()
//!     {
//!         // Get the rigid bodies of the colliders.
//!         let Ok(rb1) = bodies.get(collider_of1.body) else {
//!             continue;
//!         };
//!         let Ok(rb2) = bodies.get(collider_of2.body) else {
//!             continue;
//!         };
//!
//!         // Skip pairs where both bodies are non-dynamic.
//!         if !rb1.is_dynamic() && !rb2.is_dynamic() {
//!             continue;
//!         }
//!
//!         // Check if the AABBs intersect.
//!         if !aabb1.intersects(aabb2) {
//!             continue;
//!         }
//!
//!         // Check collision layers.
//!         if !layers1.interacts_with(*layers2) {
//!             continue;
//!         }
//!
//!         // Check if a joint disables contacts between the two bodies.
//!         if joint_graph
//!             .joints_between(collider_of1.body, collider_of2.body)
//!             .any(|edge| edge.collision_disabled)
//!         {
//!             continue;
//!         }
//!
//!         // Create a contact in the contact graph.
//!         let mut contact_edge = ContactEdge::new(collider1, collider2);
//!         contact_edge.body1 = Some(collider_of1.body);
//!         contact_edge.body2 = Some(collider_of2.body);
//!         contact_graph.add_edge(contact_edge);
//!     }
//! }
//! ```
//!
//! Now, we can simply replace the [`BvhBroadPhasePlugin`] with our custom
//! `BruteForceBroadPhasePlugin` when building the app:
//!
//! ```
#![cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#![cfg_attr(not(feature = "2d"), doc = "# use avian3d::prelude::*;")]
//! # use bevy::prelude::*;
//! #
//! # fn main() {
//! #     let mut app = App::new();
//! #
//! app.add_plugins(
//!     PhysicsPlugins::default()
//!         .build()
//!         .disable::<BvhBroadPhasePlugin>()
//!         .add(BruteForceBroadPhasePlugin)
//! );
//! # }
//! ```

mod diagnostics;
pub use diagnostics::BroadPhaseDiagnostics;

mod bvh_broad_phase;
pub use bvh_broad_phase::BvhBroadPhasePlugin;

use crate::prelude::*;
use bevy::prelude::*;

/// The core [broad phase](crate::collision::broad_phase) plugin that sets up the
/// resources, system sets, and diagnostics required for broad phase collision detection.
///
/// This does *not* implement any specific broad phase algorithm by itself,
/// but provides the foundation for other broad phase plugins to build upon.
/// By default, the [`BvhBroadPhasePlugin`] is used, but it can be replaced
/// with a custom strategy if desired.
///
/// See the [module-level documentation](crate::collision::broad_phase) for more information
/// and an example of creating a custom broad phase plugin.
pub struct BroadPhaseCorePlugin;

impl Plugin for BroadPhaseCorePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ContactGraph>();

        app.configure_sets(
            PhysicsSchedule,
            (
                BroadPhaseSystems::First,
                BroadPhaseSystems::CollectCollisions,
                BroadPhaseSystems::Last,
            )
                .chain()
                .in_set(PhysicsStepSystems::BroadPhase),
        );
    }

    fn finish(&self, app: &mut App) {
        // Register timer diagnostics for the broad phase.
        app.register_physics_diagnostics::<BroadPhaseDiagnostics>();
    }
}

/// System sets for systems running in [`PhysicsStepSystems::BroadPhase`].
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum BroadPhaseSystems {
    /// Runs at the start of the broad phase. Empty by default.
    First,
    /// Finds pairs of entities with overlapping [`ColliderAabb`]s
    /// and creates contact pairs for them in [`Collisions`].
    CollectCollisions,
    /// Runs at the end of the broad phase. Empty by default.
    Last,
}
