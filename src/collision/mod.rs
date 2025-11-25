//! Collision detection for [`Collider`]s.
//!
//! Collision detection involves determining pairs of objects that may currently be in contact
//! (or are expected to come into contact), and computing contact data for each intersection.
//! These contacts are then used by the [solver](dynamics::solver) to generate [`ContactConstraint`]s
//! and finally resolve overlap.
//!
//! [`ContactConstraint`]: dynamics::solver::contact::ContactConstraint
//!
//! # Plugins
//!
//! In Avian, collision detection is split into two plugins:
//!
//! - [`BroadPhasePlugin`]: Finds pairs of entities with overlapping [AABBs](ColliderAabb) to reduce the number of potential contacts for the [narrow phase](narrow_phase).
//! - [`NarrowPhasePlugin`]: Updates and manages contact pairs in the [`ContactGraph`], and generates [`ContactConstraint`]s for the solver.
//!
//! Spatial queries are handled separately by the [`SpatialQueryPlugin`].
//!
//! You can also find several utility methods for computing contacts in the [`contact_query`](collider::contact_query) module.
//!
//! # Accessing Collisions
//!
//! Contact pairs found by Avian are stored in the [`ContactGraph`] resource.
//! It contains all contacs between entities with overlapping [`ColliderAabb`]s,
//! including contacts where the colliders themselves may not be touching.
//!
//! To make it easier to access relevant collision data, Avian provides a [`Collisions`]
//! system parameter that only provides touching contacts. This is a light wrapper
//! around the [`ContactGraph`] that can often be more convenient to use.
//!
//! See the documentation of [`Collisions`] for more information and usage examples.
//!
//! # Collision Events
//!
//! Collision events can be used for detecting when colliders start or stop touching.
//!
//! Avian provides two buffered collision event types that can be read using an [`EventReader`](bevy::ecs::event::EventReader):
//!
//! - [`CollisionStarted`]
//! - [`CollisionEnded`]
//!
//! These events are good for efficiently processing large numbers of collision events between pairs of entities,
//! such as for detecting bullet hits or playing impact sounds when two objects collide.
//!
//! Avian also provides two collision event types that are triggered for observers:
//!
//! - [`OnCollisionStart`]
//! - [`OnCollisionEnd`]
//!
//! These events are good for entity-specific collision scenarios, such as for detecting when a player
//! steps on a pressure plate or enters a trigger volume.
//!
//! Collision events are only sent or triggered for entities that have the [`CollisionEventsEnabled`] component.
//!
//! See the documentation of the event types and the [`collision_events`] module
//! for more information and usage examples.
//!
//! # Contact Filtering and Modification
//!
//! Some advanced contact scenarios may need to filter or modify contacts
//! with user-defined logic. This can include:
//!
//! - One-way platforms
//! - Conveyor belts
//! - Non-uniform friction and restitution
//!
//! In Avian, this can be done by defining [`CollisionHooks`]. They let you hook into
//! the collision pipeline, and filter or modify contacts with (almost) full ECS access.
//!
//! See the documentation of [`CollisionHooks`] for more information and usage examples.

pub mod broad_phase;
pub mod collider;
pub mod collision_events;
pub mod contact_types;
pub mod hooks;
pub mod narrow_phase;

mod diagnostics;
pub use diagnostics::CollisionDiagnostics;

/// Re-exports common types related to collision detection functionality.
pub mod prelude {
    pub use super::broad_phase::{BroadPhasePlugin, BroadPhaseSet};
    pub use super::collider::{
        collider_hierarchy::{
            ColliderHierarchyPlugin, ColliderOf, RigidBodyColliders, RigidBodyHelper,
        },
        collider_transform::{ColliderTransform, ColliderTransformPlugin},
        AabbContext, AnyCollider, Collider, ColliderAabb, ColliderBackendPlugin,
        ColliderConstructor, ColliderConstructorHierarchy, ColliderDisabled, ColliderMarker,
        CollidingEntities, CollisionLayers, CollisionMargin, ContactManifoldContext, FillMode,
        IntoCollider, LayerMask, PhysicsLayer, ScalableCollider, Sensor, SimpleCollider,
        TrimeshFlags, VhacdParameters,
    };
    pub use super::collision_events::{
        CollisionEnded, CollisionEventsEnabled, CollisionStarted, OnCollisionEnd, OnCollisionStart,
    };
    pub use super::contact_types::{
        Collisions, ContactGraph, ContactManifold, ContactPair, ContactPairFlags, ContactPoint,
    };
    pub use super::hooks::{ActiveCollisionHooks, CollisionHooks};
    pub use super::narrow_phase::{NarrowPhaseConfig, NarrowPhasePlugin, NarrowPhaseSet};
}

#[expect(unused_imports)]
use crate::prelude::*;
