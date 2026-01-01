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
//! In Avian, collision detection is split into two:
//!
//! - [`broad_phase`]: Finds pairs of entities with overlapping [AABBs](ColliderAabb) to reduce the number of potential contacts for the [narrow phase](narrow_phase).
//! - [`narrow_phase`]: Updates and manages contact pairs in the [`ContactGraph`].
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
//! [Collision events](collision_events) can be used for detecting when colliders start or stop touching.
//!
//! Avian provides two collision event types:
//!
//! - [`CollisionStart`]: Triggered when two colliders start touching.
//! - [`CollisionEnd`]: Triggered when two colliders stop touching.
//!
//! Depending on your use case, you may want to read them as [`Message`]s with a [`MessageReader`],
//! or observe them as [`Event`]s with an [observer]. Avian supports both options.
//!
//! Collision events are only sent or triggered for entities that have the [`CollisionEventsEnabled`] component.
//!
//! See the documentation of the event types and the [`collision_events`] module
//! for more information and usage examples.
//!
//! [`Message`]: bevy::ecs::message::Message
//! [`MessageReader`]: bevy::ecs::message::MessageReader
//! [`Event`]: bevy::ecs::event::Event
//! [observer]: bevy::ecs::observer::Observer
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
    pub use super::broad_phase::{BroadPhaseCorePlugin, BroadPhaseSystems, BvhBroadPhasePlugin};
    #[cfg(all(feature = "collider-from-mesh", feature = "default-collider"))]
    pub use super::collider::ColliderCachePlugin;
    pub use super::collider::{
        AabbContext, AnyCollider, ColliderAabb, ColliderBackendPlugin, ColliderDisabled,
        ColliderMarker, CollidingEntities, CollisionLayers, CollisionMargin,
        ContactManifoldContext, IntoCollider, LayerMask, PhysicsLayer, ScalableCollider, Sensor,
        SimpleCollider,
        collider_hierarchy::{ColliderHierarchyPlugin, ColliderOf, RigidBodyColliders},
        collider_transform::{ColliderTransform, ColliderTransformPlugin},
    };
    #[cfg(all(
        feature = "default-collider",
        any(feature = "parry-f32", feature = "parry-f64")
    ))]
    pub use super::collider::{
        Collider, ColliderConstructor, ColliderConstructorHierarchy,
        ColliderConstructorHierarchyReady, ColliderConstructorReady, FillMode, TrimeshFlags,
        VhacdParameters,
    };
    #[expect(deprecated)]
    pub use super::collision_events::{
        CollisionEnd, CollisionEventsEnabled, CollisionStart, OnCollisionEnd, OnCollisionStart,
    };
    pub use super::contact_types::{
        Collisions, ContactEdge, ContactGraph, ContactManifold, ContactPair, ContactPairFlags,
        ContactPoint,
    };
    pub use super::hooks::{ActiveCollisionHooks, CollisionHooks};
    #[expect(deprecated)]
    pub use super::narrow_phase::{
        NarrowPhaseConfig, NarrowPhasePlugin, NarrowPhaseSet, NarrowPhaseSystems,
    };
}

#[expect(unused_imports)]
use crate::prelude::*;
