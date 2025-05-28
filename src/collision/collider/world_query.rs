// This is at the module-level because `QueryData` generates a `ColliderQueryItem` type,
// and we cannot easily document or allow missing docs for that.
#![allow(missing_docs)]

use crate::prelude::*;
use bevy::{ecs::query::QueryData, prelude::*};

/// A [`QueryData`] struct to make code handling colliders cleaner.
///
/// This is mostly an internal type, but can also be used by users.
#[derive(QueryData)]
pub struct ColliderQuery<C: AnyCollider> {
    pub entity: Entity,
    pub of: Option<&'static ColliderOf>,
    pub position: Ref<'static, Position>,
    pub rotation: Ref<'static, Rotation>,
    pub transform: Option<&'static ColliderTransform>,
    pub aabb: Ref<'static, ColliderAabb>,
    pub collision_margin: Option<&'static CollisionMargin>,
    pub speculative_margin: Option<&'static SpeculativeMargin>,
    pub is_rb: Has<RigidBody>,
    pub is_sensor: Has<Sensor>,
    pub collision_events_enabled: Has<CollisionEventsEnabled>,
    pub friction: Option<&'static Friction>,
    pub restitution: Option<&'static Restitution>,
    pub shape: &'static C,
    pub layers: &'static CollisionLayers,
    pub active_hooks: Option<&'static ActiveCollisionHooks>,
}

impl<C: AnyCollider> ColliderQueryItem<'_, C> {
    /// Returns the entity of the rigid body that the collider is attached to.
    ///
    /// If the collider is not attached to a rigid body, this will return `None`.
    #[inline(always)]
    pub fn body(&self) -> Option<Entity> {
        self.of.map(|ColliderOf { body }| *body)
    }

    /// Returns the [`ActiveCollisionHooks`] for the collider.
    pub fn active_hooks(&self) -> ActiveCollisionHooks {
        self.active_hooks
            .map_or(ActiveCollisionHooks::empty(), |h| *h)
    }
}
