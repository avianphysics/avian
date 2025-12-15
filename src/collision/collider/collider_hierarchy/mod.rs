//! [`ColliderOf`] relationships for attaching colliders to rigid bodies
//! based on the entity hierarchy.

mod plugin;

pub use plugin::ColliderHierarchyPlugin;

use crate::prelude::*;
use bevy::{
    ecs::{
        lifecycle::HookContext,
        relationship::{Relationship, RelationshipHookMode, RelationshipSourceCollection},
        world::DeferredWorld,
    },
    prelude::*,
};

/// A [`Relationship`] component that attaches a [`Collider`] to a [`RigidBody`].
///
/// Unless manually specified, the [`ColliderOf`] component is automatically initialized
/// with the nearest rigid body up the chain in the entity hierarchy.
///
/// For example, given the following entities:
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// # fn setup(mut commands: Commands) {
/// commands.spawn((
///     RigidBody::Dynamic,
///     Collider::capsule(0.5, 1.5),
///     children![
///         (Collider::capsule(0.5, 1.5), Transform::from_xyz(-2.0, 0.0, 0.0)),
///         (Collider::capsule(0.5, 1.5), Transform::from_xyz(2.0, 0.0, 0.0)),
///     ],
/// ));
/// # }
/// ```
///
/// all three colliders will be attached to the same rigid body.
///
/// However, it also possible to explicitly specify which rigid body a collider is attached to
/// by inserting the [`ColliderOf`] component manually.
///
/// [`Relationship`]: bevy::ecs::relationship::Relationship
#[derive(Component, Clone, Copy, Debug, PartialEq, Eq, Reflect)]
#[component(immutable, on_insert = <ColliderOf as Relationship>::on_insert, on_replace = <ColliderOf as Relationship>::on_replace)]
#[require(ColliderTransform)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
pub struct ColliderOf {
    /// The [`Entity`] ID of the [`RigidBody`] that this collider is attached to.
    pub body: Entity,
}

impl FromWorld for ColliderOf {
    #[inline(always)]
    fn from_world(_world: &mut World) -> Self {
        ColliderOf {
            body: Entity::PLACEHOLDER,
        }
    }
}

// Bevy does not currently allow relationships that point to their own entity,
// so we implement the relationship manually to work around this limitation.
impl Relationship for ColliderOf {
    type RelationshipTarget = RigidBodyColliders;

    fn get(&self) -> Entity {
        self.body
    }

    fn from(entity: Entity) -> Self {
        ColliderOf { body: entity }
    }

    fn on_insert(
        mut world: DeferredWorld,
        HookContext {
            entity,
            caller,
            relationship_hook_mode,
            ..
        }: HookContext,
    ) {
        let collider_ref = world.entity(entity);

        let &ColliderOf { body } = collider_ref.get::<ColliderOf>().unwrap();

        // Get the global transform of the collider and its rigid body.
        let Some(collider_global_transform) = collider_ref.get::<GlobalTransform>() else {
            return;
        };
        let Some(body_global_transform) = world.get::<GlobalTransform>(body) else {
            return;
        };

        // Get the collider's transform relative to the rigid body.
        let collider_transform = collider_global_transform.reparented_to(body_global_transform);

        // Update the collider transform.
        *world.get_mut::<ColliderTransform>(entity).unwrap() =
            ColliderTransform::from(collider_transform);

        // The rest is largely the same as the default implementation,
        // but allows relationships to point to their own entity.

        match relationship_hook_mode {
            RelationshipHookMode::Run => {}
            RelationshipHookMode::Skip => return,
            RelationshipHookMode::RunIfNotLinked => {
                if RigidBodyColliders::LINKED_SPAWN {
                    return;
                }
            }
        }

        let collider = entity;
        let body = world.entity(collider).get::<ColliderOf>().unwrap().body;

        if let Some(mut body_mut) = world
            .get_entity_mut(body)
            .ok()
            .filter(|e| e.contains::<RigidBody>())
        {
            // Attach the collider to the rigid body.
            if let Some(mut colliders) = body_mut.get_mut::<RigidBodyColliders>() {
                colliders.0.push(collider);
            } else {
                world
                    .commands()
                    .entity(body)
                    .insert(RigidBodyColliders(vec![collider]));
            }
        } else {
            warn!(
                "{}Tried to attach collider on entity {collider} to rigid body on entity {body}, but the rigid body does not exist.",
                caller
                    .map(|location| format!("{location}: "))
                    .unwrap_or_default(),
            );
        }
    }

    fn on_replace(
        mut world: DeferredWorld,
        HookContext {
            entity,
            relationship_hook_mode,
            ..
        }: HookContext,
    ) {
        // This is largely the same as the default implementation,
        // but does not panic if the relationship target does not exist.

        match relationship_hook_mode {
            RelationshipHookMode::Run => {}
            RelationshipHookMode::Skip => return,
            RelationshipHookMode::RunIfNotLinked => {
                if <Self::RelationshipTarget as RelationshipTarget>::LINKED_SPAWN {
                    return;
                }
            }
        }
        let body = world.entity(entity).get::<Self>().unwrap().get();
        if let Ok(mut body_mut) = world.get_entity_mut(body)
            && let Some(mut relationship_target) = body_mut.get_mut::<Self::RelationshipTarget>()
        {
            RelationshipSourceCollection::remove(
                relationship_target.collection_mut_risky(),
                entity,
            );
            if relationship_target.is_empty()
                && let Ok(mut entity) = world.commands().get_entity(body)
            {
                // this "remove" operation must check emptiness because in the event that an identical
                // relationship is inserted on top, this despawn would result in the removal of that identical
                // relationship ... not what we want!
                entity.queue_handled(
                    |mut entity: EntityWorldMut| {
                        if entity
                            .get::<Self::RelationshipTarget>()
                            .is_some_and(RelationshipTarget::is_empty)
                        {
                            entity.remove::<Self::RelationshipTarget>();
                        }
                    },
                    |_, _| {},
                );
            }
        }
    }

    fn set_risky(&mut self, entity: Entity) {
        self.body = entity;
    }
}

/// A [`RelationshipTarget`] component that tracks which colliders are attached to a [`RigidBody`].
///
/// This is automatically inserted and pupulated with entities that are attached to a rigid body
/// using the [`ColliderOf`] [`Relationship`] component.
///
/// You should not modify this component directly to avoid desynchronization.
/// Instead, modify the [`ColliderOf`] components on the colliders.
///
/// [`Relationship`]: bevy::ecs::relationship::Relationship
#[derive(Component, Clone, Debug, Default, PartialEq, Reflect)]
#[relationship_target(relationship = ColliderOf, linked_spawn)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct RigidBodyColliders(Vec<Entity>);

impl<'a> IntoIterator for &'a RigidBodyColliders {
    type Item = <Self::IntoIter as Iterator>::Item;

    type IntoIter = core::iter::Copied<core::slice::Iter<'a, Entity>>;

    #[inline(always)]
    fn into_iter(self) -> Self::IntoIter {
        self.0.iter()
    }
}

impl core::ops::Deref for RigidBodyColliders {
    type Target = [Entity];

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
