//! [`ColliderOf`] relationships for attaching colliders to rigid bodies
//! based on the entity hierarchy.

mod plugin;

pub use plugin::ColliderHierarchyPlugin;

use crate::prelude::*;
use bevy::{
    ecs::{
        lifecycle::HookContext, relationship::RelationshipSourceCollection, world::DeferredWorld,
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
#[component(immutable, on_insert = Self::on_insert)]
#[relationship(relationship_target = RigidBodyColliders, allow_self_referential)]
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

impl ColliderOf {
    fn on_insert(mut world: DeferredWorld, HookContext { entity, .. }: HookContext) {
        let collider_ref = world.entity(entity);

        let &ColliderOf { body } = collider_ref.get::<ColliderOf>().unwrap();

        let body_ref = world.entity(body);

        // Get the global transform of the collider and its rigid body.
        let Some(collider_global_transform) = collider_ref.get::<GlobalTransform>() else {
            return;
        };
        let Some(body_global_transform) = body_ref.get::<GlobalTransform>() else {
            return;
        };

        // Get the collider's transform relative to the rigid body.
        let collider_transform = collider_global_transform.reparented_to(body_global_transform);

        // Update the collider transform.
        *world.get_mut::<ColliderTransform>(entity).unwrap() =
            ColliderTransform::from(collider_transform);
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

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::mesh::MeshPlugin;
    #[cfg(feature = "bevy_scene")]
    use bevy::world_serialization::WorldSerializationPlugin;

    #[test]
    fn relationship_behavior() {
        let mut app = App::new();

        app.add_plugins((
            MinimalPlugins,
            AssetPlugin::default(),
            #[cfg(feature = "bevy_scene")]
            WorldSerializationPlugin,
            MeshPlugin,
            PhysicsPlugins::default(),
        ));

        let world = app.world_mut();

        // [`ColliderBackendPlugin`] should have registered [`ColliderMarker`] as
        // a required component of [`Collider`], so the [`ColliderHierarchyPlugin`]
        // should pick up these colliders.

        let body_id = world.spawn((RigidBody::Static, Collider::default())).id();

        let child_id = world.spawn((Collider::default(), ChildOf(body_id))).id();

        let other_id = world
            .spawn((Collider::default(), ColliderOf { body: body_id }))
            .id();

        app.update();

        let world = app.world();

        let body_of_body = world.entity(body_id).get::<ColliderOf>().unwrap().body;
        let body_of_child = world.entity(child_id).get::<ColliderOf>().unwrap().body;
        let body_of_other = world.entity(other_id).get::<ColliderOf>().unwrap().body;

        assert_eq!(body_of_body, body_id);
        assert_eq!(body_of_child, body_id);
        assert_eq!(body_of_other, body_id);

        let colliders = world.entity(body_id).get::<RigidBodyColliders>().unwrap();

        assert!(colliders.contains(&body_id));
        assert!(colliders.contains(&child_id));
        assert!(colliders.contains(&other_id));
        assert_eq!(colliders.len(), 3);
    }
}
