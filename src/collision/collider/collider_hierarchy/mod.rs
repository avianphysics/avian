//! [`ColliderOf`] relationships for attaching colliders to rigid bodies
//! based on the entity hierarchy.

mod plugin;

pub use plugin::ColliderHierarchyPlugin;

use crate::prelude::*;
use bevy::{
    ecs::{
        component::HookContext,
        relationship::{Relationship, RelationshipHookMode, RelationshipSourceCollection},
        system::SystemParam,
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
        // This is largely the same as the default implementation,
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
                caller.map(|location| format!("{location}: ")).unwrap_or_default(),
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
        if let Ok(mut body_mut) = world.get_entity_mut(body) {
            if let Some(mut relationship_target) = body_mut.get_mut::<Self::RelationshipTarget>() {
                RelationshipSourceCollection::remove(
                    relationship_target.collection_mut_risky(),
                    entity,
                );
                if relationship_target.len() == 0 {
                    if let Ok(mut entity) = world.commands().get_entity(body) {
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
        }
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

#[derive(SystemParam)]
pub struct RigidBodyHelper<'w, 's> {
    rigid_bodies: Query<
        'w,
        's,
        (
            &'static Position,
            &'static Rotation,
            &'static RigidBodyColliders,
        ),
    >,
    colliders: Query<
        'w,
        's,
        (
            &'static Position,
            &'static Rotation,
            &'static Collider,
            Option<&'static CollisionLayers>,
        ),
    >,
}

impl<'w, 's> RigidBodyHelper<'w, 's> {
    pub fn create_compound_collider(&self, entity: Entity) -> Result<Collider> {
        self.create_compound_collider_filtered(entity, &SpatialQueryFilter::DEFAULT)
    }
    pub fn create_compound_collider_filtered(
        &self,
        entity: Entity,
        filter: &SpatialQueryFilter,
    ) -> Result<Collider> {
        let (root_position, root_rotation, colliders) =
            self.rigid_bodies.get(entity).map_err(|_| {
                BevyError::from(
                    format!("Entity {entity} is not a valid rigid body, or has been filtered by a default filter."),
                )
            })?;
        let mut compound_colliders = Vec::new();
        for (position, rotation, collider, layers) in self.colliders.iter_many(colliders.iter()) {
            let layers = layers.copied().unwrap_or_default();
            if !filter.test(entity, layers) {
                continue;
            };
            let relative_position = position.0 - root_position.0;
            let relative_rotation = *root_rotation * rotation.inverse();
            if let Some(compound) = collider.shape_scaled().as_compound() {
                // Need to unpack compound shapes because we are later returning a big compound collider for the whole rigid body
                // and parry crashes on nested compound shapes
                for (isometry, shape) in compound.shapes() {
                    let translation = Vector::from(isometry.translation);
                    #[cfg(feature = "2d")]
                    let rotation = Rotation::radians(isometry.rotation.angle());
                    #[cfg(feature = "3d")]
                    let rotation = Rotation(Quaternion::from(isometry.rotation));
                    compound_colliders.push((
                        relative_position + translation,
                        relative_rotation * rotation,
                        shape.clone().into(),
                    ));
                }
            } else {
                compound_colliders.push((relative_position, relative_rotation, collider.clone()));
            }
        }
        if compound_colliders.is_empty() {
            return Err(BevyError::from(format!(
                "Rigid body {entity} contains no colliders."
            )));
        }
        Ok(Collider::compound(compound_colliders))
    }
    pub fn compute_compound_aabb(&self, entity: Entity) -> Result<ColliderAabb> {
        self.compute_compound_aabb_filtered(entity, &SpatialQueryFilter::DEFAULT)
    }

    pub fn compute_compound_aabb_filtered(
        &self,
        entity: Entity,
        filter: &SpatialQueryFilter,
    ) -> Result<ColliderAabb> {
        todo!()
    }
}

#[cfg(test)]
mod tests {
    #[cfg(feature = "f32")]
    use std::f32::consts::TAU;
    use std::f32::consts::TAU as TAU_F32;
    #[cfg(feature = "f64")]
    use std::f64::consts::TAU;

    use bevy::{scene::ScenePlugin, time::TimeUpdateStrategy};

    use super::*;

    #[test]
    fn creates_compound_collider() {
        let mut app = App::new();
        app.add_plugins(min_plugins);
        app.finish();

        let rigid_body = app
            .world_mut()
            .spawn((
                RigidBody::Dynamic,
                // [1 0 0], 90°
                Transform::from_xyz(1.0, 0.0, 0.0)
                    .with_rotation(Quat::from_rotation_y(TAU_F32 / 4.0)),
                children![(
                    // [1 1 0], 135°
                    Transform::from_xyz(0.0, 1.0, 0.0)
                        .with_rotation(Quat::from_rotation_y(TAU_F32 / 8.0)),
                    children![
                        (
                            // Collider 1:
                            // [1 1 1], 225°
                            Transform::from_xyz(0.0, 0.0, 1.0)
                                .with_rotation(Quat::from_rotation_y(TAU_F32 / 4.0)),
                            Collider::default(),
                        ),
                        (
                            // [2 1 0], 225°
                            Transform::from_xyz(0.0, 0.0, 1.0)
                                .with_rotation(Quat::from_rotation_y(TAU_F32 / 4.0)),
                            Collider::compound(vec![(
                                // Collider 2:
                                // [3 1 0], 270°
                                Vector::X,
                                Quaternion::from_rotation_y(TAU / 8.0),
                                Collider::default()
                            )]),
                        )
                    ]
                )],
            ))
            .id();
        app.update();
        app.update();

        fn validate_compound(In(rigid_body): In<Entity>, rb_helper: RigidBodyHelper) {
            let compound = rb_helper.create_compound_collider(rigid_body).unwrap();
            let compound = compound.shape().as_compound().unwrap();
            let colliders = compound.shapes();

            #[cfg(feature = "2d")]
            let expected = make_isometry(
                Position(vec2(1.0, 0.0).adjust_precision()),
                Rotation::degrees(225.0),
            );
            #[cfg(feature = "3d")]
            let expected = make_isometry(
                Position(vec3(1.0, 0.0, 0.0).adjust_precision()),
                Rotation(Quat::from_rotation_y(225.0_f32.to_radians()).adjust_precision()),
            );

            assert_eq!(expected, colliders[0].0);

            #[cfg(feature = "2d")]
            let expected = make_isometry(
                Position(vec2(1.0, 0.0).adjust_precision()),
                Rotation::degrees(225.0),
            );
            #[cfg(feature = "3d")]
            let expected = make_isometry(
                Position(vec3(1.0, 0.0, 0.0).adjust_precision()),
                Rotation(Quat::from_rotation_y(225.0_f32.to_radians()).adjust_precision()),
            );

            assert_eq!(expected, colliders[1].0);
        }
        app.world_mut()
            .run_system_cached_with(validate_compound, rigid_body)
            .unwrap();
    }

    fn min_plugins(app: &mut App) {
        app.add_plugins((
            MinimalPlugins,
            TransformPlugin,
            AssetPlugin::default(),
            ScenePlugin,
            PhysicsPlugins::default(),
        ))
        .init_resource::<Assets<Mesh>>()
        .insert_resource(TimeUpdateStrategy::ManualDuration(
            Time::<Fixed>::default().timestep(),
        ));
    }
}
