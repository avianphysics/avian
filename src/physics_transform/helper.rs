use bevy::{
    ecs::{
        entity::{Entity, EntityNotSpawnedError},
        hierarchy::{ChildOf, Children},
        query::With,
        system::{Query, SystemParam, lifetimeless::Write},
        world::Mut,
    },
    transform::components::{GlobalTransform, Transform},
    transform::helper::{ComputeGlobalTransformError, TransformHelper},
};
use thiserror::Error;

#[cfg(feature = "2d")]
use crate::math::Quaternion;
use crate::{
    math::{AdjustPrecision, AsF32},
    prelude::{Position, Rotation},
};

/// A system parameter for computing up-to-date [`Position`] and [`Rotation`] components
/// of entities based on their [`Transform`]s.
///
/// This can be useful to ensure that physics transforms are immediately updated after changes
/// to the [`Transform`], before transform propagation systems are run.
///
/// Computing the global transform of each entity individually can be expensive,
/// so it is recommended to only use this for specific entities that require immediate updates,
/// such as right after teleporting an entity.
///
/// [`Transform`]: bevy::transform::components::Transform
#[derive(SystemParam)]
pub struct PhysicsTransformHelper<'w, 's> {
    /// The [`TransformHelper`] used to compute the global transform.
    pub transform_helper: TransformHelper<'w, 's>,
    /// A query for the [`Position`] and [`Rotation`] components.
    pub query: Query<'w, 's, (Write<Position>, Write<Rotation>)>,
}

impl PhysicsTransformHelper<'_, '_> {
    /// Computes the [`GlobalTransform`] of the given entity from its [`Transform`] and ancestors.
    ///
    /// [`Transform`]: bevy::transform::components::Transform
    pub fn compute_global_transform(
        &self,
        entity: Entity,
    ) -> Result<GlobalTransform, ComputeGlobalTransformError> {
        self.transform_helper.compute_global_transform(entity)
    }

    /// Updates the [`Position`] and [`Rotation`] components of the given entity based on its
    /// [`Transform`] and ancestors.
    ///
    /// Returns a mutable reference to the updated [`Position`] and [`Rotation`] components.
    ///
    /// [`Transform`]: bevy::transform::components::Transform
    pub fn update_physics_transform(
        &mut self,
        entity: Entity,
    ) -> Result<(Mut<'_, Position>, Mut<'_, Rotation>), UpdatePhysicsTransformError> {
        use ComputeGlobalTransformError::*;

        // Compute the global transform.
        let global_transform = self
            .transform_helper
            .compute_global_transform(entity)
            .map_err(|err| match err {
                MissingTransform(e) => UpdatePhysicsTransformError::MissingTransform(e),
                NoSuchEntity(e) => UpdatePhysicsTransformError::NoSuchEntity(e),
                MalformedHierarchy(e) => UpdatePhysicsTransformError::MalformedHierarchy(e),
            })?;

        // Update the physics transform components.
        let Ok((mut position, mut rotation)) = self.query.get_mut(entity) else {
            return Err(UpdatePhysicsTransformError::MissingTransform(entity));
        };
        #[cfg(feature = "2d")]
        {
            position.0 = global_transform.translation().truncate().adjust_precision();
            *rotation = Rotation::from(global_transform.rotation().adjust_precision());
        }
        #[cfg(feature = "3d")]
        {
            position.0 = global_transform.translation().adjust_precision();
            rotation.0 = global_transform.rotation().adjust_precision();
        }

        Ok((position, rotation))
    }
}

/// Error returned by [`PhysicsTransformHelper::update_physics_transform`].
#[derive(Debug, Error)]
pub enum UpdatePhysicsTransformError {
    /// The entity or one of its ancestors is missing either the [`Transform`], [`Position`], or [`Rotation`] component.
    ///
    /// [`Transform`]: bevy::transform::components::Transform
    #[error(
        "The entity {0:?} or one of its ancestors is missing either the `Transform`, `Position`, or `Rotation` component"
    )]
    MissingTransform(Entity),
    /// The entity does not exist.
    #[error("{0}")]
    NoSuchEntity(EntityNotSpawnedError),
    /// An ancestor is missing.
    /// This probably means that your hierarchy has been improperly maintained.
    #[error("{0}")]
    MalformedHierarchy(EntityNotSpawnedError),
}

/// A system parameter for writing up-to-date [`Transform`] components from
/// [`Position`] and [`Rotation`] on demand, without advancing the simulation.
///
/// This is the inverse of [`PhysicsTransformHelper::update_physics_transform`]:
/// it pushes the physics pose out to [`Transform`] immediately, for cases like
/// setting [`Position`]/[`Rotation`] directly and then reading or rendering the
/// result before the schedule (and its `position_to_transform` writeback) runs.
///
/// Like the `position_to_transform` system, nested bodies are resolved against
/// their parent's [`GlobalTransform`].
#[derive(SystemParam)]
pub struct PhysicsTransformWriter<'w, 's> {
    bodies: Query<
        'w,
        's,
        (
            &'static mut Transform,
            &'static Position,
            &'static Rotation,
            Option<&'static ChildOf>,
        ),
    >,
    parents: Query<
        'w,
        's,
        (
            &'static GlobalTransform,
            Option<&'static Position>,
            Option<&'static Rotation>,
        ),
        With<Children>,
    >,
}

impl PhysicsTransformWriter<'_, '_> {
    /// Updates the [`Transform`] of `entity` from its [`Position`] and
    /// [`Rotation`] (and its parent's, for nested bodies). The inverse of
    /// [`PhysicsTransformHelper::update_physics_transform`].
    ///
    /// [`Transform`]: bevy::transform::components::Transform
    pub fn update_transform(&mut self, entity: Entity) -> Result<(), UpdatePhysicsTransformError> {
        let (pos, rot, parent) = {
            let (_, pos, rot, parent) = self
                .bodies
                .get(entity)
                .map_err(|_| UpdatePhysicsTransformError::MissingTransform(entity))?;
            (*pos, *rot, parent.map(|&ChildOf(parent)| parent))
        };

        #[cfg(feature = "3d")]
        let (translation, rotation) = if let Some(parent) = parent {
            let (parent_transform, parent_pos, parent_rot) = self
                .parents
                .get(parent)
                .map_err(|_| UpdatePhysicsTransformError::MissingTransform(parent))?;
            let parent_transform = parent_transform.compute_transform();
            let parent_pos = parent_pos.map_or(parent_transform.translation, |pos| pos.f32());
            let parent_rot = parent_rot.map_or(parent_transform.rotation, |rot| rot.f32());
            let parent_transform = Transform::from_translation(parent_pos)
                .with_rotation(parent_rot)
                .with_scale(parent_transform.scale);
            let new_transform = GlobalTransform::from(
                Transform::from_translation(pos.f32()).with_rotation(rot.f32()),
            )
            .reparented_to(&GlobalTransform::from(parent_transform));
            (new_transform.translation, new_transform.rotation)
        } else {
            (pos.f32(), rot.f32())
        };

        #[cfg(feature = "2d")]
        let (translation, rotation) = {
            let z = self
                .bodies
                .get(entity)
                .map(|(transform, ..)| transform.translation.z)
                .unwrap_or(0.0);
            if let Some(parent) = parent {
                let (parent_transform, parent_pos, parent_rot) = self
                    .parents
                    .get(parent)
                    .map_err(|_| UpdatePhysicsTransformError::MissingTransform(parent))?;
                let parent_transform = parent_transform.compute_transform();
                let parent_pos = parent_pos.map_or(parent_transform.translation, |pos| {
                    pos.f32().extend(parent_transform.translation.z)
                });
                let parent_rot = parent_rot.map_or(parent_transform.rotation, |rot| {
                    Quaternion::from(*rot).f32()
                });
                let parent_scale = parent_transform.scale;
                let parent_transform = Transform::from_translation(parent_pos)
                    .with_rotation(parent_rot)
                    .with_scale(parent_scale);
                let new_transform = GlobalTransform::from(
                    Transform::from_translation(
                        pos.f32().extend(parent_pos.z + z * parent_scale.z),
                    )
                    .with_rotation(Quaternion::from(rot).f32()),
                )
                .reparented_to(&GlobalTransform::from(parent_transform));
                (new_transform.translation, new_transform.rotation)
            } else {
                (pos.f32().extend(z), Quaternion::from(rot).f32())
            }
        };

        let (mut transform, ..) = self
            .bodies
            .get_mut(entity)
            .map_err(|_| UpdatePhysicsTransformError::MissingTransform(entity))?;
        transform.translation = translation;
        transform.rotation = rotation;
        Ok(())
    }
}
