use bevy::{
    ecs::{
        entity::{Entity, EntityNotSpawnedError},
        system::{Query, SystemParam, lifetimeless::Write},
        world::Mut,
    },
    transform::{
        components::GlobalTransform,
        helper::{ComputeGlobalTransformError, TransformHelper},
    },
};
use thiserror::Error;

use crate::{
    math::AdjustPrecision,
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
