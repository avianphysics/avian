//! Transform management and types for colliders.

mod plugin;

pub use plugin::ColliderTransformPlugin;

use crate::prelude::*;
use bevy::prelude::*;

/// The transform of a collider relative to the rigid body it's attached to.
/// This is in the local space of the body, not the collider itself.
///
/// This is used for computing things like contact positions and a body's center of mass
/// without having to traverse deeply nested hierarchies. It's updated automatically,
/// so you shouldn't modify it manually.
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
pub struct ColliderTransform {
    /// The translation of a collider in a rigid body's frame of reference.
    pub translation: Vector,
    /// The rotation of a collider in a rigid body's frame of reference.
    pub rotation: Rotation,
    /// The global scale of a collider. Equivalent to the `GlobalTransform` scale.
    pub scale: Vector,
}

impl ColliderTransform {
    /// Transforms a given point by applying the translation, rotation and scale of
    /// this [`ColliderTransform`].
    pub fn transform_point(&self, mut point: Vector) -> Vector {
        point *= self.scale;
        point = self.rotation * point;
        point += self.translation;
        point
    }
}

impl Default for ColliderTransform {
    fn default() -> Self {
        Self {
            translation: Vector::ZERO,
            rotation: Rotation::default(),
            scale: Vector::ONE,
        }
    }
}

impl From<Transform> for ColliderTransform {
    fn from(value: Transform) -> Self {
        Self {
            #[cfg(feature = "2d")]
            translation: value.translation.truncate().adjust_precision(),
            #[cfg(feature = "3d")]
            translation: value.translation.adjust_precision(),
            rotation: Rotation::from(value.rotation.adjust_precision()),
            #[cfg(feature = "2d")]
            scale: value.scale.truncate().adjust_precision(),
            #[cfg(feature = "3d")]
            scale: value.scale.adjust_precision(),
        }
    }
}
