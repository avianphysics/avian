//! Components for physics positions and rotations.

#![allow(clippy::unnecessary_cast)]

#[cfg(feature = "3d")]
use crate::math::QuatExt;
#[cfg(feature = "2d")]
use crate::math::Rot2Ext;
use crate::{math::Real, physics_transform::PhysicsTransformConfig, prelude::*};
use bevy::{
    ecs::{lifecycle::HookContext, world::DeferredWorld},
    prelude::*,
};
use core::ops::*;
use derive_more::From;

/// The global physics transform of a [rigid body](RigidBody) or a [collider](Collider).
///
/// # Large Worlds
///
/// By default, Avian uses single-precision floating point numbers (`f32`).
/// However, for large worlds, it is possible to enable double-precision support
/// for world-space coordinates by enabling the `f64` feature.
///
/// For the [`PhysicsTransform`], the translation is stored as an [`RVector`],
/// whose underlying type is chosen as follows:
///
/// | Precision | 2D        | 3D        |
/// |-----------|-----------|-----------|
/// | `f32`     | [`Vec2`]  | [`Vec3`]  |
/// | `f64`     | [`DVec2`] | [`DVec3`] |
///
/// While the translation can be stored in double-precision when the `f64`
/// feature is enabled, the rotation is always stored in single-precision,
/// since its precision does not depend on the distance from the origin.
///
/// See the [crate-level documentation](crate#large-worlds) for more information
/// on large worlds and the `f64` feature.
///
/// [`DVec2`]: bevy::math::DVec2
/// [`DVec3`]: bevy::math::DVec3
#[derive(Component, Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
pub struct PhysicsTransform {
    /// The global translation of a [rigid body](RigidBody) or a [collider](Collider).
    pub translation: RVector,
    /// The global rotation of a [rigid body](RigidBody) or a [collider](Collider).
    pub rotation: Rot,
}

impl PhysicsTransform {
    /// Identity transform representing no translation and no rotation.
    pub const IDENTITY: Self = Self {
        translation: RVector::ZERO,
        rotation: Rot::IDENTITY,
    };

    /// A placeholder transform. This is an invalid transform and should *not*
    /// be used to an actually position or rotate entities in the world, but can be used
    /// to indicate that a transform has not yet been initialized.
    pub const PLACEHOLDER: Self = Self {
        translation: RVector::MAX,
        #[cfg(feature = "2d")]
        rotation: Rot2 {
            cos: f32::MAX,
            sin: f32::MAX,
        },
        #[cfg(feature = "3d")]
        rotation: Quat::from_xyzw(f32::MAX, f32::MAX, f32::MAX, f32::MAX),
    };

    /// Creates a new [`PhysicsTransform`] with the given `translation` and `rotation`.
    #[inline(always)]
    pub const fn new(translation: RVector, rotation: Rot) -> Self {
        Self {
            translation,
            rotation,
        }
    }

    /// Creates a new [`PhysicsTransform`] with the given `translation` and no rotation.
    #[inline(always)]
    pub const fn from_translation(translation: RVector) -> Self {
        Self {
            translation,
            rotation: Rot::IDENTITY,
        }
    }

    /// Creates a new [`PhysicsTransform`] with the given `x` and `y` coordinates and no rotation.
    #[cfg(feature = "2d")]
    #[inline(always)]
    pub const fn from_xy(x: Real, y: Real) -> Self {
        Self {
            translation: RVector::new(x, y),
            rotation: Rot::IDENTITY,
        }
    }

    /// Creates a new [`PhysicsTransform`] with the given `x`, `y`, and `z` coordinates and no rotation.
    #[cfg(feature = "3d")]
    #[inline(always)]
    pub const fn from_xyz(x: Real, y: Real, z: Real) -> Self {
        Self {
            translation: RVector::new(x, y, z),
            rotation: Rot::IDENTITY,
        }
    }

    /// Creates a new [`PhysicsTransform`] with the given `rotation` and no translation.
    #[inline(always)]
    pub const fn from_rotation(rotation: Rot) -> Self {
        Self {
            translation: RVector::ZERO,
            rotation,
        }
    }

    /// Returns `true` if both the translation and rotation are neither infinite nor NaN.
    #[inline]
    pub fn is_finite(&self) -> bool {
        self.translation.is_finite() && self.rotation.is_finite()
    }

    /// Returns the inverse of the [`PhysicsTransform`].
    #[inline]
    pub fn inverse(&self) -> Self {
        let inv_rotation = self.rotation.inverse();
        Self {
            translation: inv_rotation * -self.translation,
            rotation: inv_rotation,
        }
    }

    /// Computes `self.inverse() * other` in a more efficient way for one-off calculations.
    ///
    /// If the same inverse is needed multiple times, it is more efficient
    /// to invert `self` once and then use that for each transformation.
    #[inline]
    pub fn inverse_mul(&self, other: &Self) -> Self {
        let inv_rotation = self.rotation.inverse();
        Self {
            translation: inv_rotation * (other.translation - self.translation),
            rotation: inv_rotation * other.rotation,
        }
    }

    /// Transforms a point by rotating and then translating it by `self`.
    #[inline]
    pub fn transform_point(&self, point: RVector) -> RVector {
        self.rotation * point + self.translation
    }

    /// Transforms a point by rotating and then translating it by the inverse of `self`,
    /// in a more efficient way than `self.inverse().transform_point(point)`.
    ///
    /// If the same inverse is needed multiple times, it is more efficient
    /// to invert `self` once and then use that for each transformation.
    #[inline]
    pub fn inverse_transform_point(&self, point: RVector) -> RVector {
        let inv_rotation = self.rotation.inverse();
        inv_rotation * (point - self.translation)
    }
}

impl Default for PhysicsTransform {
    #[inline(always)]
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl From<Isometry> for PhysicsTransform {
    #[inline]
    fn from(isometry: Isometry) -> Self {
        Self {
            #[cfg(feature = "2d")]
            translation: isometry.translation.real(),
            #[cfg(feature = "3d")]
            translation: Vec3::from(isometry.translation).real(),
            rotation: isometry.rotation,
        }
    }
}

impl From<PhysicsTransform> for Isometry {
    #[inline]
    fn from(transform: PhysicsTransform) -> Self {
        Self {
            #[cfg(feature = "2d")]
            translation: transform.translation.f32(),
            #[cfg(feature = "3d")]
            translation: transform.translation.f32().into(),
            rotation: transform.rotation,
        }
    }
}

impl From<(RVector, Rot)> for PhysicsTransform {
    #[inline]
    fn from((translation, rotation): (RVector, Rot)) -> Self {
        Self {
            translation,
            rotation,
        }
    }
}

impl From<PhysicsTransform> for (RVector, Rot) {
    #[inline]
    fn from(transform: PhysicsTransform) -> Self {
        (transform.translation, transform.rotation)
    }
}

impl From<Transform> for PhysicsTransform {
    #[inline]
    fn from(transform: Transform) -> Self {
        Self {
            #[cfg(feature = "2d")]
            translation: transform.translation.truncate().real(),
            #[cfg(feature = "3d")]
            translation: transform.translation.real(),
            rotation: Rot::from_quat(transform.rotation),
        }
    }
}

impl From<GlobalTransform> for PhysicsTransform {
    #[inline]
    fn from(transform: GlobalTransform) -> Self {
        Self::from(&transform)
    }
}

impl From<&GlobalTransform> for PhysicsTransform {
    #[inline]
    fn from(transform: &GlobalTransform) -> Self {
        let (_, rotation, translation) = transform.to_scale_rotation_translation();
        Self {
            #[cfg(feature = "2d")]
            translation: translation.truncate().real(),
            #[cfg(feature = "3d")]
            translation: translation.real(),
            rotation: Rot::from_quat(rotation),
        }
    }
}

impl Mul for PhysicsTransform {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: Self) -> Self::Output {
        Self {
            translation: self.rotation * rhs.translation + self.translation,
            rotation: self.rotation * rhs.rotation,
        }
    }
}

impl MulAssign for PhysicsTransform {
    #[inline]
    fn mul_assign(&mut self, rhs: Self) {
        *self = *self * rhs;
    }
}

impl Mul<RVector> for PhysicsTransform {
    type Output = RVector;

    #[inline]
    fn mul(self, rhs: RVector) -> Self::Output {
        self.transform_point(rhs)
    }
}

#[cfg(feature = "f64")]
impl Mul<Vector> for PhysicsTransform {
    type Output = RVector;

    #[inline]
    fn mul(self, rhs: Vector) -> Self::Output {
        self.translation + (self.rotation * rhs).real()
    }
}

impl Mul<Dir> for PhysicsTransform {
    type Output = Dir;

    #[inline]
    fn mul(self, rhs: Dir) -> Self::Output {
        self.rotation * rhs
    }
}

impl Ease for PhysicsTransform {
    fn interpolating_curve_unbounded(start: Self, end: Self) -> impl Curve<Self> {
        FunctionCurve::new(Interval::UNIT, move |t| PhysicsTransform {
            translation: RVector::lerp(start.translation, end.translation, t as Real),
            rotation: Rot::slerp(start.rotation, end.rotation, t as Real),
        })
    }
}

/// The translation accumulated before the XPBD position solve.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct PreSolveDeltaPosition(pub Vector);

/// The rotation accumulated before the XPBD position solve.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct PreSolveDeltaRotation(pub Rot);

pub(crate) fn init_physics_transform(world: &mut DeferredWorld, ctx: &HookContext) {
    let entity_ref = world.entity(ctx.entity);

    // Get the global `PhysicsTransform`, tracking which parts have not been initialized yet.
    let (mut physics_transform, is_translation_placeholder, is_rotation_placeholder) = entity_ref
        .get::<PhysicsTransform>()
        .map_or((PhysicsTransform::IDENTITY, true, true), |transform| {
            (
                *transform,
                transform.translation == PhysicsTransform::PLACEHOLDER.translation,
                transform.rotation == PhysicsTransform::PLACEHOLDER.rotation,
            )
        });

    if is_translation_placeholder {
        physics_transform.translation = RVector::ZERO;
    }
    if is_rotation_placeholder {
        physics_transform.rotation = Rot::IDENTITY;
    }

    // If either the translation or rotation was set manually,
    // we want to set `Transform` to match later.
    let is_not_placeholder = !is_translation_placeholder || !is_rotation_placeholder;

    let config = world
        .get_resource::<PhysicsTransformConfig>()
        .cloned()
        .unwrap_or_default();

    let mut parent_global_transform = GlobalTransform::default();

    // Compute the global transform by traversing up the hierarchy.
    let mut curr_parent = world.get::<ChildOf>(ctx.entity);
    while let Some(parent) = curr_parent {
        if let Some(parent_transform) = world.get::<Transform>(parent.0) {
            parent_global_transform = *parent_transform * parent_global_transform;
        }
        curr_parent = world.get::<ChildOf>(parent.0);
    }

    let transform = world.get::<Transform>(ctx.entity).copied();
    let global_transform = transform.map(|transform| {
        let global_transform = parent_global_transform * GlobalTransform::from(transform);
        // Update the global transform.
        *world.get_mut::<GlobalTransform>(ctx.entity).unwrap() = global_transform;
        global_transform
    });

    // If either the translation or rotation was not a placeholder,
    // we need to update the `Transform` to match the current values.
    if is_not_placeholder && config.position_to_transform {
        // Get the parent's global transform if it exists.
        if parent_global_transform != GlobalTransform::default() {
            #[cfg(feature = "2d")]
            let Some(transform) = transform else {
                return;
            };

            // The new local transform of the child body, computed from the its global transform
            // and its parents global transform.
            #[cfg(feature = "2d")]
            let new_transform =
                {
                    let (parent_translation, parent_scale) = (
                        parent_global_transform.translation(),
                        parent_global_transform.scale(),
                    );
                    GlobalTransform::from(
                        Transform::from_translation(physics_transform.translation.f32().extend(
                            parent_translation.z + transform.translation.z * parent_scale.z,
                        ))
                        .with_rotation(physics_transform.rotation.to_quat()),
                    )
                    .reparented_to(&parent_global_transform)
                };
            #[cfg(feature = "3d")]
            let new_transform = GlobalTransform::from(
                Transform::from_translation(physics_transform.translation.f32())
                    .with_rotation(physics_transform.rotation),
            )
            .reparented_to(&parent_global_transform);

            // Update the `Transform` of the entity with the new local transform.
            if let Some(mut transform) = world.get_mut::<Transform>(ctx.entity) {
                transform.translation = new_transform.translation;
                transform.rotation = new_transform.rotation;
            }
        } else if let Some(mut transform) = world.get_mut::<Transform>(ctx.entity) {
            // If the entity has no parent, we can set the transform directly.
            if !is_translation_placeholder {
                #[cfg(feature = "2d")]
                {
                    transform.translation = physics_transform
                        .translation
                        .f32()
                        .extend(transform.translation.z);
                }
                #[cfg(feature = "3d")]
                {
                    transform.translation = physics_transform.translation.f32();
                }
            }
            if !is_rotation_placeholder {
                transform.rotation = physics_transform.rotation.to_quat();
            }
        }
    }

    if !config.transform_to_position {
        if (is_translation_placeholder || is_rotation_placeholder)
            && let Some(mut transform) = world.get_mut::<PhysicsTransform>(ctx.entity)
        {
            if is_translation_placeholder {
                transform.translation = RVector::ZERO;
            }
            if is_rotation_placeholder {
                transform.rotation = Rot::IDENTITY;
            }
        }
    } else if is_translation_placeholder || is_rotation_placeholder {
        // If either the translation or rotation is a placeholder, we need to compute the global
        // transform from the hierarchy and set the uninitialized parts to the computed values.

        if let Some(global_transform) = global_transform {
            let global = PhysicsTransform::from(&global_transform);
            if is_translation_placeholder {
                physics_transform.translation = global.translation;
            }
            if is_rotation_placeholder {
                physics_transform.rotation = global.rotation;
            }
        } else {
            // No transform was set. Set the uninitialized parts to default values.
            if is_translation_placeholder {
                physics_transform.translation = RVector::ZERO;
            }
            if is_rotation_placeholder {
                physics_transform.rotation = Rot::IDENTITY;
            }
        }

        // Now we update the actual component value based on the computed global transform,
        // leaving any part that was already set explicitly untouched.
        if let Some(mut transform) = world.get_mut::<PhysicsTransform>(ctx.entity) {
            if transform.translation == PhysicsTransform::PLACEHOLDER.translation {
                transform.translation = physics_transform.translation;
            }
            if transform.rotation == PhysicsTransform::PLACEHOLDER.rotation {
                transform.rotation = physics_transform.rotation;
            }
        }
    }
}
