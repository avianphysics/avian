use super::ToRealPrecision;
use crate::{math::DRot2, physics_transform::Rotation};
use bevy_math::*;
use glam_matrix_extras::*;

/// The real number type used by Avian.
///
/// This is a type alias for `f32` in single-precision mode
/// and `f64` in double-precision mode.
pub type Real = f64;

/// The real number vector type used by Avian.
///
/// The type is chosen as follows:
///
/// | Feature | `2d`    | `3d`    |
/// |---------|---------|---------|
/// | `f32`   | `Vec2`  | `Vec3`  |
/// | `f64`   | `DVec2` | `DVec3` |
#[cfg(feature = "2d")]
pub type RVector = DVec2;

/// The real number vector type used by Avian.
///
/// The type is chosen as follows:
///
/// | Feature | `2d`    | `3d`    |
/// |---------|---------|---------|
/// | `f32`   | `Vec2`  | `Vec3`  |
/// | `f64`   | `DVec2` | `DVec3` |
#[cfg(feature = "3d")]
pub type RVector = DVec3;

/// The real number matrix type used by Avian.
///
/// The type is chosen as follows:
///
/// | Feature | `2d`    | `3d`    |
/// |---------|---------|---------|
/// | `f32`   | `Mat2`  | `Mat3`  |
/// | `f64`   | `DMat2` | `DMat3` |
#[cfg(feature = "2d")]
pub type RMatrix = DMat2;

/// The real number matrix type used by Avian.
///
/// The type is chosen as follows:
///
/// | Feature | `2d`    | `3d`    |
/// |---------|---------|---------|
/// | `f32`   | `Mat2`  | `Mat3`  |
/// | `f64`   | `DMat2` | `DMat3` |
#[cfg(feature = "3d")]
pub type RMatrix = DMat3;

impl ToRealPrecision for f32 {
    type Adjusted = Real;
    fn real(&self) -> Self::Adjusted {
        *self as Real
    }
}

impl ToRealPrecision for f64 {
    type Adjusted = Real;
    fn real(&self) -> Self::Adjusted {
        *self
    }
}

impl ToRealPrecision for Vec3 {
    type Adjusted = DVec3;
    fn real(&self) -> Self::Adjusted {
        self.as_dvec3()
    }
}

impl ToRealPrecision for DVec3 {
    type Adjusted = DVec3;
    fn real(&self) -> Self::Adjusted {
        *self
    }
}

impl ToRealPrecision for Vec2 {
    type Adjusted = DVec2;
    fn real(&self) -> Self::Adjusted {
        self.as_dvec2()
    }
}

impl ToRealPrecision for DVec2 {
    type Adjusted = DVec2;
    fn real(&self) -> Self::Adjusted {
        *self
    }
}

impl ToRealPrecision for Quat {
    type Adjusted = DQuat;
    fn real(&self) -> Self::Adjusted {
        self.as_dquat()
    }
}

impl ToRealPrecision for DQuat {
    type Adjusted = DQuat;
    fn real(&self) -> Self::Adjusted {
        *self
    }
}

impl ToRealPrecision for Rot2 {
    type Adjusted = DRot2;
    fn real(&self) -> Self::Adjusted {
        DRot2::from_sin_cos(self.sin as f64, self.cos as f64)
    }
}

impl ToRealPrecision for DRot2 {
    type Adjusted = DRot2;
    fn real(&self) -> Self::Adjusted {
        *self
    }
}

#[cfg(feature = "2d")]
impl ToRealPrecision for Rotation {
    type Adjusted = DRot2;
    fn real(&self) -> Self::Adjusted {
        DRot2::from_sin_cos(self.sin as f64, self.cos as f64)
    }
}

#[cfg(feature = "3d")]
impl ToRealPrecision for Rotation {
    type Adjusted = DQuat;
    fn real(&self) -> Self::Adjusted {
        self.as_dquat()
    }
}
