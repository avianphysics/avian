use super::ToRealPrecision;
use crate::{math::DRot2, physics_transform::Rotation};
use bevy_math::*;
use glam_matrix_extras::*;

/// The real number type used by Avian.
///
/// This is a type alias for `f32` in single-precision mode
/// and `f64` in double-precision mode.
pub type Real = f32;

/// The real number 2D vector type used by Avian.
///
/// This is a type alias for `Vec2` in single-precision mode
/// and `DVec2` in double-precision mode.
pub type RVec2 = Vec2;

/// The real number 3D vector type used by Avian.
///
/// This is a type alias for `Vec3` in single-precision mode
/// and `DVec3` in double-precision mode.
pub type RVec3 = Vec3;

/// The real number 2x2 matrix type used by Avian.
///
/// This is a type alias for `Mat2` in single-precision mode
/// and `DMat2` in double-precision mode.
pub type RMat2 = Mat2;

/// The real number 3x3 matrix type used by Avian.
///
/// This is a type alias for `Mat3` in single-precision mode
/// and `DMat3` in double-precision mode.
pub type RMat3 = Mat3;

impl ToRealPrecision for f32 {
    type Adjusted = Real;
    fn real(&self) -> Self::Adjusted {
        *self as Real
    }
}

impl ToRealPrecision for f64 {
    type Adjusted = Real;
    fn real(&self) -> Self::Adjusted {
        *self as Real
    }
}

impl ToRealPrecision for Vec3 {
    type Adjusted = Vec3;
    fn real(&self) -> Self::Adjusted {
        *self
    }
}

impl ToRealPrecision for DVec3 {
    type Adjusted = Vec3;
    fn real(&self) -> Self::Adjusted {
        self.as_vec3()
    }
}

impl ToRealPrecision for Vec2 {
    type Adjusted = Vec2;
    fn real(&self) -> Self::Adjusted {
        *self
    }
}

impl ToRealPrecision for DVec2 {
    type Adjusted = Vec2;
    fn real(&self) -> Self::Adjusted {
        self.as_vec2()
    }
}

impl ToRealPrecision for Quat {
    type Adjusted = Quat;
    fn real(&self) -> Self::Adjusted {
        *self
    }
}

impl ToRealPrecision for DQuat {
    type Adjusted = Quat;
    fn real(&self) -> Self::Adjusted {
        self.as_quat()
    }
}

impl ToRealPrecision for Rot2 {
    type Adjusted = Rot2;
    fn real(&self) -> Self::Adjusted {
        *self
    }
}

impl ToRealPrecision for DRot2 {
    type Adjusted = Rot2;
    fn real(&self) -> Self::Adjusted {
        Rot2::from_sin_cos(self.sin as f32, self.cos as f32)
    }
}

#[cfg(feature = "2d")]
impl ToRealPrecision for Rotation {
    type Adjusted = Rot2;
    fn real(&self) -> Self::Adjusted {
        Rot2::from_sin_cos(self.sin, self.cos)
    }
}

#[cfg(feature = "3d")]
impl ToRealPrecision for Rotation {
    type Adjusted = Quat;
    fn real(&self) -> Self::Adjusted {
        self.0
    }
}
