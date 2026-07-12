use super::ToRealPrecision;
use crate::physics_transform::Rotation;
use bevy_math::*;
use glam_matrix_extras::*;

/// The real number type used by Avian.
///
/// This is a type alias for `f32` in single precision mode
/// and `f64` in double precision mode.
pub type Real = f32;

/// The real number vector type used by Avian.
///
/// The type is chosen as follows:
///
/// | Feature | `2d`    | `3d`    |
/// |---------|---------|---------|
/// | `f32`   | `Vec2`  | `Vec3`  |
/// | `f64`   | `DVec2` | `DVec3` |
#[cfg(feature = "2d")]
pub type RVector = Vec2;

/// The real number vector type used by Avian.
///
/// The type is chosen as follows:
///
/// | Feature | `2d`    | `3d`    |
/// |---------|---------|---------|
/// | `f32`   | `Vec2`  | `Vec3`  |
/// | `f64`   | `DVec2` | `DVec3` |
#[cfg(feature = "3d")]
pub type RVector = Vec3;

/// The real number matrix type used by Avian.
///
/// The type is chosen as follows:
///
/// | Feature | `2d`    | `3d`    |
/// |---------|---------|---------|
/// | `f32`   | `Mat2`  | `Mat3`  |
/// | `f64`   | `DMat2` | `DMat3` |
#[cfg(feature = "2d")]
pub type RMatrix = Mat2;

/// The real number matrix type used by Avian.
///
/// The type is chosen as follows:
///
/// | Feature | `2d`    | `3d`    |
/// |---------|---------|---------|
/// | `f32`   | `Mat2`  | `Mat3`  |
/// | `f64`   | `DMat2` | `DMat3` |
#[cfg(feature = "3d")]
pub type RMatrix = Mat3;

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

impl ToRealPrecision for SymmetricMat2 {
    type Adjusted = SymmetricMatrix2;
    fn real(&self) -> Self::Adjusted {
        *self
    }
}

impl ToRealPrecision for SymmetricDMat2 {
    type Adjusted = SymmetricMatrix2;
    fn real(&self) -> Self::Adjusted {
        self.as_symmetric_mat2()
    }
}

impl ToRealPrecision for SymmetricMat3 {
    type Adjusted = SymmetricMatrix3;
    fn real(&self) -> Self::Adjusted {
        *self
    }
}

impl ToRealPrecision for SymmetricDMat3 {
    type Adjusted = SymmetricMatrix3;
    fn real(&self) -> Self::Adjusted {
        self.as_symmetric_mat3()
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
        self.as_rot2()
    }
}

#[cfg(feature = "2d")]
impl ToRealPrecision for Rotation {
    type Adjusted = Rot2;
    fn real(&self) -> Self::Adjusted {
        Rot2::from_sin_cos(self.sin(), self.cos())
    }
}

#[cfg(feature = "3d")]
impl ToRealPrecision for Rotation {
    type Adjusted = Quat;
    fn real(&self) -> Self::Adjusted {
        self.as_quat()
    }
}
