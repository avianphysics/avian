use super::AdjustPrecision;
use crate::physics_transform::Rotation;
use bevy_math::*;
use glam_matrix_extras::*;

/// The floating point number type used by Avian.
pub type Scalar = f32;

/// The vector type used by Avian.
#[cfg(feature = "2d")]
pub type Vector = Vec2;
/// The vector type used by Avian.
#[cfg(feature = "3d")]
pub type Vector = Vec3;
/// The vector type used by Avian. This is always a 2D vector regardless of the chosen dimension.
pub type Vector2 = Vec2;
/// The vector type used by Avian. This is always a 3D vector regardless of the chosen dimension.
pub type Vector3 = Vec3;

/// The dimension-specific matrix type used by Avian.
#[cfg(feature = "2d")]
pub type Matrix = Mat2;
/// The dimension-specific matrix type used by Avian.
#[cfg(feature = "3d")]
pub type Matrix = Mat3;
/// The 2x2 matrix type used by Avian.
pub type Matrix2 = Mat2;
/// The 3x3 matrix type used by Avian.
pub type Matrix3 = Mat3;

/// The 2D rotation type used by Avian.
pub type Rotation2 = Rot2;
/// The quaternion type used by Avian.
pub type Quaternion = Quat;

impl AdjustPrecision for f32 {
    type Adjusted = Scalar;
    fn adjust_precision(&self) -> Self::Adjusted {
        *self as Scalar
    }
}

impl AdjustPrecision for f64 {
    type Adjusted = Scalar;
    fn adjust_precision(&self) -> Self::Adjusted {
        *self as Scalar
    }
}

impl AdjustPrecision for Vec3 {
    type Adjusted = Vector3;
    fn adjust_precision(&self) -> Self::Adjusted {
        *self
    }
}

impl AdjustPrecision for DVec3 {
    type Adjusted = Vector3;
    fn adjust_precision(&self) -> Self::Adjusted {
        self.as_vec3()
    }
}

impl AdjustPrecision for Vec2 {
    type Adjusted = Vector2;
    fn adjust_precision(&self) -> Self::Adjusted {
        *self
    }
}

impl AdjustPrecision for DVec2 {
    type Adjusted = Vector2;
    fn adjust_precision(&self) -> Self::Adjusted {
        self.as_vec2()
    }
}

impl AdjustPrecision for Quat {
    type Adjusted = Quaternion;
    fn adjust_precision(&self) -> Self::Adjusted {
        *self
    }
}

impl AdjustPrecision for DQuat {
    type Adjusted = Quaternion;
    fn adjust_precision(&self) -> Self::Adjusted {
        self.as_quat()
    }
}

impl AdjustPrecision for Mat3 {
    type Adjusted = Matrix3;
    fn adjust_precision(&self) -> Self::Adjusted {
        *self
    }
}

impl AdjustPrecision for DMat3 {
    type Adjusted = Matrix3;
    fn adjust_precision(&self) -> Self::Adjusted {
        self.as_mat3()
    }
}

impl AdjustPrecision for SymmetricMat2 {
    type Adjusted = SymmetricMatrix2;
    fn adjust_precision(&self) -> Self::Adjusted {
        *self
    }
}

impl AdjustPrecision for SymmetricDMat2 {
    type Adjusted = SymmetricMatrix2;
    fn adjust_precision(&self) -> Self::Adjusted {
        self.as_symmetric_mat2()
    }
}

impl AdjustPrecision for SymmetricMat3 {
    type Adjusted = SymmetricMatrix3;
    fn adjust_precision(&self) -> Self::Adjusted {
        *self
    }
}

impl AdjustPrecision for SymmetricDMat3 {
    type Adjusted = SymmetricMatrix3;
    fn adjust_precision(&self) -> Self::Adjusted {
        self.as_symmetric_mat3()
    }
}

impl AdjustPrecision for Rot2 {
    type Adjusted = Rot2;
    fn adjust_precision(&self) -> Self::Adjusted {
        *self
    }
}

impl AdjustPrecision for DRot2 {
    type Adjusted = Rot2;
    fn adjust_precision(&self) -> Self::Adjusted {
        self.as_rot2()
    }
}

#[cfg(feature = "2d")]
impl AdjustPrecision for Rotation {
    type Adjusted = Rot2;
    fn adjust_precision(&self) -> Self::Adjusted {
        Rot2::from_sin_cos(self.sin(), self.cos())
    }
}

#[cfg(feature = "3d")]
impl AdjustPrecision for Rotation {
    type Adjusted = Quat;
    fn adjust_precision(&self) -> Self::Adjusted {
        self.as_quat()
    }
}
