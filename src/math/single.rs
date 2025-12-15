use super::AdjustPrecision;
use bevy_math::*;
use glam_matrix_extras::*;

/// The floating point number type used by Avian.
pub type Scalar = f32;
/// The PI/2 constant.
pub const FRAC_PI_2: Scalar = core::f32::consts::FRAC_PI_2;
/// The PI constant.
pub const PI: Scalar = core::f32::consts::PI;
/// The TAU constant.
pub const TAU: Scalar = core::f32::consts::TAU;
/// 1/sqrt(2)
pub const FRAC_1_SQRT_2: Scalar = core::f32::consts::FRAC_1_SQRT_2;

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
/// The dimension-specific matrix type used by Avian.
#[cfg(feature = "2d")]
pub type SymmetricMatrix = SymmetricMat2;
/// The dimension-specific matrix type used by Avian.
#[cfg(feature = "3d")]
pub type SymmetricMatrix = SymmetricMat3;
/// The 2x2 matrix type used by Avian.
pub type SymmetricMatrix2 = SymmetricMat2;
/// The 3x3 matrix type used by Avian.
pub type SymmetricMatrix3 = SymmetricMat3;
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
