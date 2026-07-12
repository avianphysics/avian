use super::AdjustPrecision;
use crate::{math::DRot2, physics_transform::Rotation};
use bevy_math::*;
use glam_matrix_extras::*;

/// The floating point number type used by Avian.
pub type Scalar = f64;

/// The vector type used by Avian.
#[cfg(feature = "2d")]
pub type Vector = DVec2;
/// The vector type used by Avian.
#[cfg(feature = "3d")]
pub type Vector = DVec3;
/// The vector type used by Avian. This is always a 2D vector regardless of the chosen dimension.
pub type Vector2 = DVec2;
/// The vector type used by Avian. This is always a 3D vector regardless of the chosen dimension.
pub type Vector3 = DVec3;

/// The dimension-specific matrix type used by Avian.
#[cfg(feature = "2d")]
pub type Matrix = DMat2;
/// The dimension-specific matrix type used by Avian.
#[cfg(feature = "3d")]
pub type Matrix = DMat3;
/// The 2x2 matrix type used by Avian.
pub type Matrix2 = DMat2;
/// The 3x3 matrix type used by Avian.
pub type Matrix3 = DMat3;

/// The 2D rotation type used by Avian.
pub type Rotation2 = DRot2;
/// The quaternion type used by Avian.
pub type Quaternion = DQuat;

impl AdjustPrecision for f32 {
    type Adjusted = Scalar;
    fn adjust_precision(&self) -> Self::Adjusted {
        *self as Scalar
    }
}

impl AdjustPrecision for f64 {
    type Adjusted = Scalar;
    fn adjust_precision(&self) -> Self::Adjusted {
        *self
    }
}

impl AdjustPrecision for Vec3 {
    type Adjusted = Vector3;
    fn adjust_precision(&self) -> Self::Adjusted {
        self.as_dvec3()
    }
}

impl AdjustPrecision for DVec3 {
    type Adjusted = Vector3;
    fn adjust_precision(&self) -> Self::Adjusted {
        *self
    }
}

impl AdjustPrecision for Vec2 {
    type Adjusted = Vector2;
    fn adjust_precision(&self) -> Self::Adjusted {
        self.as_dvec2()
    }
}

impl AdjustPrecision for DVec2 {
    type Adjusted = Vector2;
    fn adjust_precision(&self) -> Self::Adjusted {
        *self
    }
}

impl AdjustPrecision for Quat {
    type Adjusted = Quaternion;
    fn adjust_precision(&self) -> Self::Adjusted {
        self.as_dquat()
    }
}

impl AdjustPrecision for DQuat {
    type Adjusted = Quaternion;
    fn adjust_precision(&self) -> Self::Adjusted {
        *self
    }
}

impl AdjustPrecision for Mat3 {
    type Adjusted = Matrix3;
    fn adjust_precision(&self) -> Self::Adjusted {
        self.as_dmat3()
    }
}

impl AdjustPrecision for DMat3 {
    type Adjusted = Matrix3;
    fn adjust_precision(&self) -> Self::Adjusted {
        *self
    }
}

impl AdjustPrecision for Rot2 {
    type Adjusted = DRot2;
    fn adjust_precision(&self) -> Self::Adjusted {
        DRot2::from_sin_cos(self.sin as f64, self.cos as f64)
    }
}

impl AdjustPrecision for DRot2 {
    type Adjusted = DRot2;
    fn adjust_precision(&self) -> Self::Adjusted {
        *self
    }
}

#[cfg(feature = "2d")]
impl AdjustPrecision for Rotation {
    type Adjusted = DRot2;
    fn adjust_precision(&self) -> Self::Adjusted {
        DRot2::from_sin_cos(self.sin as f64, self.cos as f64)
    }
}

#[cfg(feature = "3d")]
impl AdjustPrecision for Rotation {
    type Adjusted = DQuat;
    fn adjust_precision(&self) -> Self::Adjusted {
        self.as_dquat()
    }
}
