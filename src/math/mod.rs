//! Math types and traits used by the crate.
//!
//! Most of the math types are feature-dependent, so they will be different for `2d`/`3d` and `f32`/`f64`.

#[cfg(feature = "f32")]
mod single;
use approx::abs_diff_ne;
#[cfg(feature = "f32")]
pub use single::*;

#[cfg(feature = "f64")]
mod double;
#[cfg(feature = "f64")]
pub use double::*;

use bevy_math::{prelude::*, *};

/// The active dimension.
#[cfg(feature = "2d")]
pub const DIM: usize = 2;
/// The active dimension.
#[cfg(feature = "3d")]
pub const DIM: usize = 3;

/// The `f32` vector type chosen based on the dimension.
#[cfg(feature = "2d")]
pub(crate) use bevy_math::Vec2 as VectorF32;

/// The `f32` vector type chosen based on the dimension.
#[cfg(feature = "3d")]
pub(crate) use bevy_math::Vec3 as VectorF32;

/// The ray type chosen based on the dimension.
#[cfg(feature = "2d")]
pub(crate) type Ray = Ray2d;
/// The ray type chosen based on the dimension.
#[cfg(feature = "3d")]
pub(crate) type Ray = Ray3d;

// Note: This is called `Dir` instead of `Direction` because Bevy has a conflicting `Direction` type.
/// The direction type chosen based on the dimension.
#[cfg(feature = "2d")]
pub(crate) type Dir = Dir2;
/// The direction type chosen based on the dimension.
#[cfg(feature = "3d")]
pub(crate) type Dir = Dir3;

/// The tensor type chosen based on the dimension.
/// Often used for angular inertia.
///
/// In 2D, this is a scalar, while in 3D, it is a 3x3 matrix.
#[cfg(feature = "2d")]
pub(crate) type Tensor = Scalar;
/// The tensor type chosen based on the dimension.
/// Often used for angular inertia.
///
/// In 2D, this is a scalar, while in 3D, it is a 3x3 matrix.
#[cfg(feature = "3d")]
pub(crate) type Tensor = Matrix3;

/// Adjust the precision of the math construct to the precision chosen for compilation.
pub trait AdjustPrecision {
    /// A math construct type with the desired precision.
    type Adjusted;
    /// Adjusts the precision of [`self`] to [`Self::Adjusted`](#associatedtype.Adjusted).
    fn adjust_precision(&self) -> Self::Adjusted;
}

/// Adjust the precision down to `f32` regardless of compilation.
pub trait AsF32 {
    /// The `f32` version of a math construct.
    type F32;
    /// Returns the `f32` version of this type.
    fn f32(&self) -> Self::F32;
}

impl AsF32 for DVec3 {
    type F32 = Vec3;
    fn f32(&self) -> Self::F32 {
        self.as_vec3()
    }
}

impl AsF32 for Vec3 {
    type F32 = Self;
    fn f32(&self) -> Self::F32 {
        *self
    }
}

impl AsF32 for DVec2 {
    type F32 = Vec2;
    fn f32(&self) -> Self::F32 {
        self.as_vec2()
    }
}

impl AsF32 for Vec2 {
    type F32 = Self;
    fn f32(&self) -> Self::F32 {
        *self
    }
}

impl AsF32 for Vec4 {
    type F32 = Self;
    fn f32(&self) -> Self::F32 {
        *self
    }
}

impl AsF32 for DQuat {
    type F32 = Quat;
    fn f32(&self) -> Self::F32 {
        self.as_quat()
    }
}

impl AsF32 for Quat {
    type F32 = Self;
    fn f32(&self) -> Self::F32 {
        *self
    }
}

impl AsF32 for DMat2 {
    type F32 = Mat2;
    fn f32(&self) -> Self::F32 {
        self.as_mat2()
    }
}

impl AsF32 for Mat2 {
    type F32 = Self;
    fn f32(&self) -> Self::F32 {
        *self
    }
}

impl AsF32 for DMat3 {
    type F32 = Mat3;
    fn f32(&self) -> Self::F32 {
        self.as_mat3()
    }
}

impl AsF32 for Mat3 {
    type F32 = Self;
    fn f32(&self) -> Self::F32 {
        *self
    }
}

#[cfg(feature = "2d")]
pub(crate) fn cross(a: Vector, b: Vector) -> Scalar {
    a.perp_dot(b)
}

#[cfg(feature = "3d")]
pub(crate) fn cross(a: Vector, b: Vector) -> Vector {
    a.cross(b)
}

/// An extension trait for computing reciprocals without division by zero.
pub trait RecipOrZero {
    /// Computes the reciprocal of `self` if `self` is not zero,
    /// and returns zero otherwise to avoid division by zero.
    fn recip_or_zero(self) -> Self;
}

impl RecipOrZero for f32 {
    #[inline]
    fn recip_or_zero(self) -> Self {
        if self != 0.0 && self.is_finite() {
            self.recip()
        } else {
            0.0
        }
    }
}

impl RecipOrZero for f64 {
    #[inline]
    fn recip_or_zero(self) -> Self {
        if self != 0.0 && self.is_finite() {
            self.recip()
        } else {
            0.0
        }
    }
}

impl RecipOrZero for Vec2 {
    #[inline]
    fn recip_or_zero(self) -> Self {
        Self::new(self.x.recip_or_zero(), self.y.recip_or_zero())
    }
}

impl RecipOrZero for Vec3 {
    #[inline]
    fn recip_or_zero(self) -> Self {
        Self::new(
            self.x.recip_or_zero(),
            self.y.recip_or_zero(),
            self.z.recip_or_zero(),
        )
    }
}

impl RecipOrZero for DVec2 {
    #[inline]
    fn recip_or_zero(self) -> Self {
        Self::new(self.x.recip_or_zero(), self.y.recip_or_zero())
    }
}

impl RecipOrZero for DVec3 {
    #[inline]
    fn recip_or_zero(self) -> Self {
        Self::new(
            self.x.recip_or_zero(),
            self.y.recip_or_zero(),
            self.z.recip_or_zero(),
        )
    }
}

/// An extension trait for matrix types.
pub trait MatExt {
    /// The scalar type of the matrix.
    type Scalar;

    /// Computes the inverse of `self` if `self` is not zero,
    /// and returns zero otherwise to avoid division by zero.
    fn inverse_or_zero(self) -> Self;

    /// Checks if the matrix is isotropic, meaning that it is invariant
    /// under all rotations of the coordinate system.
    ///
    /// For second-order tensors, this means that the diagonal elements
    /// are equal and the off-diagonal elements are zero.
    fn is_isotropic(&self, epsilon: Self::Scalar) -> bool;
}

impl MatExt for Mat2 {
    type Scalar = f32;

    #[inline]
    fn inverse_or_zero(self) -> Self {
        if self.determinant() == 0.0 {
            Self::ZERO
        } else {
            self.inverse()
        }
    }

    #[inline]
    fn is_isotropic(&self, epsilon: f32) -> bool {
        // Extract diagonal elements.
        let diag = Vec2::new(self.x_axis.x, self.y_axis.y);

        // All diagonal elements must be approximately equal.
        if abs_diff_ne!(diag.x, diag.y, epsilon = epsilon) {
            return false;
        }

        // Extract off-diagonal elements.
        let off_diag = [self.x_axis.y, self.y_axis.x];

        // All off-diagonal elements must be approximately zero.
        off_diag.iter().all(|&x| x.abs() < epsilon)
    }
}

impl MatExt for DMat2 {
    type Scalar = f64;

    #[inline]
    fn inverse_or_zero(self) -> Self {
        if self.determinant() == 0.0 {
            Self::ZERO
        } else {
            self.inverse()
        }
    }

    #[inline]
    fn is_isotropic(&self, epsilon: f64) -> bool {
        // Extract diagonal elements.
        let diag = DVec2::new(self.x_axis.x, self.y_axis.y);

        // All diagonal elements must be approximately equal.
        if abs_diff_ne!(diag.x, diag.y, epsilon = epsilon) {
            return false;
        }

        // Extract off-diagonal elements.
        let off_diag = [self.x_axis.y, self.y_axis.x];

        // All off-diagonal elements must be approximately zero.
        off_diag.iter().all(|&x| x.abs() < epsilon)
    }
}

impl MatExt for Mat3 {
    type Scalar = f32;

    #[inline]
    fn inverse_or_zero(self) -> Self {
        if self.determinant() == 0.0 {
            Self::ZERO
        } else {
            self.inverse()
        }
    }

    #[inline]
    fn is_isotropic(&self, epsilon: f32) -> bool {
        // Extract diagonal elements.
        let diag = Vec3::new(self.x_axis.x, self.y_axis.y, self.z_axis.z);

        // All diagonal elements must be approximately equal.
        if abs_diff_ne!(diag.x, diag.y, epsilon = epsilon)
            || abs_diff_ne!(diag.y, diag.z, epsilon = epsilon)
        {
            return false;
        }

        // Extract off-diagonal elements.
        let off_diag = [
            self.x_axis.y,
            self.x_axis.z,
            self.y_axis.x,
            self.y_axis.z,
            self.z_axis.x,
            self.z_axis.y,
        ];

        // All off-diagonal elements must be approximately zero.
        off_diag.iter().all(|&x| x.abs() < epsilon)
    }
}

impl MatExt for DMat3 {
    type Scalar = f64;

    #[inline]
    fn inverse_or_zero(self) -> Self {
        if self.determinant() == 0.0 {
            Self::ZERO
        } else {
            self.inverse()
        }
    }

    #[inline]
    fn is_isotropic(&self, epsilon: f64) -> bool {
        // Extract diagonal elements.
        let diag = DVec3::new(self.x_axis.x, self.y_axis.y, self.z_axis.z);

        // All diagonal elements must be approximately equal.
        if abs_diff_ne!(diag.x, diag.y, epsilon = epsilon)
            || abs_diff_ne!(diag.y, diag.z, epsilon = epsilon)
        {
            return false;
        }

        // Extract off-diagonal elements.
        let off_diag = [
            self.x_axis.y,
            self.x_axis.z,
            self.y_axis.x,
            self.y_axis.z,
            self.z_axis.x,
            self.z_axis.y,
        ];

        // All off-diagonal elements must be approximately zero.
        off_diag.iter().all(|&x| x.abs() < epsilon)
    }
}

#[expect(clippy::unnecessary_cast)]
#[cfg(all(feature = "2d", any(feature = "parry-f32", feature = "parry-f64")))]
pub(crate) fn na_iso_to_iso(isometry: &parry::math::Isometry<Scalar>) -> Isometry2d {
    Isometry2d::new(
        Vector::from(isometry.translation).f32(),
        Rot2::from_sin_cos(isometry.rotation.im as f32, isometry.rotation.re as f32),
    )
}

#[cfg(all(
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
use crate::prelude::*;
#[cfg(all(
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
use parry::math::Isometry;

#[cfg(all(
    feature = "2d",
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
pub(crate) fn make_isometry(
    position: impl Into<Position>,
    rotation: impl Into<Rotation>,
) -> Isometry<Scalar> {
    let position: Position = position.into();
    let rotation: Rotation = rotation.into();
    Isometry::<Scalar>::new(position.0.into(), rotation.into())
}

#[cfg(all(
    feature = "3d",
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
pub(crate) fn make_isometry(
    position: impl Into<Position>,
    rotation: impl Into<Rotation>,
) -> Isometry<Scalar> {
    let position: Position = position.into();
    let rotation: Rotation = rotation.into();
    Isometry::<Scalar>::new(position.0.into(), rotation.to_scaled_axis().into())
}

/// Computes the skew-symmetric matrix corresponding to the given vector.
///
/// ```text
///                          [   0  -v.z  v.y ]
/// skew_symmetric_mat3(v) = [  v.z   0  -v.x ]
///                          [ -v.y  v.x   0  ]
/// ```
#[inline]
#[must_use]
#[cfg(feature = "3d")]
pub fn skew_symmetric_mat3(v: Vector3) -> Matrix3 {
    Matrix3::from_cols_array(&[0.0, v.z, -v.y, -v.z, 0.0, v.x, v.y, -v.x, 0.0])
}
