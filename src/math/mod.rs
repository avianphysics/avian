//! Math types and traits used by the crate.
//!
//! Most of the math types are feature-dependent, so they will be different for `2d`/`3d` and `f32`/`f64`.

#![allow(unused_imports)]

mod drot2;
pub use drot2::*;

#[cfg(feature = "f32")]
mod single;
#[cfg(feature = "f32")]
pub use single::*;

#[cfg(feature = "f64")]
mod double;
#[cfg(feature = "f64")]
pub use double::*;

use approx::abs_diff_ne;
use bevy_math::{prelude::*, *};
use glam_matrix_extras::{SymmetricDMat2, SymmetricDMat3, SymmetricMat2, SymmetricMat3};

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

/// The `i32` vector type chosen based on the dimension.
#[cfg(feature = "2d")]
pub(crate) use bevy_math::IVec2 as IVector;

/// The `i32` vector type chosen based on the dimension.
#[cfg(feature = "3d")]
pub(crate) use bevy_math::IVec3 as IVector;

/// The ray type chosen based on the dimension.
#[cfg(feature = "2d")]
pub(crate) type Ray = Ray2d;

/// The ray type chosen based on the dimension.
#[cfg(feature = "3d")]
pub(crate) type Ray = Ray3d;

// Note: This is called `Dir` instead of `Direction` because Bevy has a conflicting `Direction` type.
/// The direction type chosen based on the dimension.
#[cfg(feature = "2d")]
pub type Dir = Dir2;

/// The direction type chosen based on the dimension.
#[cfg(feature = "3d")]
pub type Dir = Dir3;

/// The `f32` vector type for angular values chosen based on the dimension.
#[cfg(feature = "2d")]
pub(crate) type AngularVectorF32 = f32;

/// The `f32` vector type for angular values chosen based on the dimension.
#[cfg(feature = "3d")]
pub(crate) type AngularVectorF32 = VectorF32;

/// The `f32` matrix type chosen based on the dimension.
#[cfg(feature = "3d")]
pub(crate) type MatrixF32 = Mat3;

/// The symmetric tensor type chosen based on the dimension.
/// Often used for angular inertia.
///
/// In 2D, this is a scalar, while in 3D, it is a 3x3 matrix.
#[cfg(feature = "2d")]
pub(crate) type SymmetricTensor = f32;

/// The symmetric tensor type chosen based on the dimension.
/// Often used for angular inertia.
///
/// In 2D, this is a scalar, while in 3D, it is a 3x3 matrix.
#[cfg(feature = "3d")]
pub(crate) type SymmetricTensor = SymmetricMat3;

/// The `f32` rotation type chosen based on the dimension.
#[cfg(feature = "2d")]
pub(crate) type RotF32 = Rot2;

/// The `f32` rotation type chosen based on the dimension.
#[cfg(feature = "3d")]
pub(crate) type RotF32 = Quat;

/// The isometry type chosen based on the dimension.
#[cfg(feature = "2d")]
pub(crate) type Isometry = Isometry2d;

/// The isometry type chosen based on the dimension.
#[cfg(feature = "3d")]
pub(crate) type Isometry = Isometry3d;

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

impl AsF32 for f32 {
    type F32 = Self;
    fn f32(&self) -> Self::F32 {
        *self
    }
}

impl AsF32 for f64 {
    type F32 = f32;
    fn f32(&self) -> Self::F32 {
        *self as f32
    }
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

impl AsF32 for SymmetricDMat2 {
    type F32 = SymmetricMat2;
    fn f32(&self) -> Self::F32 {
        SymmetricMat2 {
            m00: self.m00 as f32,
            m01: self.m01 as f32,
            m11: self.m11 as f32,
        }
    }
}

impl AsF32 for SymmetricMat2 {
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

impl AsF32 for SymmetricDMat3 {
    type F32 = SymmetricMat3;
    fn f32(&self) -> Self::F32 {
        SymmetricMat3 {
            m00: self.m00 as f32,
            m01: self.m01 as f32,
            m02: self.m02 as f32,
            m11: self.m11 as f32,
            m12: self.m12 as f32,
            m22: self.m22 as f32,
        }
    }
}

impl AsF32 for SymmetricMat3 {
    type F32 = Self;
    fn f32(&self) -> Self::F32 {
        *self
    }
}

#[cfg(feature = "2d")]
impl AsF32 for Rotation {
    type F32 = Rot2;
    fn f32(&self) -> Self::F32 {
        Rot2::from_sin_cos(self.sin, self.cos)
    }
}

#[cfg(feature = "3d")]
impl AsF32 for Rotation {
    type F32 = Quat;
    fn f32(&self) -> Self::F32 {
        self.0.f32()
    }
}

impl AsF32 for Rot2 {
    type F32 = Self;
    fn f32(&self) -> Self::F32 {
        *self
    }
}

#[cfg(feature = "2d")]
pub(crate) fn cross(a: Vec2, b: Vec2) -> f32 {
    a.perp_dot(b)
}

#[cfg(feature = "3d")]
pub(crate) fn cross(a: Vec3, b: Vec3) -> Vec3 {
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

impl MatExt for SymmetricMat2 {
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
        let diag = Vec2::new(self.m00, self.m11);

        // All diagonal elements must be approximately equal.
        if abs_diff_ne!(diag.x, diag.y, epsilon = epsilon) {
            return false;
        }

        // All off-diagonal elements must be approximately zero.
        self.m01.abs() < epsilon
    }
}

impl MatExt for SymmetricDMat2 {
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
        let diag = DVec2::new(self.m00, self.m11);

        // All diagonal elements must be approximately equal.
        if abs_diff_ne!(diag.x, diag.y, epsilon = epsilon) {
            return false;
        }

        // All off-diagonal elements must be approximately zero.
        self.m01.abs() < epsilon
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

impl MatExt for SymmetricMat3 {
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
        let diag = Vec3::new(self.m00, self.m11, self.m22);

        // All diagonal elements must be approximately equal.
        if abs_diff_ne!(diag.x, diag.y, epsilon = epsilon)
            || abs_diff_ne!(diag.y, diag.z, epsilon = epsilon)
        {
            return false;
        }

        // Extract off-diagonal elements.
        let off_diag = [self.m01, self.m02, self.m12];

        // All off-diagonal elements must be approximately zero.
        off_diag.iter().all(|&x| x.abs() < epsilon)
    }
}

impl MatExt for SymmetricDMat3 {
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
        let diag = DVec3::new(self.m00, self.m11, self.m22);

        // All diagonal elements must be approximately equal.
        if abs_diff_ne!(diag.x, diag.y, epsilon = epsilon)
            || abs_diff_ne!(diag.y, diag.z, epsilon = epsilon)
        {
            return false;
        }

        // Extract off-diagonal elements.
        let off_diag = [self.m01, self.m02, self.m12];

        // All off-diagonal elements must be approximately zero.
        off_diag.iter().all(|&x| x.abs() < epsilon)
    }
}

#[cfg(feature = "2d")]
pub(crate) trait Rot2Ext {
    /// Computes the chord length of the rotation, which is the straight-line
    /// distance between the start and end points of the rotation on a unit circle.
    fn chord_length(self) -> f32;

    /// Adds the given counterclockiwise angle in radians to the [`Rotation`].
    /// Uses small-angle approximation.
    #[must_use]
    fn add_angle_fast(&self, radians: f32) -> Self;
}

#[cfg(feature = "2d")]
impl Rot2Ext for Rot2 {
    #[inline]
    fn chord_length(self) -> f32 {
        // The chord length traveled by a point rotated by `θ` on a unit circle
        // is `2 * sin(θ / 2)`.
        //
        // In 2D, `2 * sin(θ / 2) = sqrt(2 * (1 - cos(θ)))`, using the stored cosine.
        //
        // TODO: A "2D quaternion" that stores cos(theta / 2) and sin(theta / 2)
        //       could avoid the sqrt and be more accurate in some places.
        (2.0 * (1.0 - self.cos)).max(0.0).sqrt()
    }

    #[inline]
    fn add_angle_fast(&self, radians: f32) -> Self {
        let (sin, cos) = (self.sin + radians * self.cos, self.cos - radians * self.sin);
        let magnitude_squared = sin * sin + cos * cos;
        let magnitude_recip = if magnitude_squared > 0.0 {
            magnitude_squared.sqrt().recip()
        } else {
            0.0
        };
        Rot2::from_sin_cos(sin * magnitude_recip, cos * magnitude_recip)
    }
}

#[cfg(feature = "3d")]
pub(crate) trait QuatExt {
    /// Computes the chord length of the rotation, which is the straight-line
    /// distance between the start and end points of the rotation on a unit sphere.
    fn chord_length(self) -> f32;

    /// Returns `self` after an approximate normalization,
    /// assuming the value is already nearly normalized.
    /// Useful for preventing numerical error accumulation.
    #[must_use]
    fn fast_renormalize(self) -> Self;
}

#[cfg(feature = "3d")]
impl QuatExt for Quat {
    #[inline]
    fn chord_length(self) -> f32 {
        2.0 * self.xyz().length()
    }

    #[inline]
    fn fast_renormalize(self) -> Self {
        // First-order Tayor approximation
        // 1/L = (L^2)^(-1/2) ≈ 1 - (L^2 - 1) / 2 = (3 - L^2) / 2
        let length_squared = self.length_squared();
        let approx_inv_length = 0.5 * (3.0 - length_squared);
        self * approx_inv_length
    }
}

#[allow(clippy::unnecessary_cast)]
#[cfg(all(feature = "2d", any(feature = "parry-f32", feature = "parry-f64")))]
pub(crate) fn pose_to_isometry(pose: &parry::math::Pose) -> Isometry2d {
    let rotation = Rot2::from_sin_cos(pose.rotation.im as f32, pose.rotation.re as f32);
    Isometry2d::new(pose.translation.f32(), rotation)
}

#[cfg(all(
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
use crate::prelude::*;

#[cfg(all(
    feature = "2d",
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
pub(crate) fn make_pose(
    position: impl Into<Position>,
    rotation: impl Into<RotF32>,
) -> parry::math::Pose2 {
    let position: Position = position.into();
    let rotation: RotF32 = rotation.into();
    parry::math::Pose2::from_parts(
        position.0,
        parry::math::Rot2::from_cos_sin_unchecked(
            rotation.cos.adjust_precision(),
            rotation.sin.adjust_precision(),
        ),
    )
}

#[cfg(all(
    feature = "3d",
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
pub(crate) fn make_pose(
    position: impl Into<Position>,
    rotation: impl Into<RotF32>,
) -> parry::math::Pose3 {
    let position: Position = position.into();
    let rotation: RotF32 = rotation.into();
    parry::math::Pose3::from_parts(position.0, rotation.adjust_precision())
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

/// Computes the rotation matrix of the orthonormal basis computed from the given axis.
///
/// The `axis` must be a unit vector.
#[inline]
#[must_use]
pub fn orthonormal_basis_from_vec(axis: VectorF32) -> RotF32 {
    #[cfg(feature = "2d")]
    {
        let normal = axis.perp();
        orthonormal_basis([axis, normal])
    }
    #[cfg(feature = "3d")]
    {
        let (normal1, normal2) = axis.any_orthonormal_pair();
        orthonormal_basis([axis, normal1, normal2])
    }
}

/// Computes the rotation matrix of the orthonormal basis computed from the given axes.
///
/// Each axis must be a unit vector.
#[inline]
#[must_use]
pub fn orthonormal_basis(axes: [VectorF32; DIM]) -> RotF32 {
    #[cfg(feature = "2d")]
    {
        Rot2::from_sin_cos(axes[1].x, axes[0].x)
    }
    #[cfg(feature = "3d")]
    {
        let mat = Mat3::from_cols(axes[0], axes[1], axes[2]);
        Quat::from_mat3(&mat)
    }
}

/// Returns a single-precision vector with each component
/// being at least as small as the corresponding component
/// of the input vector.
///
/// This is used for converting double-precision vectors
/// to single-precision vectors for the minimum coordinates
/// of bounding boxes, where they must contain the original
/// shape even after precision loss far from the origin.
#[inline(always)]
#[must_use]
pub fn next_down_vector(vec: Vector) -> VectorF32 {
    #[cfg(feature = "f32")]
    {
        vec
    }
    #[cfg(feature = "f64")]
    {
        #[cfg(feature = "2d")]
        {
            let [x, y] = [vec.x as f32, vec.y as f32];
            VectorF32::new(
                if x as f64 <= vec.x { x } else { x.next_down() },
                if y as f64 <= vec.y { y } else { y.next_down() },
            )
        }
        #[cfg(feature = "3d")]
        {
            let [x, y, z] = [vec.x as f32, vec.y as f32, vec.z as f32];
            VectorF32::new(
                if x as f64 <= vec.x { x } else { x.next_down() },
                if y as f64 <= vec.y { y } else { y.next_down() },
                if z as f64 <= vec.z { z } else { z.next_down() },
            )
        }
    }
}

/// Returns a single-precision vector with each component
/// being at least as large as the corresponding component
/// of the input vector.
///
/// This is used for converting double-precision vectors
/// to single-precision vectors for the maximum coordinates
/// of bounding boxes, where they must contain the original
/// shape even after precision loss far from the origin.
#[inline(always)]
#[must_use]
pub fn next_up_vector(vec: Vector) -> VectorF32 {
    #[cfg(feature = "f32")]
    {
        vec
    }
    #[cfg(feature = "f64")]
    {
        #[cfg(feature = "2d")]
        {
            let [x, y] = [vec.x as f32, vec.y as f32];
            VectorF32::new(
                if x as f64 >= vec.x { x } else { x.next_up() },
                if y as f64 >= vec.y { y } else { y.next_up() },
            )
        }
        #[cfg(feature = "3d")]
        {
            let [x, y, z] = [vec.x as f32, vec.y as f32, vec.z as f32];
            VectorF32::new(
                if x as f64 >= vec.x { x } else { x.next_up() },
                if y as f64 >= vec.y { y } else { y.next_up() },
                if z as f64 >= vec.z { z } else { z.next_up() },
            )
        }
    }
}
