//! Components for physics positions and rotations.

#![allow(clippy::unnecessary_cast)]

use crate::prelude::*;
use bevy::{math::DQuat, prelude::*};
use derive_more::From;

#[cfg(feature = "2d")]
use crate::math::Matrix;

/// The global position of a [rigid body](RigidBody) or a [collider](Collider).
///
/// # Relation to `Transform` and `GlobalTransform`
///
/// [`Position`] is used for physics internally and kept in sync with `Transform`
/// by the [`SyncPlugin`]. It rarely needs to be used directly in your own code, as `Transform` can still
/// be used for almost everything. Using [`Position`] should only be required for managing positions
/// in systems running in the [`SubstepSchedule`]. However, if you prefer, you can also use [`Position`]
/// for everything.
///
/// The reasons why the engine uses a separate [`Position`] component can be found
/// [here](crate#why-are-there-separate-position-and-rotation-components).
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// fn setup(mut commands: Commands) {
///     commands.spawn((
///         RigidBody::Dynamic,
#[cfg_attr(feature = "2d", doc = "         Position::from_xy(0.0, 20.0),")]
#[cfg_attr(feature = "3d", doc = "         Position::from_xyz(0.0, 2.0, 0.0),")]
///     ));
/// }
/// ```
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct Position(pub Vector);

impl Position {
    /// Creates a [`Position`] component with the given global `position`.
    pub fn new(position: Vector) -> Self {
        Self(position)
    }

    /// Creates a [`Position`] component with the global position `(x, y)`.
    #[cfg(feature = "2d")]
    pub fn from_xy(x: Scalar, y: Scalar) -> Self {
        Self(Vector::new(x, y))
    }

    /// Creates a [`Position`] component with the global position `(x, y, z)`.
    #[cfg(feature = "3d")]
    pub fn from_xyz(x: Scalar, y: Scalar, z: Scalar) -> Self {
        Self(Vector::new(x, y, z))
    }
}

impl From<GlobalTransform> for Position {
    #[cfg(feature = "2d")]
    fn from(value: GlobalTransform) -> Self {
        Self::from_xy(
            value.translation().adjust_precision().x,
            value.translation().adjust_precision().y,
        )
    }

    #[cfg(feature = "3d")]
    fn from(value: GlobalTransform) -> Self {
        Self::from_xyz(
            value.translation().adjust_precision().x,
            value.translation().adjust_precision().y,
            value.translation().adjust_precision().z,
        )
    }
}

impl From<&GlobalTransform> for Position {
    #[cfg(feature = "2d")]
    fn from(value: &GlobalTransform) -> Self {
        Self::from_xy(
            value.translation().adjust_precision().x,
            value.translation().adjust_precision().y,
        )
    }

    #[cfg(feature = "3d")]
    fn from(value: &GlobalTransform) -> Self {
        Self::from_xyz(
            value.translation().adjust_precision().x,
            value.translation().adjust_precision().y,
            value.translation().adjust_precision().z,
        )
    }
}

impl Ease for Position {
    fn interpolating_curve_unbounded(start: Self, end: Self) -> impl Curve<Self> {
        FunctionCurve::new(Interval::UNIT, move |t| {
            Position(Vector::lerp(start.0, end.0, t as Scalar))
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
pub struct PreSolveDeltaRotation(pub Rotation);

/// Radians
#[cfg(all(feature = "2d", feature = "default-collider"))]
pub(crate) type RotationValue = Scalar;
/// Quaternion
#[cfg(all(feature = "3d", feature = "default-collider"))]
pub(crate) type RotationValue = Quaternion;

/// The global counterclockwise physics rotation of a [rigid body](RigidBody)
/// or a [collider](Collider) in radians.
///
/// The rotation angle is wrapped to be within the `(-pi, pi]` range.
///
/// # Relation to `Transform` and `GlobalTransform`
///
/// [`Rotation`] is used for physics internally and kept in sync with `Transform`
/// by the [`SyncPlugin`]. It rarely needs to be used directly in your own code, as `Transform` can still
/// be used for almost everything. Using [`Rotation`] should only be required for managing rotations
/// in systems running in the [`SubstepSchedule`], but if you prefer, you can also use [`Rotation`]
/// for everything.
///
/// The reasons why the engine uses a separate [`Rotation`] component can be found
/// [here](crate#why-are-there-separate-position-and-rotation-components).
///
/// # Example
///
/// ```
/// use avian2d::prelude::*;
/// use bevy::prelude::*;
///
/// fn setup(mut commands: Commands) {
///     // Spawn a dynamic rigid body rotated by 90 degrees
///     commands.spawn((RigidBody::Dynamic, Rotation::degrees(90.0)));
/// }
/// ```
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
#[cfg(feature = "2d")]
pub struct Rotation {
    /// The cosine of the rotation angle in radians.
    ///
    /// This is the real part of the unit complex number representing the rotation.
    pub cos: Scalar,
    /// The sine of the rotation angle in radians.
    ///
    /// This is the imaginary part of the unit complex number representing the rotation.
    pub sin: Scalar,
}

#[cfg(feature = "2d")]
impl Default for Rotation {
    fn default() -> Self {
        Self::IDENTITY
    }
}

#[cfg(feature = "2d")]
impl Rotation {
    /// No rotation.
    pub const IDENTITY: Self = Self { cos: 1.0, sin: 0.0 };

    /// A rotation of π radians.
    pub const PI: Self = Self {
        cos: -1.0,
        sin: 0.0,
    };

    /// A counterclockwise rotation of π/2 radians.
    pub const FRAC_PI_2: Self = Self { cos: 0.0, sin: 1.0 };

    /// A counterclockwise rotation of π/3 radians.
    pub const FRAC_PI_3: Self = Self {
        cos: 0.5,
        sin: 0.866_025_4,
    };

    /// A counterclockwise rotation of π/4 radians.
    pub const FRAC_PI_4: Self = Self {
        cos: FRAC_1_SQRT_2,
        sin: FRAC_1_SQRT_2,
    };

    /// A counterclockwise rotation of π/6 radians.
    pub const FRAC_PI_6: Self = Self {
        cos: 0.866_025_4,
        sin: 0.5,
    };

    /// A counterclockwise rotation of π/8 radians.
    pub const FRAC_PI_8: Self = Self {
        cos: 0.923_879_5,
        sin: 0.382_683_43,
    };

    /// Creates a [`Rotation`] from a counterclockwise angle in radians.
    #[inline]
    pub fn radians(radians: Scalar) -> Self {
        #[cfg(feature = "enhanced-determinism")]
        let (sin, cos) = (
            libm::sin(radians as f64) as Scalar,
            libm::cos(radians as f64) as Scalar,
        );
        #[cfg(not(feature = "enhanced-determinism"))]
        let (sin, cos) = radians.sin_cos();

        Self::from_sin_cos(sin, cos)
    }

    /// Creates a [`Rotation`] from a counterclockwise angle in degrees.
    #[inline]
    pub fn degrees(degrees: Scalar) -> Self {
        Self::radians(degrees.to_radians())
    }

    /// Creates a [`Rotation`] from radians.
    #[deprecated(note = "renamed to just `radians` to match Bevy")]
    pub fn from_radians(radians: Scalar) -> Self {
        Self::radians(radians)
    }

    /// Creates a [`Rotation`] from degrees.
    #[deprecated(note = "renamed to just `degrees` to match Bevy")]
    pub fn from_degrees(degrees: Scalar) -> Self {
        Self::degrees(degrees)
    }

    /// Creates a [`Rotation`] from the sine and cosine of an angle in radians.
    ///
    /// The rotation is only valid if `sin * sin + cos * cos == 1.0`.
    ///
    /// # Panics
    ///
    /// Panics if `sin * sin + cos * cos != 1.0` when `debug_assertions` are enabled.
    #[inline]
    pub fn from_sin_cos(sin: Scalar, cos: Scalar) -> Self {
        let rotation = Self { sin, cos };
        debug_assert!(
            rotation.is_normalized(),
            "the given sine and cosine produce an invalid rotation"
        );
        rotation
    }

    /// Returns the rotation in radians in the `(-pi, pi]` range.
    #[inline]
    pub fn as_radians(self) -> Scalar {
        #[cfg(feature = "enhanced-determinism")]
        {
            libm::atan2(self.sin as f64, self.cos as f64) as Scalar
        }
        #[cfg(not(feature = "enhanced-determinism"))]
        {
            Scalar::atan2(self.sin, self.cos)
        }
    }

    /// Returns the rotation in degrees in the `(-180, 180]` range.
    #[inline]
    pub fn as_degrees(self) -> Scalar {
        self.as_radians().to_degrees()
    }

    /// Returns the sine and cosine of the rotation angle in radians.
    #[inline]
    pub const fn sin_cos(self) -> (Scalar, Scalar) {
        (self.sin, self.cos)
    }

    /// Rotates the given vector by `self`.
    #[deprecated(note = "use the `Mul` impl instead, like `rot * vec`")]
    pub fn rotate(&self, vec: Vector) -> Vector {
        self * vec
    }

    /// Computes the length or norm of the complex number used to represent the rotation.
    ///
    /// The length is typically expected to be `1.0`. Unexpectedly denormalized rotations
    /// can be a result of incorrect construction or floating point error caused by
    /// successive operations.
    #[inline]
    #[doc(alias = "norm")]
    pub fn length(self) -> Scalar {
        Vector::new(self.sin, self.cos).length()
    }

    /// Computes the squared length or norm of the complex number used to represent the rotation.
    ///
    /// This is generally faster than [`Rotation::length()`], as it avoids a square
    /// root operation.
    ///
    /// The length is typically expected to be `1.0`. Unexpectedly denormalized rotations
    /// can be a result of incorrect construction or floating point error caused by
    /// successive operations.
    #[inline]
    #[doc(alias = "norm2")]
    pub fn length_squared(self) -> Scalar {
        Vector::new(self.sin, self.cos).length_squared()
    }

    /// Computes `1.0 / self.length()`.
    ///
    /// For valid results, `self` must _not_ have a length of zero.
    #[inline]
    pub fn length_recip(self) -> Scalar {
        Vector::new(self.sin, self.cos).length_recip()
    }

    /// Returns `self` with a length of `1.0` if possible, and `None` otherwise.
    ///
    /// `None` will be returned if the sine and cosine of `self` are both zero (or very close to zero),
    /// or if either of them is NaN or infinite.
    ///
    /// Note that [`Rotation`] should typically already be normalized by design.
    /// Manual normalization is only needed when successive operations result in
    /// accumulated floating point error, or if the rotation was constructed
    /// with invalid values.
    #[inline]
    #[must_use]
    pub fn try_normalize(self) -> Option<Self> {
        let recip = self.length_recip();
        if recip.is_finite() && recip > 0.0 {
            Some(Self::from_sin_cos(self.sin * recip, self.cos * recip))
        } else {
            None
        }
    }

    /// Returns `self` with a length of `1.0`.
    ///
    /// Note that [`Rotation`] should typically already be normalized by design.
    /// Manual normalization is only needed when successive operations result in
    /// accumulated floating point error, or if the rotation was constructed
    /// with invalid values.
    ///
    /// # Panics
    ///
    /// Panics if `self` has a length of zero, NaN, or infinity when debug assertions are enabled.
    #[inline]
    #[must_use]
    pub fn normalize(self) -> Self {
        let length_recip = self.length_recip();
        Self::from_sin_cos(self.sin * length_recip, self.cos * length_recip)
    }

    /// Returns `self` after an approximate normalization,
    /// assuming the value is already nearly normalized.
    /// Useful for preventing numerical error accumulation.
    #[inline]
    #[must_use]
    pub fn fast_renormalize(self) -> Self {
        // First-order Tayor approximation
        // 1/L = (L^2)^(-1/2) ≈ 1 - (L^2 - 1) / 2 = (3 - L^2) / 2
        let length_squared = self.length_squared();
        let approx_inv_length = 0.5 * (3.0 - length_squared);
        Self::from_sin_cos(self.sin * approx_inv_length, self.cos * approx_inv_length)
    }

    /// Returns `true` if the rotation is neither infinite nor NaN.
    #[inline]
    pub fn is_finite(self) -> bool {
        self.sin.is_finite() && self.cos.is_finite()
    }

    /// Returns `true` if the rotation is NaN.
    #[inline]
    pub fn is_nan(self) -> bool {
        self.sin.is_nan() || self.cos.is_nan()
    }

    /// Returns whether `self` has a length of `1.0` or not.
    ///
    /// Uses a precision threshold of approximately `1e-4`.
    #[inline]
    pub fn is_normalized(self) -> bool {
        // The allowed length is 1 +/- 1e-4, so the largest allowed
        // squared length is (1 + 1e-4)^2 = 1.00020001, which makes
        // the threshold for the squared length approximately 2e-4.
        (self.length_squared() - 1.0).abs() <= 2e-4
    }

    /// Returns `true` if the rotation is near [`Rotation::IDENTITY`].
    #[inline]
    pub fn is_near_identity(self) -> bool {
        // Same as `Quat::is_near_identity`, but using sine and cosine
        let threshold_angle_sin = 0.000_049_692_047; // let threshold_angle = 0.002_847_144_6;
        self.cos > 0.0 && self.sin.abs() < threshold_angle_sin
    }

    /// Returns the angle in radians needed to make `self` and `other` coincide.
    #[inline]
    pub fn angle_between(self, other: Self) -> Scalar {
        (other * self.inverse()).as_radians()
    }

    /// Returns the inverse of the rotation. This is also the conjugate
    /// of the unit complex number representing the rotation.
    #[inline]
    #[must_use]
    #[doc(alias = "conjugate")]
    pub fn inverse(self) -> Self {
        Self {
            cos: self.cos,
            sin: -self.sin,
        }
    }

    #[inline]
    #[must_use]
    /// Adds the given counterclockiwise angle in radians to the [`Rotation`].
    /// Uses small-angle approximation
    pub fn add_angle_fast(&self, radians: Scalar) -> Self {
        let (sin, cos) = (self.sin + radians * self.cos, self.cos - radians * self.sin);
        let magnitude_squared = sin * sin + cos * cos;
        let magnitude_recip = if magnitude_squared > 0.0 {
            magnitude_squared.sqrt().recip()
        } else {
            0.0
        };
        Rotation::from_sin_cos(sin * magnitude_recip, cos * magnitude_recip)
    }

    /// Performs a linear interpolation between `self` and `rhs` based on
    /// the value `s`, and normalizes the rotation afterwards.
    ///
    /// When `s == 0.0`, the result will be equal to `self`.
    /// When `s == 1.0`, the result will be equal to `rhs`.
    ///
    /// This is slightly more efficient than [`slerp`](Self::slerp), and produces a similar result
    /// when the difference between the two rotations is small. At larger differences,
    /// the result resembles a kind of ease-in-out effect.
    ///
    /// If you would like the angular velocity to remain constant, consider using [`slerp`](Self::slerp) instead.
    ///
    /// # Details
    ///
    /// `nlerp` corresponds to computing an angle for a point at position `s` on a line drawn
    /// between the endpoints of the arc formed by `self` and `rhs` on a unit circle,
    /// and normalizing the result afterwards.
    ///
    /// Note that if the angles are opposite like 0 and π, the line will pass through the origin,
    /// and the resulting angle will always be either `self` or `rhs` depending on `s`.
    /// If `s` happens to be `0.5` in this case, a valid rotation cannot be computed, and `self`
    /// will be returned as a fallback.
    ///
    /// # Example
    ///
    /// ```
    /// # use approx::assert_relative_eq;
    /// # use avian2d::prelude::Rotation;
    /// #
    /// let rot1 = Rotation::IDENTITY;
    /// let rot2 = Rotation::degrees(135.0);
    ///
    /// let result1 = rot1.nlerp(rot2, 1.0 / 3.0);
    /// assert_relative_eq!(result1.as_degrees(), 28.675055, epsilon = 0.0001);
    ///
    /// let result2 = rot1.nlerp(rot2, 0.5);
    /// assert_relative_eq!(result2.as_degrees(), 67.5);
    /// ```
    #[inline]
    pub fn nlerp(self, end: Self, s: Scalar) -> Self {
        Self {
            sin: self.sin.lerp(end.sin, s),
            cos: self.cos.lerp(end.cos, s),
        }
        .try_normalize()
        // Fall back to the start rotation.
        // This can happen when `self` and `end` are opposite angles and `s == 0.5`,
        // because the resulting rotation would be zero, which cannot be normalized.
        .unwrap_or(self)
    }

    /// Performs a spherical linear interpolation between `self` and `end`
    /// based on the value `s`.
    ///
    /// This corresponds to interpolating between the two angles at a constant angular velocity.
    ///
    /// When `s == 0.0`, the result will be equal to `self`.
    /// When `s == 1.0`, the result will be equal to `rhs`.
    ///
    /// If you would like the rotation to have a kind of ease-in-out effect, consider
    /// using the slightly more efficient [`nlerp`](Self::nlerp) instead.
    ///
    /// # Example
    ///
    /// ```
    /// # use avian2d::prelude::Rotation;
    /// #
    /// let rot1 = Rotation::IDENTITY;
    /// let rot2 = Rotation::degrees(135.0);
    ///
    /// let result1 = rot1.slerp(rot2, 1.0 / 3.0);
    /// assert_eq!(result1.as_degrees(), 45.0);
    ///
    /// let result2 = rot1.slerp(rot2, 0.5);
    /// assert_eq!(result2.as_degrees(), 67.5);
    /// ```
    #[inline]
    pub fn slerp(self, end: Self, s: Scalar) -> Self {
        self * Self::radians(self.angle_between(end) * s)
    }
}

#[cfg(feature = "2d")]
impl From<Scalar> for Rotation {
    /// Creates a [`Rotation`] from a counterclockwise angle in radians.
    fn from(rotation: Scalar) -> Self {
        Self::radians(rotation)
    }
}

#[cfg(feature = "2d")]
impl From<Rotation> for Matrix {
    /// Creates a [`Matrix`] rotation matrix from a [`Rotation`].
    fn from(rot: Rotation) -> Self {
        Matrix::from_cols_array(&[rot.cos, -rot.sin, rot.sin, rot.cos])
    }
}

#[cfg(feature = "2d")]
impl From<Rot2> for Rotation {
    /// Creates a [`Rotation`] from a [`Rot2`].
    fn from(rot: Rot2) -> Self {
        Self::from_sin_cos(rot.sin as Scalar, rot.cos as Scalar)
    }
}

#[cfg(feature = "2d")]
impl From<Rotation> for Rot2 {
    /// Creates a [`Rot2`] from a [`Rotation`].
    fn from(rot: Rotation) -> Self {
        Self::from_sin_cos(rot.sin as f32, rot.cos as f32)
    }
}

#[cfg(feature = "2d")]
impl core::ops::Mul for Rotation {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        Self {
            cos: self.cos * rhs.cos - self.sin * rhs.sin,
            sin: self.sin * rhs.cos + self.cos * rhs.sin,
        }
    }
}

#[cfg(feature = "2d")]
impl core::ops::MulAssign for Rotation {
    fn mul_assign(&mut self, rhs: Self) {
        *self = *self * rhs;
    }
}

#[cfg(feature = "2d")]
impl core::ops::Mul<Vector> for Rotation {
    type Output = Vector;

    /// Rotates a [`Vector`] by a [`Rotation`].
    fn mul(self, rhs: Vector) -> Self::Output {
        Vector::new(
            rhs.x * self.cos - rhs.y * self.sin,
            rhs.x * self.sin + rhs.y * self.cos,
        )
    }
}

#[cfg(feature = "2d")]
impl core::ops::Mul<Vector3> for Rotation {
    type Output = Vector3;

    fn mul(self, rhs: Vector3) -> Self::Output {
        Vector3::new(
            rhs.x * self.cos - rhs.y * self.sin,
            rhs.x * self.sin + rhs.y * self.cos,
            rhs.z,
        )
    }
}

#[cfg(feature = "2d")]
impl core::ops::Mul<&Vector3> for Rotation {
    type Output = Vector3;

    fn mul(self, rhs: &Vector3) -> Self::Output {
        self * *rhs
    }
}

#[cfg(feature = "2d")]
impl core::ops::Mul<&mut Vector3> for Rotation {
    type Output = Vector3;

    fn mul(self, rhs: &mut Vector3) -> Self::Output {
        self * *rhs
    }
}

#[cfg(feature = "2d")]
impl core::ops::Mul<Vector3> for &Rotation {
    type Output = Vector3;

    fn mul(self, rhs: Vector3) -> Self::Output {
        *self * rhs
    }
}

#[cfg(feature = "2d")]
impl core::ops::Mul<&Vector3> for &Rotation {
    type Output = Vector3;

    fn mul(self, rhs: &Vector3) -> Self::Output {
        *self * *rhs
    }
}

#[cfg(feature = "2d")]
impl core::ops::Mul<&mut Vector3> for &Rotation {
    type Output = Vector3;

    fn mul(self, rhs: &mut Vector3) -> Self::Output {
        *self * *rhs
    }
}

#[cfg(feature = "2d")]
impl core::ops::Mul<Vector3> for &mut Rotation {
    type Output = Vector3;

    fn mul(self, rhs: Vector3) -> Self::Output {
        *self * rhs
    }
}

#[cfg(feature = "2d")]
impl core::ops::Mul<&Vector3> for &mut Rotation {
    type Output = Vector3;

    fn mul(self, rhs: &Vector3) -> Self::Output {
        *self * *rhs
    }
}

#[cfg(feature = "2d")]
impl core::ops::Mul<&mut Vector3> for &mut Rotation {
    type Output = Vector3;

    fn mul(self, rhs: &mut Vector3) -> Self::Output {
        *self * *rhs
    }
}

impl Ease for Rotation {
    fn interpolating_curve_unbounded(start: Self, end: Self) -> impl Curve<Self> {
        FunctionCurve::new(Interval::UNIT, move |t| {
            Rotation::slerp(start, end, t as Scalar)
        })
    }
}

/// The global physics rotation of a [rigid body](RigidBody) or a [collider](Collider).
///
/// # Relation to `Transform` and `GlobalTransform`
///
/// [`Rotation`] is used for physics internally and kept in sync with `Transform`
/// by the [`SyncPlugin`]. It rarely needs to be used directly in your own code, as `Transform` can still
/// be used for almost everything. Using [`Rotation`] should only be required for managing rotations
/// in systems running in the [`SubstepSchedule`], but if you prefer, you can also use [`Rotation`]
/// for everything.
///
/// The reasons why the engine uses a separate [`Rotation`] component can be found
/// [here](crate#why-are-there-separate-position-and-rotation-components).
///
/// # Example
///
/// ```
/// use avian3d::prelude::*;
/// use bevy::prelude::*;
///
/// # #[cfg(feature = "f32")]
/// fn setup(mut commands: Commands) {
///     // Spawn a dynamic rigid body rotated by 1.5 radians around the x axis
///     commands.spawn((RigidBody::Dynamic, Rotation(Quat::from_rotation_x(1.5))));
/// }
/// ```
#[cfg(feature = "3d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct Rotation(pub Quaternion);

#[cfg(feature = "3d")]
impl Rotation {
    /// No rotation.
    pub const IDENTITY: Self = Self(Quaternion::IDENTITY);

    /// Inverts the rotation.
    #[inline]
    #[must_use]
    pub fn inverse(&self) -> Self {
        Self(self.0.inverse())
    }

    /// Performs a linear interpolation between `self` and `end` based on
    /// the value `s`, and normalizes the rotation afterwards.
    ///
    /// When `s == 0.0`, the result will be equal to `self`.
    /// When `s == 1.0`, the result will be equal to `end`.
    ///
    /// This is slightly more efficient than [`slerp`](Self::slerp), and produces a similar result
    /// when the difference between the two rotations is small. At larger differences,
    /// the result resembles a kind of ease-in-out effect.
    ///
    /// If you would like the angular velocity to remain constant, consider using [`slerp`](Self::slerp) instead.
    #[inline]
    pub fn nlerp(self, end: Self, t: Scalar) -> Self {
        Self(self.0.lerp(end.0, t))
    }

    /// Performs a spherical linear interpolation between `self` and `end`
    /// based on the value `s`.
    ///
    /// This corresponds to interpolating between the two angles at a constant angular velocity.
    ///
    /// When `s == 0.0`, the result will be equal to `self`.
    /// When `s == 1.0`, the result will be equal to `end`.
    ///
    /// If you would like the rotation to have a kind of ease-in-out effect, consider
    /// using the slightly more efficient [`nlerp`](Self::nlerp) instead.
    #[inline]
    pub fn slerp(self, end: Self, t: Scalar) -> Self {
        Self(self.0.slerp(end.0, t))
    }

    /// Returns `self` after an approximate normalization,
    /// assuming the value is already nearly normalized.
    /// Useful for preventing numerical error accumulation.
    #[inline]
    #[must_use]
    pub fn fast_renormalize(self) -> Self {
        // First-order Tayor approximation
        // 1/L = (L^2)^(-1/2) ≈ 1 - (L^2 - 1) / 2 = (3 - L^2) / 2
        let length_squared = self.length_squared();
        let approx_inv_length = 0.5 * (3.0 - length_squared);
        Self(self.0 * approx_inv_length)
    }
}

#[cfg(feature = "3d")]
impl core::ops::Mul<Vector> for Rotation {
    type Output = Vector;

    fn mul(self, vector: Vector) -> Self::Output {
        self.0 * vector
    }
}

#[cfg(feature = "3d")]
impl core::ops::Mul for Rotation {
    type Output = Rotation;

    fn mul(self, rhs: Self) -> Self::Output {
        Self(self.0 * rhs.0)
    }
}

#[cfg(feature = "3d")]
impl core::ops::MulAssign for Rotation {
    fn mul_assign(&mut self, rhs: Self) {
        self.0 *= rhs.0;
    }
}

impl core::ops::Mul<Dir> for Rotation {
    type Output = Dir;

    fn mul(self, direction: Dir) -> Self::Output {
        Dir::new_unchecked((self * direction.adjust_precision()).f32())
    }
}

impl core::ops::Mul<Vector> for &Rotation {
    type Output = Vector;

    fn mul(self, vector: Vector) -> Self::Output {
        *self * vector
    }
}

impl core::ops::Mul<Dir> for &Rotation {
    type Output = Dir;

    fn mul(self, direction: Dir) -> Self::Output {
        Dir::new_unchecked((*self * direction.adjust_precision()).f32())
    }
}

impl core::ops::Mul<Vector> for &mut Rotation {
    type Output = Vector;

    fn mul(self, vector: Vector) -> Self::Output {
        *self * vector
    }
}

impl core::ops::Mul<Dir> for &mut Rotation {
    type Output = Dir;

    fn mul(self, direction: Dir) -> Self::Output {
        Dir::new_unchecked((*self * direction.adjust_precision()).f32())
    }
}

impl core::ops::Mul<&Vector> for Rotation {
    type Output = Vector;

    fn mul(self, vector: &Vector) -> Self::Output {
        self * *vector
    }
}

impl core::ops::Mul<&Dir> for Rotation {
    type Output = Dir;

    fn mul(self, direction: &Dir) -> Self::Output {
        Dir::new_unchecked((self * direction.adjust_precision()).f32())
    }
}

impl core::ops::Mul<&mut Vector> for Rotation {
    type Output = Vector;

    fn mul(self, vector: &mut Vector) -> Self::Output {
        self * *vector
    }
}

impl core::ops::Mul<&mut Dir> for Rotation {
    type Output = Dir;

    fn mul(self, direction: &mut Dir) -> Self::Output {
        Dir::new_unchecked((self * direction.adjust_precision()).f32())
    }
}

impl core::ops::Mul<&Vector> for &Rotation {
    type Output = Vector;

    fn mul(self, vector: &Vector) -> Self::Output {
        *self * *vector
    }
}

impl core::ops::Mul<&Dir> for &Rotation {
    type Output = Dir;

    fn mul(self, direction: &Dir) -> Self::Output {
        Dir::new_unchecked((*self * direction.adjust_precision()).f32())
    }
}

impl core::ops::Mul<&Vector> for &mut Rotation {
    type Output = Vector;

    fn mul(self, vector: &Vector) -> Self::Output {
        *self * *vector
    }
}

impl core::ops::Mul<&Dir> for &mut Rotation {
    type Output = Dir;

    fn mul(self, direction: &Dir) -> Self::Output {
        Dir::new_unchecked((*self * direction.adjust_precision()).f32())
    }
}

impl core::ops::Mul<&mut Vector> for &Rotation {
    type Output = Vector;

    fn mul(self, vector: &mut Vector) -> Self::Output {
        *self * *vector
    }
}

impl core::ops::Mul<&mut Dir> for &Rotation {
    type Output = Dir;

    fn mul(self, direction: &mut Dir) -> Self::Output {
        Dir::new_unchecked((*self * direction.adjust_precision()).f32())
    }
}

impl core::ops::Mul<&mut Vector> for &mut Rotation {
    type Output = Vector;

    fn mul(self, vector: &mut Vector) -> Self::Output {
        *self * *vector
    }
}

impl core::ops::Mul<&mut Dir> for &mut Rotation {
    type Output = Dir;

    fn mul(self, direction: &mut Dir) -> Self::Output {
        Dir::new_unchecked((*self * direction.adjust_precision()).f32())
    }
}

#[cfg(feature = "2d")]
impl From<Rotation> for Scalar {
    fn from(rot: Rotation) -> Self {
        rot.as_radians()
    }
}

#[cfg(feature = "2d")]
impl From<Rotation> for Quaternion {
    fn from(rot: Rotation) -> Self {
        let z = rot.sin.signum() * ((1.0 - rot.cos) / 2.0).abs().sqrt();
        let w = ((1.0 + rot.cos) / 2.0).abs().sqrt();
        Quaternion::from_xyzw(0.0, 0.0, z, w)
    }
}

#[cfg(feature = "3d")]
impl From<Rotation> for Quaternion {
    fn from(rot: Rotation) -> Self {
        rot.0
    }
}

impl From<Transform> for Rotation {
    fn from(value: Transform) -> Self {
        Self::from(value.rotation)
    }
}

impl From<GlobalTransform> for Rotation {
    fn from(value: GlobalTransform) -> Self {
        Self::from(value.compute_transform().rotation)
    }
}

impl From<&GlobalTransform> for Rotation {
    fn from(value: &GlobalTransform) -> Self {
        Self::from(value.compute_transform().rotation)
    }
}

#[cfg(feature = "2d")]
impl From<Quat> for Rotation {
    fn from(quat: Quat) -> Self {
        let angle = quat.to_euler(EulerRot::XYZ).2;
        Self::radians(angle as Scalar)
    }
}

#[cfg(feature = "2d")]
impl From<DQuat> for Rotation {
    fn from(quat: DQuat) -> Self {
        let angle = quat.to_euler(EulerRot::XYZ).2;
        Self::radians(angle as Scalar)
    }
}

#[cfg(feature = "3d")]
impl From<Quat> for Rotation {
    fn from(quat: Quat) -> Self {
        Self(Quaternion::from_xyzw(
            quat.x as Scalar,
            quat.y as Scalar,
            quat.z as Scalar,
            quat.w as Scalar,
        ))
    }
}

#[cfg(feature = "3d")]
impl From<DQuat> for Rotation {
    fn from(quat: DQuat) -> Self {
        Self(Quaternion::from_xyzw(
            quat.x as Scalar,
            quat.y as Scalar,
            quat.z as Scalar,
            quat.w as Scalar,
        ))
    }
}
