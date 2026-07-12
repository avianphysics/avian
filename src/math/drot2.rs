use core::f64::consts::TAU;

use crate::ops;
use bevy_math::{DMat2, DVec2, prelude::*};

use bevy::reflect::prelude::*;
#[cfg(feature = "serialize")]
use bevy::reflect::{ReflectDeserialize, ReflectSerialize};

/// A double-precision version of [`Rot2`].
#[derive(Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[doc(alias = "drotation", alias = "drotation2d", alias = "drotation_2d")]
pub struct DRot2 {
    /// The cosine of the rotation angle.
    ///
    /// This is the real part of the unit complex number representing the rotation.
    pub cos: f64,
    /// The sine of the rotation angle.
    ///
    /// This is the imaginary part of the unit complex number representing the rotation.
    pub sin: f64,
}

impl Default for DRot2 {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl DRot2 {
    /// The identity rotation.
    pub const IDENTITY: Self = Self { cos: 1.0, sin: 0.0 };

    /// A rotation of π radians.
    /// Corresponds to a half-turn.
    pub const PI: Self = Self {
        cos: -1.0,
        sin: 0.0,
    };

    /// A counterclockwise rotation of π/2 radians.
    /// Corresponds to a counterclockwise quarter-turn.
    pub const FRAC_PI_2: Self = Self { cos: 0.0, sin: 1.0 };

    /// A counterclockwise rotation of π/3 radians.
    /// Corresponds to a counterclockwise turn by 60°.
    pub const FRAC_PI_3: Self = Self {
        cos: 0.5,
        sin: 0.866_025_4,
    };

    /// A counterclockwise rotation of π/4 radians.
    /// Corresponds to a counterclockwise turn by 45°.
    pub const FRAC_PI_4: Self = Self {
        cos: core::f64::consts::FRAC_1_SQRT_2,
        sin: core::f64::consts::FRAC_1_SQRT_2,
    };

    /// A counterclockwise rotation of π/6 radians.
    /// Corresponds to a counterclockwise turn by 30°.
    pub const FRAC_PI_6: Self = Self {
        cos: 0.866_025_4,
        sin: 0.5,
    };

    /// A counterclockwise rotation of π/8 radians.
    /// Corresponds to a counterclockwise turn by 22.5°.
    pub const FRAC_PI_8: Self = Self {
        cos: 0.923_879_5,
        sin: 0.382_683_43,
    };

    /// Creates a [`DRot2`] from a counterclockwise angle in radians.
    /// A negative argument corresponds to a clockwise rotation.
    ///
    /// # Note
    ///
    /// Angles larger than or equal to 2π (in either direction) loop around to smaller rotations,
    /// since a full rotation returns an object to its starting orientation.
    #[inline]
    pub fn radians(radians: f64) -> Self {
        #[cfg(feature = "enhanced-determinism")]
        let (sin, cos) = (libm::sin(radians), libm::cos(radians));
        #[cfg(not(feature = "enhanced-determinism"))]
        let (sin, cos) = radians.sin_cos();

        Self::from_sin_cos(sin, cos)
    }

    /// Creates a [`DRot2`] from a counterclockwise angle in degrees.
    /// A negative argument corresponds to a clockwise rotation.
    ///
    /// # Note
    ///
    /// Angles larger than or equal to 360° (in either direction) loop around to smaller rotations,
    /// since a full rotation returns an object to its starting orientation.
    #[inline]
    pub fn degrees(degrees: f64) -> Self {
        Self::radians(degrees.to_radians())
    }

    /// Creates a [`DRot2`] from a counterclockwise fraction of a full turn of 360 degrees.
    /// A negative argument corresponds to a clockwise rotation.
    ///
    /// # Note
    ///
    /// Angles larger than or equal to 1 turn (in either direction) loop around to smaller rotations,
    /// since a full rotation returns an object to its starting orientation.
    #[inline]
    pub fn turn_fraction(fraction: f64) -> Self {
        Self::radians(TAU * fraction)
    }

    /// Creates a [`DRot2`] from the sine and cosine of an angle.
    ///
    /// The rotation is only valid if `sin * sin + cos * cos == 1.0`.
    ///
    /// # Panics
    ///
    /// Panics if `sin * sin + cos * cos != 1.0` with debug assertions enabled.
    #[inline]
    pub fn from_sin_cos(sin: f64, cos: f64) -> Self {
        let rotation = Self { sin, cos };
        debug_assert!(
            rotation.is_normalized(),
            "the given sine and cosine produce an invalid rotation"
        );
        rotation
    }

    /// Returns a corresponding rotation angle in radians in the `(-pi, pi]` range.
    #[inline]
    pub fn as_radians(self) -> f64 {
        #[cfg(feature = "enhanced-determinism")]
        {
            libm::atan2(self.sin, self.cos)
        }
        #[cfg(not(feature = "enhanced-determinism"))]
        {
            f64::atan2(self.sin, self.cos)
        }
    }

    /// Returns a corresponding rotation angle in degrees in the `(-180, 180]` range.
    #[inline]
    pub fn as_degrees(self) -> f64 {
        self.as_radians().to_degrees()
    }

    /// Returns a corresponding rotation angle as a fraction of a full 360 degree turn in the `(-0.5, 0.5]` range.
    #[inline]
    pub fn as_turn_fraction(self) -> f64 {
        self.as_radians() / TAU
    }

    /// Returns the sine and cosine of the rotation angle.
    #[inline]
    pub const fn sin_cos(self) -> (f64, f64) {
        (self.sin, self.cos)
    }

    /// Computes the length or norm of the complex number used to represent the rotation.
    ///
    /// The length is typically expected to be `1.0`. Unexpectedly denormalized rotations
    /// can be a result of incorrect construction or floating point error caused by
    /// successive operations.
    #[inline]
    #[doc(alias = "norm")]
    pub fn length(self) -> f64 {
        DVec2::new(self.sin, self.cos).length()
    }

    /// Computes the squared length or norm of the complex number used to represent the rotation.
    ///
    /// This is generally faster than [`DRot2::length()`], as it avoids a square
    /// root operation.
    ///
    /// The length is typically expected to be `1.0`. Unexpectedly denormalized rotations
    /// can be a result of incorrect construction or floating point error caused by
    /// successive operations.
    #[inline]
    #[doc(alias = "norm2")]
    pub fn length_squared(self) -> f64 {
        DVec2::new(self.sin, self.cos).length_squared()
    }

    /// Computes `1.0 / self.length()`.
    ///
    /// For valid results, `self` must _not_ have a length of zero.
    #[inline]
    pub fn length_recip(self) -> f64 {
        DVec2::new(self.sin, self.cos).length_recip()
    }

    /// Returns `self` with a length of `1.0` if possible, and `None` otherwise.
    ///
    /// `None` will be returned if the sine and cosine of `self` are both zero (or very close to zero),
    /// or if either of them is NaN or infinite.
    ///
    /// Note that [`DRot2`] should typically already be normalized by design.
    /// Manual normalization is only needed when successive operations result in
    /// accumulated floating point error, or if the rotation was constructed
    /// with invalid values.
    #[inline]
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
    /// Note that [`DRot2`] should typically already be normalized by design.
    /// Manual normalization is only needed when successive operations result in
    /// accumulated floating point error, or if the rotation was constructed
    /// with invalid values.
    ///
    /// # Panics
    ///
    /// Panics if `self` has a length of zero, NaN, or infinity when debug assertions are enabled.
    #[inline]
    pub fn normalize(self) -> Self {
        let length_recip = self.length_recip();
        Self::from_sin_cos(self.sin * length_recip, self.cos * length_recip)
    }

    /// Returns `self` after an approximate normalization, assuming the value is already nearly normalized.
    /// Useful for preventing numerical error accumulation.
    #[inline]
    pub fn fast_renormalize(self) -> Self {
        let length_squared = self.length_squared();
        // Based on a Taylor approximation of the inverse square root
        let length_recip_approx = 0.5 * (3.0 - length_squared);
        DRot2 {
            sin: self.sin * length_recip_approx,
            cos: self.cos * length_recip_approx,
        }
    }

    /// Returns `true` if the rotation is neither infinite nor NaN.
    #[inline]
    pub const fn is_finite(self) -> bool {
        self.sin.is_finite() && self.cos.is_finite()
    }

    /// Returns `true` if the rotation is NaN.
    #[inline]
    pub const fn is_nan(self) -> bool {
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

    /// Returns `true` if the rotation is near [`DRot2::IDENTITY`].
    #[inline]
    pub fn is_near_identity(self) -> bool {
        // Same as `Quat::is_near_identity`, but using sine and cosine
        let threshold_angle_sin = 0.000_049_692_047; // let threshold_angle = 0.002_847_144_6;
        self.cos > 0.0 && self.sin.abs() < threshold_angle_sin
    }

    /// Returns the angle in radians needed to make `self` and `other` coincide.
    #[inline]
    pub fn angle_to(self, other: Self) -> f64 {
        (other * self.inverse()).as_radians()
    }

    /// Returns the inverse of the rotation. This is also the conjugate
    /// of the unit complex number representing the rotation.
    #[inline]
    #[must_use]
    #[doc(alias = "conjugate")]
    pub const fn inverse(self) -> Self {
        Self {
            cos: self.cos,
            sin: -self.sin,
        }
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
    ///
    /// # Details
    ///
    /// `nlerp` corresponds to computing an angle for a point at position `s` on a line drawn
    /// between the endpoints of the arc formed by `self` and `end` on a unit circle,
    /// and normalizing the result afterwards.
    ///
    /// Note that if the angles are opposite like 0 and π, the line will pass through the origin,
    /// and the resulting angle will always be either `self` or `end` depending on `s`.
    /// If `s` happens to be `0.5` in this case, a valid rotation cannot be computed, and `self`
    /// will be returned as a fallback.
    #[inline]
    pub fn nlerp(self, end: Self, s: f64) -> Self {
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
    /// When `s == 1.0`, the result will be equal to `end`.
    ///
    /// If you would like the rotation to have a kind of ease-in-out effect, consider
    /// using the slightly more efficient [`nlerp`](Self::nlerp) instead.
    #[inline]
    pub fn slerp(self, end: Self, s: f64) -> Self {
        self * Self::radians(self.angle_to(end) * s)
    }
}

impl From<f64> for DRot2 {
    /// Creates a [`DRot2`] from a counterclockwise angle in radians.
    fn from(rotation: f64) -> Self {
        Self::radians(rotation)
    }
}

impl From<DRot2> for DMat2 {
    /// Creates a [`DMat2`] rotation matrix from a [`DRot2`].
    fn from(rot: DRot2) -> Self {
        DMat2::from_cols_array(&[rot.cos, rot.sin, -rot.sin, rot.cos])
    }
}

impl core::ops::Mul for DRot2 {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        Self {
            cos: self.cos * rhs.cos - self.sin * rhs.sin,
            sin: self.sin * rhs.cos + self.cos * rhs.sin,
        }
    }
}

impl core::ops::MulAssign for DRot2 {
    fn mul_assign(&mut self, rhs: Self) {
        *self = *self * rhs;
    }
}

impl core::ops::Mul<DVec2> for DRot2 {
    type Output = DVec2;

    /// Rotates a [`DVec2`] by a [`DRot2`].
    fn mul(self, rhs: DVec2) -> Self::Output {
        DVec2::new(
            rhs.x * self.cos - rhs.y * self.sin,
            rhs.x * self.sin + rhs.y * self.cos,
        )
    }
}
