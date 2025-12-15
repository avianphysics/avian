//! Soft constraints are spring-like constraints that dampen constraint responses
//! using intuitive tuning parameters, a damping ratio and a frequency in Hertz.
//!
//! The following section contains an overview of soft constraints
//! and their mathematical background.
//!
#![doc = include_str!("README.md")]

use bevy::reflect::Reflect;
#[cfg(feature = "serialize")]
use bevy::reflect::{ReflectDeserialize, ReflectSerialize};

use crate::{Scalar, TAU};

/// Soft constraint tuning parameters used for dampening
/// constraint response and controlling stiffness.
#[derive(Clone, Copy, Debug, PartialEq, Reflect)]
pub struct SoftnessParameters {
    /// 2x the damping ratio (zeta ζ). Controls the amount of oscillation.
    ///
    /// This is stored as two times the damping ratio to avoid
    /// unnecessary computations in [`SoftnessParameters::compute_coefficients`].
    double_damping_ratio: Scalar,

    /// The angular frequency (omega ω). Controls the rate of oscillation.
    angular_frequency: Scalar,
}

impl SoftnessParameters {
    /// Creates a new [`SoftnessParameters`] configuration based on
    /// a given damping ratio and a frequency in Hertz.
    ///
    /// The damping ratio (zeta ζ) controls the amount of oscillation,
    /// and the frequency controls the constraint's cycles per second.
    #[inline]
    pub fn new(damping_ratio: Scalar, frequency_hz: Scalar) -> Self {
        Self {
            double_damping_ratio: 2.0 * damping_ratio,
            angular_frequency: TAU * frequency_hz,
        }
    }

    /// Returns the damping ratio that controls the amount of oscillation.
    #[inline]
    pub fn damping_ratio(self) -> Scalar {
        self.double_damping_ratio * 0.5
    }

    /// Returns the frequency that controls the rate of oscillation.
    #[inline]
    pub fn frequency(self) -> Scalar {
        self.angular_frequency / TAU
    }

    /// Returns the angular frequency that controls the rate of oscillation.
    /// This is the [`frequency`](Self::frequency) multiplied by `2.0 * PI`.
    #[inline]
    pub const fn angular_frequency(self) -> Scalar {
        self.angular_frequency
    }

    /// Computes [`SoftnessCoefficients`] based on the parameters in `self` and the time step.
    #[inline]
    pub fn compute_coefficients(self, delta_secs: Scalar) -> SoftnessCoefficients {
        // Largely based on Erin Catto's Solver2D and Box2D: https://box2d.org/posts/2024/02/solver2d#soft-constraints
        // See /docs/soft_constraint.md for in-depth explanation and derivation.

        // Expressions shared by computations.
        let a1 = self.double_damping_ratio + self.angular_frequency * delta_secs;
        let a2 = self.angular_frequency * delta_secs * a1;
        let a3 = 1.0 / (1.0 + a2);

        // The coefficients used for soft constraints.
        SoftnessCoefficients {
            bias: self.angular_frequency / a1,
            impulse_scale: a3,
            mass_scale: a2 * a3,
        }
    }
}

/// Coefficients used by soft constraints.
#[derive(Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct SoftnessCoefficients {
    /// The bias coefficient used for scaling how strongly impulses
    /// are biased based on the separation distance.
    pub bias: Scalar,

    /// The mass coefficient used for scaling the effective mass
    /// "seen" by the constraint.
    pub mass_scale: Scalar,

    /// The impulse coefficient used for scaling the accumulated impulse
    /// that is subtracted from the total impulse to prevent
    /// the total impulse from becoming too large.
    pub impulse_scale: Scalar,
}
