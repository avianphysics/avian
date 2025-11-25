use crate::{dynamics::solver::softness_parameters::SoftnessCoefficients, prelude::*};
use bevy::reflect::Reflect;
#[cfg(feature = "serialize")]
use bevy::reflect::{ReflectDeserialize, ReflectSerialize};

pub type NormalImpulse = Scalar;

// TODO: Block solver for solving two contact points simultaneously
// TODO: One-body constraint version
/// The normal part of a [`ContactConstraintPoint`](super::ContactConstraintPoint).
/// Aims to resolve overlap.
#[derive(Clone, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct ContactNormalPart {
    /// The magnitude of the contact impulse along the contact normal.
    pub impulse: NormalImpulse,

    /// The total normal impulse applied across substeps and restitution.
    /// The unit is typically N⋅s or kg⋅m/s.
    ///
    /// This is used by the solver to identify speculative contacts that did not generate
    /// an impulse, but is also useful for users to determine how "strong" the contact is.
    /// This is stored for contact manifolds as [`ContactPoint::normal_impulse`].
    pub total_impulse: Scalar,

    /// The inertial properties of the bodies projected onto the contact normal,
    /// or in other words, the mass "seen" by the constraint along the normal.
    pub effective_mass: Scalar,

    /// The softness parameters used for tuning contact response.
    pub softness: SoftnessCoefficients,
}

impl ContactNormalPart {
    /// Generates a new [`ContactNormalPart`].
    #[allow(clippy::too_many_arguments)]
    pub fn generate(
        effective_inverse_mass_sum: Vector,
        inverse_angular_inertia1: &SymmetricTensor,
        inverse_angular_inertia2: &SymmetricTensor,
        r1: Vector,
        r2: Vector,
        normal: Vector,
        warm_start_impulse: Option<NormalImpulse>,
        softness: SoftnessCoefficients,
    ) -> Self {
        let i1 = inverse_angular_inertia1;
        let i2 = inverse_angular_inertia2;

        // Derivation for the projected normal mass. This is for 3D, but the 2D version is basically equivalent.
        //
        // The penetration constraint function is the following:
        //
        // C(s) = dot(p2 - p1, n) = dot(x2 + r2 - x1 - r1, n)
        //
        // where
        // - p1 and p2 are world-space contact points for each body
        // - n is the surface normal pointing from the first body towards the second (the order matters)
        // - x1 and x2 are the centers of mass
        // - r1 and r2 are vectors from the centers of mass to the corresponding contact points
        //
        // The contact constraint is satisfied when the bodies are not penetrating:
        //
        // C(s) >= 0
        //
        // We can compute the velocity constraint by getting the time derivative:
        //
        // C_vel(s) = d/dt(dot(x2 + r2 - x1 - r1, n))
        //          = dot(d/dt(x2 + r2 - x1 - r1), n) + dot(x2 + r2 - x1 - r1, d/dt(n))
        //
        // The penetration is assumed to be small, so we can ignore the second term:
        //
        // C_vel(s) = dot(d/dt(x2 + r2 - x1 - r1), n)
        //          = dot(lin_vel2 + ang_vel2 x r2 - lin_vel1 - ang_vel1 x r1, n)
        //          = dot(lin_vel2, n) + dot(ang_vel2, r2 x n) - dot(v1, n) - dot(ang_vel1, r1 x n)
        //
        // By inspection, we can see that the Jacobian is the following:
        //
        //      linear1  angular1 linear2 angular2
        // J = [ -normal, -(r1 x n), n, r2 x n ]
        //
        // From this, we can derive the effective inverse mass:
        //
        // K = J_x * M^-1 * J_x^T
        //   = 1/m1 + 1/m2 + (r1 x n)^T * I1^-1 * (r1 x n) + (r2 x n)^T * I2^-1 * (r2 x n)
        //
        // See "Constraints Derivation for Rigid Body Simulation in 3D" section 2.1.3
        // by Daniel Chappuis for the full derivation of the effective inverse mass.
        //
        // Finally, the transposes can be simplified with dot products, because a^T * b = dot(a, b),
        // where a and b are two column vectors.
        //
        // K = 1/m1 + 1/m2 + dot(r1 x n, I1^-1 * (r1 x n)) + dot(r2 x n, I2^-1 * (r2 x n))

        let r1_cross_n = cross(r1, normal);
        let r2_cross_n = cross(r2, normal);

        let k_linear = normal.dot(effective_inverse_mass_sum * normal);
        #[cfg(feature = "2d")]
        let k = k_linear + i1 * r1_cross_n * r1_cross_n + i2 * r2_cross_n * r2_cross_n;
        #[cfg(feature = "3d")]
        let k = k_linear + r1_cross_n.dot(i1 * r1_cross_n) + r2_cross_n.dot(i2 * r2_cross_n);

        Self {
            impulse: warm_start_impulse.unwrap_or_default(),
            total_impulse: 0.0,
            effective_mass: k.recip_or_zero(),
            softness,
        }
    }

    /// Solves the non-penetration constraint, updating the total impulse in `self` and returning
    /// the incremental impulse to apply to each body.
    pub fn solve_impulse(
        &mut self,
        separation: Scalar,
        relative_velocity: Vector,
        normal: Vector,
        use_bias: bool,
        max_overlap_solve_speed: Scalar,
        delta_secs: Scalar,
    ) -> Scalar {
        // Compute the relative velocity along the normal.
        let normal_speed = relative_velocity.dot(normal);

        // Compute the incremental normal impulse.
        let mut impulse = if separation > 0.0 {
            // Speculative contact: Push back the part of the velocity that would cause penetration.
            -self.effective_mass * (normal_speed + separation / delta_secs)
        } else if use_bias {
            // Contact using bias: Incorporate softness parameters.
            //
            // 1. Velocity bias: Allows the constraint to solve overlap by boosting
            //    the constraint response a bit, taking into account the current overlap.
            //    This is known as Baumgarte stabilization.
            //
            // 2. Mass coefficient: Scales the effective mass "seen" by the constraint.
            //
            // 3. Impulse coefficient: Scales the accumulated impulse that is subtracted
            //    from the total impulse to prevent the total impulse from becoming too large.
            //
            // See the `softness_parameters` module for more details and references.

            // TODO: We might want optional slop, a small amount of allowed penetration.
            let bias = (self.softness.bias * separation).max(-max_overlap_solve_speed);
            let scaled_mass = self.softness.mass_scale * self.effective_mass;
            let scaled_impulse = self.softness.impulse_scale * self.impulse;

            -scaled_mass * (normal_speed + bias) - scaled_impulse
        } else {
            // Contact without bias: Solve normally without softness parameters.
            // This is useful for a "relaxation" phase which helps get rid of overshoot.
            -self.effective_mass * normal_speed
        };

        // Clamp the accumulated impulse.
        let new_impulse = (self.impulse + impulse).max(0.0);
        impulse = new_impulse - self.impulse;
        self.impulse = new_impulse;
        self.total_impulse += new_impulse;

        // Return the clamped incremental normal impulse.
        impulse
    }
}
