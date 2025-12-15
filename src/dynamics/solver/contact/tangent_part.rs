use crate::prelude::*;
use bevy::reflect::Reflect;
#[cfg(feature = "serialize")]
use bevy::reflect::{ReflectDeserialize, ReflectSerialize};

#[cfg(feature = "2d")]
pub type TangentImpulse = Scalar;
#[cfg(feature = "3d")]
pub type TangentImpulse = Vector2;

// TODO: One-body constraint version
/// The tangential friction part of a [`ContactConstraintPoint`](super::ContactConstraintPoint).
#[derive(Clone, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct ContactTangentPart {
    /// The contact impulse magnitude along the contact tangent.
    ///
    /// This corresponds to the magnitude of the friction impulse.
    pub impulse: TangentImpulse,

    /// The inertial properties of the bodies projected onto the contact tangent,
    /// or in other words, the mass "seen" by the constraint along the tangent.
    #[cfg(feature = "2d")]
    pub effective_mass: Scalar,
    /// The inverse of the inertial properties of the bodies projected onto the contact tangents,
    /// or in other words, the inverse mass "seen" by the constraint along the tangents.
    #[cfg(feature = "3d")]
    pub effective_inverse_mass: [Scalar; 3],
}

impl ContactTangentPart {
    /// Generates a new [`ContactTangentPart`].
    pub fn generate(
        effective_inverse_mass_sum: Vector,
        inverse_angular_inertia1: &SymmetricTensor,
        inverse_angular_inertia2: &SymmetricTensor,
        r1: Vector,
        r2: Vector,
        tangents: [Vector; DIM - 1],
        warm_start_impulse: Option<TangentImpulse>,
    ) -> Self {
        let i1 = inverse_angular_inertia1;
        let i2 = inverse_angular_inertia2;

        let mut part = Self {
            impulse: warm_start_impulse.unwrap_or_default(),
            #[cfg(feature = "2d")]
            effective_mass: 0.0,
            #[cfg(feature = "3d")]
            effective_inverse_mass: [0.0; 3],
        };

        // Derivation for the projected tangent masses. This is for 3D, but the 2D version is largely the same.
        //
        // Friction constraints aim to prevent relative tangential motion at contact points.
        // The velocity constraint is satisfied when the relative velocity along the tangent
        // is equal to zero.
        //
        // In 3D, there are two tangent directions and therefore two constraints:
        //
        // dot(lin_vel1_p, tangent_x) = dot(lin_vel2_p, tangent_x)
        // dot(lin_vel1_p, tangent_y) = dot(lin_vel2_p, tangent_y)
        //
        // where lin_vel1_p and lin_vel2_p are the velocities of the bodies at the contact point p:
        //
        // lin_vel1_p = lin_vel1 + ang_vel1 x r1
        // lin_vel2_p = lin_vel2 + ang_vel2 x r2
        //
        // Based on this, we get:
        //
        // dot(lin_vel1_p, tangent_x) = dot(lin_vel1, tangent_x) + dot(ang_vel1 x r1, tangent_x)
        //                            = dot(lin_vel1, tangent_x) + dot(r1 x tangent_x, ang_vel1)
        //
        // Restating the original constraints with the derived formula:
        //
        // dot(lin_vel1, tangent_x) + dot(r1 x tangent_x, ang_vel1) = dot(lin_vel2, tangent_x) + dot(r2 x tangent_x, ang_vel2)
        // dot(lin_vel1, tangent_y) + dot(r1 x tangent_y, ang_vel1) = dot(lin_vel2, tangent_y) + dot(r2 x tangent_y, ang_vel2)
        //
        // Finally, moving the right-hand side to the left:
        //
        // dot(lin_vel1, tangent_x) + dot(r1 x tangent_x, ang_vel1) - dot(lin_vel2, tangent_x) - dot(r2 x tangent_x, ang_vel2) = 0
        // dot(lin_vel1, tangent_y) + dot(r1 x tangent_y, ang_vel1) - dot(lin_vel2, tangent_y) - dot(r2 x tangent_y, ang_vel2) = 0
        //
        // By inspection, we can see that the Jacobian is the following:
        //
        //          linear1      angular1       linear2       angular2
        // J_x = [ -tangent_x, -(r1 x tangent_x), tangent_x, r2 x tangent_x ]
        // J_y = [ -tangent_y, -(r1 x tangent_y), tangent_y, r2 x tangent_y ]
        //
        // From this, we can derive the effective inverse mass for both tangent directions:
        //
        // K_x = J_x * M^-1 * J_x^T
        //     = m1 + m2 + (r1 x tangent_x)^T * I1 * (r1 x tangent_x) + (r2 x tangent_x)^T * I2 * (r2 x tangent_x)
        // K_y = J_y * M^-1 * J_y^T
        //     = m1 + m2 + (r1 x tangent_y)^T * I1 * (r1 x tangent_y) + (r2 x tangent_y)^T * I2 * (r2 x tangent_y)
        //
        // See "Constraints Derivation for Rigid Body Simulation in 3D" section 2.1.3
        // by Daniel Chappuis for the full derivation of the effective inverse mass.
        //
        // Finally, the transposes can be simplified with dot products, because a^T * b = dot(a, b),
        // where a and b are two column vectors.
        //
        // K_x = m1 + m2 + dot(r1 x tangent_x, I1 * (r1 x tangent_x)) + dot(r2 x tangent_x, I2 * (r2 x tangent_x))
        // K_y = m1 + m2 + dot(r1 x tangent_y, I1 * (r1 x tangent_y)) + dot(r2 x tangent_y, I2 * (r2 x tangent_y))

        #[cfg(feature = "2d")]
        {
            let rt1 = cross(r1, tangents[0]);
            let rt2 = cross(r2, tangents[0]);

            let k_linear = tangents[0].dot(effective_inverse_mass_sum * tangents[0]);
            let k = k_linear + i1 * rt1 * rt1 + i2 * rt2 * rt2;

            part.effective_mass = k.recip_or_zero();
        }

        #[cfg(feature = "3d")]
        {
            // Based on Rapier's two-body constraint.
            // https://github.com/dimforge/rapier/blob/af1ac9baa26b1199ae2728e91adf5345bcd1c693/src/dynamics/solver/contact_constraint/two_body_constraint.rs#L257-L289

            let rt11 = cross(r1, tangents[0]);
            let rt12 = cross(r2, tangents[0]);
            let rt21 = cross(r1, tangents[1]);
            let rt22 = cross(r2, tangents[1]);

            // Multiply by the inverse inertia early to reuse the values.
            let i1_rt11 = i1 * rt11;
            let i2_rt12 = i2 * rt12;
            let i1_rt21 = i1 * rt21;
            let i2_rt22 = i2 * rt22;

            let k_linear1 = tangents[0].dot(effective_inverse_mass_sum * tangents[0]);
            let k_linear2 = tangents[1].dot(effective_inverse_mass_sum * tangents[1]);
            let k1 = k_linear1 + rt11.dot(i1_rt11) + rt12.dot(i2_rt12);
            let k2 = k_linear2 + rt21.dot(i1_rt21) + rt22.dot(i2_rt22);

            // Note: The invertion is done in `solve_impulse`, unlike in 2D.
            part.effective_inverse_mass[0] = k1;
            part.effective_inverse_mass[1] = k2;

            // This is needed for solving the two tangent directions simultaneously.
            // TODO. Derive and explain the math for this, or consider an alternative approach,
            //       like using the Jacobians to compute the actual effective mass matrix.
            part.effective_inverse_mass[2] = 2.0 * (rt11.dot(i1_rt21) + rt12.dot(i2_rt22));
        }

        part
    }

    /// Solves the friction constraint, updating the total impulse in `self` and returning
    /// the incremental impulse to apply to each body.
    pub fn solve_impulse(
        &mut self,
        tangent_directions: [Vector; DIM - 1],
        relative_velocity: Vector,
        // The desired relative velocity along the contact surface, used to simulate things like conveyor belts.
        #[cfg(feature = "2d")] surface_speed: Scalar,
        #[cfg(feature = "3d")] surface_velocity: Vector,
        friction: Scalar,
        normal_impulse: Scalar,
    ) -> Vector {
        // Compute the maximum bound for the friction impulse.
        //
        // According to the Coulomb friction law:
        //
        // length(friction_force) <= coefficient * length(normal_force)
        //
        // Now, we need to find the Lagrange multiplier, which corresponds
        // to the constraint force magnitude.
        //
        // F_c = J^T * lagrange, where J is the Jacobian, which in this case is of unit length.
        //
        // We get the following:
        //
        // length(J^T * force_magnitude) <= coefficient * length(normal_force)
        // <=> abs(force_magnitude) <= coefficient * length(normal_force)
        // <=> -coefficient * length(normal_force) <= force_magnitude <= coefficient * length(normal_force)
        //
        // We are dealing with impulses instead of forces. Multiplying by delta time,
        // we get the minimum and maximum bound for the friction impulse:
        //
        // -coefficient * length(normal_impulse) <= impulse_magnitude <= coefficient * length(normal_impulse)

        let impulse_limit = friction * normal_impulse;

        #[cfg(feature = "2d")]
        {
            // Compute the relative velocity along the tangent.
            // Add the relative speed along the surface to the total tangent speed.
            let tangent = tangent_directions[0];
            let tangent_speed = relative_velocity.dot(tangent) + surface_speed;

            // Compute the incremental tangent impoulse magnitude.
            let mut impulse = self.effective_mass * (-tangent_speed);
            // Clamp the accumulated impulse.
            let new_impulse = (self.impulse + impulse).clamp(-impulse_limit, impulse_limit);
            impulse = new_impulse - self.impulse;
            self.impulse = new_impulse;
            // Return the incremental friction impulse.
            impulse * tangent
        }
        #[cfg(feature = "3d")]
        {
            // Compute the relative velocity along the tangents.
            // Add the relative velocity along the surface to the total tangent speed.
            let relative_velocity = relative_velocity + surface_velocity;
            let tangent_speed1 = relative_velocity.dot(tangent_directions[0]);
            let tangent_speed2 = relative_velocity.dot(tangent_directions[1]);

            // Solve the two tangent directions simultaneously.
            // Based on Rapier's two-body constraint.
            // https://github.com/dimforge/rapier/blob/af1ac9baa26b1199ae2728e91adf5345bcd1c693/src/dynamics/solver/contact_constraint/two_body_constraint_element.rs#L127-L133
            let t11 = tangent_speed1.powi(2);
            let t22 = tangent_speed2.powi(2);
            let t12 = tangent_speed1 * tangent_speed2;
            let inv = t11 * self.effective_inverse_mass[0]
                + t22 * self.effective_inverse_mass[1]
                + t12 * self.effective_inverse_mass[2];

            // Compute the effective mass "seen" by the constraint along the tangent.
            // Note the guard against division by zero.
            let effective_mass = (t11 + t22) * inv.recip();

            // TODO: Could this be handled earlier reliably?
            if !effective_mass.is_finite() {
                return Vector::ZERO;
            }

            // Compute the incremental tangent impoulse.
            let delta_impulse = effective_mass * Vector2::new(tangent_speed1, tangent_speed2);

            // Clamp the accumulated impulse.
            let new_impulse = (self.impulse - delta_impulse).clamp_length_max(impulse_limit);
            let impulse = new_impulse - self.impulse;

            self.impulse = new_impulse;

            // Return the clamped incremental friction impulse.
            impulse.x * tangent_directions[0] + impulse.y * tangent_directions[1]
        }
    }
}
