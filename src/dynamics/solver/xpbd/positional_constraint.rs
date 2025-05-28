use dynamics::solver::solver_body::SolverBody;

use super::XpbdConstraint;
use crate::{dynamics::solver::solver_body::SolverBodyInertia, prelude::*};

/// A positional constraint applies a positional correction
/// with a given direction and magnitude at the local contact points `r1` and  `r2`.
pub trait PositionConstraint: XpbdConstraint<2> {
    /// Applies a positional correction to two bodies.
    ///
    /// Returns the positional impulse that is applied proportional to the inverse masses of the bodies.
    #[allow(clippy::too_many_arguments)]
    fn apply_positional_lagrange_update(
        &self,
        body1: &mut SolverBody,
        body2: &mut SolverBody,
        inertia1: &SolverBodyInertia,
        inertia2: &SolverBodyInertia,
        delta_lagrange: Scalar,
        direction: Vector,
        r1: Vector,
        r2: Vector,
    ) -> Vector {
        if delta_lagrange.abs() <= Scalar::EPSILON {
            return Vector::ZERO;
        }

        let impulse = delta_lagrange * direction;

        self.apply_positional_impulse(body1, body2, inertia1, inertia2, impulse, r1, r2)
    }

    /// Applies a positional impulse to two bodies.
    ///
    /// Returns the impulse that is applied proportional to the inverse masses of the bodies.
    fn apply_positional_impulse(
        &self,
        body1: &mut SolverBody,
        body2: &mut SolverBody,
        inertia1: &SolverBodyInertia,
        inertia2: &SolverBodyInertia,
        impulse: Vector,
        r1: Vector,
        r2: Vector,
    ) -> Vector {
        let inv_mass1 = inertia1.effective_inv_mass();
        let inv_mass2 = inertia2.effective_inv_mass();
        let inv_angular_inertia1 = inertia1.effective_inv_angular_inertia();
        let inv_angular_inertia2 = inertia2.effective_inv_angular_inertia();

        body1.delta_position += impulse * inv_mass1;

        #[cfg(feature = "2d")]
        {
            let delta_angle = Self::get_delta_rot(inv_angular_inertia1, r1, impulse);
            body1.delta_rotation = body1.delta_rotation.add_angle_fast(delta_angle);
        }
        #[cfg(feature = "3d")]
        {
            let delta_quat = Self::get_delta_rot(inv_angular_inertia1, r1, impulse);
            body1.delta_rotation.0 = delta_quat * body1.delta_rotation.0;
        }

        body2.delta_position -= impulse * inv_mass2;

        #[cfg(feature = "2d")]
        {
            let delta_angle = Self::get_delta_rot(inv_angular_inertia2, r2, -impulse);
            body2.delta_rotation = body2.delta_rotation.add_angle_fast(delta_angle);
        }
        #[cfg(feature = "3d")]
        {
            let delta_quat = Self::get_delta_rot(inv_angular_inertia2, r2, -impulse);
            body2.delta_rotation.0 = delta_quat * body2.delta_rotation.0;
        }

        impulse
    }

    /// Computes the generalized inverse mass of a body when applying a positional correction
    /// at point `r` along the vector `n`.
    #[cfg(feature = "2d")]
    fn compute_generalized_inverse_mass(
        &self,
        inverse_mass: Scalar,
        inverse_angular_inertia: Scalar,
        r: Vector,
        n: Vector,
    ) -> Scalar {
        inverse_mass + inverse_angular_inertia * r.perp_dot(n).powi(2)
    }

    /// Computes the generalized inverse mass of a body when applying a positional correction
    /// at point `r` along the vector `n`.
    #[cfg(feature = "3d")]
    fn compute_generalized_inverse_mass(
        &self,
        inverse_mass: Scalar,
        inverse_angular_inertia: Matrix3,
        r: Vector,
        n: Vector,
    ) -> Scalar {
        let r_cross_n = r.cross(n); // Compute the cross product only once

        // The line below is equivalent to Eq (2) because the component-wise multiplication of a transposed vector and another vector is equal to the dot product of the two vectors.
        // a^T * b = a • b
        inverse_mass + r_cross_n.dot(inverse_angular_inertia * r_cross_n)
    }

    /// Computes the update in rotation when applying a positional correction `p` at point `r`.
    #[cfg(feature = "2d")]
    fn get_delta_rot(inverse_angular_inertia: Scalar, r: Vector, p: Vector) -> Scalar {
        // Equation 8/9 but in 2D
        inverse_angular_inertia * r.perp_dot(p)
    }

    /// Computes the update in rotation when applying a positional correction `p` at point `r`.
    #[cfg(feature = "3d")]
    fn get_delta_rot(inverse_angular_inertia: Matrix3, r: Vector, p: Vector) -> Quaternion {
        // Equation 8/9
        Quaternion::from_scaled_axis(inverse_angular_inertia * r.cross(p))
    }

    /// Computes the force acting along the constraint using the equation f = lambda * n / h^2
    fn compute_force(&self, lagrange: Scalar, direction: Vector, dt: Scalar) -> Vector {
        lagrange * direction / dt.powi(2)
    }
}
