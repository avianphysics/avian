use crate::dynamics::solver::solver_body::SolverBody;
use crate::dynamics::solver::xpbd;
use crate::prelude::*;

/// An angular constraint applies an angular correction around a given axis.
pub trait AngularConstraint {
    /// Applies an angular correction to two bodies.
    ///
    /// Returns the angular impulse that is applied proportional
    /// to the inverse moments of inertia of the bodies.
    #[cfg(feature = "2d")]
    fn apply_angular_lagrange_update(
        &self,
        body1: &mut SolverBody,
        body2: &mut SolverBody,
        inv_angular_inertia1: SymmetricTensor,
        inv_angular_inertia2: SymmetricTensor,
        delta_lagrange: Scalar,
    ) -> Scalar {
        if delta_lagrange.abs() <= Scalar::EPSILON {
            return 0.0;
        }

        self.apply_angular_impulse(
            body1,
            body2,
            inv_angular_inertia1,
            inv_angular_inertia2,
            -delta_lagrange,
        )
    }

    /// Applies an angular impulse to two bodies.
    ///
    /// Returns the impulse that is applied proportional
    /// to the inverse moments of inertia of the bodies.
    #[cfg(feature = "2d")]
    fn apply_angular_impulse(
        &self,
        body1: &mut SolverBody,
        body2: &mut SolverBody,
        inv_angular_inertia1: SymmetricTensor,
        inv_angular_inertia2: SymmetricTensor,
        impulse: Scalar,
    ) -> Scalar {
        // Apply rotational updates
        let delta_angle = Self::get_delta_rot(inv_angular_inertia1, impulse);
        body1.delta_rotation = body1.delta_rotation.add_angle_fast(delta_angle);

        let delta_angle = Self::get_delta_rot(inv_angular_inertia2, -impulse);
        body2.delta_rotation = body2.delta_rotation.add_angle_fast(delta_angle);

        impulse
    }

    /// Applies an angular correction to two bodies.
    ///
    /// Returns the angular impulse that is applied proportional
    /// to the inverse moments of inertia of the bodies.
    #[cfg(feature = "3d")]
    fn apply_angular_lagrange_update(
        &self,
        body1: &mut SolverBody,
        body2: &mut SolverBody,
        inv_angular_inertia1: SymmetricTensor,
        inv_angular_inertia2: SymmetricTensor,
        delta_lagrange: Scalar,
        axis: Vector,
    ) -> Vector {
        if delta_lagrange.abs() <= Scalar::EPSILON {
            return Vector::ZERO;
        }

        let impulse = -delta_lagrange * axis;

        self.apply_angular_impulse(
            body1,
            body2,
            inv_angular_inertia1,
            inv_angular_inertia2,
            impulse,
        )
    }

    /// Applies an angular impulse to two bodies.
    ///
    /// Returns the impulse that is applied proportional
    /// to the inverse moments of inertia of the bodies.
    #[cfg(feature = "3d")]
    fn apply_angular_impulse(
        &self,
        body1: &mut SolverBody,
        body2: &mut SolverBody,
        inv_angular_inertia1: SymmetricTensor,
        inv_angular_inertia2: SymmetricTensor,
        impulse: Vector,
    ) -> Vector {
        // Apply rotational updates
        let delta_quat = Self::get_delta_rot(inv_angular_inertia1, impulse);
        body1.delta_rotation.0 = delta_quat * body1.delta_rotation.0;

        let delta_quat = Self::get_delta_rot(inv_angular_inertia2, -impulse);
        body2.delta_rotation.0 = delta_quat * body2.delta_rotation.0;

        impulse
    }

    /// Applies an angular correction that aligns the orientation of the bodies.
    ///
    /// Returns the Lagrange multiplier update.
    #[cfg(feature = "2d")]
    fn align_orientation(
        &self,
        body1: &mut SolverBody,
        body2: &mut SolverBody,
        inv_angular_inertia1: SymmetricTensor,
        inv_angular_inertia2: SymmetricTensor,
        angle: Scalar,
        lagrange: Scalar,
        compliance: Scalar,
        dt: Scalar,
    ) -> AngularVector {
        if angle.abs() <= Scalar::EPSILON {
            return AngularVector::ZERO;
        }

        let w = [inv_angular_inertia1, inv_angular_inertia2];

        // Compute Lagrange multiplier update
        let delta_lagrange = xpbd::compute_lagrange_update(lagrange, angle, &w, compliance, dt);

        // Apply angular correction to aling the bodies
        self.apply_angular_lagrange_update(
            body1,
            body2,
            inv_angular_inertia1,
            inv_angular_inertia2,
            delta_lagrange,
        );

        // Return Lagrange multiplier update
        delta_lagrange
    }

    /// Applies an angular correction that aligns the orientation of the bodies.
    ///
    /// Returns the Lagrange multiplier update.
    #[cfg(feature = "3d")]
    fn align_orientation(
        &self,
        body1: &mut SolverBody,
        body2: &mut SolverBody,
        inv_angular_inertia1: SymmetricTensor,
        inv_angular_inertia2: SymmetricTensor,
        rotation_difference: Vector,
        lagrange: Scalar,
        compliance: Scalar,
        dt: Scalar,
    ) -> AngularVector {
        let angle = rotation_difference.length();

        if angle <= Scalar::EPSILON {
            return AngularVector::ZERO;
        }

        let axis = rotation_difference / angle;

        // Compute generalized inverse masses
        let w1 =
            AngularConstraint::compute_generalized_inverse_mass(self, inv_angular_inertia1, axis);
        let w2 =
            AngularConstraint::compute_generalized_inverse_mass(self, inv_angular_inertia2, axis);
        let w = [w1, w2];

        // Compute Lagrange multiplier update
        let delta_lagrange = xpbd::compute_lagrange_update(lagrange, angle, &w, compliance, dt);

        // Apply angular correction to aling the bodies
        self.apply_angular_lagrange_update(
            body1,
            body2,
            inv_angular_inertia1,
            inv_angular_inertia2,
            delta_lagrange,
            axis,
        );

        // Return Lagrange multiplier update
        delta_lagrange * axis
    }

    /// Applies angular constraints for interactions between two bodies.
    ///
    /// Here in 2D, `axis` is a unit vector with the Z coordinate set to 1 or -1. It controls if the body should rotate counterclockwise or clockwise.
    ///
    /// Returns the angular impulse that is applied proportional to the inverse masses of the bodies.
    #[cfg(feature = "2d")]
    fn apply_angular_correction(
        &self,
        body1: &mut SolverBody,
        body2: &mut SolverBody,
        inv_angular_inertia1: SymmetricTensor,
        inv_angular_inertia2: SymmetricTensor,
        delta_lagrange: Scalar,
        axis: Vector3,
    ) -> Scalar {
        if delta_lagrange.abs() <= Scalar::EPSILON {
            return 0.0;
        }

        // Compute angular impulse
        // `axis.z` is 1 or -1 and it controls if the body should rotate counterclockwise or clockwise
        let p = -delta_lagrange * axis.z;

        // Apply rotational updates
        let delta_angle = Self::get_delta_rot(inv_angular_inertia1, p);
        body1.delta_rotation = body1.delta_rotation.add_angle_fast(delta_angle);

        let delta_angle = Self::get_delta_rot(inv_angular_inertia2, -p);
        body2.delta_rotation = body2.delta_rotation.add_angle_fast(delta_angle);

        p
    }

    /// Applies angular constraints for interactions between two bodies.
    ///
    /// Returns the angular impulse that is applied proportional to the inverse masses of the bodies.
    #[cfg(feature = "3d")]
    fn apply_angular_correction(
        &self,
        body1: &mut SolverBody,
        body2: &mut SolverBody,
        inv_angular_inertia1: SymmetricTensor,
        inv_angular_inertia2: SymmetricTensor,
        delta_lagrange: Scalar,
        axis: Vector,
    ) -> Vector {
        if delta_lagrange.abs() <= Scalar::EPSILON {
            return Vector::ZERO;
        }

        // Compute angular impulse
        let p = -delta_lagrange * axis;

        // Apply rotational updates
        let delta_quat = Self::get_delta_rot(inv_angular_inertia1, p);
        body1.delta_rotation.0 = delta_quat * body1.delta_rotation.0;

        let delta_quat = Self::get_delta_rot(inv_angular_inertia2, -p);
        body2.delta_rotation.0 = delta_quat * body2.delta_rotation.0;

        p
    }

    /// Computes the generalized inverse mass of a body when applying an angular correction
    /// around `axis`.
    ///
    /// In 2D, `axis` should only have the z axis set to either -1 or 1 to indicate counterclockwise or
    /// clockwise rotation.
    fn compute_generalized_inverse_mass(
        &self,
        inv_angular_inertia: SymmetricTensor,
        axis: Vector3,
    ) -> Scalar {
        axis.dot(inv_angular_inertia * axis)
    }

    /// Computes the update in rotation when applying an angular correction `p`.
    #[cfg(feature = "2d")]
    fn get_delta_rot(inverse_inertia: SymmetricTensor, p: Scalar) -> Scalar {
        // Equation 8/9 but in 2D
        inverse_inertia * p
    }

    /// Computes the update in rotation when applying an angular correction `p`.
    #[cfg(feature = "3d")]
    fn get_delta_rot(inverse_inertia: SymmetricTensor, p: Vector) -> Quaternion {
        // Equation 8/9
        Quaternion::from_scaled_axis(inverse_inertia * p)
    }

    /// Computes the torque acting along the constraint using the equation `tau = lambda * n / h^2`,
    /// where `n` is the Z axis in 2D.
    #[cfg(feature = "2d")]
    fn compute_torque(&self, lagrange: Scalar, dt: Scalar) -> AngularVector {
        // Eq (17)
        lagrange / dt.powi(2)
    }

    /// Computes the torque acting along the constraint using the equation `tau = lambda * n / h^2`
    #[cfg(feature = "3d")]
    fn compute_torque(&self, lagrange: Scalar, axis: Vector, dt: Scalar) -> AngularVector {
        // Eq (17)
        lagrange * axis / dt.powi(2)
    }
}
