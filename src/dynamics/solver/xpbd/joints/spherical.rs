use super::PointConstraintShared;
use crate::{
    dynamics::solver::{
        solver_body::{SolverBody, SolverBodyInertia},
        xpbd::*,
    },
    prelude::*,
};
use bevy::prelude::*;

/// Constraint data required by the XPBD constraint solver for a [`SphericalJoint`].
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, PartialEq)]
pub struct SphericalJointSolverData {
    pub(super) point_constraint: PointConstraintShared,
    pub(super) swing_axis1: Vector,
    pub(super) swing_axis2: Vector,
    pub(super) twist_axis1: Vector,
    pub(super) twist_axis2: Vector,
    pub(super) total_swing_lagrange: Vector,
    pub(super) total_twist_lagrange: Vector,
}

impl XpbdConstraintSolverData for SphericalJointSolverData {
    fn clear_lagrange_multipliers(&mut self) {
        self.point_constraint.clear_lagrange_multipliers();
        self.total_swing_lagrange = Vector::ZERO;
        self.total_twist_lagrange = Vector::ZERO;
    }

    fn total_position_lagrange(&self) -> Vector {
        self.point_constraint.total_position_lagrange()
    }

    fn total_rotation_lagrange(&self) -> AngularVector {
        self.total_swing_lagrange + self.total_twist_lagrange
    }
}

impl XpbdConstraint<2> for SphericalJoint {
    type SolverData = SphericalJointSolverData;

    fn prepare(
        &mut self,
        bodies: [&RigidBodyQueryReadOnlyItem; 2],
        solver_data: &mut SphericalJointSolverData,
    ) {
        let [body1, body2] = bodies;

        let Some(local_anchor1) = self.local_anchor1() else {
            return;
        };
        let Some(local_anchor2) = self.local_anchor2() else {
            return;
        };
        let Some(local_basis1) = self.local_basis1() else {
            return;
        };
        let Some(local_basis2) = self.local_basis2() else {
            return;
        };

        // Compute the rotation matrices since we're performing so many rotations.
        let rot1_mat = Matrix::from_quat(body1.rotation.0);
        let rot2_mat = Matrix::from_quat(body2.rotation.0);

        // Prepare the point-to-point constraint.
        let point_constraint = &mut solver_data.point_constraint;
        point_constraint.world_r1 = rot1_mat * (local_anchor1 - body1.center_of_mass.0);
        point_constraint.world_r2 = rot2_mat * (local_anchor2 - body2.center_of_mass.0);
        point_constraint.center_difference = (body2.position.0 - body1.position.0)
            + (body2.rotation * body2.center_of_mass.0 - body1.rotation * body1.center_of_mass.0);

        // Prepare the base swing and twist axes.
        let swing_axis = self.twist_axis.any_orthonormal_vector();
        solver_data.swing_axis1 = rot1_mat * (local_basis1 * swing_axis);
        solver_data.swing_axis2 = rot2_mat * (local_basis2 * swing_axis);
        solver_data.twist_axis1 = rot1_mat * (local_basis1 * self.twist_axis);
        solver_data.twist_axis2 = rot2_mat * (local_basis2 * self.twist_axis);
    }

    fn solve(
        &mut self,
        bodies: [&mut SolverBody; 2],
        inertias: [&SolverBodyInertia; 2],
        solver_data: &mut SphericalJointSolverData,
        dt: Scalar,
    ) {
        let [body1, body2] = bodies;
        let [inertia1, inertia2] = inertias;

        // Align positions
        solver_data.point_constraint.solve(
            [body1, body2],
            [inertia1, inertia2],
            self.point_compliance,
            dt,
        );

        // Apply swing limits
        self.apply_swing_limits(body1, body2, inertia1, inertia2, solver_data, dt);

        // Apply twist limits
        self.apply_twist_limits(body1, body2, inertia1, inertia2, solver_data, dt);
    }
}

impl SphericalJoint {
    /// Applies angle limits to limit the relative rotation of the bodies around the `swing_axis`.
    fn apply_swing_limits(
        &self,
        body1: &mut SolverBody,
        body2: &mut SolverBody,
        inertia1: &SolverBodyInertia,
        inertia2: &SolverBodyInertia,
        solver_data: &mut SphericalJointSolverData,
        dt: Scalar,
    ) {
        if let Some(joint_limit) = self.swing_limit {
            let a1 = body1.delta_rotation * solver_data.swing_axis1;
            let a2 = body2.delta_rotation * solver_data.swing_axis2;

            let n = a1.cross(a2);
            let n_magnitude = n.length();

            if n_magnitude <= Scalar::EPSILON {
                return;
            }

            let n = n / n_magnitude;

            if let Some(correction) = joint_limit.compute_correction(n, a1, a2, PI) {
                let inv_inertia1 = inertia1.effective_inv_angular_inertia();
                let inv_inertia2 = inertia2.effective_inv_angular_inertia();

                solver_data.total_swing_lagrange += self.align_orientation(
                    body1,
                    body2,
                    inv_inertia1,
                    inv_inertia2,
                    correction,
                    0.0,
                    self.swing_compliance,
                    dt,
                );
            }
        }
    }

    /// Applies angle limits to limit the relative rotation of the bodies around the `twist_axis`.
    fn apply_twist_limits(
        &self,
        body1: &mut SolverBody,
        body2: &mut SolverBody,
        inertia1: &SolverBodyInertia,
        inertia2: &SolverBodyInertia,
        solver_data: &mut SphericalJointSolverData,
        dt: Scalar,
    ) {
        if let Some(joint_limit) = self.twist_limit {
            let a1 = body1.delta_rotation * solver_data.swing_axis1;
            let a2 = body2.delta_rotation * solver_data.swing_axis2;

            let n = a1 + a2;
            let n_magnitude = n.length();

            if n_magnitude <= Scalar::EPSILON {
                return;
            }

            let b1 = body1.delta_rotation * solver_data.twist_axis1;
            let b2 = body2.delta_rotation * solver_data.twist_axis2;

            let n = n / n_magnitude;

            let n1 = b1 - n.dot(b1) * n;
            let n2 = b2 - n.dot(b2) * n;
            let n1_magnitude = n1.length();
            let n2_magnitude = n2.length();

            if n1_magnitude <= Scalar::EPSILON || n2_magnitude <= Scalar::EPSILON {
                return;
            }

            let n1 = n1 / n1_magnitude;
            let n2 = n2 / n2_magnitude;

            let max_correction = if a1.dot(a2) > -0.5 { 2.0 * PI } else { dt };

            if let Some(correction) = joint_limit.compute_correction(n, n1, n2, max_correction) {
                let inv_inertia1 = inertia1.effective_inv_angular_inertia();
                let inv_inertia2 = inertia2.effective_inv_angular_inertia();

                solver_data.total_twist_lagrange += self.align_orientation(
                    body1,
                    body2,
                    inv_inertia1,
                    inv_inertia2,
                    correction,
                    0.0,
                    self.twist_compliance,
                    dt,
                );
            }
        }
    }
}

impl PositionConstraint for SphericalJoint {}

impl AngularConstraint for SphericalJoint {}
