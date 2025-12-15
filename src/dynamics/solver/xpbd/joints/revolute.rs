use super::PointConstraintShared;
use crate::{
    dynamics::solver::{
        solver_body::{SolverBody, SolverBodyInertia},
        xpbd::*,
    },
    prelude::*,
};
use bevy::prelude::*;

/// Constraint data required by the XPBD constraint solver for a [`RevoluteJoint`].
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, PartialEq)]
pub struct RevoluteJointSolverData {
    pub(super) point_constraint: PointConstraintShared,
    #[cfg(feature = "2d")]
    pub(super) rotation_difference: Scalar,
    #[cfg(feature = "3d")]
    pub(super) a1: Vector,
    #[cfg(feature = "3d")]
    pub(super) a2: Vector,
    #[cfg(feature = "3d")]
    pub(super) b1: Vector,
    #[cfg(feature = "3d")]
    pub(super) b2: Vector,
    pub(super) total_align_lagrange: AngularVector,
    pub(super) total_limit_lagrange: AngularVector,
}

impl XpbdConstraintSolverData for RevoluteJointSolverData {
    fn clear_lagrange_multipliers(&mut self) {
        self.point_constraint.clear_lagrange_multipliers();
        self.total_align_lagrange = AngularVector::ZERO;
        self.total_limit_lagrange = AngularVector::ZERO;
    }

    fn total_position_lagrange(&self) -> Vector {
        self.point_constraint.total_position_lagrange()
    }

    fn total_rotation_lagrange(&self) -> AngularVector {
        self.total_align_lagrange + self.total_limit_lagrange
    }
}

impl XpbdConstraint<2> for RevoluteJoint {
    type SolverData = RevoluteJointSolverData;

    fn prepare(
        &mut self,
        bodies: [&RigidBodyQueryReadOnlyItem; 2],
        solver_data: &mut RevoluteJointSolverData,
    ) {
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

        // Prepare the point-to-point constraint.
        solver_data
            .point_constraint
            .prepare(bodies, local_anchor1, local_anchor2);

        // Prepare the base rotation difference.
        #[cfg(feature = "2d")]
        {
            solver_data.rotation_difference = (*bodies[0].rotation * local_basis1)
                .angle_between(*bodies[1].rotation * local_basis2);
        }
        #[cfg(feature = "3d")]
        {
            // Prepare the base axes.
            solver_data.a1 = *bodies[0].rotation * local_basis1 * self.hinge_axis;
            solver_data.a2 = *bodies[1].rotation * local_basis2 * self.hinge_axis;
            solver_data.b1 =
                *bodies[0].rotation * local_basis1 * self.hinge_axis.any_orthonormal_vector();
            solver_data.b2 =
                *bodies[1].rotation * local_basis2 * self.hinge_axis.any_orthonormal_vector();
        }
    }

    fn solve(
        &mut self,
        bodies: [&mut SolverBody; 2],
        inertias: [&SolverBodyInertia; 2],
        solver_data: &mut RevoluteJointSolverData,
        dt: Scalar,
    ) {
        let [body1, body2] = bodies;
        let [inertia1, inertia2] = inertias;

        // Get the effective inverse angular inertia of the bodies.
        let inv_angular_inertia1 = inertia1.effective_inv_angular_inertia();
        let inv_angular_inertia2 = inertia2.effective_inv_angular_inertia();

        #[cfg(feature = "3d")]
        {
            // Constrain the relative rotation of the bodies, only allowing rotation around one free axis
            let a1 = body1.delta_rotation * solver_data.a1;
            let a2 = body2.delta_rotation * solver_data.a2;
            let difference = a1.cross(a2);

            solver_data.total_align_lagrange += self.align_orientation(
                body1,
                body2,
                inv_angular_inertia1,
                inv_angular_inertia2,
                difference,
                0.0,
                self.align_compliance,
                dt,
            );
        }

        // Apply angle limits when rotating around the free axis
        self.apply_angle_limits(
            body1,
            body2,
            inv_angular_inertia1,
            inv_angular_inertia2,
            solver_data,
            dt,
        );

        // Align positions
        solver_data
            .point_constraint
            .solve([body1, body2], inertias, self.point_compliance, dt);
    }
}

impl RevoluteJoint {
    /// Applies angle limits to limit the relative rotation of the bodies around the `hinge_axis`.
    fn apply_angle_limits(
        &self,
        body1: &mut SolverBody,
        body2: &mut SolverBody,
        inv_angular_inertia1: SymmetricTensor,
        inv_angular_inertia2: SymmetricTensor,
        solver_data: &mut RevoluteJointSolverData,
        dt: Scalar,
    ) {
        let Some(Some(correction)) = self.angle_limit.map(|angle_limit| {
            #[cfg(feature = "2d")]
            {
                let rotation_difference = solver_data.rotation_difference
                    + body1.delta_rotation.angle_between(body2.delta_rotation);
                angle_limit.compute_correction(rotation_difference, PI)
            }
            #[cfg(feature = "3d")]
            {
                // [n, n1, n2] = [a1, b1, b2], where [a, b, c] are perpendicular unit axes on the bodies.
                let a1 = body1.delta_rotation * solver_data.a1;
                let b1 = body1.delta_rotation * solver_data.b1;
                let b2 = body2.delta_rotation * solver_data.b2;
                angle_limit.compute_correction(a1, b1, b2, PI)
            }
        }) else {
            return;
        };

        solver_data.total_limit_lagrange += self.align_orientation(
            body1,
            body2,
            inv_angular_inertia1,
            inv_angular_inertia2,
            correction,
            0.0,
            self.limit_compliance,
            dt,
        );
    }
}

impl PositionConstraint for RevoluteJoint {}

impl AngularConstraint for RevoluteJoint {}
