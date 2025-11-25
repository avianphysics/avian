use super::FixedAngleConstraintShared;
use crate::{
    dynamics::solver::{
        solver_body::{SolverBody, SolverBodyInertia},
        xpbd::*,
    },
    prelude::*,
};
use bevy::prelude::*;

/// Constraint data required by the XPBD constraint solver for a [`PrismaticJoint`].
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, PartialEq)]
pub struct PrismaticJointSolverData {
    pub(super) world_r1: Vector,
    pub(super) world_r2: Vector,
    pub(super) center_difference: Vector,
    pub(super) free_axis1: Vector,
    pub(super) total_position_lagrange: Vector,
    pub(super) angle_constraint: FixedAngleConstraintShared,
}

impl XpbdConstraintSolverData for PrismaticJointSolverData {
    fn clear_lagrange_multipliers(&mut self) {
        self.total_position_lagrange = Vector::ZERO;
        self.angle_constraint.clear_lagrange_multipliers();
    }

    fn total_position_lagrange(&self) -> Vector {
        self.total_position_lagrange
    }

    fn total_rotation_lagrange(&self) -> AngularVector {
        self.angle_constraint.total_rotation_lagrange()
    }
}

impl XpbdConstraint<2> for PrismaticJoint {
    type SolverData = PrismaticJointSolverData;

    fn prepare(
        &mut self,
        bodies: [&RigidBodyQueryReadOnlyItem; 2],
        solver_data: &mut PrismaticJointSolverData,
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

        // Prepare the point-to-point constraint.
        solver_data.angle_constraint.prepare(
            body1.rotation,
            body2.rotation,
            local_basis1,
            local_basis2,
        );

        // Prepare the prismatic joint.
        solver_data.world_r1 = body1.rotation * (local_anchor1 - body1.center_of_mass.0);
        solver_data.world_r2 = body2.rotation * (local_anchor2 - body2.center_of_mass.0);
        solver_data.center_difference = (body2.position.0 - body1.position.0)
            + (body2.rotation * body2.center_of_mass.0 - body1.rotation * body1.center_of_mass.0);
        solver_data.free_axis1 = *body1.rotation * local_basis1 * self.slider_axis;
    }

    fn solve(
        &mut self,
        bodies: [&mut SolverBody; 2],
        inertias: [&SolverBodyInertia; 2],
        solver_data: &mut PrismaticJointSolverData,
        dt: Scalar,
    ) {
        let [body1, body2] = bodies;

        // Solve the angular constraint.
        solver_data
            .angle_constraint
            .solve([body1, body2], inertias, self.angle_compliance, dt);

        // Constrain the relative positions of the bodies, only allowing translation along one free axis.
        self.constrain_positions(body1, body2, inertias[0], inertias[1], solver_data, dt);
    }
}

impl PrismaticJoint {
    /// Constrains the relative positions of the bodies, only allowing translation along one free axis.
    ///
    /// Returns the force exerted by this constraint.
    fn constrain_positions(
        &self,
        body1: &mut SolverBody,
        body2: &mut SolverBody,
        inertia1: &SolverBodyInertia,
        inertia2: &SolverBodyInertia,
        solver_data: &mut PrismaticJointSolverData,
        dt: Scalar,
    ) {
        // Compute the effective inverse masses and angular inertias of the bodies.
        let inv_mass1 = inertia1.effective_inv_mass();
        let inv_mass2 = inertia2.effective_inv_mass();
        let inv_angular_inertia1 = inertia1.effective_inv_angular_inertia();
        let inv_angular_inertia2 = inertia2.effective_inv_angular_inertia();

        let world_r1 = body1.delta_rotation * solver_data.world_r1;
        let world_r2 = body2.delta_rotation * solver_data.world_r2;

        let mut delta_x = Vector::ZERO;

        let axis1 = body1.delta_rotation * solver_data.free_axis1;
        if let Some(limits) = self.limits {
            let separation = (body2.delta_position - body1.delta_position)
                + (world_r2 - world_r1)
                + solver_data.center_difference;
            delta_x += limits.compute_correction_along_axis(separation, axis1);
        }

        let zero_distance_limit = DistanceLimit::ZERO;

        #[cfg(feature = "2d")]
        {
            let axis2 = Vector::new(axis1.y, -axis1.x);

            let separation = (body2.delta_position - body1.delta_position)
                + (world_r2 - world_r1)
                + solver_data.center_difference;
            delta_x += zero_distance_limit.compute_correction_along_axis(separation, axis2);
        }
        #[cfg(feature = "3d")]
        {
            let axis2 = axis1.any_orthogonal_vector();
            let axis3 = axis1.cross(axis2);

            let separation = (body2.delta_position - body1.delta_position)
                + (world_r2 - world_r1)
                + solver_data.center_difference;
            delta_x += zero_distance_limit.compute_correction_along_axis(separation, axis2);

            let separation = (body2.delta_position - body1.delta_position)
                + (world_r2 - world_r1)
                + solver_data.center_difference;
            delta_x += zero_distance_limit.compute_correction_along_axis(separation, axis3);
        }

        let magnitude = delta_x.length();

        if magnitude <= Scalar::EPSILON {
            return;
        }

        let dir = delta_x / magnitude;

        // Compute generalized inverse masses
        let w1 = PositionConstraint::compute_generalized_inverse_mass(
            self,
            inv_mass1.max_element(),
            inv_angular_inertia1,
            world_r1,
            dir,
        );
        let w2 = PositionConstraint::compute_generalized_inverse_mass(
            self,
            inv_mass2.max_element(),
            inv_angular_inertia2,
            world_r2,
            dir,
        );

        // Compute Lagrange multiplier update
        let delta_lagrange =
            compute_lagrange_update(0.0, magnitude, &[w1, w2], self.align_compliance, dt);
        let impulse = delta_lagrange * dir;
        solver_data.total_position_lagrange += impulse;

        // Apply positional correction to align the positions of the bodies
        self.apply_positional_impulse(
            body1, body2, inertia1, inertia2, impulse, world_r1, world_r2,
        );
    }
}

impl PositionConstraint for PrismaticJoint {}

impl AngularConstraint for PrismaticJoint {}
