use crate::{
    dynamics::solver::{
        solver_body::{SolverBody, SolverBodyInertia},
        xpbd::*,
    },
    prelude::*,
};
use bevy::prelude::*;

/// Constraint data required by the XPBD constraint solver for a [`DistanceJoint`].
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, PartialEq)]
pub struct DistanceJointSolverData {
    pub(super) world_r1: Vector,
    pub(super) world_r2: Vector,
    pub(super) center_difference: Vector,
    pub(super) total_lagrange: Vector,
}

impl XpbdConstraintSolverData for DistanceJointSolverData {
    fn clear_lagrange_multipliers(&mut self) {
        self.total_lagrange = Vector::ZERO;
    }

    fn total_position_lagrange(&self) -> Vector {
        self.total_lagrange
    }
}

impl XpbdConstraint<2> for DistanceJoint {
    type SolverData = DistanceJointSolverData;

    fn prepare(
        &mut self,
        bodies: [&RigidBodyQueryReadOnlyItem; 2],
        solver_data: &mut DistanceJointSolverData,
    ) {
        let [body1, body2] = bodies;

        let JointAnchor::Local(local_anchor1) = self.anchor1 else {
            return;
        };
        let JointAnchor::Local(local_anchor2) = self.anchor2 else {
            return;
        };

        // Prepare the base rotation difference.
        solver_data.world_r1 = body1.rotation * (local_anchor1 - body1.center_of_mass.0);
        solver_data.world_r2 = body2.rotation * (local_anchor2 - body2.center_of_mass.0);
        solver_data.center_difference = (body2.position.0 - body1.position.0)
            + (body2.rotation * body2.center_of_mass.0 - body1.rotation * body1.center_of_mass.0);
    }

    fn solve(
        &mut self,
        bodies: [&mut SolverBody; 2],
        inertias: [&SolverBodyInertia; 2],
        solver_data: &mut DistanceJointSolverData,
        dt: Scalar,
    ) {
        let [body1, body2] = bodies;
        let [inertia1, inertia2] = inertias;

        let inv_mass1 = inertia1.effective_inv_mass();
        let inv_mass2 = inertia2.effective_inv_mass();
        let inv_angular_inertia1 = inertia1.effective_inv_angular_inertia();
        let inv_angular_inertia2 = inertia2.effective_inv_angular_inertia();

        let world_r1 = body1.delta_rotation * solver_data.world_r1;
        let world_r2 = body2.delta_rotation * solver_data.world_r2;

        let separation = (body2.delta_position - body1.delta_position)
            + (world_r2 - world_r1)
            + solver_data.center_difference;

        // Compute the direction and magnitude of the positional correction required
        // to keep the bodies within a certain distance from each other.
        let (dir, distance) = self.limits.compute_correction(separation);

        if distance <= Scalar::EPSILON {
            // No separation, no need to apply a correction.
            return;
        }

        // Compute generalized inverse masses
        let w1 = PositionConstraint::compute_generalized_inverse_mass(
            self,
            inv_mass1.max_element(), // TODO: Do this properly.
            inv_angular_inertia1,
            world_r1,
            dir,
        );
        let w2 = PositionConstraint::compute_generalized_inverse_mass(
            self,
            inv_mass2.max_element(), // TODO: Do this properly.
            inv_angular_inertia2,
            world_r2,
            dir,
        );
        let w = [w1, w2];

        // Compute Lagrange multiplier update, essentially the signed magnitude of the correction.
        let delta_lagrange = compute_lagrange_update(0.0, distance, &w, self.compliance, dt);
        let impulse = delta_lagrange * dir;
        solver_data.total_lagrange += impulse;

        // Apply positional correction (method from PositionConstraint)
        self.apply_positional_impulse(
            body1, body2, inertia1, inertia2, impulse, world_r1, world_r2,
        );
    }
}

impl PositionConstraint for DistanceJoint {}

impl AngularConstraint for DistanceJoint {}
