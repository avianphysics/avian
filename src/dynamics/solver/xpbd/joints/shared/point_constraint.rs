use crate::{
    dynamics::solver::{
        solver_body::{SolverBody, SolverBodyInertia},
        xpbd::*,
    },
    prelude::*,
};
use bevy::prelude::*;

/// Constraint data required by the XPBD constraint solver for a point constraint.
#[derive(Clone, Copy, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct PointConstraintShared {
    /// The world-space anchor point relative to the center of mass of the first body.
    pub world_r1: Vector,
    /// The world-space anchor point relative to the center of mass of the second body.
    pub world_r2: Vector,
    /// The difference in center of mass positions between the two bodies.
    pub center_difference: Vector,
    /// The total Lagrange multiplier across the whole time step.
    pub total_lagrange: Vector,
}

impl XpbdConstraintSolverData for PointConstraintShared {
    fn clear_lagrange_multipliers(&mut self) {
        self.total_lagrange = Vector::ZERO;
    }

    fn total_position_lagrange(&self) -> Vector {
        self.total_lagrange
    }
}

impl PointConstraintShared {
    /// Prepares the constraint with the given bodies and local anchor points.
    pub fn prepare(
        &mut self,
        bodies: [&RigidBodyQueryReadOnlyItem; 2],
        local_anchor1: Vector,
        local_anchor2: Vector,
    ) {
        let [body1, body2] = bodies;

        self.world_r1 = body1.rotation * (local_anchor1 - body1.center_of_mass.0);
        self.world_r2 = body2.rotation * (local_anchor2 - body2.center_of_mass.0);
        self.center_difference = (body2.position.0 - body1.position.0)
            + (body2.rotation * body2.center_of_mass.0 - body1.rotation * body1.center_of_mass.0);
    }

    /// Solves the constraint for the given bodies.
    pub fn solve(
        &mut self,
        bodies: [&mut SolverBody; 2],
        inertias: [&SolverBodyInertia; 2],
        compliance: Scalar,
        dt: Scalar,
    ) {
        let [body1, body2] = bodies;
        let [inertia1, inertia2] = inertias;

        let inv_mass1 = inertia1.effective_inv_mass();
        let inv_mass2 = inertia2.effective_inv_mass();
        let inv_angular_inertia1 = inertia1.effective_inv_angular_inertia();
        let inv_angular_inertia2 = inertia2.effective_inv_angular_inertia();

        let world_r1 = body1.delta_rotation * self.world_r1;
        let world_r2 = body2.delta_rotation * self.world_r2;

        let separation = (body2.delta_position - body1.delta_position)
            + (world_r2 - world_r1)
            + self.center_difference;

        let magnitude_squared = separation.length_squared();

        if magnitude_squared == 0.0 {
            // No separation, no need to apply a correction.
            return;
        }

        let magnitude = magnitude_squared.sqrt();
        let dir = -separation / magnitude;

        // Compute generalized inverse masses
        let w1 = self.compute_generalized_inverse_mass(
            inv_mass1.max_element(), // TODO: Do this properly.
            inv_angular_inertia1,
            world_r1,
            dir,
        );
        let w2 = self.compute_generalized_inverse_mass(
            inv_mass2.max_element(), // TODO: Do this properly.
            inv_angular_inertia2,
            world_r2,
            dir,
        );

        // Compute Lagrange multiplier update
        let delta_lagrange = compute_lagrange_update(0.0, magnitude, &[w1, w2], compliance, dt);
        let impulse = delta_lagrange * dir;
        self.total_lagrange += impulse;

        // Apply positional correction to align the positions of the bodies
        self.apply_positional_impulse(
            body1, body2, inertia1, inertia2, impulse, world_r1, world_r2,
        );
    }
}

impl PositionConstraint for PointConstraintShared {}
