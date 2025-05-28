use crate::{
    dynamics::solver::{
        solver_body::{SolverBody, SolverBodyInertia},
        xpbd::*,
    },
    prelude::*,
};
use bevy::prelude::*;

/// A point-to-point constraint that constrains two points to each other.
#[derive(Clone, Copy, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct PointConstraint {
    /// Attachment point on the first body.
    pub local_anchor1: Vector,
    /// Attachment point on the second body.
    pub local_anchor2: Vector,
    /// The joint's compliance, the inverse of stiffness (m / N).
    pub compliance: Scalar,
    /// Lagrange multiplier for the positional correction.
    lagrange: Scalar,
    /// The force exerted by the joint.
    force: Vector,
    /// Pre-step data for the solver.
    pub(crate) pre_step: PointConstraintPreStepData,
}

/// Pre-step data for the [`PointConstraint`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub(crate) struct PointConstraintPreStepData {
    pub(crate) world_r1: Vector,
    pub(crate) world_r2: Vector,
    pub(crate) center_difference: Vector,
}

impl XpbdConstraint<2> for PointConstraint {
    fn clear_lagrange_multipliers(&mut self) {
        self.lagrange = 0.0;
    }

    fn prepare(&mut self, bodies: [&RigidBodyQueryReadOnlyItem; 2], _dt: Scalar) {
        let [body1, body2] = bodies;

        self.pre_step.world_r1 = body1.rotation * (self.local_anchor1 - body1.center_of_mass.0);
        self.pre_step.world_r2 = body2.rotation * (self.local_anchor2 - body2.center_of_mass.0);
        self.pre_step.center_difference = body2.position.0 - body1.position.0;
    }

    fn solve(
        &mut self,
        bodies: [&mut SolverBody; 2],
        inertias: [&SolverBodyInertia; 2],
        dt: Scalar,
    ) {
        let [body1, body2] = bodies;
        let [inertia1, inertia2] = inertias;

        let inv_mass1 = inertia1.effective_inv_mass();
        let inv_mass2 = inertia2.effective_inv_mass();
        let inv_angular_inertia1 = inertia1.effective_inv_angular_inertia();
        let inv_angular_inertia2 = inertia2.effective_inv_angular_inertia();

        let world_r1 = body1.delta_rotation * self.pre_step.world_r1;
        let world_r2 = body2.delta_rotation * self.pre_step.world_r2;

        let separation = (body2.delta_position - body1.delta_position)
            + (world_r2 - world_r1)
            + self.pre_step.center_difference;

        let magnitude_squared = separation.length_squared();

        if magnitude_squared <= Scalar::EPSILON {
            // No separation, no need to apply a correction.
            self.force = Vector::ZERO;
            return;
        }

        let magnitude = magnitude_squared.sqrt();
        let dir = -separation / magnitude;

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

        // Compute Lagrange multiplier update
        let delta_lagrange =
            self.compute_lagrange_update(self.lagrange, magnitude, &[w1, w2], self.compliance, dt);
        self.lagrange += delta_lagrange;

        // Apply positional correction to align the positions of the bodies
        self.apply_positional_lagrange_update(
            body1,
            body2,
            inertia1,
            inertia2,
            delta_lagrange,
            dir,
            world_r1,
            world_r2,
        );

        self.force = self.compute_force(self.lagrange, dir, dt);
    }
}

impl PositionConstraint for PointConstraint {}

impl PointConstraint {
    /// Sets the joint's compliance (inverse of stiffness, m / N).
    #[inline]
    pub fn with_compliance(mut self, compliance: Scalar) -> Self {
        self.compliance = compliance;
        self
    }

    /// Sets the attachment point on the first body.
    #[inline]
    pub fn with_local_anchor_1(mut self, anchor: Vector) -> Self {
        self.local_anchor1 = anchor;
        self
    }

    /// Sets the attachment point on the second body.
    #[inline]
    pub fn with_local_anchor_2(mut self, anchor: Vector) -> Self {
        self.local_anchor2 = anchor;
        self
    }

    /// Returns the Lagrange multiplier used for the positional correction.
    #[inline]
    pub fn lagrange(&self) -> Scalar {
        self.lagrange
    }

    /// Returns the force exerted by the joint.
    #[inline]
    pub fn force(&self) -> Vector {
        self.force
    }
}
