use crate::{
    dynamics::solver::{
        solver_body::{SolverBody, SolverBodyInertia},
        xpbd::*,
    },
    prelude::*,
};
use bevy::prelude::*;

/// Constraint data required by the XPBD constraint solver for a fixed angle constraint.
#[derive(Clone, Copy, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct FixedAngleConstraintShared {
    /// The target rotation difference between the two bodies.
    #[cfg(feature = "2d")]
    pub rotation_difference: f32,
    /// The target rotation difference between the two bodies.
    #[cfg(feature = "3d")]
    pub rotation_difference: Quat,
    /// The total Lagrange multiplier across the whole time step.
    pub total_lagrange: AngularVector,
}

impl XpbdConstraintSolverData for FixedAngleConstraintShared {
    fn clear_lagrange_multipliers(&mut self) {
        self.total_lagrange = AngularVector::default();
    }

    fn total_rotation_lagrange(&self) -> AngularVector {
        self.total_lagrange
    }
}

impl FixedAngleConstraintShared {
    /// Prepares the constraint with the given rotations and local basis orientations.
    pub fn prepare(
        &mut self,
        rotation1: Rot,
        rotation2: Rot,
        local_basis1: Rot,
        local_basis2: Rot,
    ) {
        // Prepare the base rotation difference.
        #[cfg(feature = "2d")]
        {
            self.rotation_difference =
                (rotation1 * local_basis1).angle_to(rotation2 * local_basis2);
        }
        #[cfg(feature = "3d")]
        {
            self.rotation_difference =
                (rotation1 * local_basis1) * (rotation2 * local_basis2).inverse();
        }
    }

    /// Solves the constraint for the given bodies.
    pub fn solve(
        &mut self,
        bodies: [&mut SolverBody; 2],
        inertias: [&SolverBodyInertia; 2],
        compliance: f32,
        dt: f32,
    ) {
        let [body1, body2] = bodies;
        let [inertia1, inertia2] = inertias;

        let inv_inertia1 = inertia1.effective_inv_angular_inertia();
        let inv_inertia2 = inertia2.effective_inv_angular_inertia();

        #[cfg(feature = "2d")]
        let difference =
            self.rotation_difference + body1.delta_rotation.angle_to(body2.delta_rotation);
        #[cfg(feature = "3d")]
        // TODO: The XPBD paper doesn't have this minus sign, but it seems to be needed for stability.
        //       The angular correction code might have a wrong sign elsewhere.
        let difference = -2.0
            * (self.rotation_difference * body1.delta_rotation * body2.delta_rotation.inverse())
                .xyz();

        // Align orientation
        self.total_lagrange += self.align_orientation(
            body1,
            body2,
            inv_inertia1,
            inv_inertia2,
            difference,
            0.0,
            compliance,
            dt,
        );
    }
}

impl AngularConstraint for FixedAngleConstraintShared {}
