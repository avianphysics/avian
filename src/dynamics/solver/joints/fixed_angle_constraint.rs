use crate::{
    dynamics::solver::{
        solver_body::{SolverBody, SolverBodyInertia},
        xpbd::*,
    },
    prelude::*,
};
use bevy::prelude::*;

// TODO: Support relative base rotation.
/// A constraint that keeps the attached bodies at a fixed angle
/// from each other while while allowing translation along all axes.
#[derive(Clone, Copy, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct FixedAngleConstraint {
    /// The constraint's compliance, the inverse of stiffness (N * m / rad).
    pub compliance: Scalar,
    /// Lagrange multiplier for the angular correction caused by the alignment of the bodies.
    lagrange: Scalar,
    /// The torque exerted by the constraint when aligning the bodies.
    torque: Torque,
    /// Pre-step data for the solver.
    pub(crate) pre_step: FixedAngleConstraintPreStepData,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub(crate) struct FixedAngleConstraintPreStepData {
    #[cfg(feature = "2d")]
    pub(crate) rotation_difference: Scalar,
    #[cfg(feature = "3d")]
    pub(crate) rotation_difference: Quaternion,
}

impl XpbdConstraint<2> for FixedAngleConstraint {
    fn clear_lagrange_multipliers(&mut self) {
        self.lagrange = 0.0;
    }

    fn prepare(&mut self, bodies: [&RigidBodyQueryReadOnlyItem; 2], _dt: Scalar) {
        let [body1, body2] = bodies;

        // Prepare the base rotation difference.
        #[cfg(feature = "2d")]
        {
            self.pre_step.rotation_difference = body1.rotation.angle_between(*body2.rotation);
        }
        #[cfg(feature = "3d")]
        {
            self.pre_step.rotation_difference = body1.rotation.0 * body2.rotation.inverse().0;
        }
    }

    fn solve(
        &mut self,
        bodies: [&mut SolverBody; 2],
        inertias: [&SolverBodyInertia; 2],
        dt: Scalar,
    ) {
        let [body1, body2] = bodies;
        let [inertia1, inertia2] = inertias;

        let inv_inertia1 = inertia1.effective_inv_angular_inertia();
        let inv_inertia2 = inertia2.effective_inv_angular_inertia();

        #[cfg(feature = "2d")]
        let difference = self.pre_step.rotation_difference
            + body1.delta_rotation.angle_between(body2.delta_rotation);
        #[cfg(feature = "3d")]
        // TODO: The XPBD paper doesn't have this minus sign, but it seems to be needed for stability.
        //       The angular correction code might have a wrong sign elsewhere.
        let difference = -2.0
            * (self.pre_step.rotation_difference
                * body1.delta_rotation.0
                * body2.delta_rotation.0.inverse())
            .xyz();

        // Align orientation
        let mut lagrange = self.lagrange;
        self.torque = self.align_orientation(
            body1,
            body2,
            inv_inertia1,
            inv_inertia2,
            difference,
            &mut lagrange,
            self.compliance,
            dt,
        );
        self.lagrange = lagrange;
    }
}

impl AngularConstraint for FixedAngleConstraint {}

impl FixedAngleConstraint {
    /// Sets the joint's compliance (inverse of stiffness, (N * m / rad).
    #[inline]
    pub fn with_compliance(mut self, compliance: Scalar) -> Self {
        self.compliance = compliance;
        self
    }

    /// Returns the Lagrange multiplier used for the positional correction.
    #[inline]
    pub fn lagrange(&self) -> Scalar {
        self.lagrange
    }

    /// Returns the torque exerted by the joint.
    #[inline]
    pub fn torque(&self) -> Torque {
        self.torque
    }
}
