use super::PointConstraintShared;
use crate::{
    dynamics::{
        joints::{AngularMotor, MotorModel},
        solver::{
            solver_body::{SolverBody, SolverBodyInertia},
            xpbd::*,
        },
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
    /// Accumulated motor Lagrange multiplier for this frame.
    pub(super) total_motor_lagrange: AngularVector,
    /// Motor Lagrange multiplier from the previous frame, used for warm starting.
    /// This is zeroed after being applied in the first substep.
    pub(super) warm_start_motor_lagrange: AngularVector,
}

impl XpbdConstraintSolverData for RevoluteJointSolverData {
    fn clear_lagrange_multipliers(&mut self) {
        self.point_constraint.clear_lagrange_multipliers();
        self.total_align_lagrange = AngularVector::ZERO;
        self.total_limit_lagrange = AngularVector::ZERO;
        // Save motor lagrange for warm starting before clearing.
        self.warm_start_motor_lagrange = self.total_motor_lagrange;
        self.total_motor_lagrange = AngularVector::ZERO;
    }

    fn total_motor_lagrange(&self) -> Scalar {
        #[cfg(feature = "2d")]
        {
            self.total_motor_lagrange
        }
        #[cfg(feature = "3d")]
        {
            self.total_motor_lagrange.length()
        }
    }

    fn total_position_lagrange(&self) -> Vector {
        self.point_constraint.total_position_lagrange()
    }

    fn total_rotation_lagrange(&self) -> AngularVector {
        self.total_align_lagrange + self.total_limit_lagrange + self.total_motor_lagrange
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

        // Solve motors before limits to give limits higher priority.
        self.apply_motor(
            body1,
            body2,
            inv_angular_inertia1,
            inv_angular_inertia2,
            solver_data,
            dt,
        );

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

    fn warm_start_motors(
        &self,
        bodies: [&mut SolverBody; 2],
        inertias: [&SolverBodyInertia; 2],
        solver_data: &mut RevoluteJointSolverData,
        _dt: Scalar,
        warm_start_coefficient: Scalar,
    ) {
        if !self.motor.enabled {
            return;
        }

        let [body1, body2] = bodies;
        let [inertia1, inertia2] = inertias;

        let inv_angular_inertia1 = inertia1.effective_inv_angular_inertia();
        let inv_angular_inertia2 = inertia2.effective_inv_angular_inertia();

        let impulse = warm_start_coefficient * solver_data.warm_start_motor_lagrange;

        body1.angular_velocity -= inv_angular_inertia1 * impulse;
        body2.angular_velocity += inv_angular_inertia2 * impulse;

        solver_data.warm_start_motor_lagrange = AngularVector::ZERO;
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

    /// Applies motor forces to drive the joint towards the target velocity and/or position.
    ///
    /// Uses a PD controller approach with optional implicit Euler integration improved stability.
    fn apply_motor(
        &self,
        body1: &mut SolverBody,
        body2: &mut SolverBody,
        inv_angular_inertia1: SymmetricTensor,
        inv_angular_inertia2: SymmetricTensor,
        solver_data: &mut RevoluteJointSolverData,
        dt: Scalar,
    ) {
        let motor = &self.motor;

        if !motor.enabled {
            return;
        }

        #[cfg(feature = "2d")]
        let current_angle = solver_data.rotation_difference
            + body1.delta_rotation.angle_between(body2.delta_rotation);
        #[cfg(feature = "3d")]
        let a1 = body1.delta_rotation * solver_data.a1;
        #[cfg(feature = "3d")]
        let current_angle = {
            let b1 = body1.delta_rotation * solver_data.b1;
            let b2 = body2.delta_rotation * solver_data.b2;
            let sin_angle = b1.cross(b2).dot(a1);
            let cos_angle = b1.dot(b2);
            sin_angle.atan2(cos_angle)
        };

        #[cfg(feature = "2d")]
        let relative_angular_velocity = body2.angular_velocity - body1.angular_velocity;
        #[cfg(feature = "3d")]
        let relative_angular_velocity = (body2.angular_velocity - body1.angular_velocity).dot(a1);

        #[cfg(feature = "2d")]
        let w_sum = inv_angular_inertia1 + inv_angular_inertia2;
        #[cfg(feature = "3d")]
        let w_sum =
            AngularConstraint::compute_generalized_inverse_mass(self, inv_angular_inertia1, a1)
                + AngularConstraint::compute_generalized_inverse_mass(
                    self,
                    inv_angular_inertia2,
                    a1,
                );

        if w_sum <= Scalar::EPSILON {
            return;
        }

        let velocity_error = motor.target_velocity - relative_angular_velocity;

        // Wrap position error to [-PI, PI] for shortest path rotation.
        let raw_error = motor.target_position - current_angle;
        let position_error = (raw_error + PI).rem_euclid(TAU) - PI;

        let Some(delta_lagrange) =
            Self::compute_angular_motor_lagrange(velocity_error, position_error, w_sum, motor, dt)
        else {
            return;
        };

        #[cfg(feature = "2d")]
        {
            solver_data.total_motor_lagrange += delta_lagrange;
        }
        #[cfg(feature = "3d")]
        {
            solver_data.total_motor_lagrange += delta_lagrange * a1;
        }

        // Positive delta_lagrange increases body2's angular velocity relative to body1.
        #[cfg(feature = "2d")]
        self.apply_angular_lagrange_update(
            body1,
            body2,
            inv_angular_inertia1,
            inv_angular_inertia2,
            delta_lagrange,
        );
        #[cfg(feature = "3d")]
        self.apply_angular_lagrange_update(
            body1,
            body2,
            inv_angular_inertia1,
            inv_angular_inertia2,
            delta_lagrange,
            a1,
        );
    }

    /// Computes the Lagrange multiplier delta for an angular motor.
    ///
    /// Returns `None` if the correction is negligible.
    fn compute_angular_motor_lagrange(
        velocity_error: Scalar,
        position_error: Scalar,
        w_sum: Scalar,
        motor: &AngularMotor,
        dt: Scalar,
    ) -> Option<Scalar> {
        let target_velocity_change = match motor.motor_model {
            MotorModel::SpringDamper {
                frequency,
                damping_ratio,
            } => {
                // Implicit Euler formulation for stable spring-damper behavior.
                let omega = TAU * frequency;
                let omega_sq = omega * omega;
                let two_zeta_omega = 2.0 * damping_ratio * omega;
                let inv_denominator = 1.0 / (1.0 + two_zeta_omega * dt + omega_sq * dt * dt);
                (omega_sq * position_error + two_zeta_omega * velocity_error) * dt * inv_denominator
            }
            MotorModel::AccelerationBased { stiffness, damping } => {
                damping * velocity_error + stiffness * position_error * dt
            }
            MotorModel::ForceBased { stiffness, damping } => {
                // Velocity change = (stiffness * pos_error + damping * vel_error) * inv_inertia
                (stiffness * position_error + damping * velocity_error) * w_sum
            }
        };

        let correction = target_velocity_change * dt;
        if correction.abs() <= Scalar::EPSILON {
            return None;
        }

        let delta_lagrange = correction / w_sum;

        // Clamp to limit instantaneous torque per substep.
        let delta_lagrange = if motor.max_torque < Scalar::MAX && motor.max_torque > 0.0 {
            let max_delta = motor.max_torque * dt * dt;
            delta_lagrange.clamp(-max_delta, max_delta)
        } else {
            delta_lagrange
        };

        Some(delta_lagrange)
    }
}

impl PositionConstraint for RevoluteJoint {}

impl AngularConstraint for RevoluteJoint {}
