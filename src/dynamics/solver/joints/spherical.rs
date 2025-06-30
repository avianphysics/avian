//! [`SphericalJoint`] component.

use crate::{
    dynamics::solver::{
        solver_body::{SolverBody, SolverBodyInertia},
        xpbd::*,
    },
    prelude::*,
};
use bevy::{
    ecs::{
        entity::{EntityMapper, MapEntities},
        reflect::ReflectMapEntities,
    },
    prelude::*,
};

/// A spherical joint prevents relative translation of the attached bodies while allowing rotation around all axes.
///
/// Spherical joints can be useful for things like pendula, chains, ragdolls etc.
#[derive(Component, Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, MapEntities, PartialEq)]
pub struct SphericalJoint {
    /// First entity constrained by the joint.
    pub entity1: Entity,
    /// Second entity constrained by the joint.
    pub entity2: Entity,
    /// The point-to-point constraint that prevents relative translation of the attached bodies.
    pub point_constraint: PointConstraint,
    /// An axis that the attached bodies can swing around. This is normally the x-axis.
    pub swing_axis: Vector3,
    /// An axis that the attached bodies can twist around. This is normally the y-axis.
    pub twist_axis: Vector3,
    /// The extents of the allowed relative rotation of the bodies around the `swing_axis`.
    pub swing_limit: Option<AngleLimit>,
    /// The extents of the allowed relative rotation of the bodies around the `twist_axis`.
    pub twist_limit: Option<AngleLimit>,
    /// Linear damping applied by the joint.
    pub damping_linear: Scalar,
    /// Angular damping applied by the joint.
    pub damping_angular: Scalar,
    /// The relative dominance of the bodies.
    ///
    /// If the relative dominance is positive, the first body is dominant
    /// and is considered to have infinite mass.
    pub relative_dominance: i16,
    /// The joint's compliance for swing, the inverse of stiffness (N * m / rad).
    pub swing_compliance: Scalar,
    /// The joint's compliance for twist, the inverse of stiffness (N * m / rad).
    pub twist_compliance: Scalar,
    /// Lagrange multiplier for the angular correction caused by the swing limits.
    pub swing_lagrange: Scalar,
    /// Lagrange multiplier for the angular correction caused by the twist limits.
    pub twist_lagrange: Scalar,
    /// The torque exerted by the joint when limiting the relative rotation of the bodies around the `swing_axis`.
    pub swing_torque: Torque,
    /// The torque exerted by the joint when limiting the relative rotation of the bodies around the `twist_axis`.
    pub twist_torque: Torque,
    pre_step: SphericalJointPreStepData,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
struct SphericalJointPreStepData {
    swing_axis1: Vector,
    swing_axis2: Vector,
    twist_axis1: Vector,
    twist_axis2: Vector,
}

impl EntityConstraint<2> for SphericalJoint {
    fn entities(&self) -> [Entity; 2] {
        [self.entity1, self.entity2]
    }
}

impl XpbdConstraint<2> for SphericalJoint {
    fn clear_lagrange_multipliers(&mut self) {
        self.point_constraint.clear_lagrange_multipliers();
        self.swing_lagrange = 0.0;
        self.twist_lagrange = 0.0;
    }

    fn prepare(&mut self, bodies: [&RigidBodyQueryReadOnlyItem; 2], _dt: Scalar) {
        let [body1, body2] = bodies;

        // Compute the rotation matrices since we're performing so many rotations.
        let rot1_mat = Matrix::from_quat(body1.rotation.0);
        let rot2_mat = Matrix::from_quat(body2.rotation.0);

        // Prepare the point-to-point constraint.
        self.point_constraint.pre_step.world_r1 =
            rot1_mat * (self.point_constraint.local_anchor1 - body1.center_of_mass.0);
        self.point_constraint.pre_step.world_r2 =
            rot2_mat * (self.point_constraint.local_anchor2 - body2.center_of_mass.0);
        self.point_constraint.pre_step.center_difference = (body2.position.0 - body1.position.0)
            + (body2.rotation * body2.center_of_mass.0 - body1.rotation * body1.center_of_mass.0);

        // Prepare the base swing and twist axes.
        self.pre_step.swing_axis1 = rot1_mat * self.swing_axis;
        self.pre_step.swing_axis2 = rot2_mat * self.swing_axis;
        self.pre_step.twist_axis1 = rot1_mat * self.twist_axis;
        self.pre_step.twist_axis2 = rot2_mat * self.twist_axis;

        // Prepare the relative dominance.
        self.relative_dominance = body1.dominance() - body2.dominance();
    }

    fn solve(
        &mut self,
        bodies: [&mut SolverBody; 2],
        inertias: [&SolverBodyInertia; 2],
        dt: Scalar,
    ) {
        let [body1, body2] = bodies;
        let [inertia1, inertia2] = inertias;

        // Align positions
        self.point_constraint
            .solve([body1, body2], [inertia1, inertia2], dt);

        // Apply swing limits
        self.swing_torque = self.apply_swing_limits(body1, body2, inertia1, inertia2, dt);

        // Apply twist limits
        self.twist_torque = self.apply_twist_limits(body1, body2, inertia1, inertia2, dt);
    }
}

impl Joint for SphericalJoint {
    fn new(entity1: Entity, entity2: Entity) -> Self {
        Self {
            entity1,
            entity2,
            point_constraint: PointConstraint::default(),
            swing_axis: Vector3::X,
            twist_axis: Vector3::Y,
            swing_limit: None,
            twist_limit: None,
            damping_linear: 1.0,
            damping_angular: 1.0,
            relative_dominance: 0,
            swing_compliance: 0.0,
            twist_compliance: 0.0,
            swing_lagrange: 0.0,
            twist_lagrange: 0.0,
            #[cfg(feature = "2d")]
            swing_torque: 0.0,
            #[cfg(feature = "3d")]
            swing_torque: Vector::ZERO,
            #[cfg(feature = "2d")]
            twist_torque: 0.0,
            #[cfg(feature = "3d")]
            twist_torque: Vector::ZERO,
            pre_step: SphericalJointPreStepData::default(),
        }
    }

    #[inline]
    fn local_anchor_1(&self) -> Vector {
        self.point_constraint.local_anchor1
    }

    #[inline]
    fn local_anchor_2(&self) -> Vector {
        self.point_constraint.local_anchor2
    }

    #[inline]
    fn damping_linear(&self) -> Scalar {
        self.damping_linear
    }

    #[inline]
    fn damping_angular(&self) -> Scalar {
        self.damping_angular
    }

    #[inline]
    fn relative_dominance(&self) -> i16 {
        self.relative_dominance
    }
}

impl SphericalJoint {
    /// Sets the joint's compliance (inverse of stiffness, m / N).
    #[inline]
    #[deprecated(
        since = "0.4.0",
        note = "Use `with_point_compliance`, `with_swing_compliance`, and `with_twist_compliance` instead."
    )]
    pub fn with_compliance(mut self, compliance: Scalar) -> Self {
        self.point_constraint.compliance = compliance;
        self.swing_compliance = compliance;
        self.twist_compliance = compliance;
        self
    }

    /// Sets the compliance of the axis alignment constraint (inverse of stiffness, m / N).
    #[inline]
    pub fn with_point_compliance(mut self, compliance: Scalar) -> Self {
        self.point_constraint.compliance = compliance;
        self
    }

    /// Sets the compliance of the swing axis constraint (inverse of stiffness, N * m / rad).
    #[inline]
    pub fn with_swing_compliance(mut self, compliance: Scalar) -> Self {
        self.swing_compliance = compliance;
        self
    }

    /// Sets the compliance of the twist axis constraint (inverse of stiffness, N * m / rad).
    #[inline]
    pub fn with_twist_compliance(mut self, compliance: Scalar) -> Self {
        self.twist_compliance = compliance;
        self
    }

    /// Sets the attachment point on the first body.
    #[inline]
    pub fn with_local_anchor_1(mut self, anchor: Vector) -> Self {
        self.point_constraint.local_anchor1 = anchor;
        self
    }

    /// Sets the attachment point on the second body.
    #[inline]
    pub fn with_local_anchor_2(mut self, anchor: Vector) -> Self {
        self.point_constraint.local_anchor2 = anchor;
        self
    }

    /// Sets the linear velocity damping caused by the joint.
    #[inline]
    pub fn with_linear_velocity_damping(self, damping: Scalar) -> Self {
        Self {
            damping_linear: damping,
            ..self
        }
    }

    /// Sets the angular velocity damping caused by the joint.
    #[inline]
    pub fn with_angular_velocity_damping(self, damping: Scalar) -> Self {
        Self {
            damping_angular: damping,
            ..self
        }
    }

    /// Returns the Lagrange multiplier used for the positional correction.
    #[inline]
    pub fn point_lagrange(&self) -> Scalar {
        self.point_constraint.lagrange()
    }

    /// Returns the Lagrange multiplier used for the swing limit angular correction.
    #[inline]
    pub fn swing_lagrange(&self) -> Scalar {
        self.swing_lagrange
    }

    /// Returns the Lagrange multiplier used for the twist limit angular correction.
    #[inline]
    pub fn twist_lagrange(&self) -> Scalar {
        self.twist_lagrange
    }

    /// Returns the force exerted by the joint.
    #[inline]
    pub fn force(&self) -> Vector {
        self.point_constraint.force()
    }

    /// Sets the limits of the allowed relative rotation around the `swing_axis`.
    pub fn with_swing_limits(self, min: Scalar, max: Scalar) -> Self {
        Self {
            swing_limit: Some(AngleLimit::new(min, max)),
            ..self
        }
    }

    /// Sets the limits of the allowed relative rotation around the `twist_axis`.
    #[cfg(feature = "3d")]
    pub fn with_twist_limits(self, min: Scalar, max: Scalar) -> Self {
        Self {
            twist_limit: Some(AngleLimit::new(min, max)),
            ..self
        }
    }

    /// Applies angle limits to limit the relative rotation of the bodies around the `swing_axis`.
    fn apply_swing_limits(
        &mut self,
        body1: &mut SolverBody,
        body2: &mut SolverBody,
        inertia1: &SolverBodyInertia,
        inertia2: &SolverBodyInertia,
        dt: Scalar,
    ) -> Torque {
        if let Some(joint_limit) = self.swing_limit {
            let a1 = body1.delta_rotation * self.pre_step.swing_axis1;
            let a2 = body2.delta_rotation * self.pre_step.swing_axis2;

            let n = a1.cross(a2);
            let n_magnitude = n.length();

            if n_magnitude <= Scalar::EPSILON {
                return Torque::ZERO;
            }

            let n = n / n_magnitude;

            if let Some(correction) = joint_limit.compute_correction(n, a1, a2, PI) {
                let mut lagrange = self.swing_lagrange;

                let inv_inertia1 = inertia1.effective_inv_angular_inertia();
                let inv_inertia2 = inertia2.effective_inv_angular_inertia();

                let torque = self.align_orientation(
                    body1,
                    body2,
                    inv_inertia1,
                    inv_inertia2,
                    correction,
                    &mut lagrange,
                    self.swing_compliance,
                    dt,
                );
                self.swing_lagrange = lagrange;
                return torque;
            }
        }
        Torque::ZERO
    }

    /// Applies angle limits to limit the relative rotation of the bodies around the `twist_axis`.
    fn apply_twist_limits(
        &mut self,
        body1: &mut SolverBody,
        body2: &mut SolverBody,
        inertia1: &SolverBodyInertia,
        inertia2: &SolverBodyInertia,
        dt: Scalar,
    ) -> Torque {
        if let Some(joint_limit) = self.twist_limit {
            let a1 = body1.delta_rotation * self.pre_step.swing_axis1;
            let a2 = body2.delta_rotation * self.pre_step.swing_axis2;

            let n = a1 + a2;
            let n_magnitude = n.length();

            if n_magnitude <= Scalar::EPSILON {
                return Torque::ZERO;
            }

            let b1 = body1.delta_rotation * self.pre_step.twist_axis1;
            let b2 = body2.delta_rotation * self.pre_step.twist_axis2;

            let n = n / n_magnitude;

            let n1 = b1 - n.dot(b1) * n;
            let n2 = b2 - n.dot(b2) * n;
            let n1_magnitude = n1.length();
            let n2_magnitude = n2.length();

            if n1_magnitude <= Scalar::EPSILON || n2_magnitude <= Scalar::EPSILON {
                return Torque::ZERO;
            }

            let n1 = n1 / n1_magnitude;
            let n2 = n2 / n2_magnitude;

            let max_correction = if a1.dot(a2) > -0.5 { 2.0 * PI } else { dt };

            if let Some(correction) = joint_limit.compute_correction(n, n1, n2, max_correction) {
                let mut lagrange = self.twist_lagrange;

                let inv_inertia1 = inertia1.effective_inv_angular_inertia();
                let inv_inertia2 = inertia2.effective_inv_angular_inertia();

                let torque = self.align_orientation(
                    body1,
                    body2,
                    inv_inertia1,
                    inv_inertia2,
                    correction,
                    &mut lagrange,
                    self.twist_compliance,
                    dt,
                );
                self.twist_lagrange = lagrange;
                return torque;
            }
        }
        Torque::ZERO
    }
}

impl PositionConstraint for SphericalJoint {}

impl AngularConstraint for SphericalJoint {}

impl MapEntities for SphericalJoint {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.entity1 = entity_mapper.get_mapped(self.entity1);
        self.entity2 = entity_mapper.get_mapped(self.entity2);
    }
}
