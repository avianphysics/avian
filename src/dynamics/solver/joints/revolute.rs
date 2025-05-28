//! [`RevoluteJoint`] component.

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

/// A revolute joint prevents relative movement of the attached bodies, except for rotation around one `aligned_axis`.
///
/// Revolute joints can be useful for things like wheels, fans, revolving doors etc.
#[derive(Component, Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, MapEntities, PartialEq)]
pub struct RevoluteJoint {
    /// First entity constrained by the joint.
    pub entity1: Entity,
    /// Second entity constrained by the joint.
    pub entity2: Entity,
    /// The point-to-point constraint that prevents relative translation of the attached bodies.
    pub point_constraint: PointConstraint,
    /// A unit vector that controls which axis should be aligned for both entities.
    ///
    /// In 2D this should always be the Z axis.
    #[cfg(feature = "2d")]
    pub(crate) aligned_axis: Vector3,
    /// A unit vector that controls which axis should be aligned for both bodies.
    #[cfg(feature = "3d")]
    pub aligned_axis: Vector,
    /// The extents of the allowed relative rotation of the bodies around the `aligned_axis`.
    pub angle_limit: Option<AngleLimit>,
    /// Linear damping applied by the joint.
    pub damping_linear: Scalar,
    /// Angular damping applied by the joint.
    pub damping_angular: Scalar,
    /// The relative dominance of the bodies.
    ///
    /// If the relative dominance is positive, the first body is dominant
    /// and is considered to have infinite mass.
    pub relative_dominance: i16,
    /// The joint's compliance for aligning the bodies along the `aligned_axis`, the inverse of stiffness (N * m / rad).
    pub align_compliance: Scalar,
    /// The joint's compliance for the angle limit, the inverse of stiffness (N * m / rad).
    pub limit_compliance: Scalar,
    /// Lagrange multiplier for the angular correction caused by the alignment of the bodies.
    pub align_lagrange: Scalar,
    /// Lagrange multiplier for the angular correction caused by the angle limits.
    pub limit_lagrange: Scalar,
    /// The torque exerted by the joint when aligning the bodies.
    pub align_torque: Torque,
    /// The torque exerted by the joint when limiting the relative rotation of the bodies around the `aligned_axis`.
    pub limit_torque: Torque,
    pre_step: RevoluteJointPreStepData,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
struct RevoluteJointPreStepData {
    #[cfg(feature = "2d")]
    rotation_difference: Scalar,
    #[cfg(feature = "3d")]
    axis1: Vector,
    #[cfg(feature = "3d")]
    axis2: Vector,
}

impl EntityConstraint<2> for RevoluteJoint {
    fn entities(&self) -> [Entity; 2] {
        [self.entity1, self.entity2]
    }
}

impl XpbdConstraint<2> for RevoluteJoint {
    fn clear_lagrange_multipliers(&mut self) {
        self.point_constraint.clear_lagrange_multipliers();
        self.align_lagrange = 0.0;
        self.limit_lagrange = 0.0;
    }

    fn prepare(&mut self, bodies: [&RigidBodyQueryReadOnlyItem; 2], _dt: Scalar) {
        // Prepare the point-to-point constraint.
        self.point_constraint.prepare(bodies, _dt);

        // Prepare the base rotation difference.
        #[cfg(feature = "2d")]
        {
            self.pre_step.rotation_difference =
                bodies[0].rotation.angle_between(*bodies[1].rotation);
        }
        #[cfg(feature = "3d")]
        {
            // Prepare the base axes.
            self.pre_step.axis1 = bodies[0].rotation * self.aligned_axis;
            self.pre_step.axis2 = bodies[1].rotation * self.aligned_axis;
        }

        // Prepare the relative dominance.
        self.relative_dominance = bodies[0].dominance() - bodies[1].dominance();
    }

    fn solve(
        &mut self,
        bodies: [&mut SolverBody; 2],
        inertias: [&SolverBodyInertia; 2],
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
            let a1 = body1.delta_rotation * self.pre_step.axis1;
            let a2 = body2.delta_rotation * self.pre_step.axis2;
            let difference = a1.cross(a2);

            let mut lagrange = self.align_lagrange;
            self.align_torque = self.align_orientation(
                body1,
                body2,
                inv_angular_inertia1,
                inv_angular_inertia2,
                difference,
                &mut lagrange,
                self.align_compliance,
                dt,
            );
            self.align_lagrange = lagrange;
        }

        // Apply angle limits when rotating around the free axis
        self.limit_torque =
            self.apply_angle_limits(body1, body2, inv_angular_inertia1, inv_angular_inertia2, dt);

        // Align positions
        self.point_constraint.solve([body1, body2], inertias, dt);
    }
}

impl Joint for RevoluteJoint {
    fn new(entity1: Entity, entity2: Entity) -> Self {
        Self {
            entity1,
            entity2,
            point_constraint: PointConstraint::default(),
            aligned_axis: Vector3::Z,
            angle_limit: None,
            damping_linear: 1.0,
            damping_angular: 1.0,
            relative_dominance: 0,
            align_lagrange: 0.0,
            limit_lagrange: 0.0,
            align_compliance: 0.0,
            limit_compliance: 0.0,
            #[cfg(feature = "2d")]
            align_torque: 0.0,
            #[cfg(feature = "3d")]
            align_torque: Vector::ZERO,
            #[cfg(feature = "2d")]
            limit_torque: 0.0,
            #[cfg(feature = "3d")]
            limit_torque: Vector::ZERO,
            pre_step: RevoluteJointPreStepData::default(),
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

impl RevoluteJoint {
    /// Sets the joint's compliance (inverse of stiffness, m / N).
    #[inline]
    #[deprecated(
        since = "0.4.0",
        note = "Use `with_point_compliance`, `with_align_compliance`, and `with_limit_compliance` instead."
    )]
    pub fn with_compliance(mut self, compliance: Scalar) -> Self {
        self.point_constraint.compliance = compliance;
        self.align_compliance = compliance;
        self.limit_compliance = compliance;
        self
    }

    /// Sets the compliance of the point-to-point constraint (inverse of stiffness, m / N).
    #[inline]
    pub fn with_point_compliance(mut self, compliance: Scalar) -> Self {
        self.point_constraint.compliance = compliance;
        self
    }

    /// Sets the compliance of the axis alignment constraint (inverse of stiffness, N * m / rad).
    #[inline]
    pub fn with_align_compliance(mut self, compliance: Scalar) -> Self {
        self.align_compliance = compliance;
        self
    }

    /// Sets the compliance of the angle limit (inverse of stiffness, N * m / rad).
    #[inline]
    pub fn with_limit_compliance(mut self, compliance: Scalar) -> Self {
        self.limit_compliance = compliance;
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

    /// Returns the Lagrange multiplier used for the axis alignment correction.
    #[inline]
    pub fn align_lagrange(&self) -> Scalar {
        self.align_lagrange
    }

    /// Returns the Lagrange multiplier used for the angle limit correction.
    #[inline]
    pub fn limit_lagrange(&self) -> Scalar {
        self.limit_lagrange
    }

    /// Returns the force exerted by the joint.
    #[inline]
    pub fn force(&self) -> Vector {
        self.point_constraint.force()
    }

    /// Sets the axis that the bodies should be aligned on.
    #[cfg(feature = "3d")]
    pub fn with_aligned_axis(self, axis: Vector) -> Self {
        Self {
            aligned_axis: axis,
            ..self
        }
    }

    /// Sets the limits of the allowed relative rotation around the `aligned_axis`.
    pub fn with_angle_limits(self, min: Scalar, max: Scalar) -> Self {
        Self {
            angle_limit: Some(AngleLimit::new(min, max)),
            ..self
        }
    }

    /// Applies angle limits to limit the relative rotation of the bodies around the `aligned_axis`.
    #[allow(clippy::too_many_arguments)]
    fn apply_angle_limits(
        &mut self,
        body1: &mut SolverBody,
        body2: &mut SolverBody,
        inv_angular_inertia1: Tensor,
        inv_angular_inertia2: Tensor,
        dt: Scalar,
    ) -> Torque {
        let Some(Some(correction)) = self.angle_limit.map(|angle_limit| {
            #[cfg(feature = "2d")]
            {
                let rotation_difference = self.pre_step.rotation_difference
                    + body1.delta_rotation.angle_between(body2.delta_rotation);
                angle_limit.compute_correction(rotation_difference, PI)
            }
            #[cfg(feature = "3d")]
            {
                // [n, n1, n2] = [a1, b1, b2], where [a, b, c] are perpendicular unit axes on the bodies.
                let a1 = body1.delta_rotation * self.aligned_axis;
                let b1 = a1.any_orthonormal_vector();
                let b2 = body2.delta_rotation * self.aligned_axis.any_orthonormal_vector();
                angle_limit.compute_correction(a1, b1, b2, PI)
            }
        }) else {
            return Torque::ZERO;
        };

        let mut lagrange = self.limit_lagrange;
        let torque = self.align_orientation(
            body1,
            body2,
            inv_angular_inertia1,
            inv_angular_inertia2,
            correction,
            &mut lagrange,
            self.align_compliance,
            dt,
        );
        self.limit_lagrange = lagrange;
        torque
    }
}

impl PositionConstraint for RevoluteJoint {}

impl AngularConstraint for RevoluteJoint {}

impl MapEntities for RevoluteJoint {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.entity1 = entity_mapper.get_mapped(self.entity1);
        self.entity2 = entity_mapper.get_mapped(self.entity2);
    }
}
