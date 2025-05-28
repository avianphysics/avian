//! [`PrismaticJoint`] component.

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

/// A prismatic joint prevents relative movement of the attached bodies, except for translation along one `free_axis`.
///
/// Prismatic joints can be useful for things like elevators, pistons, sliding doors and moving platforms.
#[derive(Component, Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, MapEntities, PartialEq)]
pub struct PrismaticJoint {
    /// First entity constrained by the joint.
    pub entity1: Entity,
    /// Second entity constrained by the joint.
    pub entity2: Entity,
    /// The angle constraint that prevents relative rotation of the attached bodies.
    pub angle_constraint: FixedAngleConstraint,
    /// Attachment point on the first body.
    pub local_anchor1: Vector,
    /// Attachment point on the second body.
    pub local_anchor2: Vector,
    /// A free axis that the attached bodies can translate along relative to each other.
    pub free_axis: Vector,
    /// The extents of the allowed relative translation along the free axis.
    pub free_axis_limits: Option<DistanceLimit>,
    /// Linear damping applied by the joint.
    pub damping_linear: Scalar,
    /// Angular damping applied by the joint.
    pub damping_angular: Scalar,
    /// The relative dominance of the bodies.
    ///
    /// If the relative dominance is positive, the first body is dominant
    /// and is considered to have infinite mass.
    pub relative_dominance: i16,
    /// Lagrange multiplier for the positional correction.
    position_lagrange: Scalar,
    /// The joint's compliance for aligning the positions of the bodies to the `free_axis`, the inverse of stiffness (m / N).
    pub axis_compliance: Scalar,
    /// The joint's compliance for the distance limit, the inverse of stiffness (m / N).
    pub limit_compliance: Scalar,
    /// The force exerted by the joint.
    force: Vector,
    /// Pre-step data for the solver.
    pre_step: PrismaticJointPreStepData,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
struct PrismaticJointPreStepData {
    world_r1: Vector,
    world_r2: Vector,
    center_difference: Vector,
    free_axis1: Vector,
}

impl EntityConstraint<2> for PrismaticJoint {
    fn entities(&self) -> [Entity; 2] {
        [self.entity1, self.entity2]
    }
}

impl XpbdConstraint<2> for PrismaticJoint {
    fn clear_lagrange_multipliers(&mut self) {
        self.position_lagrange = 0.0;
        self.angle_constraint.clear_lagrange_multipliers();
    }

    fn prepare(&mut self, bodies: [&RigidBodyQueryReadOnlyItem; 2], dt: Scalar) {
        let [body1, body2] = bodies;

        // Prepare the point-to-point constraint.
        self.angle_constraint.prepare(bodies, dt);

        // Prepare the prismatic joint.
        self.pre_step.world_r1 = body1.rotation * (self.local_anchor1 - body1.center_of_mass.0);
        self.pre_step.world_r2 = body2.rotation * (self.local_anchor2 - body2.center_of_mass.0);
        self.pre_step.center_difference = body2.position.0 - body1.position.0;
        self.pre_step.free_axis1 = body1.rotation * self.free_axis;

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

        // Solve the angular constraint.
        self.angle_constraint.solve([body1, body2], inertias, dt);

        // Constrain the relative positions of the bodies, only allowing translation along one free axis.
        self.force = self.constrain_positions(body1, body2, inertias[0], inertias[1], dt);
    }
}

impl Joint for PrismaticJoint {
    fn new(entity1: Entity, entity2: Entity) -> Self {
        Self {
            entity1,
            entity2,
            angle_constraint: FixedAngleConstraint::default(),
            local_anchor1: Vector::ZERO,
            local_anchor2: Vector::ZERO,
            free_axis: Vector::X,
            free_axis_limits: None,
            damping_linear: 1.0,
            damping_angular: 1.0,
            relative_dominance: 0,
            position_lagrange: 0.0,
            axis_compliance: 0.0,
            limit_compliance: 0.0,
            force: Vector::ZERO,
            pre_step: PrismaticJointPreStepData::default(),
        }
    }

    #[inline]
    fn local_anchor_1(&self) -> Vector {
        self.local_anchor1
    }

    #[inline]
    fn local_anchor_2(&self) -> Vector {
        self.local_anchor2
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

impl PrismaticJoint {
    /// Sets the joint's compliance (inverse of stiffness).
    #[inline]
    #[deprecated(
        since = "0.4.0",
        note = "Use `with_axis_compliance`, `with_limit_compliance`, and `with_angle_compliance` instead."
    )]
    pub fn with_compliance(mut self, compliance: Scalar) -> Self {
        self.angle_constraint.compliance = compliance;
        self.axis_compliance = compliance;
        self.limit_compliance = compliance;
        self
    }

    /// Sets the compliance of the axis alignment constraint (inverse of stiffness, m / N).
    #[inline]
    pub fn with_axis_compliance(mut self, compliance: Scalar) -> Self {
        self.axis_compliance = compliance;
        self
    }

    /// Sets the compliance of the distance limit (inverse of stiffness, m / N).
    #[inline]
    pub fn with_limit_compliance(mut self, compliance: Scalar) -> Self {
        self.limit_compliance = compliance;
        self
    }

    /// Sets the compliance of the angular constraint (inverse of stiffness, N * m / rad).
    #[inline]
    pub fn with_angle_compliance(mut self, compliance: Scalar) -> Self {
        self.angle_constraint.compliance = compliance;
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
    pub fn position_lagrange(&self) -> Scalar {
        self.position_lagrange
    }

    /// Returns the Lagrange multiplier used for the angular correction.
    #[inline]
    pub fn angle_lagrange(&self) -> Scalar {
        self.angle_constraint.lagrange()
    }

    /// Returns the force exerted by the joint.
    #[inline]
    pub fn force(&self) -> Vector {
        self.force
    }

    /// Constrains the relative positions of the bodies, only allowing translation along one free axis.
    ///
    /// Returns the force exerted by this constraint.
    fn constrain_positions(
        &mut self,
        body1: &mut SolverBody,
        body2: &mut SolverBody,
        inertia1: &SolverBodyInertia,
        inertia2: &SolverBodyInertia,
        dt: Scalar,
    ) -> Vector {
        // Compute the effective inverse masses and angular inertias of the bodies.
        let inv_mass1 = inertia1.effective_inv_mass();
        let inv_mass2 = inertia2.effective_inv_mass();
        let inv_angular_inertia1 = inertia1.effective_inv_angular_inertia();
        let inv_angular_inertia2 = inertia2.effective_inv_angular_inertia();

        let world_r1 = body1.delta_rotation * self.pre_step.world_r1;
        let world_r2 = body2.delta_rotation * self.pre_step.world_r2;

        let mut delta_x = Vector::ZERO;

        let axis1 = body1.delta_rotation * self.pre_step.free_axis1;
        if let Some(limits) = self.free_axis_limits {
            let separation = (body2.delta_position - body1.delta_position)
                + (world_r2 - world_r1)
                + self.pre_step.center_difference;
            delta_x += limits.compute_correction_along_axis(separation, axis1);
        }

        let zero_distance_limit = DistanceLimit::ZERO;

        #[cfg(feature = "2d")]
        {
            let axis2 = Vector::new(axis1.y, -axis1.x);

            let separation = (body2.delta_position - body1.delta_position)
                + (world_r2 - world_r1)
                + self.pre_step.center_difference;
            delta_x += zero_distance_limit.compute_correction_along_axis(separation, axis2);
        }
        #[cfg(feature = "3d")]
        {
            let axis2 = axis1.any_orthogonal_vector();
            let axis3 = axis1.cross(axis2);

            let separation = (body2.delta_position - body1.delta_position)
                + (world_r2 - world_r1)
                + self.pre_step.center_difference;
            delta_x += zero_distance_limit.compute_correction_along_axis(separation, axis2);

            let separation = (body2.delta_position - body1.delta_position)
                + (world_r2 - world_r1)
                + self.pre_step.center_difference;
            delta_x += zero_distance_limit.compute_correction_along_axis(separation, axis3);
        }

        let magnitude = delta_x.length();

        if magnitude <= Scalar::EPSILON {
            return Vector::ZERO;
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
        let delta_lagrange = self.compute_lagrange_update(
            self.position_lagrange,
            magnitude,
            &[w1, w2],
            self.axis_compliance,
            dt,
        );
        self.position_lagrange += delta_lagrange;

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

        // Return constraint force
        self.compute_force(self.position_lagrange, dir, dt)
    }

    /// Sets the joint's free axis. Relative translations are allowed along this free axis.
    pub fn with_free_axis(self, axis: Vector) -> Self {
        Self {
            free_axis: axis,
            ..self
        }
    }

    /// Sets the translational limits along the joint's free axis.
    pub fn with_limits(self, min: Scalar, max: Scalar) -> Self {
        Self {
            free_axis_limits: Some(DistanceLimit::new(min, max)),
            ..self
        }
    }
}

impl PositionConstraint for PrismaticJoint {}

impl AngularConstraint for PrismaticJoint {}

impl MapEntities for PrismaticJoint {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.entity1 = entity_mapper.get_mapped(self.entity1);
        self.entity2 = entity_mapper.get_mapped(self.entity2);
    }
}
