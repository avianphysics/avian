//! [`DistanceJoint`] component.

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

/// A distance joint keeps the attached bodies at a certain distance from each other while while allowing rotation around all axes.
///
/// Distance joints can be useful for things like springs, muscles, and mass-spring networks.
#[derive(Component, Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, MapEntities, PartialEq)]
pub struct DistanceJoint {
    /// First entity constrained by the joint.
    pub entity1: Entity,
    /// Second entity constrained by the joint.
    pub entity2: Entity,
    /// Attachment point on the first body.
    pub local_anchor1: Vector,
    /// Attachment point on the second body.
    pub local_anchor2: Vector,
    /// The distance the attached bodies will be kept relative to each other.
    pub rest_length: Scalar,
    /// The extents of the allowed relative translation between the attached bodies.
    pub length_limits: Option<DistanceLimit>,
    /// Linear damping applied by the joint.
    pub damping_linear: Scalar,
    /// Angular damping applied by the joint.
    pub damping_angular: Scalar,
    /// The relative dominance of the bodies.
    ///
    /// If the relative dominance is positive, the first body is dominant
    /// and is considered to have infinite mass.
    pub relative_dominance: i16,
    /// The joint's compliance, the inverse of stiffness (m / N).
    pub compliance: Scalar,
    /// Lagrange multiplier for the positional correction.
    lagrange: Scalar,
    /// The force exerted by the joint.
    force: Vector,
    // Pre-step data for the solver.
    pre_step: DistanceJointPreStepData,
}

/// Pre-step data for the [`DistanceJoint`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
struct DistanceJointPreStepData {
    world_r1: Vector,
    world_r2: Vector,
    center_difference: Vector,
}

impl EntityConstraint<2> for DistanceJoint {
    fn entities(&self) -> [Entity; 2] {
        [self.entity1, self.entity2]
    }
}

impl XpbdConstraint<2> for DistanceJoint {
    fn clear_lagrange_multipliers(&mut self) {
        self.lagrange = 0.0;
    }

    fn prepare(&mut self, bodies: [&RigidBodyQueryReadOnlyItem; 2], _dt: Scalar) {
        let [body1, body2] = bodies;

        // Prepare the base rotation difference.
        self.pre_step.world_r1 = body1.rotation * (self.local_anchor1 - body1.center_of_mass.0);
        self.pre_step.world_r2 = body2.rotation * (self.local_anchor2 - body2.center_of_mass.0);
        self.pre_step.center_difference = body2.position.0 - body1.position.0;

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

        let inv_mass1 = inertia1.effective_inv_mass();
        let inv_mass2 = inertia2.effective_inv_mass();
        let inv_angular_inertia1 = inertia1.effective_inv_angular_inertia();
        let inv_angular_inertia2 = inertia2.effective_inv_angular_inertia();

        let world_r1 = body1.delta_rotation * self.pre_step.world_r1;
        let world_r2 = body2.delta_rotation * self.pre_step.world_r2;

        let separation = (body2.delta_position - body1.delta_position)
            + (world_r2 - world_r1)
            + self.pre_step.center_difference;

        // If min and max limits aren't specified, use rest length
        // TODO: Remove rest length, just use min/max limits.
        let limits = self
            .length_limits
            .unwrap_or(DistanceLimit::new(self.rest_length, self.rest_length));

        // Compute the direction and magnitude of the positional correction required
        // to keep the bodies within a certain distance from each other.
        let (dir, distance) = limits.compute_correction(separation);

        if distance <= Scalar::EPSILON {
            // No separation, no need to apply a correction.
            self.force = Vector::ZERO;
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
        let delta_lagrange =
            self.compute_lagrange_update(self.lagrange, distance, &w, self.compliance, dt);
        self.lagrange += delta_lagrange;

        // Apply positional correction (method from PositionConstraint)
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
        self.force = self.compute_force(self.lagrange, dir, dt);
    }
}

impl Joint for DistanceJoint {
    fn new(entity1: Entity, entity2: Entity) -> Self {
        Self {
            entity1,
            entity2,
            local_anchor1: Vector::ZERO,
            local_anchor2: Vector::ZERO,
            rest_length: 0.0,
            length_limits: None,
            damping_linear: 0.0,
            damping_angular: 0.0,
            relative_dominance: 0,
            lagrange: 0.0,
            compliance: 0.0,
            force: Vector::ZERO,
            pre_step: DistanceJointPreStepData::default(),
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

impl DistanceJoint {
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
    pub fn lagrange(&self) -> Scalar {
        self.lagrange
    }

    /// Returns the force exerted by the joint.
    #[inline]
    pub fn force(&self) -> Vector {
        self.force
    }

    /// Sets the minimum and maximum distances between the attached bodies.
    pub fn with_limits(self, min: Scalar, max: Scalar) -> Self {
        Self {
            length_limits: Some(DistanceLimit::new(min, max)),
            ..self
        }
    }

    /// Sets the joint's rest length, or distance the bodies will be kept at.
    pub fn with_rest_length(self, rest_length: Scalar) -> Self {
        Self {
            rest_length,
            ..self
        }
    }
}

impl PositionConstraint for DistanceJoint {}

impl AngularConstraint for DistanceJoint {}

impl MapEntities for DistanceJoint {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.entity1 = entity_mapper.get_mapped(self.entity1);
        self.entity2 = entity_mapper.get_mapped(self.entity2);
    }
}
