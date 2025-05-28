//! [`FixedJoint`] component.

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

/// A fixed joint prevents any relative movement of the attached bodies.
///
/// You should generally prefer using a single body instead of multiple bodies fixed together,
/// but fixed joints can be useful for things like rigid structures where a force can dynamically break the joints connecting individual bodies.
#[derive(Component, Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, MapEntities, PartialEq)]
pub struct FixedJoint {
    /// First entity constrained by the joint.
    pub entity1: Entity,
    /// Second entity constrained by the joint.
    pub entity2: Entity,
    /// The point-to-point constraint that prevents relative translation of the attached bodies.
    pub point_constraint: PointConstraint,
    /// The angular constraint that prevents relative rotation of the attached bodies.
    pub angle_constraint: FixedAngleConstraint,
    /// Linear damping applied by the joint.
    pub damping_linear: Scalar,
    /// Angular damping applied by the joint.
    pub damping_angular: Scalar,
    /// The relative dominance of the bodies.
    ///
    /// If the relative dominance is positive, the first body is dominant
    /// and is considered to have infinite mass.
    pub relative_dominance: i16,
}

impl EntityConstraint<2> for FixedJoint {
    fn entities(&self) -> [Entity; 2] {
        [self.entity1, self.entity2]
    }
}

impl XpbdConstraint<2> for FixedJoint {
    fn clear_lagrange_multipliers(&mut self) {
        self.point_constraint.clear_lagrange_multipliers();
        self.angle_constraint.clear_lagrange_multipliers();
    }

    fn prepare(&mut self, bodies: [&RigidBodyQueryReadOnlyItem; 2], dt: Scalar) {
        // Prepare the point-to-point constraint.
        self.point_constraint.prepare(bodies, dt);

        // Prepare the angular constraint.
        self.angle_constraint.prepare(bodies, dt);

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

        // Solve the angular constraint.
        self.angle_constraint.solve([body1, body2], inertias, dt);

        // Solve the point-to-point constraint.
        self.point_constraint.solve([body1, body2], inertias, dt);
    }
}

impl Joint for FixedJoint {
    fn new(entity1: Entity, entity2: Entity) -> Self {
        Self {
            entity1,
            entity2,
            point_constraint: PointConstraint::default(),
            angle_constraint: FixedAngleConstraint::default(),
            damping_linear: 1.0,
            damping_angular: 1.0,
            relative_dominance: 0,
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

impl FixedJoint {
    /// Sets the joint's compliance (inverse of stiffness).
    #[inline]
    #[deprecated(
        since = "0.4.0",
        note = "Use `with_point_compliance` and `with_angle_compliance` instead."
    )]
    pub fn with_compliance(mut self, compliance: Scalar) -> Self {
        self.point_constraint.compliance = compliance;
        self.angle_constraint.compliance = compliance;
        self
    }

    /// Sets the compliance of the point-to-point compliance (inverse of stiffness, m / N).
    #[inline]
    pub fn with_point_compliance(mut self, compliance: Scalar) -> Self {
        self.point_constraint.compliance = compliance;
        self
    }

    /// Sets the compliance of the angular constraint (inverse of stiffness, (N * m / rad).
    #[inline]
    pub fn with_angle_compliance(mut self, compliance: Scalar) -> Self {
        self.angle_constraint.compliance = compliance;
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

    /// Returns the Lagrange multiplier used for the angular correction.
    #[inline]
    pub fn angle_lagrange(&self) -> Scalar {
        self.angle_constraint.lagrange()
    }

    /// Returns the force exerted by the joint.
    #[inline]
    pub fn force(&self) -> Vector {
        self.point_constraint.force()
    }
}

impl PositionConstraint for FixedJoint {}

impl AngularConstraint for FixedJoint {}

impl MapEntities for FixedJoint {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.entity1 = entity_mapper.get_mapped(self.entity1);
        self.entity2 = entity_mapper.get_mapped(self.entity2);
    }
}
