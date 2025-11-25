//! Constraints and other types used for solving contacts.

mod normal_part;
mod tangent_part;

pub use normal_part::ContactNormalPart;
pub use tangent_part::ContactTangentPart;

use core::cmp::Ordering;

use crate::{
    collision::contact_types::ContactId,
    dynamics::solver::{BodyQueryItem, ContactSoftnessCoefficients},
    prelude::*,
};
#[cfg(feature = "serialize")]
use bevy::reflect::{ReflectDeserialize, ReflectSerialize};
use bevy::{
    ecs::entity::{Entity, EntityMapper, MapEntities},
    reflect::Reflect,
    utils::default,
};

use super::solver_body::{SolverBody, SolverBodyInertia};

// TODO: One-body constraint version
/// Data and logic for solving a single contact point for a [`ContactConstraint`].
#[derive(Clone, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct ContactConstraintPoint {
    /// The normal part of the contact constraint.
    pub normal_part: ContactNormalPart,

    /// The tangential friction part of the contact constraint.
    ///
    /// `None` if the coefficient of friction is zero.
    pub tangent_part: Option<ContactTangentPart>,

    /// The world-space contact point relative to the center of mass of the first body.
    pub anchor1: Vector,

    /// The world-space contact point relative to the center of mass of the second body.
    pub anchor2: Vector,

    /// The pre-solve relative velocity of the bodies along the normal at the contact point.
    pub normal_speed: Scalar,

    /// The pre-solve separation distance between the bodies.
    ///
    /// A negative separation indicates penetration.
    pub initial_separation: Scalar,
}

/// A contact constraint used for resolving inter-penetration between two bodies.
///
/// Each constraint corresponds to a [`ContactManifold`] indicated by the `manifold_index`.
/// The contact points are stored in `points`, and they all share the same `normal`.
#[derive(Clone, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct ContactConstraint {
    /// The first rigid body entity in the contact.
    pub body1: Entity,
    /// The second rigid body entity in the contact.
    pub body2: Entity,
    /// The relative dominance of the bodies.
    ///
    /// If the relative dominance is positive, the first body is dominant
    /// and is considered to have infinite mass.
    pub relative_dominance: i16,
    /// The combined coefficient of dynamic [friction](Friction) of the bodies.
    pub friction: Scalar,
    /// The combined coefficient of [restitution](Restitution) of the bodies.
    pub restitution: Scalar,
    /// The desired relative linear speed of the bodies along the surface,
    /// expressed in world space as `tangent_speed2 - tangent_speed1`.
    ///
    /// Defaults to zero. If set to a non-zero value, this can be used to simulate effects
    /// such as conveyor belts.
    #[cfg(feature = "2d")]
    pub tangent_speed: Scalar,
    /// The desired relative linear velocity of the bodies along the surface,
    /// expressed in world space as `tangent_velocity2 - tangent_velocity1`.
    ///
    /// Defaults to zero. If set to a non-zero value, this can be used to simulate effects
    /// such as conveyor belts.
    #[cfg(feature = "3d")]
    pub tangent_velocity: Vector,
    /// The world-space contact normal shared by all points in the contact manifold.
    pub normal: Vector,
    /// The first world-space tangent direction shared by all points in the contact manifold.
    #[cfg(feature = "3d")]
    pub tangent1: Vector,
    /// The contact points in the manifold. Each point shares the same `normal`.
    // TODO: Use a `SmallVec`
    pub points: Vec<ContactConstraintPoint>,
    /// The stable identifier of the [`ContactEdge`] in the [`ContactGraph`].
    ///
    /// [`ContactEdge`]: crate::collision::contact_types::ContactEdge
    pub contact_id: ContactId,
    /// The index of the contact manifold in the [`ContactPair`].
    pub manifold_index: usize,
}

impl ContactConstraint {
    /// Generates a new [`ContactConstraint`] from the given bodies and contact manifold.
    pub(super) fn generate(
        body1_entity: Entity,
        body2_entity: Entity,
        body1: BodyQueryItem,
        body2: BodyQueryItem,
        contact_id: ContactId,
        manifold: &ContactManifold,
        manifold_index: usize,
        warm_start_enabled: bool,
        softness: &ContactSoftnessCoefficients,
    ) -> Self {
        // Get the solver body inertia if it exists, or use a dummy inertia for static bodies.
        let inertia1 = body1.inertia.unwrap_or(&SolverBodyInertia::DUMMY);
        let inertia2 = body2.inertia.unwrap_or(&SolverBodyInertia::DUMMY);

        // Compute the relative dominance of the bodies.
        let relative_dominance = inertia1.dominance() - inertia2.dominance();

        // Compute the inverse mass and angular inertia, taking into account the relative dominance.
        let (inv_mass1, i1, inv_mass2, i2) = match relative_dominance.cmp(&0) {
            Ordering::Equal => (
                inertia1.effective_inv_mass(),
                inertia1.effective_inv_angular_inertia(),
                inertia2.effective_inv_mass(),
                inertia2.effective_inv_angular_inertia(),
            ),
            Ordering::Greater => (
                Vector::ZERO,
                SymmetricTensor::ZERO,
                inertia2.effective_inv_mass(),
                inertia2.effective_inv_angular_inertia(),
            ),
            Ordering::Less => (
                inertia1.effective_inv_mass(),
                inertia1.effective_inv_angular_inertia(),
                Vector::ZERO,
                SymmetricTensor::ZERO,
            ),
        };

        let softness = if relative_dominance != 0 {
            softness.non_dynamic
        } else {
            softness.dynamic
        };

        let effective_inverse_mass_sum = inv_mass1 + inv_mass2;

        let tangents = compute_tangent_directions(
            manifold.normal,
            body1.linear_velocity.0,
            body2.linear_velocity.0,
        );

        let mut points = Vec::with_capacity(manifold.points.len());

        for point in manifold.points.iter() {
            // Use fixed world-space anchors.
            // This improves rolling behavior for shapes like balls and capsules.
            let anchor1 = point.anchor1;
            let anchor2 = point.anchor2;

            let point = ContactConstraintPoint {
                // TODO: Apply warm starting scale here instead of in `warm_start`?
                normal_part: ContactNormalPart::generate(
                    effective_inverse_mass_sum,
                    &i1,
                    &i2,
                    anchor1,
                    anchor2,
                    manifold.normal,
                    warm_start_enabled.then_some(point.warm_start_normal_impulse),
                    softness,
                ),
                // There should only be a friction part if the coefficient of friction is non-negative.
                tangent_part: (manifold.friction > 0.0).then_some(ContactTangentPart::generate(
                    effective_inverse_mass_sum,
                    &i1,
                    &i2,
                    anchor1,
                    anchor2,
                    tangents,
                    warm_start_enabled.then_some(point.warm_start_tangent_impulse),
                )),
                anchor1,
                anchor2,
                normal_speed: point.normal_speed,
                initial_separation: -point.penetration - (anchor2 - anchor1).dot(manifold.normal),
            };

            points.push(point);
        }

        ContactConstraint {
            body1: body1_entity,
            body2: body2_entity,
            relative_dominance,
            friction: manifold.friction,
            restitution: manifold.restitution,
            #[cfg(feature = "2d")]
            tangent_speed: manifold.tangent_speed,
            #[cfg(feature = "3d")]
            tangent_velocity: manifold.tangent_velocity,
            normal: manifold.normal,
            #[cfg(feature = "3d")]
            tangent1: tangents[0],
            points,
            contact_id,
            manifold_index,
        }
    }

    /// Warm starts the contact constraint by applying the impulses from the previous frame or substep.
    pub fn warm_start(
        &self,
        body1: &mut SolverBody,
        body2: &mut SolverBody,
        inertia1: &SolverBodyInertia,
        inertia2: &SolverBodyInertia,
        warm_start_coefficient: Scalar,
    ) {
        let inv_mass1 = inertia1.effective_inv_mass();
        let inv_mass2 = inertia2.effective_inv_mass();
        let inv_angular_inertia1 = inertia1.effective_inv_angular_inertia();
        let inv_angular_inertia2 = inertia2.effective_inv_angular_inertia();

        let tangent_directions = self.tangent_directions();

        for point in self.points.iter() {
            // Fixed anchors
            let r1 = point.anchor1;
            let r2 = point.anchor2;

            let tangent_impulse = point
                .tangent_part
                .as_ref()
                .map_or(default(), |part| part.impulse);

            #[cfg(feature = "2d")]
            let p = warm_start_coefficient
                * (point.normal_part.impulse * self.normal
                    + tangent_impulse * tangent_directions[0]);
            #[cfg(feature = "3d")]
            let p = warm_start_coefficient
                * (point.normal_part.impulse * self.normal
                    + tangent_impulse.x * tangent_directions[0]
                    + tangent_impulse.y * tangent_directions[1]);

            body1.linear_velocity -= p * inv_mass1;
            body1.angular_velocity -= inv_angular_inertia1 * cross(r1, p);

            body2.linear_velocity += p * inv_mass2;
            body2.angular_velocity += inv_angular_inertia2 * cross(r2, p);
        }
    }

    /// Solves the [`ContactConstraint`], applying an impulse to the given bodies.
    pub fn solve(
        &mut self,
        body1: &mut SolverBody,
        body2: &mut SolverBody,
        inertia1: &SolverBodyInertia,
        inertia2: &SolverBodyInertia,
        delta_secs: Scalar,
        use_bias: bool,
        max_overlap_solve_speed: Scalar,
    ) {
        let inv_mass1 = inertia1.effective_inv_mass();
        let inv_mass2 = inertia2.effective_inv_mass();
        let inv_angular_inertia1 = inertia1.effective_inv_angular_inertia();
        let inv_angular_inertia2 = inertia2.effective_inv_angular_inertia();

        let delta_translation = body2.delta_position - body1.delta_position;

        // Normal impulses
        for point in self.points.iter_mut() {
            let r1 = body1.delta_rotation * point.anchor1;
            let r2 = body2.delta_rotation * point.anchor2;

            // Compute current saparation.
            let delta_separation = delta_translation + (r2 - r1);
            let separation = delta_separation.dot(self.normal) + point.initial_separation;

            // Fixed anchors
            let r1 = point.anchor1;
            let r2 = point.anchor2;

            // Relative velocity at contact
            let relative_velocity = body2.velocity_at_point(r2) - body1.velocity_at_point(r1);

            // Compute the incremental impulse. The clamping and impulse accumulation is handled by the method.
            let impulse_magnitude = point.normal_part.solve_impulse(
                separation,
                relative_velocity,
                self.normal,
                use_bias,
                max_overlap_solve_speed,
                delta_secs,
            );

            let impulse = impulse_magnitude * self.normal;

            // Apply the impulse.
            body1.linear_velocity -= impulse * inv_mass1;
            body1.angular_velocity -= inv_angular_inertia1 * cross(r1, impulse);

            body2.linear_velocity += impulse * inv_mass2;
            body2.angular_velocity += inv_angular_inertia2 * cross(r2, impulse);
        }

        let tangent_directions = self.tangent_directions();

        // Friction
        for point in self.points.iter_mut() {
            let Some(ref mut friction_part) = point.tangent_part else {
                continue;
            };

            // Fixed anchors
            let r1 = point.anchor1;
            let r2 = point.anchor2;

            // Relative velocity at contact point
            let relative_velocity = body2.velocity_at_point(r2) - body1.velocity_at_point(r1);

            // Compute the incremental impulse. The clamping and impulse accumulation is handled by the method.
            let impulse = friction_part.solve_impulse(
                tangent_directions,
                relative_velocity,
                #[cfg(feature = "2d")]
                self.tangent_speed,
                #[cfg(feature = "3d")]
                self.tangent_velocity,
                self.friction,
                point.normal_part.impulse,
            );

            // Apply the impulse.
            body1.linear_velocity -= impulse * inv_mass1;
            body1.angular_velocity -= inv_angular_inertia1 * cross(r1, impulse);

            body2.linear_velocity += impulse * inv_mass2;
            body2.angular_velocity += inv_angular_inertia2 * cross(r2, impulse);
        }
    }

    /// Applies [restitution](`Restitution`) for the given bodies if the relative speed
    /// along the contact normal exceeds the given `threshold`.
    pub fn apply_restitution(
        &mut self,
        body1: &mut SolverBody,
        body2: &mut SolverBody,
        inertia1: &SolverBodyInertia,
        inertia2: &SolverBodyInertia,
        threshold: Scalar,
    ) {
        let inv_mass1 = inertia1.effective_inv_mass();
        let inv_mass2 = inertia2.effective_inv_mass();
        let inv_angular_inertia1 = inertia1.effective_inv_angular_inertia();
        let inv_angular_inertia2 = inertia2.effective_inv_angular_inertia();

        for point in self.points.iter_mut() {
            // Skip restitution for speeds below the threshold.
            // We also skip contacts that don't apply an impulse to account for speculative contacts.
            if point.normal_speed > -threshold || point.normal_part.total_impulse == 0.0 {
                continue;
            }

            // Fixed anchors
            let r1 = point.anchor1;
            let r2 = point.anchor2;

            // Relative velocity at contact point
            let relative_velocity = body2.velocity_at_point(r2) - body1.velocity_at_point(r1);
            let normal_speed = relative_velocity.dot(self.normal);

            // Compute the incremental normal impulse to account for restitution.
            let mut impulse = -point.normal_part.effective_mass
                * (normal_speed + self.restitution * point.normal_speed);

            // Clamp the accumulated impulse.
            let new_impulse = (point.normal_part.impulse + impulse).max(0.0);
            impulse = new_impulse - point.normal_part.impulse;
            point.normal_part.impulse = new_impulse;

            // Add the incremental impulse instead of the full impulse because this is not a substep.
            point.normal_part.total_impulse += impulse;

            // Apply the impulse.
            let impulse = impulse * self.normal;

            body1.linear_velocity -= impulse * inv_mass1;
            body1.angular_velocity -= inv_angular_inertia1 * cross(r1, impulse);

            body2.linear_velocity += impulse * inv_mass2;
            body2.angular_velocity += inv_angular_inertia2 * cross(r2, impulse);
        }
    }

    /// Returns the tangent directions for the contact constraint.
    #[inline(always)]
    pub fn tangent_directions(&self) -> [Vector; DIM - 1] {
        #[cfg(feature = "2d")]
        {
            [Vector::new(self.normal.y, -self.normal.x)]
        }
        #[cfg(feature = "3d")]
        {
            // Note: The order is flipped here so that we use `-normal`.
            [self.tangent1, self.tangent1.cross(self.normal)]
        }
    }
}

/// Computes `DIM - 1` tangent directions.
#[allow(unused_variables)]
#[inline(always)]
fn compute_tangent_directions(
    normal: Vector,
    velocity1: Vector,
    velocity2: Vector,
) -> [Vector; DIM - 1] {
    #[cfg(feature = "2d")]
    {
        [Vector::new(normal.y, -normal.x)]
    }
    #[cfg(feature = "3d")]
    {
        let force_direction = -normal;
        let relative_velocity = velocity1 - velocity2;
        let tangent_velocity =
            relative_velocity - force_direction * force_direction.dot(relative_velocity);

        let tangent = tangent_velocity
            .try_normalize()
            .unwrap_or(force_direction.any_orthonormal_vector());
        let bitangent = force_direction.cross(tangent);
        [tangent, bitangent]
    }
}

impl MapEntities for ContactConstraint {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.body1 = entity_mapper.get_mapped(self.body1);
        self.body2 = entity_mapper.get_mapped(self.body2);
    }
}
