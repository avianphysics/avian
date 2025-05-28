//! Constraints and other types used for solving contacts.

mod normal_part;
mod tangent_part;

pub use normal_part::ContactNormalPart;
pub use tangent_part::ContactTangentPart;

use crate::prelude::*;
use bevy::{
    ecs::entity::{Entity, EntityMapper, MapEntities},
    reflect::Reflect,
    utils::default,
};

use super::solver_body::{SolverBody, SolverBodyInertia};

// TODO: One-body constraint version
/// Data and logic for solving a single contact point for a [`ContactConstraint`].
#[derive(Clone, Debug, PartialEq, Reflect)]
pub struct ContactConstraintPoint {
    /// The normal part of the contact constraint.
    pub normal_part: ContactNormalPart,

    /// The tangential friction part of the contact constraint.
    ///
    /// `None` if the coefficient of friction is zero.
    pub tangent_part: Option<ContactTangentPart>,

    // TODO: This could probably just be a boolean?
    /// The largest incremental contact impulse magnitude along the contact normal during this frame.
    ///
    /// This is used for determining whether restitution should be applied.
    pub max_normal_impulse: Scalar,

    /// The world-space contact point relative to the center of mass of the first body.
    pub anchor1: Vector,

    /// The world-space contact point relative to the center of mass of the second body.
    pub anchor2: Vector,

    /// The relative velocity of the bodies along the normal at the contact point.
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
pub struct ContactConstraint {
    /// The first rigid body entity in the contact.
    pub body1: Entity,
    /// The second rigid body entity in the contact.
    pub body2: Entity,
    // TODO: These aren't needed if we get rid of the lookup in `store_contact_impulses`.
    /// The first collider entity in the contact.
    pub collider1: Entity,
    /// The second collider entity in the contact.
    pub collider2: Entity,
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
    /// The contact points in the manifold. Each point shares the same `normal`.
    pub points: Vec<ContactConstraintPoint>,
    /// The index of the [`ContactPair`] in the [`ContactGraph`].
    ///
    /// This is primarily used for ordering contact constraints deterministically
    /// when parallelism is enabled. The index may be invalidated by contact removal.
    // TODO: We should figure out a better way to handle deterministic constraint generation.
    #[cfg(feature = "parallel")]
    pub pair_index: usize,
    /// The index of the [`ContactManifold`] in the [`ContactPair`] stored for the two bodies.
    pub manifold_index: usize,
}

impl ContactConstraint {
    /// Warm starts the contact constraint by applying the impulses from the previous frame or substep.
    pub fn warm_start(
        &self,
        body1: &mut SolverBody,
        body2: &mut SolverBody,
        inertia1: &SolverBodyInertia,
        inertia2: &SolverBodyInertia,
        normal: Vector,
        tangent_directions: [Vector; DIM - 1],
        warm_start_coefficient: Scalar,
    ) {
        let inv_mass1 = inertia1.effective_inv_mass();
        let inv_mass2 = inertia2.effective_inv_mass();
        let inv_angular_inertia1 = inertia1.effective_inv_angular_inertia();
        let inv_angular_inertia2 = inertia2.effective_inv_angular_inertia();

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
                * (point.normal_part.impulse * normal + tangent_impulse * tangent_directions[0]);
            #[cfg(feature = "3d")]
            let p = warm_start_coefficient
                * (point.normal_part.impulse * normal
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

            // Store the maximum impulse for restitution.
            point.max_normal_impulse = impulse_magnitude.max(point.max_normal_impulse);

            if impulse_magnitude == 0.0 {
                continue;
            }

            let impulse = impulse_magnitude * self.normal;

            // Apply the impulse.
            body1.linear_velocity -= impulse * inv_mass1;
            body1.angular_velocity -= inv_angular_inertia1 * cross(r1, impulse);

            body2.linear_velocity += impulse * inv_mass2;
            body2.angular_velocity += inv_angular_inertia2 * cross(r2, impulse);
        }

        let tangent_directions =
            self.tangent_directions(body1.linear_velocity, body2.linear_velocity);

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
            if point.normal_speed > -threshold || point.max_normal_impulse == 0.0 {
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
            point.max_normal_impulse += impulse;

            // Apply the impulse.
            let impulse = impulse * self.normal;

            body1.linear_velocity -= impulse * inv_mass1;
            body1.angular_velocity -= inv_angular_inertia1 * cross(r1, impulse);

            body2.linear_velocity += impulse * inv_mass2;
            body2.angular_velocity += inv_angular_inertia2 * cross(r2, impulse);
        }
    }

    /// Computes `DIM - 1` tangent directions.
    #[allow(unused_variables)]
    pub fn tangent_directions(&self, velocity1: Vector, velocity2: Vector) -> [Vector; DIM - 1] {
        #[cfg(feature = "2d")]
        {
            [Vector::new(self.normal.y, -self.normal.x)]
        }
        #[cfg(feature = "3d")]
        {
            let force_direction = -self.normal;
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
}

impl MapEntities for ContactConstraint {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.collider1 = entity_mapper.get_mapped(self.collider1);
        self.collider2 = entity_mapper.get_mapped(self.collider2);
    }
}
