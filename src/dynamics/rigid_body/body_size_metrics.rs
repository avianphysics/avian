//! Size metrics associated with a [`RigidBody`] and its colliders.
//!
//! See [`BodySizeMetrics`] for more information.
//!
//! [`RigidBody`]: crate::dynamics::rigid_body::RigidBody

use core::marker::PhantomData;

use bevy::{ecs::system::StaticSystemParam, prelude::*};

use crate::{
    collision::collider::{
        AnyCollider, ColliderContext,
        collider_hierarchy::{ColliderOf, RigidBodyColliders},
        collider_transform::ColliderTransform,
    },
    dynamics::rigid_body::mass_properties::components::ComputedCenterOfMass,
    math::ToRealPrecision,
    schedule::{PhysicsSchedule, PhysicsStepSystems},
};

/// Size metrics associated with a [`RigidBody`] and its colliders.
///
/// These can be used for various purposes, such as determining thresholds
/// for [Continuous Collision Detection (CCD)][CCD], [sleeping], and contact recycling.
///
/// The values are automatically computed and updated by the [`BodySizeMetricsPlugin`].
///
/// [`RigidBody`]: crate::dynamics::rigid_body::RigidBody
/// [CCD]: crate::dynamics::ccd
/// [sleeping]: crate::dynamics::sleeping
#[derive(Component, Clone, Copy, Debug, PartialEq, Reflect)]
pub struct BodySizeMetrics {
    /// A conservative minimum thickness used by [Continuous Collision Detection (CCD)][CCD]
    /// to determine how far the body can move in a single timestep before it might start to
    /// tunnel through geometry.
    ///
    /// This is the minimum [`ccd_thickness`] of the colliders attached to the body,
    /// and typically corresponds to the minimum distance from the centroid
    /// of any given shape to its surface.
    ///
    /// [CCD]: crate::dynamics::ccd
    /// [`ccd_thickness`]: crate::collision::collider::AnyCollider::ccd_thickness_with_context
    pub ccd_thickness: f32,

    /// The maximum distance from the center of mass of the body to the surface
    /// of any of its colliders.
    ///
    /// Typically corresponds to the radius of the sphere formed by sweeping
    /// the body about its center of mass.
    ///
    /// This can be useful for determining the maximum velocity that a point
    /// on the body can have, which can be used for sleeping and contact recycling
    /// thresholds, for example.
    pub sweep_radius: f32,
}

impl Default for BodySizeMetrics {
    fn default() -> Self {
        Self {
            ccd_thickness: f32::INFINITY,
            sweep_radius: 0.0,
        }
    }
}

/// A plugin for computing and updating [`BodySizeMetrics`] for rigid bodies.
#[derive(Default)]
pub struct BodySizeMetricsPlugin<C: AnyCollider> {
    _phantom: PhantomData<C>,
}

impl<C: AnyCollider> Plugin for BodySizeMetricsPlugin<C> {
    fn build(&self, app: &mut App) {
        // Update body size metrics before they are consumed by the solver and continuous collision
        // detection. Allowing ambiguities lets multiple collision backends coexist.
        app.add_systems(
            PhysicsSchedule,
            update_body_size_metrics::<C>
                .before(PhysicsStepSystems::Solver)
                .ambiguous_with_all(),
        );
    }
}

type BodySizeMetricsData = (
    &'static mut BodySizeMetrics,
    &'static RigidBodyColliders,
    &'static ComputedCenterOfMass,
);

fn update_body_size_metrics<C: AnyCollider>(
    // Body size metrics only need updating when the center of mass
    // or one of the colliders changes.
    mut bodies: ParamSet<(
        Query<BodySizeMetricsData, Changed<ComputedCenterOfMass>>,
        Query<BodySizeMetricsData>,
    )>,
    colliders: Query<(Entity, &C, &ColliderTransform)>,
    changed_colliders: Query<&ColliderOf, Or<(Changed<C>, Changed<ColliderTransform>)>>,
    context: StaticSystemParam<C::Context>,
) {
    let context = context.into_inner();

    // Computes the minimum CCD thickness and maximum sweep radius over a body's colliders.
    let compute = |rb_colliders: &RigidBodyColliders, com: &ComputedCenterOfMass| {
        let mut ccd_thickness: f32 = f32::INFINITY;
        let mut sweep_radius: f32 = 0.0;

        for (entity, collider, collider_transform) in colliders.iter_many(rb_colliders) {
            // Compute the CCD thickness
            let ctx = ColliderContext::new(entity, &context);
            let thickness = collider.ccd_thickness_with_context(ctx);
            ccd_thickness = ccd_thickness.min(thickness);

            // Compute the sweep radius
            let ctx = ColliderContext::new(entity, &context);
            let point = com.0 - collider_transform.translation;
            let distance_to_com = collider.max_distance_to_point_with_context(point.real(), ctx);
            sweep_radius = sweep_radius.max(distance_to_com);
        }

        BodySizeMetrics {
            ccd_thickness,
            sweep_radius,
        }
    };

    // Bodies whose center of mass changed.
    for (mut size_metrics, rb_colliders, com) in &mut bodies.p0() {
        *size_metrics = compute(rb_colliders, com);
    }

    // Bodies with a collider that changed. Note that bodies may be visited
    // multiple times, but the end result will be the same.
    if changed_colliders.is_empty() {
        return;
    }
    let mut all_bodies = bodies.p1();
    for collider_of in &changed_colliders {
        if let Ok((mut size_metrics, rb_colliders, com)) = all_bodies.get_mut(collider_of.body) {
            *size_metrics = compute(rb_colliders, com);
        }
    }
}
