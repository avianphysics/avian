//! Size metrics associated with a [`RigidBody`] and its colliders.
//!
//! See [`BodySizeMetrics`] for more information.
//!
//! [`RigidBody`]: crate::dynamics::rigid_body::RigidBody

use core::marker::PhantomData;

use bevy::{ecs::system::StaticSystemParam, prelude::*};

use crate::{
    collision::collider::{
        AnyCollider, ColliderContext, collider_hierarchy::RigidBodyColliders,
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

fn update_body_size_metrics<C: AnyCollider>(
    mut bodies: Query<(
        &mut BodySizeMetrics,
        &RigidBodyColliders,
        Ref<ComputedCenterOfMass>,
    )>,
    colliders: Query<(Entity, &C, &ColliderTransform)>,
    changed_colliders: Query<(), Or<(Changed<C>, Changed<ColliderTransform>)>>,
    context: StaticSystemParam<C::Context>,
) {
    let context = context.into_inner();

    for (mut size_metrics, rb_colliders, com) in bodies.iter_mut() {
        if !com.is_changed()
            && !rb_colliders
                .iter()
                .any(|collider_entity| changed_colliders.contains(collider_entity))
        {
            // Neither the center of mass nor any of the colliders have changed.
            continue;
        }

        let mut ccd_thickness: f32 = f32::INFINITY;
        let mut sweep_radius: f32 = 0.0;

        // Find the minimum CCD thickness and maximum sweep radius.
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

        size_metrics.ccd_thickness = ccd_thickness;
        size_metrics.sweep_radius = sweep_radius;
    }
}
