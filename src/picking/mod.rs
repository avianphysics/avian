//! A physics picking backend for [`bevy_picking`](bevy::picking).
//!
//! Add the [`PhysicsPickingPlugin`] to enable picking for [colliders](Collider).
//! By default, all colliders are pickable. Picking can be disabled for individual entities
//! by adding [`Pickable::IGNORE`].
//!
//! To make physics picking entirely opt-in, set [`PhysicsPickingSettings::require_markers`]
//! to `true` and add a [`PhysicsPickable`] component to the desired camera and target entities.
//!
//! Cameras can further filter which entities are pickable with the [`PhysicsPickingFilter`] component.
#![cfg_attr(
    feature = "3d",
    doc = "

Note that in 3D, only the closest intersection will be reported."
)]

use crate::{
    diagnostics::{PhysicsDiagnostics, impl_diagnostic_paths},
    prelude::*,
};
use bevy::{
    ecs::entity::hash_set::EntityHashSet,
    picking::{
        PickingSystems,
        backend::{HitData, PointerHits, ray::RayMap},
    },
    prelude::*,
};

use core::time::Duration;

use bevy::{
    diagnostic::DiagnosticPath,
    prelude::{ReflectResource, Resource},
    reflect::Reflect,
};

/// Diagnostics for [physics picking](crate::picking).
#[derive(Resource, Debug, Default, Reflect)]
#[reflect(Resource, Debug)]
pub struct PhysicsPickingDiagnostics {
    /// Time spent updating hits for the physics picking backend.
    pub update_hits: Duration,
}

impl PhysicsDiagnostics for PhysicsPickingDiagnostics {
    fn timer_paths(&self) -> Vec<(&'static DiagnosticPath, Duration)> {
        vec![(Self::UPDATE_HITS, Duration::default())]
    }
}

impl_diagnostic_paths! {
    impl PhysicsPickingDiagnostics {
        UPDATE_HITS: "avian/physics/update_hits",
    }
}

/// Adds the [physics picking](crate::picking) backend to your app, enabling picking for [colliders](Collider).
#[derive(Clone, Default)]
pub struct PhysicsPickingPlugin;

impl Plugin for PhysicsPickingPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<PhysicsPickingSettings>()
            .add_systems(PreUpdate, update_hits.in_set(PickingSystems::Backend));
    }

    fn finish(&self, app: &mut App) {
        // Register diagnostics for physics picking.
        app.register_physics_diagnostics::<PhysicsPickingDiagnostics>();
    }
}

/// Runtime settings for the [`PhysicsPickingPlugin`].
#[derive(Resource, Default, Reflect)]
#[reflect(Resource, Default)]
pub struct PhysicsPickingSettings {
    /// When set to `true`, picking will only happen between cameras and entities marked with
    /// [`PhysicsPickable`]. `false` by default.
    ///
    /// This setting is provided to give you fine-grained control over which cameras and entities
    /// should be used by the physics picking backend at runtime.
    pub require_markers: bool,
}

/// An optional component that marks cameras and target entities that should be used in the [`PhysicsPickingPlugin`].
/// Only needed if [`PhysicsPickingSettings::require_markers`] is set to true.
#[derive(Debug, Clone, Default, Component, Reflect)]
#[reflect(Component, Default)]
pub struct PhysicsPickable;

/// An optional component with a [`SpatialQueryFilter`] to determine
/// which physics entities a camera considers for [physics picking](crate::picking).
///
/// If not present on a camera, picking will consider all colliders.
#[derive(Component, Clone, Debug, Default, Reflect)]
#[reflect(Component, Debug, Default)]
pub struct PhysicsPickingFilter(pub SpatialQueryFilter);

impl PhysicsPickingFilter {
    /// Creates a new [`PhysicsPickingFilter`] with the given [`LayerMask`] determining
    /// which [collision layers] will be pickable.
    ///
    /// [collision layers]: CollisionLayers
    /// [spatial query]: crate::spatial_query
    pub fn from_mask(mask: impl Into<LayerMask>) -> Self {
        Self(SpatialQueryFilter::from_mask(mask))
    }

    /// Creates a new [`PhysicsPickingFilter`] with the given entities excluded from physics picking.
    ///
    /// [spatial query]: crate::spatial_query
    pub fn from_excluded_entities(entities: impl IntoIterator<Item = Entity>) -> Self {
        Self(SpatialQueryFilter::from_excluded_entities(entities))
    }

    /// Sets the [`LayerMask`] of the filter configuration. Only colliders with the corresponding
    /// [collision layer memberships] will be pickable.
    ///
    /// [collision layer memberships]: CollisionLayers
    /// [spatial query]: crate::spatial_query
    pub fn with_mask(mut self, masks: impl Into<LayerMask>) -> Self {
        self.0.mask = masks.into();
        self
    }

    /// Excludes the given entities from physics picking.
    pub fn with_excluded_entities(mut self, entities: impl IntoIterator<Item = Entity>) -> Self {
        self.0.excluded_entities = EntityHashSet::from_iter(entities);
        self
    }
}

// Store a const reference to the default filter to avoid unnecessary allocations.
const DEFAULT_FILTER_REF: &PhysicsPickingFilter =
    &PhysicsPickingFilter(SpatialQueryFilter::DEFAULT);

/// Queries for collider intersections with pointers using [`PhysicsPickingSettings`] and sends [`PointerHits`] events.
pub fn update_hits(
    picking_cameras: Query<(
        &Camera,
        Option<&PhysicsPickingFilter>,
        Option<&PhysicsPickable>,
    )>,
    ray_map: Res<RayMap>,
    pickables: Query<&Pickable>,
    marked_targets: Query<&PhysicsPickable>,
    backend_settings: Res<PhysicsPickingSettings>,
    spatial_query: SpatialQuery,
    mut output_events: MessageWriter<PointerHits>,
    mut diagnostics: ResMut<PhysicsPickingDiagnostics>,
) {
    let start_time = crate::utils::Instant::now();

    for (&ray_id, &ray) in ray_map.map.iter() {
        let Ok((camera, picking_filter, cam_pickable)) = picking_cameras.get(ray_id.camera) else {
            continue;
        };

        if backend_settings.require_markers && cam_pickable.is_none() || !camera.is_active {
            continue;
        }

        let filter = picking_filter.unwrap_or(DEFAULT_FILTER_REF);

        #[cfg(feature = "2d")]
        {
            let mut hits: Vec<(Entity, HitData)> = vec![];

            spatial_query.point_intersections_callback(
                ray.origin.truncate().adjust_precision(),
                &filter.0,
                |entity| {
                    let marker_requirement =
                        !backend_settings.require_markers || marked_targets.get(entity).is_ok();

                    let is_pickable = pickables
                        .get(entity)
                        .map(|p| *p != Pickable::IGNORE)
                        .unwrap_or(true);

                    if marker_requirement && is_pickable {
                        hits.push((
                            entity,
                            HitData::new(ray_id.camera, 0.0, Some(ray.origin.f32()), None),
                        ));
                    }

                    true
                },
            );

            output_events.write(PointerHits::new(ray_id.pointer, hits, camera.order as f32));
        }
        #[cfg(feature = "3d")]
        {
            if let Some((entity, hit_data)) = spatial_query
                .cast_ray_predicate(
                    ray.origin.adjust_precision(),
                    ray.direction,
                    Scalar::MAX,
                    true,
                    &filter.0,
                    &|entity| {
                        let marker_requirement =
                            !backend_settings.require_markers || marked_targets.get(entity).is_ok();

                        let is_pickable = pickables
                            .get(entity)
                            .map(|p| *p != Pickable::IGNORE)
                            .unwrap_or(true);

                        marker_requirement && is_pickable
                    },
                )
                .map(|ray_hit_data| {
                    #[allow(clippy::unnecessary_cast)]
                    let distance = ray_hit_data.distance as f32;
                    let hit_data = HitData::new(
                        ray_id.camera,
                        distance,
                        Some(ray.get_point(distance)),
                        Some(ray_hit_data.normal.f32()),
                    );
                    (ray_hit_data.entity, hit_data)
                })
            {
                output_events.write(PointerHits::new(
                    ray_id.pointer,
                    vec![(entity, hit_data)],
                    camera.order as f32,
                ));
            }
        }
    }

    diagnostics.update_hits = start_time.elapsed();
}
