use core::time::Duration;

use bevy::{
    diagnostic::DiagnosticPath,
    prelude::{ReflectResource, Resource},
    reflect::Reflect,
};

use crate::diagnostics::{impl_diagnostic_paths, PhysicsDiagnostics};

/// Diagnostics for spatial queries.
#[derive(Resource, Debug, Default, Reflect)]
#[reflect(Resource, Debug)]
pub struct SpatialQueryDiagnostics {
    /// Time spent updating the [`SpatialQueryPipeline`](super::SpatialQueryPipeline).
    pub update_pipeline: Duration,
    /// Time spent updating [`RayCaster`](super::RayCaster) hits.
    pub update_ray_casters: Duration,
    /// Time spent updating [`ShapeCaster`](super::ShapeCaster) hits.
    pub update_shape_casters: Duration,
}

impl PhysicsDiagnostics for SpatialQueryDiagnostics {
    fn timer_paths(&self) -> Vec<(&'static DiagnosticPath, Duration)> {
        vec![
            (Self::UPDATE_PIPELINE, self.update_pipeline),
            (Self::UPDATE_RAY_CASTERS, self.update_ray_casters),
            (Self::UPDATE_SHAPE_CASTERS, self.update_shape_casters),
        ]
    }
}

impl_diagnostic_paths! {
    impl SpatialQueryDiagnostics {
        UPDATE_PIPELINE: "avian/spatial_query/update_pipeline",
        UPDATE_RAY_CASTERS: "avian/spatial_query/update_ray_casters",
        UPDATE_SHAPE_CASTERS: "avian/spatial_query/update_shape_casters",
    }
}
