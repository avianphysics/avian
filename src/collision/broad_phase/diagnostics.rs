use bevy::{
    diagnostic::DiagnosticPath,
    prelude::{ReflectResource, Resource},
    reflect::Reflect,
};
use core::time::Duration;

use crate::diagnostics::{PhysicsDiagnostics, impl_diagnostic_paths};

/// Diagnostics for collision detection.
#[derive(Resource, Debug, Default, Reflect)]
#[reflect(Resource, Debug)]
pub struct BroadPhaseDiagnostics {
    /// Time spent finding potential collision pairs in the broad phase.
    pub find_pairs: Duration,
    /// Time spent optimizing acceleration structures for the broad phase.
    pub optimize: Duration,
    /// Time spent updating AABBs and BVH nodes.
    pub update: Duration,
}

impl PhysicsDiagnostics for BroadPhaseDiagnostics {
    fn timer_paths(&self) -> Vec<(&'static DiagnosticPath, Duration)> {
        vec![
            (Self::FIND_PAIRS, self.find_pairs),
            (Self::OPTIMIZE, self.optimize),
            (Self::UPDATE, self.update),
        ]
    }
}

impl_diagnostic_paths! {
    impl BroadPhaseDiagnostics {
        FIND_PAIRS: "avian/broad_phase/find_pairs",
        OPTIMIZE: "avian/broad_phase/optimize",
        UPDATE: "avian/broad_phase/update",
    }
}
