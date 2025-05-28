use bevy::{
    diagnostic::DiagnosticPath,
    prelude::{ReflectResource, Resource},
    reflect::Reflect,
};
use core::time::Duration;

use crate::diagnostics::{impl_diagnostic_paths, PhysicsDiagnostics};

/// Diagnostics for collision detection.
#[derive(Resource, Debug, Default, Reflect)]
#[reflect(Resource, Debug)]
pub struct CollisionDiagnostics {
    /// Time spent finding potential collision pairs in the broad phase.
    pub broad_phase: Duration,
    /// Time spent updating contacts in the narrow phase.
    pub narrow_phase: Duration,
    /// The number of contacts.
    pub contact_count: u32,
}

impl PhysicsDiagnostics for CollisionDiagnostics {
    fn timer_paths(&self) -> Vec<(&'static DiagnosticPath, Duration)> {
        vec![
            (Self::BROAD_PHASE, self.broad_phase),
            (Self::NARROW_PHASE, self.narrow_phase),
        ]
    }

    fn counter_paths(&self) -> Vec<(&'static DiagnosticPath, u32)> {
        vec![(Self::CONTACT_COUNT, self.contact_count)]
    }
}

impl_diagnostic_paths! {
    impl CollisionDiagnostics {
        BROAD_PHASE: "avian/collision/broad_phase",
        NARROW_PHASE: "avian/collision/update_contacts",
        CONTACT_COUNT: "avian/collision/contact_count",
    }
}
