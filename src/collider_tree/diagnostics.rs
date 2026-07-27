use bevy_diagnostic::DiagnosticPath;
use bevy_ecs::prelude::{ReflectResource, Resource};
use bevy_reflect::Reflect;
use core::time::Duration;

use crate::diagnostics::{PhysicsDiagnostics, impl_diagnostic_paths};

/// Diagnostics for [collider trees](crate::collider_tree).
#[derive(Resource, Debug, Default, Reflect)]
#[reflect(Resource, Debug)]
pub struct ColliderTreeDiagnostics {
    /// Time spent optimizing [collider trees](crate::collider_tree).
    pub optimize: Duration,
    /// Time spent updating AABBs and BVH nodes.
    pub update: Duration,
}

impl PhysicsDiagnostics for ColliderTreeDiagnostics {
    fn timer_paths(&self) -> Vec<(&'static DiagnosticPath, Duration)> {
        vec![(Self::OPTIMIZE, self.optimize), (Self::UPDATE, self.update)]
    }
}

impl_diagnostic_paths! {
    impl ColliderTreeDiagnostics {
        OPTIMIZE: "avian/collider_tree/optimize",
        UPDATE: "avian/collider_tree/update",
    }
}
