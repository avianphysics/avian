//! Diagnostics support for tracking physics timers and counters. Useful for profiling and debugging.
//!
//! # Overview
//!
//! Each physics plugin such as [`NarrowPhasePlugin`] and [`SolverPlugin`] is responsible
//! for implementing its own diagnostics resource using the [`PhysicsDiagnostics`] trait
//! and registering it using [`AppDiagnosticsExt::register_physics_diagnostics`].
//!
//! If the `bevy_diagnostic` feature is enabled and the [`PhysicsDiagnosticsPlugin`] is added to the app,
//! these diagnostics will also be automatically written to the [`DiagnosticsStore`] resource.
//!
//! If the `diagnostic_ui` feature is enabled and the [`PhysicsDiagnosticsUiPlugin`] is added to the app,
//! a debug UI will also be available for displaying these diagnostics in real-time.
//!
//! [`NarrowPhasePlugin`]: crate::collision::narrow_phase::NarrowPhasePlugin
//! [`SolverPlugin`]: crate::dynamics::solver::SolverPlugin
//! [`DiagnosticsStore`]: bevy::diagnostic::DiagnosticsStore
//! [`PhysicsDiagnosticsUiPlugin`]: crate::diagnostics::ui::PhysicsDiagnosticsUiPlugin
//!
//! # Example
//!
//! ```no_run
#![cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
//! use bevy::prelude::*;
//!
//! fn main() {
//!     App::new()
//!         .add_plugins((
//!             DefaultPlugins,
//!             PhysicsPlugins::default(),
//!             // Add the `PhysicsDiagnosticsPlugin` to write physics diagnostics
//!             // to the `DiagnosticsStore` resource in `bevy_diagnostic`.
//!             // Requires the `bevy_diagnostic` feature.
//! #           #[cfg(feature = "bevy_diagnostic")]
//!             PhysicsDiagnosticsPlugin,
//!             // Add the `PhysicsDiagnosticsUiPlugin` to display physics diagnostics
//!             // in a debug UI. Requires the `diagnostic_ui` feature.
//! #           #[cfg(feature = "diagnostic_ui")]
//!             PhysicsDiagnosticsUiPlugin,
//!         ))
//!         // ...your other plugins, systems and resources
//!         .run();
//! }
//! ```
//!
//! # Supported Diagnostics
//!
//! The following diagnostics are available but not added by default:
//!
//! - [`PhysicsTotalDiagnostics`]: Total physics timers and counters.
//! - [`PhysicsEntityDiagnostics`]: Physics entity counters.
//!
//! Additionally, physics plugins may implement their own diagnostics that *are* enabled by default.
//! These include:
//!
//! - [`CollisionDiagnostics`]: Diagnostics for collision detection.
//! - [`SolverDiagnostics`]: Diagnostics for the physics solver.
//! - [`SpatialQueryDiagnostics`]: Diagnostics for spatial queries.
//! - [`PhysicsPickingDiagnostics`]: Diagnostics for physics picking.
//!
//! [`CollisionDiagnostics`]: crate::collision::CollisionDiagnostics
//! [`SolverDiagnostics`]: crate::dynamics::solver::SolverDiagnostics
//! [`SpatialQueryDiagnostics`]: crate::spatial_query::SpatialQueryDiagnostics
//! [`PhysicsPickingDiagnostics`]: crate::picking::PhysicsPickingDiagnostics

#[cfg(feature = "bevy_diagnostic")]
mod entity_counters;
mod path_macro;
#[cfg(feature = "bevy_diagnostic")]
mod total;

#[cfg(feature = "diagnostic_ui")]
pub mod ui;
#[cfg(feature = "bevy_diagnostic")]
pub use entity_counters::{PhysicsEntityDiagnostics, PhysicsEntityDiagnosticsPlugin};
pub(crate) use path_macro::impl_diagnostic_paths;
#[cfg(feature = "bevy_diagnostic")]
pub use total::{PhysicsTotalDiagnostics, PhysicsTotalDiagnosticsPlugin};

use crate::{PhysicsStepSystems, schedule::PhysicsSchedule};
use bevy::{
    diagnostic::DiagnosticPath,
    prelude::{App, IntoScheduleConfigs, ResMut, Resource, SystemSet},
};
#[cfg(feature = "bevy_diagnostic")]
use bevy::{
    diagnostic::{Diagnostic, Diagnostics, RegisterDiagnostic},
    prelude::{Plugin, Res},
};
use core::time::Duration;

/// A plugin that enables writing [physics diagnostics](crate::diagnostics)
/// to [`bevy::diagnostic::DiagnosticsStore`]. It is not enabled by default
/// and must be added manually.
///
/// To add a debug UI for physics diagnostics, enable the `diagnostic_ui` feature, and add the
/// [`PhysicsDiagnosticsUiPlugin`] to your app.
///
/// See the [module-level documentation](crate::diagnostics) for more information.
///
/// [`PhysicsDiagnosticsUiPlugin`]: crate::diagnostics::ui::PhysicsDiagnosticsUiPlugin
#[cfg(feature = "bevy_diagnostic")]
pub struct PhysicsDiagnosticsPlugin;

#[cfg(feature = "bevy_diagnostic")]
impl Plugin for PhysicsDiagnosticsPlugin {
    fn build(&self, app: &mut App) {
        // Configure system sets for physics diagnostics.
        app.configure_sets(
            PhysicsSchedule,
            (
                PhysicsDiagnosticsSystems::Reset.before(PhysicsStepSystems::First),
                PhysicsDiagnosticsSystems::WriteDiagnostics.after(PhysicsStepSystems::Last),
            ),
        );
    }
}

/// A system set for [physics diagnostics](crate::diagnostics).
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PhysicsDiagnosticsSystems {
    /// Resets diagnostics to their default values.
    Reset,
    /// Writes physics diagnostics to other resources, commonly `DiagnosticsStore`.
    WriteDiagnostics,
}

/// A trait for resources storing timers and counters for [physics diagnostics](crate::diagnostics).
pub trait PhysicsDiagnostics: Default + Resource {
    /// Maps diagnostic paths to their respective duration fields.
    fn timer_paths(&self) -> Vec<(&'static DiagnosticPath, Duration)> {
        Vec::new()
    }

    /// Maps diagnostic paths to their respective counter fields.
    fn counter_paths(&self) -> Vec<(&'static DiagnosticPath, u32)> {
        Vec::new()
    }

    /// A system that resets the diagnostics to their default values.
    fn reset(mut physics_diagnostics: ResMut<Self>) {
        *physics_diagnostics = Self::default();
    }

    /// A system that writes diagnostics to the given [`Diagnostics`] instance.
    #[cfg(feature = "bevy_diagnostic")]
    fn write_diagnostics(physics_diagnostics: Res<Self>, mut diagnostics: Diagnostics) {
        for (path, duration) in physics_diagnostics.timer_paths() {
            diagnostics.add_measurement(path, || duration.as_secs_f64() * 1000.0);
        }

        for (path, count) in physics_diagnostics.counter_paths() {
            diagnostics.add_measurement(path, || count as f64);
        }
    }
}

/// An extension trait for registering [physics diagnostics](crate::diagnostics) in an [`App`].
pub trait AppDiagnosticsExt {
    /// Registers timer and counter diagnostics for a resource implementing [`PhysicsDiagnostics`].
    ///
    /// This method should be called in [`Plugin::finish`] to ensure that [`Diagnostic`]s
    /// are only tracked if the [`PhysicsDiagnosticsPlugin`] was added to the app.
    fn register_physics_diagnostics<T: PhysicsDiagnostics>(&mut self);
}

impl AppDiagnosticsExt for App {
    fn register_physics_diagnostics<T: PhysicsDiagnostics>(&mut self) {
        // Avoid duplicate registrations.
        if self.world().is_resource_added::<T>() {
            return;
        }

        // Initialize the diagnostics resource.
        self.init_resource::<T>();

        // Make sure the system set exists, even if `PhysicsDiagnosticsPlugin` is not added.
        self.configure_sets(
            PhysicsSchedule,
            PhysicsDiagnosticsSystems::Reset.before(PhysicsStepSystems::First),
        );

        // Add a system to reset the resource, even if `PhysicsDiagnosticsPlugin` is not added.
        self.add_systems(
            PhysicsSchedule,
            T::reset
                .in_set(PhysicsDiagnosticsSystems::Reset)
                .ambiguous_with_all(),
        );

        #[cfg(feature = "bevy_diagnostic")]
        {
            // If physics diagnostics are not enabled, return early.
            if !self.is_plugin_added::<PhysicsDiagnosticsPlugin>() {
                return;
            }

            // Register diagnostics for the paths returned by the diagnostics resource.
            let diagnostics = T::default();
            let timer_paths = diagnostics.timer_paths();
            let counter_paths = diagnostics.counter_paths();

            for path in timer_paths.iter().map(|(path, _)| *path) {
                // All timers are in milliseconds.
                self.register_diagnostic(Diagnostic::new(path.clone()).with_suffix("ms"));
            }

            for path in counter_paths.iter().map(|(path, _)| *path) {
                // All counters are in whole numbers.
                self.register_diagnostic(Diagnostic::new(path.clone()).with_smoothing_factor(0.0));
            }

            // Add systems to reset the diagnostics and write them to the `DiagnosticsStore` resource.
            self.add_systems(
                PhysicsSchedule,
                T::write_diagnostics
                    .in_set(PhysicsDiagnosticsSystems::WriteDiagnostics)
                    .ambiguous_with_all(),
            );
        }
    }
}
