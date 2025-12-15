use core::time::Duration;

use bevy::{
    diagnostic::{Diagnostic, DiagnosticPath, Diagnostics, DiagnosticsStore, RegisterDiagnostic},
    prelude::*,
};

use crate::diagnostics::impl_diagnostic_paths;

use super::{AppDiagnosticsExt, PhysicsDiagnostics, PhysicsSchedule, PhysicsStepSystems};

/// A plugin that adds diagnostics for total physics timers and counters.
pub struct PhysicsTotalDiagnosticsPlugin;

impl Plugin for PhysicsTotalDiagnosticsPlugin {
    fn build(&self, app: &mut App) {
        // Register diagnostics for total physics timers and counters.
        // NOTE: This should be done after the `PhysicsDiagnosticsPlugin` is added.
        app.register_physics_diagnostics::<PhysicsTotalDiagnostics>();

        // For timers that are not covered by other diagnostics, we add a miscellaneous diagnostic.
        // This is computed from other timers and the total time spent on the physics step.
        #[cfg(feature = "bevy_diagnostic")]
        app.register_diagnostic(
            Diagnostic::new(PhysicsTotalDiagnostics::MISCELLANEOUS.clone()).with_suffix("ms"),
        );

        // Initialize `PhysicsStepStart` for tracking the time at which the physics step started.
        app.insert_resource::<PhysicsStepStart>(PhysicsStepStart(crate::utils::Instant::now()));

        // Add systems for updating total physics diagnostics.
        app.add_systems(
            PhysicsSchedule,
            (
                (update_physics_step_start, increment_physics_step_number)
                    .in_set(PhysicsStepSystems::First),
                update_step_time.in_set(PhysicsStepSystems::Last),
                #[cfg(feature = "bevy_diagnostic")]
                update_miscellaneous_physics_timer.after(PhysicsStepSystems::Last),
            ),
        );
    }
}

/// Diagnostics for total physics timers and counters.
#[derive(Resource, Debug, Default, Reflect)]
#[reflect(Resource, Debug)]
pub struct PhysicsTotalDiagnostics {
    /// The current physics step number.
    pub step_number: u32,
    /// The total time spent on the physics step.
    pub step_time: Duration,
}

impl PhysicsDiagnostics for PhysicsTotalDiagnostics {
    fn counter_paths(&self) -> Vec<(&'static DiagnosticPath, u32)> {
        vec![(Self::STEP_NUMBER, self.step_number)]
    }

    fn timer_paths(&self) -> Vec<(&'static DiagnosticPath, Duration)> {
        vec![(Self::STEP_TIME, self.step_time)]
    }
}

impl_diagnostic_paths! {
    impl PhysicsTotalDiagnostics {
        // The current physics step number.
        STEP_NUMBER: "avian/step_number",
        // The total time spent on the physics step.
        STEP_TIME: "avian/total_step_time",
        // The time spent on the physics step not covered by other diagnostics.
        MISCELLANEOUS: "avian/miscellaneous",
    }
}

fn increment_physics_step_number(
    mut diagnostics: ResMut<PhysicsTotalDiagnostics>,
    mut step: Local<u32>,
) {
    *step += 1;
    diagnostics.step_number = *step;
}

/// The time at which the physics step started.
#[derive(Resource, Debug, Reflect)]
#[reflect(Resource, Debug)]
struct PhysicsStepStart(pub crate::utils::Instant);

fn update_physics_step_start(mut start: ResMut<PhysicsStepStart>) {
    start.0 = crate::utils::Instant::now();
}

fn update_step_time(
    start: Res<PhysicsStepStart>,
    mut diagnostics: ResMut<PhysicsTotalDiagnostics>,
) {
    diagnostics.step_time = start.0.elapsed();
}

/// Updates the time spent on the physics step not covered by other diagnostics.
#[cfg(feature = "bevy_diagnostic")]
fn update_miscellaneous_physics_timer(mut diagnostics: Diagnostics, store: Res<DiagnosticsStore>) {
    // Get the total time spent on the physics step.
    let total = store
        .get(PhysicsTotalDiagnostics::STEP_TIME)
        .and_then(|d| d.measurement().map(|m| m.value))
        .unwrap_or(0.0);

    // Get the total time spent on the physics step not covered by other diagnostics.
    let timed: f64 = store
        .iter()
        .filter_map(|d| {
            // Only consider timers that are not the total step time or this diagnostic.
            if (d.suffix != "s" && d.suffix != "ms" && d.suffix != "us")
                || d.path() == PhysicsTotalDiagnostics::STEP_TIME
                || d.path() == PhysicsTotalDiagnostics::MISCELLANEOUS
            {
                return None;
            }

            // Only consider timers that are under the "avian" path.
            d.path()
                .components()
                .next()
                .and_then(|c| (c == "avian").then_some(d.value()))
                .flatten()
        })
        .sum();

    // If the total time spent on the physics step is greater than the time covered by other diagnostics,
    // add a miscellaneous diagnostic for the difference.
    if timed > 0.0 {
        diagnostics.add_measurement(PhysicsTotalDiagnostics::MISCELLANEOUS, || {
            (total - timed).max(0.0)
        });
    }
}
