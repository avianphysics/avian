//! Sets up the default scheduling, system set configuration, and time resources
//! for the physics solver and substepping loop.
//!
//! See [`SolverSchedulePlugin`].

use crate::prelude::*;
use bevy::{
    ecs::schedule::{ExecutorKind, LogLevel, ScheduleBuildSettings, ScheduleLabel},
    prelude::*,
};
use dynamics::integrator::IntegrationSystems;

/// Sets up the default scheduling, system set configuration, and time resources for the physics solver.
#[derive(Debug, Default)]
pub struct SolverSchedulePlugin;

impl Plugin for SolverSchedulePlugin {
    fn build(&self, app: &mut App) {
        // Register types with generics.
        app.register_type::<Time<Substeps>>();

        // Initialize resources.
        app.insert_resource(Time::new_with(Substeps))
            .init_resource::<SubstepCount>();

        // Get the `PhysicsSchedule`, and panic if it doesn't exist.
        let physics = app
            .get_schedule_mut(PhysicsSchedule)
            .expect("add PhysicsSchedule first");

        // See `SolverSystems` for what each system set is responsible for.
        physics.configure_sets(
            (
                SolverSystems::PrepareSolverBodies,
                SolverSystems::PrepareJoints,
                SolverSystems::PrepareContactConstraints,
                SolverSystems::PreSubstep,
                SolverSystems::Substep,
                SolverSystems::PostSubstep,
                SolverSystems::Restitution,
                SolverSystems::Finalize,
                SolverSystems::StoreContactImpulses,
            )
                .chain()
                .in_set(PhysicsStepSystems::Solver),
        );

        // Run the substepping loop.
        physics.add_systems(run_substep_schedule.in_set(SolverSystems::Substep));

        // Set up the substep schedule, the schedule that runs systems in the inner substepping loop.
        app.edit_schedule(SubstepSchedule, |schedule| {
            schedule
                .set_executor_kind(ExecutorKind::SingleThreaded)
                .set_build_settings(ScheduleBuildSettings {
                    ambiguity_detection: LogLevel::Error,
                    ..default()
                })
                .configure_sets(
                    (
                        IntegrationSystems::Velocity,
                        SubstepSolverSystems::WarmStart,
                        SubstepSolverSystems::SolveConstraints,
                        IntegrationSystems::Position,
                        SubstepSolverSystems::Relax,
                        SubstepSolverSystems::Damping,
                    )
                        .chain(),
                );
        });
    }
}

/// The substepping schedule that runs in [`SolverSystems::Substep`].
/// The number of substeps per physics step is configured through the [`SubstepCount`] resource.
#[derive(Debug, Hash, PartialEq, Eq, Clone, ScheduleLabel)]
pub struct SubstepSchedule;

/// System sets for the constraint solver.
///
/// # Steps
///
/// Below is the core solver loop.
///
/// 1. Prepare solver bodies ([`SolverSystems::PrepareSolverBodies`])
/// 2. Prepare joints ([`SolverSystems::PrepareJoints`])
/// 3. Prepare contact constraints ([`SolverSystems::PrepareContactConstraints`])
/// 4. Substepping loop (runs the [`SubstepSchedule`] [`SubstepCount`] times; see [`SolverSystems::Substep`])
/// 5. Apply restitution ([`SolverSystems::Restitution`])
/// 6. Write back solver body data to rigid bodies. ([`SolverSystems::Finalize`])
/// 7. Store contact impulses for next frame's warm starting ([`SolverSystems::StoreContactImpulses`])
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum SolverSystems {
    /// Prepares [solver bodies] for the substepping loop.
    ///
    /// [solver bodies]: crate::dynamics::solver::solver_body::SolverBody
    PrepareSolverBodies,
    /// Prepares joint constraints for the substepping loop.
    PrepareJoints,
    /// Prepares contact constraints for the substepping loop.
    PrepareContactConstraints,
    /// A system set for systems running just before the substepping loop.
    PreSubstep,
    /// A system set for the substepping loop.
    Substep,
    /// A system set for systems running just after the substepping loop.
    PostSubstep,
    /// Applies [restitution](Restitution) for bodies after solving overlap.
    Restitution,
    /// Writes back solver body data to rigid bodies.
    Finalize,
    /// Copies contact impulses from [`ContactConstraints`] to the contacts in the [`ContactGraph`].
    /// They will be used for [warm starting](SubstepSolverSystems::WarmStart) the next frame or substep.
    ///
    /// [`ContactConstraints`]: super::ContactConstraints
    StoreContactImpulses,
}

/// A deprecated alias for [`SolverSystems`].
#[deprecated(since = "0.4.0", note = "Renamed to `SolverSystems`")]
pub type SolverSet = SolverSystems;

/// System sets for the substepped part of the constraint solver.
///
/// # Steps
///
/// 1. Integrate velocity ([`IntegrationSystems::Velocity`])
/// 2. Warm start ([`SubstepSolverSystems::WarmStart`])
/// 3. Solve constraints with bias ([`SubstepSolverSystems::SolveConstraints`])
/// 4. Integrate positions ([`IntegrationSystems::Position`])
/// 5. Solve constraints without bias to relax velocities ([`SubstepSolverSystems::Relax`])
/// 6. Apply velocity-based constraint damping ([`SubstepSolverSystems::Damping`])
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum SubstepSolverSystems {
    /// Warm starts the solver by applying the impulses from the previous frame or substep.
    ///
    /// This significantly improves convergence, but by itself can lead to overshooting.
    /// Overshooting is reduced by [relaxing](SubstepSolverSystems::Relax) the biased velocities
    /// by running the solver a second time *without* bias.
    WarmStart,
    /// Solves velocity constraints using a position bias that boosts the response
    /// to account for the constraint error.
    SolveConstraints,
    /// Solves velocity constraints without a position bias to relax the biased velocities
    /// and impulses. This reduces overshooting caused by [warm starting](SubstepSolverSystems::WarmStart).
    Relax,
    /// Applies velocity-based constraint damping, such as [`JointDamping`].
    Damping,
}

/// A deprecated alias for [`SubstepSolverSystems`].
#[deprecated(since = "0.4.0", note = "Renamed to `SubstepSolverSystems`")]
pub type SubstepSolverSet = SubstepSolverSystems;

/// The number of substeps used in the simulation.
///
/// A higher number of substeps reduces the value of [`Time`],
/// which results in a more accurate simulation, but also reduces performance. The default
/// substep count is currently 6.
///
/// If you use a very high substep count and encounter stability issues, consider enabling the `f64`
/// feature as shown in the [getting started guide](crate#getting-started) to avoid floating point
/// precision problems.
///
/// # Example
///
/// You can change the number of substeps by inserting the [`SubstepCount`] resource:
///
/// ```no_run
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// fn main() {
///     App::new()
///         .add_plugins((DefaultPlugins, PhysicsPlugins::default()))
///         .insert_resource(SubstepCount(12))
///         .run();
/// }
/// ```
#[derive(Debug, Reflect, Resource, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Resource, PartialEq)]
pub struct SubstepCount(pub u32);

impl Default for SubstepCount {
    fn default() -> Self {
        Self(6)
    }
}

/// Runs the [`SubstepSchedule`].
fn run_substep_schedule(world: &mut World) {
    let delta = world.resource::<Time<Physics>>().delta();
    let SubstepCount(substeps) = *world.resource::<SubstepCount>();
    let sub_delta = delta.div_f64(substeps as f64);

    let mut sub_delta_time = world.resource_mut::<Time<Substeps>>();
    sub_delta_time.advance_by(sub_delta);

    let _ = world.try_schedule_scope(SubstepSchedule, |world, schedule| {
        for i in 0..substeps {
            trace!("running SubstepSchedule: {i}");
            *world.resource_mut::<Time>() = world.resource::<Time<Substeps>>().as_generic();
            schedule.run(world);
        }
    });

    // Set generic `Time` resource back to `Time<Physics>`.
    // Later, it's set back to the default clock after the `PhysicsSchedule`.
    *world.resource_mut::<Time>() = world.resource::<Time<Physics>>().as_generic();
}
