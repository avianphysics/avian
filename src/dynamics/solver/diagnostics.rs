use bevy::{
    diagnostic::DiagnosticPath,
    prelude::{ReflectResource, Resource},
    reflect::Reflect,
};
use core::time::Duration;

use crate::diagnostics::{PhysicsDiagnostics, impl_diagnostic_paths};

/// Diagnostics for the physics solver.
#[derive(Resource, Debug, Default, Reflect)]
#[reflect(Resource, Debug)]
pub struct SolverDiagnostics {
    /// Time spent preparing constraints.
    pub prepare_constraints: Duration,
    /// Time spent preparing or clearing velocity increments in [`VelocityIntegrationData`]s.
    ///
    /// [`VelocityIntegrationData`]: crate::dynamics::integrator::VelocityIntegrationData
    pub update_velocity_increments: Duration,
    /// Time spent integrating velocities.
    pub integrate_velocities: Duration,
    /// Time spent warm starting the solver.
    pub warm_start: Duration,
    /// Time spent solving constraints with bias.
    pub solve_constraints: Duration,
    /// Time spent integrating positions.
    pub integrate_positions: Duration,
    /// Time spent relaxing velocities.
    pub relax_velocities: Duration,
    /// Time spent applying restitution.
    pub apply_restitution: Duration,
    /// Time spent writing the final results to the bodies.
    pub finalize: Duration,
    /// Time spent storing impulses for warm starting.
    pub store_impulses: Duration,
    /// Time spent on swept CCD.
    pub swept_ccd: Duration,
    /// The number of contact constraints generated.
    pub contact_constraint_count: u32,
}

impl PhysicsDiagnostics for SolverDiagnostics {
    fn timer_paths(&self) -> Vec<(&'static DiagnosticPath, Duration)> {
        vec![
            (Self::PREPARE_CONSTRAINTS, self.prepare_constraints),
            (
                Self::UPDATE_VELOCITY_INCREMENTS,
                self.update_velocity_increments,
            ),
            (Self::INTEGRATE_VELOCITIES, self.integrate_velocities),
            (Self::WARM_START, self.warm_start),
            (Self::SOLVE_CONSTRAINTS, self.solve_constraints),
            (Self::INTEGRATE_POSITIONS, self.integrate_positions),
            (Self::RELAX_VELOCITIES, self.relax_velocities),
            (Self::APPLY_RESTITUTION, self.apply_restitution),
            (Self::FINALIZE, self.finalize),
            (Self::STORE_IMPULSES, self.store_impulses),
            (Self::SWEPT_CCD, self.swept_ccd),
        ]
    }

    fn counter_paths(&self) -> Vec<(&'static DiagnosticPath, u32)> {
        vec![(
            Self::CONTACT_CONSTRAINT_COUNT,
            self.contact_constraint_count,
        )]
    }
}

impl_diagnostic_paths! {
    impl SolverDiagnostics {
        PREPARE_CONSTRAINTS: "avian/solver/prepare_constraints",
        UPDATE_VELOCITY_INCREMENTS: "avian/solver/update_velocity_increments",
        INTEGRATE_VELOCITIES: "avian/solver/integrate_velocities",
        WARM_START: "avian/solver/warm_start",
        SOLVE_CONSTRAINTS: "avian/solver/solve_constraints",
        INTEGRATE_POSITIONS: "avian/solver/integrate_positions",
        RELAX_VELOCITIES: "avian/solver/relax_velocities",
        APPLY_RESTITUTION: "avian/solver/apply_restitution",
        FINALIZE: "avian/solver/finalize",
        STORE_IMPULSES: "avian/solver/store_impulses",
        SWEPT_CCD: "avian/solver/swept_ccd",
        CONTACT_CONSTRAINT_COUNT: "avian/solver/contact_constraint_count",
    }
}
