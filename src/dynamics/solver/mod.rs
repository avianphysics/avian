//! The physics solver responsible for simulating rigid body dynamics, contacts, and joints.
//!
//! See [`SolverPlugins`] for a plugin group that contains Avian's default solver plugins.

mod plugin;
pub use plugin::*;

pub mod constraint_graph;
pub mod contact;
pub mod islands;
pub mod joint_graph;
pub mod schedule;
pub mod softness_parameters;
pub mod solver_body;
#[cfg(feature = "xpbd_joints")]
pub mod xpbd;

mod diagnostics;
pub use diagnostics::SolverDiagnostics;

use crate::{
    dynamics::solver::{joint_graph::JointGraphPlugin, solver_body::SolverBodyPlugin},
    prelude::*,
};
use bevy::{app::PluginGroupBuilder, prelude::*};

/// A plugin group that contains Avian's default solver plugins.
///
/// # Plugins
///
/// By default, the following plugins will be added:
///
/// | Plugin                            | Description                                                                                                                                                |
/// | --------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------- |
/// | [`SolverSchedulePlugin`]          | Sets up the solver and substepping loop by initializing the necessary schedules, sets and resources.                                                       |
/// | [`SolverBodyPlugin`]              | Manages [solver bodies](dynamics::solver::solver_body::SolverBody).                                                                                        |
/// | [`IntegratorPlugin`]              | Handles motion caused by velocity, and applies external forces and gravity.                                                                                |
/// | [`SolverPlugin`]                  | Manages and solves contacts, [joints](dynamics::joints), and other constraints.                                                                            |
/// | [`CcdPlugin`]                     | Performs sweep-based [Continuous Collision Detection](dynamics::ccd) for bodies with the [`SweptCcd`] component.                                           |
/// | [`IslandPlugin`]                  | Manages [simulation islands](dynamics::solver::islands) for sleeping and waking.                                                                           |
/// | [`IslandSleepingPlugin`]          | Manages sleeping and waking of [simulation islands](dynamics::solver::islands).                                                                            |
/// | [`JointGraphPlugin`]              | Manages the [`JointGraph`](joint_graph::JointGraph) for each joint type.                                                                                   |
/// | [`XpbdSolverPlugin`]              | Solves joints using Extended Position-Based Dynamics (XPBD). Requires the `xpbd_joints` feature.                                                           |
///
/// Refer to the documentation of the plugins for more information about their responsibilities and implementations.
#[derive(Debug, Default)]
pub struct SolverPlugins {
    length_unit: Scalar,
}

impl SolverPlugins {
    /// Creates a [`SolverPlugins`] plugin group with the given approximate dimensions of most objects.
    ///
    /// The length unit will be used for initializing the [`PhysicsLengthUnit`]
    /// resource unless it already exists.
    pub fn new_with_length_unit(unit: Scalar) -> Self {
        Self { length_unit: unit }
    }
}

impl PluginGroup for SolverPlugins {
    fn build(self) -> PluginGroupBuilder {
        let builder = PluginGroupBuilder::start::<Self>()
            .add(SolverBodyPlugin)
            .add(SolverSchedulePlugin)
            .add(IntegratorPlugin::default())
            .add(SolverPlugin::new_with_length_unit(self.length_unit))
            .add(CcdPlugin)
            .add(IslandPlugin)
            .add(IslandSleepingPlugin)
            .add(JointGraphPlugin::<FixedJoint>::default())
            .add(JointGraphPlugin::<RevoluteJoint>::default())
            .add(JointGraphPlugin::<PrismaticJoint>::default())
            .add(JointGraphPlugin::<DistanceJoint>::default());

        #[cfg(feature = "3d")]
        let builder = builder.add(JointGraphPlugin::<SphericalJoint>::default());

        #[cfg(feature = "xpbd_joints")]
        let builder = builder.add(XpbdSolverPlugin);

        builder
    }
}
