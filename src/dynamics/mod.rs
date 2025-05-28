//! Rigid body dynamics, handling the motion and physical interactions of non-deformable objects.
//!
//! A [`RigidBody`] is the core component of a physics simulation.
//! It describes a rigid, non-deformable physics object that can be either dynamic,
//! kinematic, or static.
//!
//! Rigid body dynamics includes:
//!
//! - Motion of rigid bodies based on their [`LinearVelocity`] and [`AngularVelocity`].
//! - Acceleration caused by external forces and [`Gravity`].
//! - Collision response, preventing objects from overlapping each other,
//!   considering properties such as [`Friction`] and [`Restitution`].
//! - [Joints](solver::joints) connecting rigid bodies to each other.
//! - Everything else related to the physical behavior and properties of rigid bodies.
//!
//! Rigid body dynamics does *not* include:
//!
//! - Simulation of fluids (liquids and gasses)
//! - Elasticity (soft bodies)
//! - Plasticity (permanent deformation)
//!
//! # Plugins
//!
//! | Plugin               | Description                                                                                                                           |
//! | -------------------- | ------------------------------------------------------------------------------------------------------------------------------------- |
//! | [`IntegratorPlugin`] | Handles motion caused by velocity, and applies external forces and gravity.                                                           |
//! | [`SolverPlugin`]     | Solves constraints (contacts and joints).                                                                                             |
//! | [`CcdPlugin`]        | Performs sweep-based [Continuous Collision Detection](dynamics::ccd) for bodies with the [`SweptCcd`] component to prevent tunneling. |
//! | [`SleepingPlugin`]   | Manages sleeping and waking for bodies, automatically deactivating them to save computational resources.                              |
//!
//! # Accuracy
//!
//! The engine uses iterative algorithms to approximate the simulation.
//! Thus, results may not be perfectly accurate:
//!
//! - Constraints (contacts and joints) are not perfectly rigid.
//!   - Objects can overlap, especially in extreme stacking scenarios.
//!   - Contacts can be slightly bouncy even when the [`Restitution`] is zero.
//!   - Joint chains can stretch and in extreme cases have jittery behavior.
//!   - High mass ratios, such as when a very heavy object rests on top of a lighter one,
//!     can be difficult for the engine to deal with.
//! - [`Friction`] and [`Restitution`] may not be perfectly accurate.
//! - Objects moving at high speeds can pass through thin and small geometry due to discrete time steps,
//!   a phenomenon known as *tunneling*. This can be mitigated with [Continuous Collision Detection](ccd).
//!
//! These caveats are very common for physics engines intended for real-time applications,
//! not something specific to this engine. Approximations are required for several reasons:
//!
//! - Performance.
//!   - Exact results are simply not possible, especially at the frame rates
//!     expected from games and other real-time applications.
//! - Some differential equations do not have known exact solutions.
//!   - [Semi-implicit Euler] integration is used to approximate them.
//! - Some constraints cannot be solved uniquely.
//!   - An iterative [Gauss-Seidel] solver is used to solve them approximately.
//!
//! In practice, the results should be accurate enough for most real-time applications.
//! Perfect accuracy is very rarely necessary, if ever, and most of the more visible issues
//! can typically be worked around.
//!
//! [Gauss-Seidel]: https://en.wikipedia.org/wiki/Gauss%E2%80%93Seidel_method
//! [Semi-implicit Euler]: https://en.wikipedia.org/wiki/Semi-implicit_Euler_method

pub mod ccd;
pub mod integrator;
pub mod rigid_body;
pub mod sleeping;
pub mod solver;

/// Re-exports common types related to the rigid body dynamics functionality.
pub mod prelude {
    pub(crate) use super::rigid_body::mass_properties::{
        components::GlobalAngularInertia, ComputeMassProperties, MassProperties,
    };
    pub use super::{
        ccd::{CcdPlugin, SpeculativeMargin, SweepMode, SweptCcd},
        integrator::{Gravity, IntegratorPlugin},
        rigid_body::{
            mass_properties::{
                bevy_heavy::{
                    AngularInertiaTensor, AngularInertiaTensorError, ComputeMassProperties2d,
                    ComputeMassProperties3d, MassProperties2d, MassProperties3d,
                },
                components::{
                    AngularInertia, CenterOfMass, ColliderDensity, ColliderMassProperties,
                    ComputedAngularInertia, ComputedCenterOfMass, ComputedMass, Mass,
                    MassPropertiesBundle, NoAutoAngularInertia, NoAutoCenterOfMass, NoAutoMass,
                },
                MassPropertiesExt, MassPropertyHelper, MassPropertyPlugin,
            },
            *,
        },
        sleeping::{DeactivationTime, SleepingPlugin, SleepingThreshold, WakeUpBody},
        solver::{
            joints::*,
            schedule::{SolverSchedulePlugin, SolverSet, SubstepCount, SubstepSchedule},
            solver_body::SolverBodyPlugin,
            PhysicsLengthUnit, SolverPlugin,
        },
    };
}

// For intra-doc links
#[allow(unused_imports)]
use crate::prelude::*;
