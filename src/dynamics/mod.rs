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
//! - [Joints](joints) connecting rigid bodies to each other.
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
//! | Plugin                 | Description                                                                                                                                |
//! | ---------------------- | ------------------------------------------------------------------------------------------------------------------------------------------ |
//! | [`SolverPlugins`]      | A plugin group for the physics solver's plugins. See the plugin group's documentation for more information.                                |
//! | [`JointPlugin`]        | A plugin for managing and initializing [joints]. Does *not* include the actual joint solver.                                               |
//! | [`MassPropertyPlugin`] | Manages mass properties of dynamic [rigid bodies](RigidBody).                                                                              |
//! | [`ForcePlugin`]        | Manages and applies external forces, torques, and acceleration for rigid bodies. See the [module-level documentation](rigid_body::forces). |
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
pub mod joints;
pub mod rigid_body;
pub mod solver;

/// Re-exports common types related to the rigid body dynamics functionality.
pub mod prelude {
    pub(crate) use super::rigid_body::mass_properties::{ComputeMassProperties, MassProperties};
    #[cfg(feature = "xpbd_joints")]
    pub use super::solver::xpbd::XpbdSolverPlugin;
    #[expect(deprecated)]
    pub use super::{
        ccd::{CcdPlugin, SpeculativeMargin, SweepMode, SweptCcd},
        integrator::{
            CustomPositionIntegration, CustomVelocityIntegration, Gravity, IntegratorPlugin,
        },
        joints::{
            AngleLimit, AngularMotor, DistanceJoint, DistanceLimit, FixedJoint, JointAnchor,
            JointBasis, JointCollisionDisabled, JointDamping, JointDisabled, JointForces,
            JointFrame, JointPlugin, LinearMotor, MotorModel, PrismaticJoint, RevoluteJoint,
        },
        rigid_body::{
            forces::{
                ConstantAngularAcceleration, ConstantForce, ConstantLinearAcceleration,
                ConstantLocalForce, ConstantLocalLinearAcceleration, ConstantTorque, ForcePlugin,
                ForceSystems, Forces, ReadRigidBodyForces, RigidBodyForces, WriteRigidBodyForces,
            },
            mass_properties::{
                MassPropertiesExt, MassPropertyHelper, MassPropertyPlugin,
                bevy_heavy::{
                    AngularInertiaTensor, AngularInertiaTensorError, ComputeMassProperties2d,
                    ComputeMassProperties3d, MassProperties2d, MassProperties3d,
                },
                components::{
                    AngularInertia, CenterOfMass, ColliderDensity, ColliderMassProperties,
                    ComputedAngularInertia, ComputedCenterOfMass, ComputedMass, Mass,
                    MassPropertiesBundle, NoAutoAngularInertia, NoAutoCenterOfMass, NoAutoMass,
                },
            },
            sleeping::{
                DeactivationTime, SleepThreshold, SleepTimer, Sleeping, SleepingDisabled,
                SleepingThreshold, TimeSleeping, TimeToSleep,
            },
            *,
        },
        solver::{
            PhysicsLengthUnit, SolverPlugin, SolverPlugins,
            islands::{
                IslandPlugin, IslandSleepingPlugin, SleepBody, SleepIslands, WakeBody, WakeIslands,
            },
            schedule::{
                SolverSchedulePlugin, SolverSet, SolverSystems, SubstepCount, SubstepSchedule,
            },
            solver_body::SolverBodyPlugin,
        },
    };
    #[cfg(feature = "3d")]
    pub use super::{
        joints::SphericalJoint,
        rigid_body::forces::{ConstantLocalAngularAcceleration, ConstantLocalTorque},
    };
}

// For intra-doc links
#[allow(unused_imports)]
use crate::prelude::*;
