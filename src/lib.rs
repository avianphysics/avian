//! ![Avian Physics](https://raw.githubusercontent.com/Jondolf/avian/avian/assets/branding/logo.svg)
//!
//! **Avian** is an ECS-driven 2D and 3D physics engine for the [Bevy game engine](https://bevyengine.org/).
//!
//! Check out the [GitHub repository](https://github.com/avianphysics/avian)
//! for more information about the design, read the [Getting Started](#getting-started)
//! guide below to get up to speed, and take a look at the [Table of Contents](#table-of-contents)
//! for an overview of the engine's features and their documentation.
//!
//! You can also check out the [FAQ](#frequently-asked-questions), and if you encounter
//! any further problems, consider saying hello on the [Bevy Discord](https://discord.gg/bevy)!
//!
//! # Getting Started
//!
//! This short guide should help you get started with Avian.
//!
//! ## Add the Dependency
//!
//! First, add `avian2d` or `avian3d` to the dependencies in your `Cargo.toml`:
//!
//! ```toml
//! # For 2D applications:
//! [dependencies]
//! avian2d = "0.5"
//!
//! # For 3D applications:
//! [dependencies]
//! avian3d = "0.5"
//!
//! # If you want to use the most up-to-date version, you can follow the main branch:
//! [dependencies]
//! avian3d = { git = "https://github.com/avianphysics/avian", branch = "main" }
//! ```
//!
//! You can specify features by disabling the default features and manually adding
//! the feature flags you want:
//!
//! ```toml
//! [dependencies]
//! # Add 3D Avian with double-precision floating point numbers.
//! # `parry-f64` enables collision detection using Parry.
//! avian3d = { version = "0.5", default-features = false, features = ["3d", "f64", "parry-f64", "xpbd_joints"] }
//! ```
//!
//! ## Feature Flags
//!
//! | Feature                | Description                                                                                                                                        | Default feature |
//! | ---------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------- | --------------- |
//! | `2d`                   | Enables 2D physics. Incompatible with `3d`.                                                                                                        | Yes (`avian2d`) |
//! | `3d`                   | Enables 3D physics. Incompatible with `2d`.                                                                                                        | Yes (`avian3d`) |
//! | `f32`                  | Enables `f32` precision for physics. Incompatible with `f64`.                                                                                      | Yes             |
//! | `f64`                  | Enables `f64` precision for physics. Incompatible with `f32`.                                                                                      | No              |
//! | `default-collider`     | Enables the default [`Collider`]. Required for [spatial queries](spatial_query). Requires either the `parry-f32` or `parry-f64` feature.           | Yes             |
//! | `parry-f32`            | Enables the `f32` version of the Parry collision detection library. Also enables the `default-collider` feature.                                   | Yes             |
//! | `parry-f64`            | Enables the `f64` version of the Parry collision detection library. Also enables the `default-collider` feature.                                   | No              |
//! | `xpbd_joints`          | Enables support for [XPBD joints](dynamics::solver::xpbd).                            .                                                            | Yes             |
#![cfg_attr(
    feature = "3d",
    doc = "| `collider-from-mesh`   | Allows you to create [`Collider`]s from `Mesh`es.                                                                                                  | Yes             |"
)]
//! | `bevy_scene`           | Enables [`ColliderConstructorHierarchy`] to wait until a [`Scene`] has loaded before processing it.                                                 | Yes             |
//! | `bevy_picking`         | Enables physics picking support for [`bevy_picking`] using the [`PhysicsPickingPlugin`]. The plugin must be added separately.                       | Yes             |
//! | `bevy_diagnostic`      | Enables writing [physics diagnostics] to the [`DiagnosticsStore`] with the [`PhysicsDiagnosticsPlugin`]. The plugin must be added separately.       | No              |
//! | `diagnostic_ui`        | Enables [physics diagnostics] UI for performance timers and counters using the [`PhysicsDiagnosticsUiPlugin`]. The plugin must be added separately. | No              |
//! | `debug-plugin`         | Enables physics debug rendering using the [`PhysicsDebugPlugin`]. The plugin must be added separately.                                              | Yes             |
//! | `enhanced-determinism` | Enables cross-platform deterministic math, improving determinism across architectures at a small performance cost.                                  | No              |
//! | `parallel`             | Enables some extra multithreading, which improves performance for larger simulations but can add some overhead for smaller ones.                    | Yes             |
//! | `simd`                 | Enables [SIMD] optimizations.                                                                                                                       | No              |
//! | `serialize`            | Enables support for serialization and deserialization using Serde.                                                                                  | No              |
//! | `validate`             | Enables additional correctness checks and validation at the cost of worse performance.                                                              | No              |
//!
//! [`bevy_picking`]: bevy::picking
//! [physics diagnostics]: diagnostics
//! [`DiagnosticsStore`]: bevy::diagnostic::DiagnosticsStore
//! [SIMD]: https://en.wikipedia.org/wiki/Single_instruction,_multiple_data
//!
//! ## Add the Plugins
//!
//! Avian is designed to be very modular. It is built from several [plugins](PhysicsPlugins) that
//! manage different parts of the engine. These plugins can be easily initialized and configured through
//! the [`PhysicsPlugins`] plugin group.
//!
//! ```no_run
#![cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
//! use bevy::prelude::*;
//!
//! fn main() {
//!     App::new()
//!         .add_plugins((DefaultPlugins, PhysicsPlugins::default()))
//!         // ...your other plugins, systems and resources
//!         .run();
//! }
//! ```
//!
//! Now you can use all of Avian's components and resources to build whatever you want!
//!
//! For example, adding a [rigid body](RigidBody) with a [collider](Collider) is as simple as spawning an entity
//! with the [`RigidBody`] and [`Collider`] components:
//!
//! ```
#![cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
//! use bevy::prelude::*;
//!
//! fn setup(mut commands: Commands) {
#![cfg_attr(
    feature = "2d",
    doc = "    commands.spawn((RigidBody::Dynamic, Collider::circle(0.5)));"
)]
#![cfg_attr(
    feature = "3d",
    doc = "    commands.spawn((RigidBody::Dynamic, Collider::sphere(0.5)));"
)]
//! }
//! ```
//!
//! You can find lots of [usage examples](https://github.com/avianphysics/avian#more-examples)
//! in the project's [repository](https://github.com/avianphysics/avian).
//!
//! # Table of Contents
//!
//! Below is a structured overview of the documentation for the various
//! features of the engine.
//!
//! ## Rigid Body Dynamics
//!
//! - [Rigid body types](RigidBody#rigid-body-types)
//! - [Creating rigid bodies](RigidBody#creation)
//! - [Movement](RigidBody#movement)
//!     - [Linear](LinearVelocity) and [angular](AngularVelocity) velocity
//!     - [External forces, impulses, and acceleration](dynamics::rigid_body::forces)
//! - [Gravity] and [gravity scale](GravityScale)
//! - [Mass properties](dynamics::rigid_body::mass_properties)
//! - [Linear](LinearDamping) and [angular](AngularDamping) velocity damping
//! - [Lock translational and rotational axes](LockedAxes)
//! - [Dominance]
//! - [Continuous Collision Detection (CCD)](dynamics::ccd)
//!     - [Speculative collision](dynamics::ccd#speculative-collision)
//!     - [Swept CCD](dynamics::ccd#swept-ccd)
//! - [`Transform` interpolation and extrapolation](PhysicsInterpolationPlugin)
//! - [Temporarily disabling a rigid body](RigidBodyDisabled)
//! - [Automatic deactivation with sleeping](Sleeping)
//!
//! See the [`dynamics`] module for more details about rigid body dynamics in Avian.
//!
//! ## Collision Detection
//!
//! - [Colliders](Collider)
//!     - [Creation](Collider#creation)
//!     - [Density](ColliderDensity)
//!     - [Friction] and [restitution](Restitution) (bounciness)
//!     - [Collision layers](CollisionLayers)
//!     - [Sensors](Sensor)
#![cfg_attr(
    feature = "3d",
    doc = "- Generating colliders for meshes and scenes with [`ColliderConstructor`] and [`ColliderConstructorHierarchy`]"
)]
//! - [Get colliding entities](CollidingEntities)
//! - [Collision events](collision#collision-events)
//! - [Accessing collision data](Collisions)
//! - [Filtering and modifying contacts with hooks](CollisionHooks)
//! - [Manual contact queries](collision::collider::contact_query)
//! - [Temporarily disabling a collider](ColliderDisabled)
//!
//! See the [`collision`] module for more details about collision detection and colliders in Avian.
//!
//! ## Constraints and Joints
//!
//! - [Joints](dynamics::joints)
//!     - [Fixed joint](FixedJoint)
//!     - [Distance joint](DistanceJoint)
//!     - [Prismatic joint](PrismaticJoint)
//!     - [Revolute joint](RevoluteJoint)
#![cfg_attr(feature = "3d", doc = "    - [Spherical joint](SphericalJoint)")]
//! - [Temporarily disabling a joint](JointDisabled)
#![cfg_attr(
    feature = "xpbd_joints",
    doc = "- [Custom XPBD constraints](dynamics::solver::xpbd#constraints) (advanced)"
)]
//!
//! Joint motors and articulations are not supported yet, but they will be implemented in a future release.
//!
//! ## Spatial Queries
//!
//! - [Spatial query types](spatial_query)
//!     - [Raycasting](spatial_query#raycasting) and [`RayCaster`]
//!     - [Shapecasting](spatial_query#shapecasting) and [`ShapeCaster`]
//!     - [Point projection](spatial_query#point-projection)
//!     - [Intersection tests](spatial_query#intersection-tests)
//! - [Spatial query filters](SpatialQueryFilter)
//! - [The `SpatialQuery` system parameter](SpatialQuery)
//!
//! ## Configuration
//!
//! - [Gravity]
//! - [`Transform` interpolation and extrapolation](PhysicsInterpolationPlugin)
//! - [Physics speed](Physics#physics-speed)
//! - [Configure simulation fidelity with substeps](SubstepCount)
//!
//! ## Debugging and Profiling
//!
//! - [Physics debug rendering](PhysicsDebugPlugin)
//! - [Physics diagnostics](diagnostics)
//!
//! ## Scheduling
//!
//! - [Schedules and sets](PhysicsSchedulePlugin#schedules-and-sets)
//!     - [`PhysicsSystems`]
//!     - [`PhysicsSchedule`] and [`PhysicsStepSystems`]
//!     - [`SubstepSchedule`]
//!     - [`SolverSystems`] and [`SubstepSolverSystems`](dynamics::solver::schedule::SubstepSolverSystems)
//!     - Many more internal system sets
//! - [Configure the schedule used for running physics](PhysicsPlugins#custom-schedule)
//! - [Pausing, resuming and stepping physics](Physics#pausing-resuming-and-stepping-physics)
//! - [Usage on servers](#can-the-engine-be-used-on-servers)
//!
//! ## Architecture
//!
//! - [List of plugins and their responsibilities](PhysicsPlugins)
//! - [Custom plugins](PhysicsPlugins#custom-plugins)
//!
//! # Frequently Asked Questions
//!
//! - [How does Avian compare to Rapier and bevy_rapier?](#how-does-avian-compare-to-rapier-and-bevy_rapier)
//! - [Why is nothing happening?](#why-is-nothing-happening)
//! - [Why is everything moving so slowly?](#why-is-everything-moving-so-slowly)
//! - [Why did my rigid body suddenly vanish?](#why-did-my-rigid-body-suddenly-vanish)
//! - [Why is performance so bad?](#why-is-performance-so-bad)
//! - [Why does movement look choppy?](#why-does-movement-look-choppy)
//! - [Is there a character controller?](#is-there-a-character-controller)
//! - [Why are there separate `Position` and `Rotation` components?](#why-are-there-separate-position-and-rotation-components)
//! - [Can the engine be used on servers?](#can-the-engine-be-used-on-servers)
//! - [Something else?](#something-else)
//!
//! ## How does Avian compare to Rapier and bevy_rapier?
//!
//! Rapier is the biggest and most used physics engine in the Rust ecosystem, and it is currently
//! the most mature and feature-rich option.
//!
//! `bevy_rapier` is a great physics integration for Bevy, but it does have several problems:
//!
//! - It has to maintain a separate physics world and synchronize a ton of data with Bevy each frame
//! - The source code is difficult to inspect, as the vast majority of it is glue code and wrappers
//! for Bevy
//! - It has poor docs.rs documentation, and the documentation on rapier.rs is often outdated and
//! missing features
//! - Overall, it doesn't have a native ECS-like feel
//!
//! Avian on the other hand is built *for* Bevy *with* Bevy, and it uses the ECS for both the internals
//! and the public API. This removes the need for a separate physics world, reduces overhead, and makes
//! the source code much more approachable and easy to inspect for Bevy users.
//!
//! In part thanks to Bevy's modular architecture and the ECS, Avian is also highly composable,
//! as it consists of several independent plugins and provides lots of options for configuration and extensions,
//! from [custom schedules](PhysicsPlugins#custom-schedule) and [plugins](PhysicsPlugins#custom-plugins) to
//! [custom joints](dynamics::joints#custom-joints) and [constraints](dynamics::solver::xpbd#custom-constraints).
//!
//! One disadvantage of Avian is that it is still relatively young, so it can have more bugs,
//! some missing features, and fewer community resources and third party crates. However, it is growing quite
//! rapidly, and it is already pretty close to feature-parity with Rapier.
//!
//! At the end of the day, both engines are solid options. If you are looking for a more mature and tested
//! physics integration, `bevy_rapier` is the better choice, but if you prefer an engine with less overhead
//! and a more native Bevy integration, consider using Avian. Their core APIs are also quite similar,
//! so switching between them shouldn't be too difficult.
//!
//! ## Why is nothing happening?
//!
//! Make sure you have added the [`PhysicsPlugins`] plugin group and you have given your rigid bodies
//! a [`RigidBody`] component. See the [getting started](#getting-started) section.
//!
//! ## Why is everything moving so slowly?
//!
//! If your application is in 2D, you might be using pixels as length units. This will require you to use
//! larger velocities and forces than you would in 3D. Make sure you set [`Gravity`] to some larger value
//! as well, because its magnitude is `9.81` by default, which is tiny in pixels.
//!
//! ## Why is performance so bad?
//!
//! It is highly recommended to enable some optimizations for debug builds, or to run your project
//! in release mode. This can have over a 100x performance impact in some cases.
//!
//! Add the following to your `Cargo.toml` to enable optimizations for debug builds:
//!
//! ```toml
//! # Enable a small amount of optimization in the dev profile.
//! [profile.dev]
//! opt-level = 1
//!
//! # Enable a large amount of optimization in the dev profile for dependencies.
//! [profile.dev.package."*"]
//! opt-level = 3
//! ```
//!
//! You can also further optimize release builds by setting the number of codegen units to `1`,
//! although this will also increase build times.
//!
//! ```toml
//! [profile.release]
//! codegen-units = 1
//! ```
//!
//! If you still have performance issues, consider enabling the [`PhysicsDiagnosticsPlugin`]
//! and [`PhysicsDiagnosticsUiPlugin`] (requires the `diagnostic_ui` feature) to see where time is being spent.
//! See the [diagnostics](diagnostics) module for more information.
//!
//! ## Why does movement look choppy?
//!
//! To produce consistent, frame rate independent behavior, physics by default runs
//! in the [`FixedPostUpdate`] schedule with a fixed timestep, meaning that the time between
//! physics ticks remains constant. On some frames, physics can either not run at all or run
//! more than once to catch up to real time. This can lead to visible stutter for movement.
//!
//! This stutter can be resolved by *interpolating* or *extrapolating* the positions of physics objects
//! in between physics ticks. Avian has built-in support for this through the [`PhysicsInterpolationPlugin`],
//! which is included in the [`PhysicsPlugins`] by default.
//!
//! Interpolation can be enabled for an individual entity by adding the [`TransformInterpolation`] component:
//!
//! ```
#![cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
//! # use bevy::prelude::*;
//! #
//! fn setup(mut commands: Commands) {
//!     // Enable interpolation for this rigid body.
//!     commands.spawn((
//!         RigidBody::Dynamic,
//!         Transform::default(),
//!         TransformInterpolation,
//!     ));
//! }
//! ```
//!
//! To make *all* rigid bodies interpolated by default, use [`PhysicsInterpolationPlugin::interpolate_all()`]:
//!
//! ```no_run
#![cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
//! # use bevy::prelude::*;
//! #
//! fn main() {
//!    App::new()
//!       .add_plugins(PhysicsPlugins::default().set(PhysicsInterpolationPlugin::interpolate_all()))
//!       // ...
//!       .run();
//! }
//! ```
//!
//! See the [`PhysicsInterpolationPlugin`] for more information.
//!
//! If this does not fix the choppiness, the problem could also be related to system ordering.
//! If you have a system for camera following, make sure it runs *after* physics,
//! but *before* Bevy's transform propagation in `PostUpdate`.
//!
//! ```
//! # use bevy::prelude::*;
//! #
//! # let mut app = App::new();
//! #
//! app.add_systems(
//!     PostUpdate,
//!     camera_follow_player.before(TransformSystems::Propagate),
//! );
//! #
//! # fn camera_follow_player() {}
//! ```
//!
//! ## Is there a character controller?
//!
//! Avian does not have a built-in character controller yet. However, it has a [`MoveAndSlide`]
//! system parameter with utilities for implementing your own kinematic character controllers.
//! See its documentation for more information.
//!
//! There are also some third party character controllers such as [`bevy_tnua`](https://github.com/idanarye/bevy-tnua)
//! that support Avian.
//!
//! For custom character controllers, you can take a look at the
#![cfg_attr(
    feature = "2d",
    doc = "[`dynamic_character_2d`] and [`kinematic_character_2d`] examples to get started."
)]
#![cfg_attr(
    feature = "3d",
    doc = "[`dynamic_character_3d`] and [`kinematic_character_3d`] examples to get started."
)]
//!
#![cfg_attr(
    feature = "2d",
    doc = "[`dynamic_character_2d`]: https://github.com/avianphysics/avian/tree/main/crates/avian2d/examples/dynamic_character_2d
[`kinematic_character_2d`]: https://github.com/avianphysics/avian/tree/main/crates/avian2d/examples/kinematic_character_2d"
)]
#![cfg_attr(
    feature = "3d",
    doc = "[`dynamic_character_3d`]: https://github.com/avianphysics/avian/tree/main/crates/avian3d/examples/dynamic_character_3d
[`kinematic_character_3d`]: https://github.com/avianphysics/avian/tree/main/crates/avian3d/examples/kinematic_character_3d"
)]
//!
//! ## Why are there separate `Position` and `Rotation` components?
//!
//! While `Transform` can be used for the vast majority of things, Avian internally
//! uses separate [`Position`] and [`Rotation`] components. These are automatically
//! kept in sync by the [`PhysicsTransformPlugin`].
//!
//! There are several reasons why the separate components are currently used.
//!
//! - Position and rotation should be global from the physics engine's point of view.
//! - Transform scale and shearing can cause issues and rounding errors in physics.
//! - Transform hierarchies can be problematic.
//! - There is no `f64` version of `Transform`.
//! - There is no 2D version of `Transform` (yet), and having a 2D version can optimize several computations.
//! - When position and rotation are separate, we can technically have more systems running in parallel.
//! - Only rigid bodies have rotation, particles typically don't (although we don't make a distinction yet).
//!
//! In external projects however, using [`Position`] and [`Rotation`] is only necessary when you
//! need to manage positions within [`PhysicsSystems::StepSimulation`]. Elsewhere, you should be able to use `Transform`.
//!
//! There is also a possibility that we will revisit this if/when Bevy has a `Transform2d` component.
//! Using `Transform` feels more idiomatic and simple, so it would be nice if it could be used directly
//! as long as we can get around the drawbacks.
//!
//! ## Can the engine be used on servers?
//!
//! Yes! Networking often requires running the simulation in a specific schedule, and in Avian it is straightforward
//! to [set the schedule that runs physics](PhysicsPlugins#custom-schedule) and [configure the timestep](Physics) if needed.
//! By default, physics runs at a fixed timestep in [`FixedPostUpdate`].
//!
//! ## Something else?
//!
//! Physics engines are very large and Avian is still young, so stability issues and bugs are to be expected.
//!
//! If you encounter issues, please consider first taking a look at the
//! [issues on GitHub](https://github.com/avianphysics/avian/issues) and
//! [open a new issue](https://github.com/avianphysics/avian/issues/new) if there already isn't one regarding your problem.
//!
//! You can also come and say hello on the [Bevy Discord server](https://discord.com/invite/gMUk5Ph).
//! There, you can find an Avian Physics topic on the `#ecosystem-crates` channel where you can ask questions.
//!
//! # License
//!
//! Avian is free and open source. All code in the Avian repository is dual-licensed under either:
//!
//! - MIT License ([LICENSE-MIT](https://github.com/avianphysics/avian/blob/main/LICENSE-MIT)
//! or <http://opensource.org/licenses/MIT>)
//! - Apache License, Version 2.0 ([LICENSE-APACHE](https://github.com/avianphysics/avian/blob/main/LICENSE-APACHE)
//! or <http://www.apache.org/licenses/LICENSE-2.0>)
//!
//! at your option.

#![doc(
    html_logo_url = "https://raw.githubusercontent.com/Jondolf/avian/avian/assets/branding/icon.png",
    html_favicon_url = "https://raw.githubusercontent.com/Jondolf/avian/avian/assets/branding/icon.png"
)]
#![allow(
    unexpected_cfgs,
    clippy::type_complexity,
    clippy::too_many_arguments,
    rustdoc::invalid_rust_codeblocks
)]
#![warn(clippy::doc_markdown, missing_docs)]

#[cfg(all(not(feature = "f32"), not(feature = "f64")))]
compile_error!("either feature \"f32\" or \"f64\" must be enabled");

#[cfg(all(feature = "f32", feature = "f64"))]
compile_error!("feature \"f32\" and feature \"f64\" cannot be enabled at the same time");

#[cfg(all(not(feature = "2d"), not(feature = "3d")))]
compile_error!("either feature \"2d\" or \"3d\" must be enabled");

#[cfg(all(feature = "2d", feature = "3d"))]
compile_error!("feature \"2d\" and feature \"3d\" cannot be enabled at the same time");

#[cfg(all(
    feature = "default-collider",
    feature = "f32",
    not(feature = "parry-f32")
))]
compile_error!(
    "feature \"default-collider\" requires the feature \"parry-f32\" when \"f32\" is enabled"
);

#[cfg(all(
    feature = "default-collider",
    feature = "f64",
    not(feature = "parry-f64")
))]
compile_error!(
    "feature \"default-collider\" requires the feature \"parry-f64\" when \"f64\" is enabled"
);

extern crate alloc;

#[cfg(all(feature = "2d", feature = "parry-f32"))]
pub extern crate parry2d as parry;

#[cfg(all(feature = "2d", feature = "parry-f64"))]
pub extern crate parry2d_f64 as parry;

#[cfg(all(feature = "3d", feature = "parry-f32"))]
pub extern crate parry3d as parry;

#[cfg(all(feature = "3d", feature = "parry-f64"))]
pub extern crate parry3d_f64 as parry;

#[cfg(all(
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
pub mod character_controller;
pub mod collision;
#[cfg(feature = "debug-plugin")]
pub mod debug_render;
pub mod diagnostics;
pub mod dynamics;
pub mod interpolation;
pub mod math;
pub mod physics_transform;
#[cfg(feature = "bevy_picking")]
pub mod picking;
pub mod schedule;
pub mod spatial_query;

pub mod data_structures;

// TODO: Where should this go?
pub(crate) mod ancestor_marker;

/// Re-exports common components, bundles, resources, plugins and types.
pub mod prelude {
    #[cfg(feature = "debug-plugin")]
    pub use crate::debug_render::*;
    #[cfg(feature = "bevy_diagnostic")]
    pub use crate::diagnostics::PhysicsDiagnosticsPlugin;
    #[cfg(feature = "diagnostic_ui")]
    pub use crate::diagnostics::ui::{PhysicsDiagnosticsUiPlugin, PhysicsDiagnosticsUiSettings};
    #[cfg(feature = "default-collider")]
    pub(crate) use crate::physics_transform::RotationValue;
    #[cfg(feature = "bevy_picking")]
    pub use crate::picking::{
        PhysicsPickable, PhysicsPickingFilter, PhysicsPickingPlugin, PhysicsPickingSettings,
    };
    #[expect(deprecated)]
    pub use crate::{
        PhysicsPlugins,
        collision::prelude::*,
        dynamics::{self, ccd::SpeculativeMargin, prelude::*},
        interpolation::*,
        physics_transform::{PhysicsTransformHelper, PhysicsTransformPlugin, Position, Rotation},
        schedule::{
            Physics, PhysicsSchedule, PhysicsSchedulePlugin, PhysicsSet, PhysicsStepSet,
            PhysicsStepSystems, PhysicsSystems, PhysicsTime, Substeps,
        },
        spatial_query::{self, *},
    };

    #[cfg(all(
        feature = "default-collider",
        any(feature = "parry-f32", feature = "parry-f64")
    ))]
    pub use crate::character_controller::prelude::*;
    pub(crate) use crate::{
        diagnostics::AppDiagnosticsExt,
        math::*,
        physics_transform::{PreSolveDeltaPosition, PreSolveDeltaRotation},
        schedule::TimePrecisionAdjusted,
    };
    pub use avian_derive::*;
}

mod utils;

#[cfg(test)]
mod tests;

use bevy::{
    app::PluginGroupBuilder,
    ecs::{intern::Interned, schedule::ScheduleLabel, system::SystemParamItem},
    prelude::*,
};
#[allow(unused_imports)]
use prelude::*;

/// A plugin group containing Avian's plugins.
///
/// # Plugins
///
/// By default, the following plugins will be added:
///
/// | Plugin                            | Description                                                                                                                                                |
/// | --------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------- |
/// | [`PhysicsSchedulePlugin`]         | Sets up the physics engine by initializing the necessary schedules, sets and resources.                                                                    |
/// | [`ColliderBackendPlugin`]         | Handles generic collider backend logic, like initializing colliders and AABBs and updating related components.                                             |
/// | [`ColliderHierarchyPlugin`]       | Manages [`ColliderOf`] relationships based on the entity hierarchy.                                                                                        |
/// | [`ColliderTransformPlugin`]       | Propagates and updates transforms for colliders.
#[cfg_attr(
    all(feature = "collider-from-mesh", feature = "default-collider"),
    doc = "| [`ColliderCachePlugin`]           | Caches colliders created from meshes. Requires `collider-from-mesh` and `default-collider` features.                                                       |"
)]
/// | [`BroadPhasePlugin`]              | Finds pairs of entities with overlapping [AABBs](ColliderAabb) to reduce the number of potential contacts for the [narrow phase](collision::narrow_phase). |
/// | [`NarrowPhasePlugin`]             | Manages contacts and generates contact constraints.                                                                                                        |
/// | [`SolverPlugins`]                 | A plugin group for the physics solver's plugins. See the plugin group's documentation for more information.                                                |
/// | [`JointPlugin`]                   | A plugin for managing and initializing [joints](dynamics::joints). Does *not* include the actual joint solver.                                             |
/// | [`MassPropertyPlugin`]            | Manages mass properties of dynamic [rigid bodies](RigidBody).                                                                                              |
/// | [`ForcePlugin`]                   | Manages and applies external forces, torques, and acceleration for rigid bodies. See the [module-level documentation](dynamics::rigid_body::forces).       |
/// | [`SpatialQueryPlugin`]            | Handles spatial queries like [raycasting](spatial_query#raycasting) and [shapecasting](spatial_query#shapecasting).                                        |
/// | [`PhysicsInterpolationPlugin`]    | [`Transform`] interpolation and extrapolation for rigid bodies.                                                                                            |
/// | [`PhysicsTransformPlugin`]        | Manages physics transforms and synchronizes them with [`Transform`].                                                                                       |
///
/// Optional additional plugins include:
///
/// | Plugin                            | Description                                                                                                                                                |
/// | --------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------- |
/// | [`PhysicsPickingPlugin`]          | Enables a physics picking backend for [`bevy_picking`](bevy::picking) (only with `bevy_picking` feature enabled).                                          |
/// | [`PhysicsDebugPlugin`]            | Renders physics objects and events like [AABBs](ColliderAabb) and contacts for debugging purposes (only with `debug-plugin` feature enabled).              |
/// | [`PhysicsDiagnosticsPlugin`]      | Writes [physics diagnostics](diagnostics) to the [`DiagnosticsStore`] (only with `bevy_diagnostic` feature enabled).                                       |
/// | [`PhysicsDiagnosticsUiPlugin`]    | Displays [physics diagnostics](diagnostics) with a debug UI overlay (only with `diagnostic_ui` feature enabled).                                           |
///
/// [`DiagnosticsStore`]: bevy::diagnostic::DiagnosticsStore
///
/// Refer to the documentation of the plugins for more information about their responsibilities and implementations.
///
/// # World Scale
///
/// The [`PhysicsLengthUnit`] resource is a units-per-meter scaling factor
/// that adjusts the engine's internal properties to the scale of the world.
/// It is recommended to configure the length unit to match the approximate length
/// of the average dynamic object in the world to get the best simulation results.
///
/// For example, a 2D game might use pixels as units and have an average object size
/// of around 100 pixels. By setting the length unit to `100.0`, the physics engine
/// will interpret 100 pixels as 1 meter for internal thresholds, improving stability.
///
/// The length unit can be set by inserting the resource like normal,
/// but it can also be specified through the [`PhysicsPlugins`] plugin group.
///
/// ```no_run
/// # #[cfg(feature = "2d")]
/// use avian2d::prelude::*;
/// use bevy::prelude::*;
///
/// # #[cfg(feature = "2d")]
/// fn main() {
///     App::new()
///         .add_plugins((
///             DefaultPlugins,
///             // A 2D game with 100 pixels per meter
///             PhysicsPlugins::default().with_length_unit(100.0),
///         ))
///         .run();
/// }
/// # #[cfg(not(feature = "2d"))]
/// # fn main() {} // Doc test needs main
/// ```
///
/// # Custom Schedule
///
/// You can run the [`PhysicsSchedule`] in any schedule you want by specifying the schedule when adding the plugin group:
///
/// ```no_run
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// fn main() {
///     App::new()
///         // Run physics at a variable timestep in `PostUpdate`.
///         .add_plugins((DefaultPlugins, PhysicsPlugins::new(PostUpdate)))
///         .run();
/// }
/// ```
///
/// # Custom Plugins
///
/// First, create a new plugin. If you want to run your systems in the engine's schedules, get either the [`PhysicsSchedule`]
/// or the [`SubstepSchedule`]. Then you can add your systems to that schedule and control system ordering with system sets like
/// [`PhysicsStepSystems`], [`SolverSystems`], or [`SubstepSolverSystems`](dynamics::solver::schedule::SubstepSolverSystems).
///
/// Here we will create a custom broad phase plugin that will replace the default [`BroadPhasePlugin`]:
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// pub struct CustomBroadPhasePlugin;
///
/// impl Plugin for CustomBroadPhasePlugin {
///     fn build(&self, app: &mut App) {
///         // Make sure the PhysicsSchedule is available
///         let physics_schedule = app
///             .get_schedule_mut(PhysicsSchedule)
///             .expect("add PhysicsSchedule first");
///
///         // Add the system into the broad phase system set
///         physics_schedule.add_systems(collect_collision_pairs.in_set(PhysicsStepSystems::BroadPhase));
///     }
/// }
///
/// fn collect_collision_pairs() {
///     // Implementation goes here
/// }
/// ```
///
/// Next, when creating your app, simply disable the default [`BroadPhasePlugin`] and add your custom plugin:
///
/// ```no_run
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// # struct CustomBroadPhasePlugin;
/// # impl Plugin for CustomBroadPhasePlugin {
/// #     fn build(&self, app: &mut App) {}
/// # }
/// #
/// fn main() {
///     let mut app = App::new();
///
///     app.add_plugins(DefaultPlugins);
///
///     // Add PhysicsPlugins and replace default broad phase with our custom broad phase
///     app.add_plugins(
///         PhysicsPlugins::default()
///             .build()
///             .disable::<BroadPhasePlugin>()
///             .add(CustomBroadPhasePlugin),
///     );
///
///     app.run();
/// }
/// ```
///
/// You can find a full working example
/// [here](https://github.com/avianphysics/avian/blob/main/crates/avian3d/examples/custom_broad_phase.rs).
pub struct PhysicsPlugins {
    schedule: Interned<dyn ScheduleLabel>,
    length_unit: Scalar,
}

impl PhysicsPlugins {
    /// Creates a [`PhysicsPlugins`] plugin group using the given schedule for running the [`PhysicsSchedule`].
    ///
    /// The default schedule is `FixedPostUpdate`.
    pub fn new(schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: schedule.intern(),
            length_unit: 1.0,
        }
    }

    /// Adds the given [`CollisionHooks`] for user-defined contact filtering and modification.
    ///
    /// Returns a [`PhysicsPluginsWithHooks`] plugin group, which wraps the original [`PhysicsPlugins`],
    /// and applies the provided hooks. Only one set of collision hooks can be defined per application.
    pub fn with_collision_hooks<H: CollisionHooks + 'static>(self) -> PhysicsPluginsWithHooks<H>
    where
        for<'w, 's> SystemParamItem<'w, 's, H>: CollisionHooks,
    {
        PhysicsPluginsWithHooks::<H> {
            plugins: self,
            _phantom: core::marker::PhantomData,
        }
    }

    /// Sets the value used for the [`PhysicsLengthUnit`], a units-per-meter scaling factor
    /// that adjusts the engine's internal properties to the scale of the world.
    ///
    /// For example, a 2D game might use pixels as units and have an average object size
    /// of around 100 pixels. By setting the length unit to `100.0`, the physics engine
    /// will interpret 100 pixels as 1 meter for internal thresholds, improving stability.
    ///
    /// Note that this is *not* used to scale forces or any other user-facing inputs or outputs.
    /// Instead, the value is only used to scale some internal length-based tolerances, such as
    /// [`SleepingThreshold::linear`] and [`NarrowPhaseConfig::default_speculative_margin`],
    /// as well as the scale used for [debug rendering](PhysicsDebugPlugin).
    ///
    /// Choosing the appropriate length unit can help improve stability and robustness.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # #[cfg(feature = "2d")]
    /// use avian2d::prelude::*;
    /// use bevy::prelude::*;
    ///
    /// # #[cfg(feature = "2d")]
    /// fn main() {
    ///     App::new()
    ///         .add_plugins((
    ///             DefaultPlugins,
    ///             // A 2D game with 100 pixels per meter
    ///             PhysicsPlugins::default().with_length_unit(100.0),
    ///         ))
    ///         .run();
    /// }
    /// # #[cfg(not(feature = "2d"))]
    /// # fn main() {} // Doc test needs main
    /// ```
    pub fn with_length_unit(mut self, unit: Scalar) -> Self {
        self.length_unit = unit;
        self
    }
}

impl Default for PhysicsPlugins {
    fn default() -> Self {
        Self::new(FixedPostUpdate)
    }
}

impl PluginGroup for PhysicsPlugins {
    fn build(self) -> PluginGroupBuilder {
        let builder = PluginGroupBuilder::start::<Self>()
            .add(PhysicsSchedulePlugin::new(self.schedule))
            .add(MassPropertyPlugin::new(self.schedule))
            .add(ForcePlugin)
            .add(ColliderHierarchyPlugin)
            .add(ColliderTransformPlugin::new(self.schedule));

        #[cfg(all(feature = "collider-from-mesh", feature = "default-collider"))]
        let builder = builder.add(ColliderCachePlugin);

        #[cfg(all(
            feature = "default-collider",
            any(feature = "parry-f32", feature = "parry-f64")
        ))]
        let builder = builder
            .add(ColliderBackendPlugin::<Collider>::new(self.schedule))
            .add(NarrowPhasePlugin::<Collider>::default());

        // Add solver plugins.
        let builder = builder.add_group(SolverPlugins::new_with_length_unit(self.length_unit));

        builder
            .add(BroadPhasePlugin::<()>::default())
            .add(JointPlugin)
            .add(SpatialQueryPlugin)
            .add(PhysicsTransformPlugin::new(self.schedule))
            .add(PhysicsInterpolationPlugin::default())
    }
}

// This type is separate from `PhysicsPlugins` to avoid requiring users to use generics
// like `PhysicsPlugins::<()>::default()` unless they actually want to use collision hooks.
/// A [`PhysicsPlugins`] plugin group with [`CollisionHooks`] specified.
pub struct PhysicsPluginsWithHooks<H: CollisionHooks> {
    plugins: PhysicsPlugins,
    _phantom: core::marker::PhantomData<H>,
}

impl<H: CollisionHooks> PhysicsPluginsWithHooks<H> {
    /// Creates a new [`PhysicsPluginsWithHooks`] plugin group using the given [`CollisionHooks`]
    /// and schedule for running the [`PhysicsSchedule`].
    ///
    /// The default schedule is [`FixedPostUpdate`].
    pub fn new(schedule: impl ScheduleLabel) -> Self {
        Self {
            plugins: PhysicsPlugins::new(schedule),
            _phantom: core::marker::PhantomData,
        }
    }

    /// Sets the value used for the [`PhysicsLengthUnit`], a units-per-meter scaling factor
    /// that adjusts the engine's internal properties to the scale of the world.
    ///
    /// See [`PhysicsPlugins::with_length_unit`] for more information.
    pub fn with_length_unit(mut self, unit: Scalar) -> Self {
        self.plugins.length_unit = unit;
        self
    }
}

impl<H: CollisionHooks> Default for PhysicsPluginsWithHooks<H> {
    fn default() -> Self {
        Self {
            plugins: PhysicsPlugins::default(),
            _phantom: core::marker::PhantomData,
        }
    }
}

impl<H: CollisionHooks + 'static> PluginGroup for PhysicsPluginsWithHooks<H>
where
    for<'w, 's> SystemParamItem<'w, 's, H>: CollisionHooks,
{
    fn build(self) -> PluginGroupBuilder {
        // Replace the default collision hooks with the user-defined ones.
        let builder = self
            .plugins
            .build()
            .disable::<BroadPhasePlugin>()
            .add(BroadPhasePlugin::<H>::default());

        #[cfg(all(
            feature = "default-collider",
            any(feature = "parry-f32", feature = "parry-f64")
        ))]
        let builder = builder
            .disable::<NarrowPhasePlugin<Collider>>()
            .add(NarrowPhasePlugin::<Collider, H>::default());

        builder
    }
}
