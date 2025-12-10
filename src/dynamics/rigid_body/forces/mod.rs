//! External forces, impulses, and acceleration for dynamic [rigid bodies](RigidBody).
//!
//! # Overview
//!
//! In addition to [`Gravity`], it is possible to apply your own forces, impulses, and acceleration
//! to dynamic rigid bodies to simulate various effects such as force fields, motors, and thrusters.
//! They have the following relationships:
//!
//! | Type             | Formula      | Relation to Velocity | Unit        |
//! | ---------------- | ------------ | -------------------- | ----------- |
//! | **Force**        | `F = m * a`  | `Δa = F / m`         | kg⋅m/s² (N) |
//! | **Impulse**      | `J = F * Δt` | `Δv = J / m`         | kg⋅m/s (N⋅s) |
//! | **Acceleration** | `a = F / m`  | `Δv = a * Δt`        | m/s²        |
//!
//! A force applies an acceleration to a body, which in turn modifies its velocity over time,
//! while an impulse applies an immediate change in velocity. Both forces and impulses consider [mass properties],
//! while acceleration by itself is independent of mass.
//!
//! The rotational equivalents are torques, angular impulses, and angular acceleration, which work similarly.
//!
//! [mass properties]: crate::dynamics::rigid_body::mass_properties
//!
//! ## Constant Forces
//!
//! Constant forces and accelerations that persist across time steps can be applied using the following components:
//!
//! - [`ConstantForce`]: Applies a constant force in world space.
//! - [`ConstantTorque`]: Applies a constant torque in world space.
//! - [`ConstantLinearAcceleration`]: Applies a constant linear acceleration in world space.
//! - [`ConstantAngularAcceleration`]: Applies a constant angular acceleration in world space.
//!
//! They also have local space equivalents:
//!
//! - [`ConstantLocalForce`]: Applies a constant force in local space.
#![cfg_attr(
    feature = "3d",
    doc = "- [`ConstantLocalTorque`]: Applies a constant torque in local space."
)]
//! - [`ConstantLocalLinearAcceleration`]: Applies a constant linear acceleration in local space.
#![cfg_attr(
    feature = "3d",
    doc = "- [`ConstantLocalAngularAcceleration`]: Applies a constant angular acceleration in local space."
)]
//!
//! These components are useful for simulating continuously applied forces that are expected
//! to remain the same across time steps, such as per-body gravity or force fields.
//!
//! You can use constant forces by adding the components to your entities:
//!
//! ```
#![cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
//! # use bevy::prelude::*;
//! #
//! # fn setup(mut commands: Commands) {
//! commands.spawn((
//!     RigidBody::Dynamic,
//!     Collider::capsule(0.5, 1.0),
//!     // Apply a constant force of 10 N in the positive Y direction.
#![cfg_attr(feature = "2d", doc = "    ConstantForce::new(0.0, 10.0),")]
#![cfg_attr(feature = "3d", doc = "    ConstantForce::new(0.0, 10.0, 0.0),")]
//! ));
//! # }
//! ```
//!
//! The forces are only constant in the sense that they persist across time steps.
//! They can still be modified in systems like normal.
//!
//! ## One-Time Forces and Impulses
//!
//! It is common to apply many individual forces and impulses to dynamic rigid bodies,
//! and to clear them afterwards. This can be done using the [`Forces`] helper [`QueryData`](bevy::ecs::query::QueryData).
//!
//! To use [`Forces`], add it to a [`Query`] (without `&` or `&mut`), and use the associated methods
//! to apply forces, impulses, and accelerations to the rigid bodies.
//!
//! ```
#![cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
//! # use bevy::prelude::*;
//! #
//! # #[cfg(feature = "f32")]
//! fn apply_forces(mut query: Query<Forces>) {
//!     for mut forces in &mut query {
//!         // Apply a force of 10 N in the positive Y direction to the entity.
#![cfg_attr(
    feature = "2d",
    doc = "        forces.apply_force(Vec2::new(0.0, 10.0));"
)]
#![cfg_attr(
    feature = "3d",
    doc = "        forces.apply_force(Vec3::new(0.0, 10.0, 0.0));"
)]
//!     }
//! }
//! ```
//!
//! The force is applied continuously during the physics step, and cleared automatically after the step is complete.
//!
//! By default, applying forces to [sleeping](Sleeping) bodies will wake them up. If this is not desired,
//! the [`non_waking`](ForcesItem::non_waking) method can be used to fetch a [`NonWakingForcesItem`]
//! that allows applying forces to a body without waking it up.
//!
//! ```
#![cfg_attr(feature = "2d", doc = "# use avian2d::{math::Vector, prelude::*};")]
#![cfg_attr(feature = "3d", doc = "# use avian3d::{math::Vector, prelude::*};")]
//! # use bevy::prelude::*;
//! #
//! # fn apply_forces(mut query: Query<Forces>) {
//! #     for mut forces in &mut query {
//! #         let force = Vector::default();
//! // Apply a force without waking up the body if it is sleeping.
//! forces.non_waking().apply_force(force);
//! #     }
//! # }
//! ```
//!
//! [`Forces`] can also apply forces and impulses at a specific point in the world. If the point is not aligned
//! with the [center of mass](CenterOfMass), it will apply a torque to the body.
//!
//! ```
#![cfg_attr(feature = "2d", doc = "# use avian2d::{math::Vector, prelude::*};")]
#![cfg_attr(feature = "3d", doc = "# use avian3d::{math::Vector, prelude::*};")]
//! # use bevy::prelude::*;
//! #
//! # fn apply_impulses(mut query: Query<Forces>) {
//! #     for mut forces in &mut query {
//! #         let force = Vector::default();
//! #         let point = Vector::default();
//! // Apply an impulse at a specific point in the world.
//! // Unlike forces, impulses are applied immediately to the velocity.
//! forces.apply_linear_impulse_at_point(force, point);
//! #     }
//! # }
//! ```
//!
//! As an example, you could implement radial gravity that pulls rigid bodies towards the world origin
//! with a system like the following:
//!
//! ```
#![cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
//! # use bevy::prelude::*;
//! #
//! # #[cfg(feature = "f32")]
//! fn radial_gravity(mut query: Query<(Forces, &GlobalTransform)>) {
//!     for (mut forces, global_transform) in &mut query {
//!         // Compute the direction towards the center of the world.
//!         let direction = -global_transform.translation().normalize_or_zero();
//!         // Apply a linear acceleration of 9.81 m/s² towards the center of the world.
#![cfg_attr(
    feature = "2d",
    doc = "        forces.apply_linear_acceleration(direction.truncate() * 9.81);"
)]
#![cfg_attr(
    feature = "3d",
    doc = "        forces.apply_linear_acceleration(direction * 9.81);"
)]
//!     }
//! }
//! ```
//!
//! # Applying Forces vs. Modifying Velocity
//!
//! It is possible to achieve similar effects by directly modifying the [`LinearVelocity`]
//! or [`AngularVelocity`] components instead of using the force APIs. For example, you could
//! implement gravity by simply modifying the velocity of the rigid bodies in a system:
//!
//! ```
#![cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
//! # use bevy::prelude::*;
//! #
//! // In `FixedUpdate`
//! # #[cfg(feature = "f32")]
//! fn gravity(mut bodies: Query<&mut LinearVelocity>, time: Res<Time>) {
//!     for mut velocity in &mut bodies {
//!         // Apply a constant acceleration of 9.81 m/s² in the negative Y direction.
//!         velocity.y -= 9.81 * time.delta_secs();
//!     }
//! }
//! ```
//!
//! However, using the force APIs has several advantages:
//!
//! - Forces and accelerations are spread out over several [substeps](SubstepCount),
//!   which provides higher simulation fidelity than modifying velocity once per time step.
//! - Forces and impulses consider mass properties and delta time for you, simplifying code.
//! - The force APIs make it straightforward to apply forces at specific points in the world.
//!
//! Modifying velocity directly can still be useful and simpler in some cases,
//! and the difference in simulation fidelity tends to be small for most applications.
//! Notably, it is currently not possible to apply forces to kinematic bodies, as they have infinite mass.
//!
//! Still, for convenience and best results, it is generally recommended to use the force APIs
//! for most cases where you want to apply forces, impulses, or acceleration to dynamic rigid bodies.

mod plugin;
mod query_data;
#[cfg(test)]
mod tests;

pub use plugin::{ForcePlugin, ForceSystems};
pub use query_data::{
    Forces, ForcesItem, NonWakingForcesItem, ReadRigidBodyForces, RigidBodyForces,
    WriteRigidBodyForces,
};

use crate::prelude::*;
use bevy::prelude::*;

#[cfg(feature = "2d")]
pub(crate) trait FloatZero {
    const ZERO: Self;
}

#[cfg(feature = "2d")]
impl FloatZero for Scalar {
    const ZERO: Self = 0.0;
}

/// A component for applying a constant force to a dynamic rigid body in world space.
/// The unit is typically N or kg⋅m/s².
///
/// The force persists across time steps, and is accumulated with other forces.
/// This can be useful for simulating constant forces like gravity, force fields, wind,
/// or other environmental effects.
///
/// See the [module-level documentation](self) for more general information about forces in Avian.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// commands.spawn((
///     RigidBody::Dynamic,
///     Collider::capsule(0.5, 1.0),
///     // Apply a constant force of 10 N in the positive Y direction.
#[cfg_attr(feature = "2d", doc = "    ConstantForce::new(0.0, 10.0),")]
#[cfg_attr(feature = "3d", doc = "    ConstantForce::new(0.0, 10.0, 0.0),")]
/// ));
/// # }
/// ```
///
/// # Related Types
///
/// - [`Forces`]: A helper [`QueryData`](bevy::ecs::query::QueryData) for applying forces, impulses, and acceleration to entities.
/// - [`ConstantLocalForce`]: Applies a constant force in local space.
/// - [`ConstantTorque`]: Applies a constant torque in world space.
/// - [`ConstantLinearAcceleration`]: Applies a constant linear acceleration in world space.
/// - [`ConstantAngularAcceleration`]: Applies a constant angular acceleration in world space.
#[derive(Component, Clone, Debug, Default, Deref, DerefMut, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
pub struct ConstantForce(pub Vector);

impl ConstantForce {
    /// Creates a new [`ConstantForce`] with the given `x` and `y` components.
    #[cfg(feature = "2d")]
    pub fn new(x: Scalar, y: Scalar) -> Self {
        Self(Vector::new(x, y))
    }

    /// Creates a new [`ConstantForce`] with the given `x`, `y`, and `z` components.
    #[cfg(feature = "3d")]
    pub fn new(x: Scalar, y: Scalar, z: Scalar) -> Self {
        Self(Vector::new(x, y, z))
    }
}

/// A component for applying a constant torque to a dynamic rigid body in world space.
/// The unit is typically N⋅m or kg⋅m²/s².
///
/// The torque persists across time steps, and is accumulated with other torques.
/// This can be useful for simulating constant torques like the torque from a motor,
/// or other rotational effects.
///
/// See the [module-level documentation](self) for more general information about forces in Avian.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// commands.spawn((
///     RigidBody::Dynamic,
///     Collider::capsule(0.5, 1.0),
///     // Apply a constant torque of 5 N⋅m in the positive Z direction.
#[cfg_attr(feature = "2d", doc = "    ConstantTorque(5.0),")]
#[cfg_attr(feature = "3d", doc = "    ConstantTorque::new(0.0, 0.0, 5.0),")]
/// ));
/// # }
/// ```
///
/// # Related Types
///
/// - [`Forces`]: A helper [`QueryData`](bevy::ecs::query::QueryData) for applying forces, impulses, and acceleration to entities.
#[cfg_attr(
    feature = "3d",
    doc = "- [`ConstantLocalTorque`]: Applies a constant torque in local space."
)]
/// - [`ConstantForce`]: Applies a constant force in world space.
/// - [`ConstantLinearAcceleration`]: Applies a constant linear acceleration in world space.
/// - [`ConstantAngularAcceleration`]: Applies a constant angular acceleration in world space.
#[derive(Component, Clone, Debug, Default, Deref, DerefMut, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
pub struct ConstantTorque(pub AngularVector);

#[cfg(feature = "3d")]
impl ConstantTorque {
    /// Creates a new [`ConstantTorque`] with the given `x`, `y`, and `z` components.
    pub fn new(x: Scalar, y: Scalar, z: Scalar) -> Self {
        Self(Vector::new(x, y, z))
    }
}

/// A component for applying a constant force to a dynamic rigid body in local space.
/// The unit is typically N or kg⋅m/s².
///
/// The force persists across time steps, and is accumulated with other forces.
/// This can be useful for simulating constant forces like rocket thrust,
/// or other local force effects.
///
/// See the [module-level documentation](self) for more general information about forces in Avian.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// commands.spawn((
///     RigidBody::Dynamic,
///     Collider::capsule(0.5, 1.0),
///     // Apply a constant force of 10 N in the positive Y direction in local space.
#[cfg_attr(feature = "2d", doc = "    ConstantLocalForce::new(0.0, 10.0),")]
#[cfg_attr(feature = "3d", doc = "    ConstantLocalForce::new(0.0, 10.0, 0.0),")]
/// ));
/// # }
/// ```
///
/// # Related Types
///
/// - [`Forces`]: A helper [`QueryData`](bevy::ecs::query::QueryData) for applying forces, impulses, and acceleration to entities.
/// - [`ConstantForce`]: Applies a constant force in world space.
#[cfg_attr(
    feature = "3d",
    doc = "- [`ConstantLocalTorque`]: Applies a constant torque in local space."
)]
/// - [`ConstantLocalLinearAcceleration`]: Applies a constant linear acceleration in local space.
#[cfg_attr(
    feature = "3d",
    doc = "- [`ConstantLocalAngularAcceleration`]: Applies a constant angular acceleration in local space."
)]
#[derive(Component, Clone, Debug, Default, Deref, DerefMut, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
pub struct ConstantLocalForce(pub Vector);

impl ConstantLocalForce {
    /// Creates a new [`ConstantLocalForce`] with the given `x` and `y` components.
    #[cfg(feature = "2d")]
    pub fn new(x: Scalar, y: Scalar) -> Self {
        Self(Vector::new(x, y))
    }

    /// Creates a new [`ConstantLocalForce`] with the given `x`, `y`, and `z` components.
    #[cfg(feature = "3d")]
    pub fn new(x: Scalar, y: Scalar, z: Scalar) -> Self {
        Self(Vector::new(x, y, z))
    }
}

/// A component for applying a constant torque to a dynamic rigid body in local space.
/// The unit is typically N⋅m or kg⋅m²/s²
///
/// The torque persists across time steps, and is accumulated with other torques.
/// This can be useful for simulating constant torques like torque from a motor,
/// or other rotational effects in local space.
///
/// See the [module-level documentation](self) for more general information about forces in Avian.
///
/// # Example
///
/// ```
/// # use avian3d::prelude::*;
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// commands.spawn((
///     RigidBody::Dynamic,
///     Collider::capsule(0.5, 1.0),
///     // Apply a constant torque of 5 N⋅m in the positive Z direction in local space.
///     ConstantLocalTorque::new(0.0, 0.0, 5.0),
/// ));
/// # }
/// ```
///
/// # Related Types
///
/// - [`Forces`]: A helper [`QueryData`](bevy::ecs::query::QueryData) for applying forces, impulses, and acceleration to entities.
/// - [`ConstantTorque`]: Applies a constant torque in world space.
/// - [`ConstantLocalForce`]: Applies a constant force in local space.
/// - [`ConstantLocalLinearAcceleration`]: Applies a constant linear acceleration in local space.
/// - [`ConstantLocalAngularAcceleration`]: Applies a constant angular acceleration in local space.
#[derive(Component, Clone, Debug, Default, Deref, DerefMut, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
#[cfg(feature = "3d")]
pub struct ConstantLocalTorque(pub AngularVector);

#[cfg(feature = "3d")]
impl ConstantLocalTorque {
    /// Creates a new [`ConstantLocalTorque`] with the given `x`, `y`, and `z` components.
    pub fn new(x: Scalar, y: Scalar, z: Scalar) -> Self {
        Self(Vector::new(x, y, z))
    }
}

/// A component for applying a constant linear acceleration to a dynamic rigid body in world space.
/// The unit is typically m/s².
///
/// The acceleration persists across time steps, and is accumulated with other accelerations.
/// This can be useful for simulating constant accelerations like per-body gravity.
///
/// See the [module-level documentation](self) for more general information about forces in Avian.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// commands.spawn((
///     RigidBody::Dynamic,
///     Collider::capsule(0.5, 1.0),
///     // Apply a constant linear acceleration of 9.81 m/s² in the negative Y direction.
///     // This is equivalent to using the `Gravity` resource, but only for this entity.
#[cfg_attr(
    feature = "2d",
    doc = "    ConstantLinearAcceleration::new(0.0, -9.81),"
)]
#[cfg_attr(
    feature = "3d",
    doc = "    ConstantLinearAcceleration::new(0.0, -9.81, 0.0),"
)]
/// ));
/// # }
/// ```
///
/// # Related Types
///
/// - [`Forces`]: A helper [`QueryData`](bevy::ecs::query::QueryData) for applying forces, impulses, and acceleration to entities.
/// - [`ConstantLocalLinearAcceleration`]: Applies a constant linear acceleration in local space.
/// - [`ConstantForce`]: Applies a constant force in world space.
/// - [`ConstantTorque`]: Applies a constant torque in world space.
/// - [`ConstantAngularAcceleration`]: Applies a constant angular acceleration in world space.
#[derive(Component, Clone, Debug, Default, Deref, DerefMut, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
pub struct ConstantLinearAcceleration(pub Vector);

impl ConstantLinearAcceleration {
    /// Creates a new [`ConstantLinearAcceleration`] with the given `x` and `y` components.
    #[cfg(feature = "2d")]
    pub fn new(x: Scalar, y: Scalar) -> Self {
        Self(Vector::new(x, y))
    }

    /// Creates a new [`ConstantLinearAcceleration`] with the given `x`, `y`, and `z` components.
    #[cfg(feature = "3d")]
    pub fn new(x: Scalar, y: Scalar, z: Scalar) -> Self {
        Self(Vector::new(x, y, z))
    }
}

/// A component for applying a constant angular acceleration to a dynamic rigid body in world space.
/// The unit is typically rad/s².
///
/// The acceleration persists across time steps, and is accumulated with other accelerations.
/// This can be useful for simulating constant angular accelerations like rotation from a motor,
/// or other rotational effects.
///
/// See the [module-level documentation](self) for more general information about forces in Avian.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// commands.spawn((
///     RigidBody::Dynamic,
///     Collider::capsule(0.5, 1.0),
///     // Apply a constant angular acceleration of 1.0 rad/s² in the positive Z direction.
#[cfg_attr(feature = "2d", doc = "    ConstantAngularAcceleration(1.0),")]
#[cfg_attr(
    feature = "3d",
    doc = "    ConstantAngularAcceleration::new(0.0, 0.0, 1.0),"
)]
/// ));
/// # }
/// ```
///
/// # Related Types
///
/// - [`Forces`]: A helper [`QueryData`](bevy::ecs::query::QueryData) for applying forces, impulses, and acceleration to entities.
#[cfg_attr(
    feature = "3d",
    doc = "- [`ConstantLocalAngularAcceleration`]: Applies a constant angular acceleration in local space."
)]
/// - [`ConstantForce`]: Applies a constant force in world space.
/// - [`ConstantTorque`]: Applies a constant torque in world space.
/// - [`ConstantLinearAcceleration`]: Applies a constant linear acceleration in world space.
#[derive(Component, Clone, Debug, Default, Deref, DerefMut, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
pub struct ConstantAngularAcceleration(pub AngularVector);

#[cfg(feature = "3d")]
impl ConstantAngularAcceleration {
    /// Creates a new [`ConstantAngularAcceleration`] with the given `x`, `y`, and `z` components.
    pub fn new(x: Scalar, y: Scalar, z: Scalar) -> Self {
        Self(Vector::new(x, y, z))
    }
}

/// A component for applying a constant linear acceleration to a dynamic rigid body in local space.
/// The unit is typically m/s².
///
/// The acceleration persists across time steps, and is accumulated with other accelerations.
/// This can be useful for simulating constant linear accelerations like rocket thrust,
/// or other local acceleration effects.
///
/// See the [module-level documentation](self) for more general information about forces in Avian.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// commands.spawn((
///     RigidBody::Dynamic,
///     Collider::capsule(0.5, 1.0),
///     // Apply a constant linear acceleration of 10.0 m/s² in the positive Y direction in local space.
#[cfg_attr(
    feature = "2d",
    doc = "    ConstantLocalLinearAcceleration::new(0.0, 10.0),"
)]
#[cfg_attr(
    feature = "3d",
    doc = "    ConstantLocalLinearAcceleration::new(0.0, 10.0, 0.0),"
)]
/// ));
/// # }
/// ```
///
/// # Related Types
///
/// - [`Forces`]: A helper [`QueryData`](bevy::ecs::query::QueryData) for applying forces, impulses, and acceleration to entities.
/// - [`ConstantLinearAcceleration`]: Applies a constant linear acceleration in world space.
/// - [`ConstantLocalForce`]: Applies a constant force in local space.
#[cfg_attr(
    feature = "3d",
    doc = "- [`ConstantLocalTorque`]: Applies a constant torque in local space."
)]
#[cfg_attr(
    feature = "3d",
    doc = "- [`ConstantLocalAngularAcceleration`]: Applies a constant angular acceleration in local space."
)]
#[derive(Component, Clone, Debug, Default, Deref, DerefMut, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
pub struct ConstantLocalLinearAcceleration(pub Vector);

impl ConstantLocalLinearAcceleration {
    /// Creates a new [`ConstantLocalLinearAcceleration`] with the given `x` and `y` components.
    #[cfg(feature = "2d")]
    pub fn new(x: Scalar, y: Scalar) -> Self {
        Self(Vector::new(x, y))
    }

    /// Creates a new [`ConstantLocalLinearAcceleration`] with the given `x`, `y`, and `z` components.
    #[cfg(feature = "3d")]
    pub fn new(x: Scalar, y: Scalar, z: Scalar) -> Self {
        Self(Vector::new(x, y, z))
    }
}

/// A component for applying a constant angular acceleration to a dynamic rigid body in local space.
/// The unit is typically rad/s².
///
/// The acceleration persists across time steps, and is accumulated with other accelerations.
/// This can be useful for simulating constant angular accelerations like rotation from a motor,
/// or other rotational effects in local space.
///
/// See the [module-level documentation](self) for more general information about forces in Avian.
///
/// # Example
///
/// ```
/// # use avian3d::prelude::*;
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// commands.spawn((
///     RigidBody::Dynamic,
///     Collider::capsule(0.5, 1.0),
///     // Apply a constant angular acceleration of 1.0 rad/s² in the positive Z direction in local space.
///     ConstantLocalAngularAcceleration::new(0.0, 0.0, 1.0),
/// ));
/// # }
/// ```
///
/// # Related Types
///
/// - [`Forces`]: A helper [`QueryData`](bevy::ecs::query::QueryData) for applying forces, impulses, and acceleration to entities.
/// - [`ConstantAngularAcceleration`]: Applies a constant angular acceleration in world space.
/// - [`ConstantLocalForce`]: Applies a constant force in local space.
/// - [`ConstantLocalTorque`]: Applies a constant torque in local space.
/// - [`ConstantLocalLinearAcceleration`]: Applies a constant linear acceleration in local space.
#[derive(Component, Clone, Debug, Default, Deref, DerefMut, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
#[cfg(feature = "3d")]
pub struct ConstantLocalAngularAcceleration(pub AngularVector);

#[cfg(feature = "3d")]
impl ConstantLocalAngularAcceleration {
    /// Creates a new [`ConstantLocalAngularAcceleration`] with the given `x`, `y`, and `z` components.
    pub fn new(x: Scalar, y: Scalar, z: Scalar) -> Self {
        Self(Vector::new(x, y, z))
    }
}

/// A component with the user-applied local acceleration
/// accumulated for a rigid body before the physics step.
#[derive(Component, Clone, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
pub struct AccumulatedLocalAcceleration {
    /// The accumulated linear acceleration in local space.
    pub linear: Vector,
    /// The accumulated angular acceleration in local space.
    #[cfg(feature = "3d")]
    pub angular: Vector,
}
