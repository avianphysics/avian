//! Components and resources for [`Sleeping`] rigid bodies to reduce CPU overhead.
//!
//! See the [`Sleeping`] documentation for more information.

use bevy::prelude::*;

/// A marker component indicating that a [`RigidBody`] is sleeping and not simulated until woken up again.
///
/// # What is Sleeping?
///
/// Simulating a large number of bodies can be expensive. To reduce CPU overhead, bodies that come to rest
/// enter a low-cost "sleeping" state where they are not simulated until woken up again.
///
/// To start sleeping, the [`LinearVelocity`] and [`AngularVelocity`] of a body must remain below the [`SleepThreshold`]
/// for a time specified by the [`TimeToSleep`] resource. All bodies that are either directly or indirectly connected
/// to the body through contacts or joints must also be allowed to sleep.
///
/// A body is woken up when any of the following happens:
///
/// - An awake body collides with a sleeping body.
/// - A joint is created between an awake body and a sleeping body.
/// - A joint or contact is removed from a sleeping body.
/// - The [`Transform`], [`LinearVelocity`], or [`AngularVelocity`] of a sleeping body is modified.
/// - The [`RigidBody`] type of a body is changed.
/// - A [constant force component](super::forces#constant-forces) of a sleeping body is modified.
/// - A force, impulse, or acceleration is applied via [`Forces`], without using [`non_waking`].
/// - The [`Gravity`] resource or [`GravityScale`] component is modified.
///
/// A body and all bodies connected to it can also be forced to sleep or wake up
/// by manually adding or removing the [`Sleeping`] component, or by using
/// the [`SleepBody`] and [`WakeBody`] commands.
///
/// Sleeping can be disabled for an entity by adding the [`SleepingDisabled`] component.
///
/// [`RigidBody`]: super::RigidBody
/// [`LinearVelocity`]: super::LinearVelocity
/// [`AngularVelocity`]: super::AngularVelocity
/// [`Forces`]: super::forces::Forces
/// [`non_waking`]: super::forces::ForcesItem::non_waking
/// [`Gravity`]: super::Gravity
/// [`GravityScale`]: super::GravityScale
/// [`SleepBody`]: crate::dynamics::solver::islands::SleepBody
/// [`WakeBody`]: crate::dynamics::solver::islands::WakeBody
///
/// # Implementation Details
///
/// Sleeping in Avian is handled using [simulation islands]. Each island is a collection of bodies
/// that are connected through contacts or joints. An island is only allowed to sleep if all of its bodies are resting,
/// and if any body in a sleeping island is woken up, the entire island is woken up with it.
///
/// The [`IslandPlugin`] is responsible for managing islands, and the [`IslandSleepingPlugin`]
/// is responsible for island sleeping and waking.
///
/// [simulation islands]: crate::dynamics::solver::islands
/// [`IslandPlugin`]: crate::dynamics::solver::islands::IslandPlugin
/// [`IslandSleepingPlugin`]: crate::dynamics::solver::islands::IslandSleepingPlugin
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default)]
pub struct Sleeping;

/// A marker component indicating that [`Sleeping`] is disabled for a [`RigidBody`].
///
/// [`RigidBody`]: super::RigidBody
#[derive(Component, Clone, Copy, Debug, Default, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default)]
pub struct SleepingDisabled;

/// A component for the maximum [`LinearVelocity`] and [`AngularVelocity`]
/// for a body to be allowed to be [`Sleeping`].
///
/// Setting a negative sleeping threshold disables sleeping entirely,
/// similar to [`SleepingDisabled`].
///
/// [`LinearVelocity`]: super::LinearVelocity
/// [`AngularVelocity`]: super::AngularVelocity
#[derive(Component, Clone, Copy, PartialEq, PartialOrd, Debug, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, PartialEq)]
pub struct SleepThreshold {
    /// The maximum linear velocity for the body to be allowed to be [`Sleeping`].
    ///
    /// This is implicitly scaled by the [`PhysicsLengthUnit`].
    ///
    /// Default: `0.15`
    ///
    /// [`PhysicsLengthUnit`]: crate::dynamics::solver::PhysicsLengthUnit
    pub linear: f32,
    /// The maximum angular velocity for the body to be allowed to be [`Sleeping`].
    ///
    /// Default: `0.15`
    pub angular: f32,
}

/// Deprecated alias for [`SleepThreshold`].
#[deprecated(note = "Renamed to `SleepThreshold`")]
pub type SleepingThreshold = SleepThreshold;

impl Default for SleepThreshold {
    fn default() -> Self {
        Self {
            linear: 0.15,
            angular: 0.15,
        }
    }
}

/// A component storing the time in seconds that a [`RigidBody`] has been resting
/// with its [`LinearVelocity`] and [`AngularVelocity`] below the [`SleepThreshold`].
///
/// When this time exceeds the [`TimeToSleep`], the body is allowed to be [`Sleeping`].
///
/// [`RigidBody`]: super::RigidBody
/// [`LinearVelocity`]: super::LinearVelocity
/// [`AngularVelocity`]: super::AngularVelocity
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct SleepTimer(pub f32);

/// Deprecated alias for [`SleepTimer`].
#[deprecated(note = "Renamed to `SleepTimer`")]
pub type TimeSleeping = SleepTimer;

/// A resource that specifies the time in seconds that a [`RigidBody`] must rest
/// with its [`LinearVelocity`] and [`AngularVelocity`] below the [`SleepThreshold`]
/// before it is allowed to be [`Sleeping`].
///
/// Default: `0.5`
///
/// [`RigidBody`]: super::RigidBody
/// [`LinearVelocity`]: super::LinearVelocity
/// [`AngularVelocity`]: super::AngularVelocity
#[derive(Resource, Clone, Copy, Debug, PartialEq, PartialOrd, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Default, PartialEq)]
pub struct TimeToSleep(pub f32);

/// Deprecated alias for [`TimeToSleep`].
#[deprecated(note = "Renamed to `TimeToSleep`")]
pub type DeactivationTime = TimeToSleep;

impl Default for TimeToSleep {
    fn default() -> Self {
        Self(0.5)
    }
}
