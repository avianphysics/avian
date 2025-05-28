//! Forces, torques, linear impulses, and angular impulses
//! that can be applied to dynamic rigid bodies.

use crate::prelude::*;
use bevy::prelude::*;
use core::ops::{Deref, DerefMut};
use derive_more::From;

#[cfg(feature = "2d")]
pub(crate) type Torque = Scalar;

#[cfg(feature = "3d")]
pub(crate) type Torque = Vector;

#[cfg(feature = "2d")]
pub(crate) trait FloatZero {
    const ZERO: Self;
}

#[cfg(feature = "2d")]
impl FloatZero for Scalar {
    const ZERO: Self = 0.0;
}

/// An external force applied continuously to a dynamic [rigid body](RigidBody).
///
/// The force is stored in world space. If you want to apply forces in local space, you need to
/// use the body's rotation to [transform the local force into world space](#local-forces).
///
/// By default, the force persists across frames. You can clear the force manually using
/// [`clear`](Self::clear) or set `persistent` to false.
///
/// # Example
///
/// ```
/// # #[cfg(feature = "2d")]
/// # use avian2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use avian3d::prelude::*;
/// use bevy::prelude::*;
///
/// # #[cfg(all(feature = "3d", feature = "f32"))]
/// fn setup(mut commands: Commands) {
///     // Apply a force in world space every physics frame.
///     commands.spawn((RigidBody::Dynamic, ExternalForce::new(Vec3::Y)));
///
///     // Apply an initial force and automatically clear it every physics frame.
///     commands.spawn((
///         RigidBody::Dynamic,
///         ExternalForce::new(Vec3::Y).with_persistence(false),
///     ));
///
///     // Apply multiple forces.
///     let mut force = ExternalForce::default();
///     force.apply_force(Vec3::Y).apply_force(Vec3::X);
///     commands.spawn((RigidBody::Dynamic, force));
///
///     // Apply a force at a specific point relative to the given center of mass, also applying a torque.
///     // In this case, the torque would cause the body to rotate counterclockwise.
///     let mut force = ExternalForce::default();
///     force.apply_force_at_point(Vec3::Y, Vec3::X, Vec3::ZERO);
///     commands.spawn((RigidBody::Dynamic, force));
/// }
/// ```
///
/// # Local Forces
///
/// The force stored in `ExternalForce` is in world space.
///
/// If you want to apply a force in some direction relative to the body's frame of reference,
/// you need to rotate the force using the body's `Transform` or [`Rotation`].
///
/// ```
/// # #[cfg(feature = "2d")]
/// # use avian2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use avian3d::prelude::*;
/// use bevy::prelude::*;
///
/// # #[cfg(all(feature = "3d", feature = "f32"))]
/// fn setup(mut commands: Commands) {
///     // Spawn a rotated body and apply a force in the local up direction.
///     let transform = Transform::from_rotation(Quat::from_rotation_z(0.2));
///     commands.spawn((
///         RigidBody::Dynamic,
///         ExternalForce::new(transform.rotation * Vec3::Y),
///         transform,
///     ));
/// }
/// ```
///
/// Note that the actual force stored in `ExternalForce` is still in world space.
/// If you want to apply a force in the same local direction every frame,
/// consider setting `persistent` to `false` and running [`apply_force`](Self::apply_force) in a system.
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
pub struct ExternalForce {
    /// The total external force that will be applied.
    force: Vector,
    /// True if the force persists across frames, and false if the force is automatically cleared every physics frame.
    ///
    /// If you clear the force manually, use the [`clear`](Self::clear) method. This will clear the force and
    /// the torque that is applied when the force is not applied at the center of mass.
    pub persistent: bool,
    /// The torque caused by forces applied at certain points using [`apply_force_at_point`](Self::apply_force_at_point).
    torque: Torque,
}

impl Deref for ExternalForce {
    type Target = Vector;

    fn deref(&self) -> &Self::Target {
        &self.force
    }
}

impl DerefMut for ExternalForce {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.force
    }
}

impl Default for ExternalForce {
    fn default() -> Self {
        Self {
            force: Vector::ZERO,
            persistent: true,
            torque: Torque::ZERO,
        }
    }
}

impl ExternalForce {
    /// Zero external force.
    pub const ZERO: Self = Self {
        force: Vector::ZERO,
        persistent: true,
        torque: Torque::ZERO,
    };

    /// Creates a new [`ExternalForce`] component with a given world-space `force`.
    pub fn new(force: Vector) -> Self {
        Self { force, ..default() }
    }

    /// Sets the world-space force. Note that the torque caused by any forces will not be reset.
    pub fn set_force(&mut self, force: Vector) -> &mut Self {
        **self = force;
        self
    }

    /// Adds the given world-space `force` to the force that will be applied.
    pub fn apply_force(&mut self, force: Vector) -> &mut Self {
        **self += force;
        self
    }

    /// Applies the given `force` at the specified `point`, which will also cause torque to be applied.
    ///
    /// The force, point, and center of mass must be given in world space.
    pub fn apply_force_at_point(
        &mut self,
        force: Vector,
        point: Vector,
        center_of_mass: Vector,
    ) -> &mut Self {
        **self += force;
        #[cfg(feature = "2d")]
        {
            self.torque += (point - center_of_mass).perp_dot(force);
        }
        #[cfg(feature = "3d")]
        {
            self.torque += (point - center_of_mass).cross(force);
        }
        self
    }

    /// Returns the force in world space.
    pub fn force(&self) -> Vector {
        self.force
    }

    /// Returns the torque caused by forces applied at certain points using
    /// [`apply_force_at_point`](Self::apply_force_at_point).
    pub fn torque(&self) -> Torque {
        self.torque
    }

    /// Determines if the force is persistent or if it should be automatically cleared every physics frame.
    #[doc(alias = "clear_automatically")]
    pub fn with_persistence(mut self, is_persistent: bool) -> Self {
        self.persistent = is_persistent;
        self
    }

    /// Sets the force and the potential torque caused by the force to zero.
    pub fn clear(&mut self) {
        self.force = Vector::ZERO;
        self.torque = Torque::ZERO;
    }
}

/// An external torque applied continuously to a dynamic [rigid body](RigidBody).
///
/// By default, the torque persists across frames. You can clear the torque manually using
/// [`clear`](Self::clear) or set `persistent` to false.
///
/// # Example
///
/// ```
/// # #[cfg(feature = "2d")]
/// # use avian2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use avian3d::prelude::*;
/// use bevy::prelude::*;
///
/// # #[cfg(all(feature = "3d", feature = "f32"))]
/// fn setup(mut commands: Commands) {
///     // Apply a torque every physics frame.
///     commands.spawn((RigidBody::Dynamic, ExternalTorque::new(Vec3::Y)));
///
///     // Apply an initial torque and automatically clear it every physics frame.
///     commands.spawn((
///         RigidBody::Dynamic,
///         ExternalTorque::new(Vec3::Y).with_persistence(false),
///     ));
///
///     // Apply multiple torques.
///     let mut torque = ExternalTorque::default();
///     torque.apply_torque(Vec3::Y).apply_torque(Vec3::X);
///     commands.spawn((RigidBody::Dynamic, torque));
/// }
/// ```
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
pub struct ExternalTorque {
    /// The total external torque that will be applied.
    torque: Torque,
    /// True if the torque persists across frames, and false if the torque is automatically cleared every physics frame.
    pub persistent: bool,
}

impl Deref for ExternalTorque {
    type Target = Torque;

    fn deref(&self) -> &Self::Target {
        &self.torque
    }
}

impl DerefMut for ExternalTorque {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.torque
    }
}

impl Default for ExternalTorque {
    fn default() -> Self {
        Self {
            torque: Torque::ZERO,
            persistent: true,
        }
    }
}

impl ExternalTorque {
    /// Zero external torque.
    pub const ZERO: Self = Self {
        torque: Torque::ZERO,
        persistent: true,
    };

    /// Creates a new [`ExternalTorque`] component with a given `torque`.
    pub fn new(torque: Torque) -> Self {
        Self {
            torque,
            ..default()
        }
    }

    /// Sets the torque.
    pub fn set_torque(&mut self, torque: Torque) -> &mut Self {
        **self = torque;
        self
    }

    /// Adds the given `torque` to the torque that will be applied.
    pub fn apply_torque(&mut self, torque: Torque) -> &mut Self {
        **self += torque;
        self
    }

    /// Determines if the torque is persistent or if it should be automatically cleared every physics frame.
    #[doc(alias = "clear_automatically")]
    pub fn with_persistence(mut self, is_persistent: bool) -> Self {
        self.persistent = is_persistent;
        self
    }

    /// Returns the torque.
    pub fn torque(&self) -> Torque {
        self.torque
    }

    /// Sets the torque to zero.
    pub fn clear(&mut self) {
        self.torque = Torque::ZERO;
    }
}

/// An external impulse applied instantly to a dynamic [rigid body](RigidBody).
///
/// The impulse is stored in world space. If you want to apply impulses in local space, you need to
/// use the body's rotation to [transform the local impulse into world space](#local-impulses).
///
/// By default, the impulse is cleared every frame. You can set `persistent` to true in order to persist
/// the impulse across frames.
///
/// # Example
///
/// ```
/// # #[cfg(feature = "2d")]
/// # use avian2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use avian3d::prelude::*;
/// use bevy::prelude::*;
///
/// # #[cfg(all(feature = "3d", feature = "f32"))]
/// fn setup(mut commands: Commands) {
///     // Apply an impulse in world space.
///     commands.spawn((RigidBody::Dynamic, ExternalImpulse::new(Vec3::Y)));
///
///     // Apply an impulse every physics frame.
///     commands.spawn((
///         RigidBody::Dynamic,
///         ExternalImpulse::new(Vec3::Y).with_persistence(true),
///     ));
///
///     // Apply multiple impulses.
///     let mut impulse = ExternalImpulse::default();
///     impulse.apply_impulse(Vec3::Y).apply_impulse(Vec3::X);
///     commands.spawn((RigidBody::Dynamic, impulse));
///
///     // Apply an impulse at a specific point relative to the given center of mass, also applying an angular impulse.
///     // In this case, the angular impulse would cause the body to rotate counterclockwise.
///     let mut impulse = ExternalImpulse::default();
///     impulse.apply_impulse_at_point(Vec3::Y, Vec3::X, Vec3::ZERO);
///     commands.spawn((RigidBody::Dynamic, impulse));
/// }
/// ```
///
/// # Local Impulses
///
/// The impulse stored in `ExternalImpulse` is in world space.
///
/// If you want to apply an impulse in some direction relative to the body's frame of reference,
/// you need to rotate the impulse using the body's `Transform` or [`Rotation`].
///
/// ```
/// # #[cfg(feature = "2d")]
/// # use avian2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use avian3d::prelude::*;
/// use bevy::prelude::*;
///
/// # #[cfg(all(feature = "3d", feature = "f32"))]
/// fn setup(mut commands: Commands) {
///     // Spawn a rotated body and apply an impulse in the local up direction.
///     let transform = Transform::from_rotation(Quat::from_rotation_z(0.2));
///     commands.spawn((
///         RigidBody::Dynamic,
///         ExternalImpulse::new(transform.rotation * Vec3::Y),
///         transform,
///     ));
/// }
/// ```
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
pub struct ExternalImpulse {
    /// The total external impulse that will be applied.
    impulse: Vector,
    /// True if the impulse persists across frames, and false if the impulse is automatically cleared every physics frame.
    ///
    /// If you clear the impulse manually, use the [`clear`](Self::clear) method. This will clear the impulse and
    /// the angular impulse that is applied when the impulse is not applied at the center of mass.
    pub persistent: bool,
    /// The angular impulse caused by impulses applied at certain points using [`apply_impulse_at_point`](Self::apply_impulse_at_point).
    angular_impulse: Torque,
}

impl Deref for ExternalImpulse {
    type Target = Vector;

    fn deref(&self) -> &Self::Target {
        &self.impulse
    }
}

impl DerefMut for ExternalImpulse {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.impulse
    }
}

impl Default for ExternalImpulse {
    fn default() -> Self {
        Self {
            impulse: Vector::ZERO,
            persistent: false,
            angular_impulse: Torque::ZERO,
        }
    }
}

impl ExternalImpulse {
    /// Zero external impulse.
    pub const ZERO: Self = Self {
        impulse: Vector::ZERO,
        persistent: false,
        angular_impulse: Torque::ZERO,
    };

    /// Creates a new [`ExternalImpulse`] component with a given world-space `impulse`.
    pub fn new(impulse: Vector) -> Self {
        Self {
            impulse,
            ..default()
        }
    }

    /// Sets the world-space impulse. Note that the angular impulse caused by any impulses will not be reset.
    pub fn set_impulse(&mut self, impulse: Vector) -> &mut Self {
        **self = impulse;
        self
    }

    /// Adds the given world-space `impulse` to the impulse that will be applied.
    pub fn apply_impulse(&mut self, impulse: Vector) -> &mut Self {
        **self += impulse;
        self
    }

    /// Applies the given `impulse` at the specified `point`, which will also cause an angular impulse to be applied.
    ///
    /// The impulse, point, and center of mass must be given in world space.
    pub fn apply_impulse_at_point(
        &mut self,
        impulse: Vector,
        point: Vector,
        center_of_mass: Vector,
    ) -> &mut Self {
        **self += impulse;
        #[cfg(feature = "2d")]
        {
            self.angular_impulse += (point - center_of_mass).perp_dot(impulse);
        }
        #[cfg(feature = "3d")]
        {
            self.angular_impulse += (point - center_of_mass).cross(impulse);
        }
        self
    }

    /// Returns the impulse in world space.
    pub fn impulse(&self) -> Vector {
        self.impulse
    }

    /// Returns the angular impulse caused by impulses applied at certain points using
    /// [`apply_impulse_at_point`](Self::apply_impulse_at_point).
    pub fn angular_impulse(&self) -> Torque {
        self.angular_impulse
    }

    /// Determines if the impulse is persistent or if it should be automatically cleared every physics frame.
    #[doc(alias = "clear_automatically")]
    pub fn with_persistence(mut self, is_persistent: bool) -> Self {
        self.persistent = is_persistent;
        self
    }

    /// Sets the impulse and the potential angular impulse caused by the impulse to zero.
    pub fn clear(&mut self) {
        self.impulse = Vector::ZERO;
        self.angular_impulse = Torque::ZERO;
    }
}

/// An external angular impulse applied instantly to a dynamic [rigid body](RigidBody).
///
/// By default, the angular impulse is cleared every frame. You can set `persistent` to true in order to persist
/// the impulse across frames.
///
/// # Example
///
/// ```
/// # #[cfg(feature = "2d")]
/// # use avian2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use avian3d::prelude::*;
/// use bevy::prelude::*;
///
/// # #[cfg(all(feature = "3d", feature = "f32"))]
/// fn setup(mut commands: Commands) {
///     // Apply an angular impulse.
///     commands.spawn((RigidBody::Dynamic, ExternalAngularImpulse::new(Vec3::Y)));
///
///     // Apply an angular impulse every physics frame.
///     commands.spawn((
///         RigidBody::Dynamic,
///         ExternalAngularImpulse::new(Vec3::Y).with_persistence(false),
///     ));
///
///     // Apply multiple angular impulses.
///     let mut angular_impulse = ExternalAngularImpulse::default();
///     angular_impulse.apply_impulse(Vec3::Y).apply_impulse(Vec3::X);
///     commands.spawn((RigidBody::Dynamic, angular_impulse));
/// }
/// ```
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
#[doc(alias = "ExternalTorqueImpulse")]
pub struct ExternalAngularImpulse {
    /// The total external angular impulse that will be applied.
    impulse: Torque,
    /// True if the angular impulse persists across frames, and false if
    /// the angular impulse is automatically cleared every physics frame.
    pub persistent: bool,
}

impl Deref for ExternalAngularImpulse {
    type Target = Torque;

    fn deref(&self) -> &Self::Target {
        &self.impulse
    }
}

impl DerefMut for ExternalAngularImpulse {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.impulse
    }
}

impl Default for ExternalAngularImpulse {
    fn default() -> Self {
        Self {
            impulse: Torque::ZERO,
            persistent: false,
        }
    }
}

impl ExternalAngularImpulse {
    /// Zero external angular impulse.
    pub const ZERO: Self = Self {
        impulse: Torque::ZERO,
        persistent: false,
    };

    /// Creates a new [`ExternalAngularImpulse`] component with a given `impulse`.
    pub fn new(impulse: Torque) -> Self {
        Self {
            impulse,
            ..default()
        }
    }

    /// Sets the angular impulse.
    pub fn set_impulse(&mut self, impulse: Torque) -> &mut Self {
        **self = impulse;
        self
    }

    /// Adds the given `impulse` to the angular impulse that will be applied.
    pub fn apply_impulse(&mut self, impulse: Torque) -> &mut Self {
        **self += impulse;
        self
    }

    /// Determines if the angular impulse is persistent or if it should be automatically cleared every physics frame.
    #[doc(alias = "clear_automatically")]
    pub fn with_persistence(mut self, is_persistent: bool) -> Self {
        self.persistent = is_persistent;
        self
    }

    /// Returns the angular impulse.
    pub fn impulse(&self) -> Torque {
        self.impulse
    }

    /// Sets the angular impulse to zero.
    pub fn clear(&mut self) {
        self.impulse = Torque::ZERO;
    }
}
