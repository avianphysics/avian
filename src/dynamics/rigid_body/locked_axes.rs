use bevy::prelude::*;
use derive_more::From;

use crate::prelude::*;

/// A component that specifies which translational and rotational axes of a [rigid body](RigidBody) are locked.
///
/// The axes are represented using a total of six bits, one for each axis. The easiest way to lock or unlock
/// specific axes is to use methods like [`lock_translation_x`](Self::lock_translation_x), but you can also
/// use bits directly with the [`from_bits`](Self::from_bits) and [`to_bits`](Self::to_bits) methods.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// fn spawn(mut commands: Commands) {
///     commands.spawn((
///         RigidBody::Dynamic,
///         Collider::capsule(0.5, 1.0),
#[cfg_attr(feature = "2d", doc = "        LockedAxes::ROTATION_LOCKED,")]
#[cfg_attr(feature = "3d", doc = "        LockedAxes::new().lock_rotation_z(),")]
///     ));
/// }
/// ```
#[derive(Component, Reflect, Clone, Copy, Debug, Default, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default)]
pub struct LockedAxes(u8);

impl LockedAxes {
    /// All translational axes are locked, but all rotational axes are unlocked.
    pub const TRANSLATION_LOCKED: Self = Self(0b111_000);
    /// All rotational axes are locked, but all translational axes are unlocked.
    pub const ROTATION_LOCKED: Self = Self(0b000_111);
    /// All translational and rotational axes are locked.
    pub const ALL_LOCKED: Self = Self(0b111_111);

    /// Creates a new [`LockedAxes`] configuration with all axes unlocked by default.
    pub const fn new() -> Self {
        Self(0)
    }

    /// Creates a new [`LockedAxes`] configuration using bits.
    ///
    /// The first three bits correspond to translational axes, while the last three bits correspond to rotational
    /// axes. For example, `0b100_010` would lock translation along the `X` axis and rotation around the `Y` axis.
    pub const fn from_bits(bits: u8) -> Self {
        Self(bits)
    }

    /// Returns the locked axes as bits.
    ///
    /// The first three bits correspond to translational axes, while the last three bits correspond to rotational
    /// axes. For example, `0b100_010` would mean that translation along the `X` axis and rotation around the `Y` axis
    /// are locked.
    pub const fn to_bits(&self) -> u8 {
        self.0
    }

    /// Locks translation along the `X` axis.
    #[must_use]
    pub const fn lock_translation_x(mut self) -> Self {
        self.0 |= 0b100_000;
        self
    }

    /// Locks translation along the `Y` axis.
    #[must_use]
    pub const fn lock_translation_y(mut self) -> Self {
        self.0 |= 0b010_000;
        self
    }

    /// Locks translation along the `Z` axis.
    #[cfg(feature = "3d")]
    #[must_use]
    pub const fn lock_translation_z(mut self) -> Self {
        self.0 |= 0b001_000;
        self
    }

    /// Locks rotation around the `X` axis.
    #[cfg(feature = "3d")]
    pub const fn lock_rotation_x(mut self) -> Self {
        self.0 |= 0b000_100;
        self
    }

    /// Locks rotation around the `Y` axis.
    #[cfg(feature = "3d")]
    #[must_use]
    pub const fn lock_rotation_y(mut self) -> Self {
        self.0 |= 0b000_010;
        self
    }

    /// Locks rotation around the `Z` axis.
    #[cfg(feature = "3d")]
    #[must_use]
    pub const fn lock_rotation_z(mut self) -> Self {
        self.0 |= 0b000_001;
        self
    }

    /// Locks all rotation.
    #[cfg(feature = "2d")]
    #[must_use]
    pub const fn lock_rotation(mut self) -> Self {
        self.0 |= 0b000_001;
        self
    }

    /// Unlocks translation along the `X` axis.
    #[must_use]
    pub const fn unlock_translation_x(mut self) -> Self {
        self.0 &= !0b100_000;
        self
    }

    /// Unlocks translation along the `Y` axis.
    #[must_use]
    pub const fn unlock_translation_y(mut self) -> Self {
        self.0 &= !0b010_000;
        self
    }

    /// Unlocks translation along the `Z` axis.
    #[cfg(feature = "3d")]
    #[must_use]
    pub const fn unlock_translation_z(mut self) -> Self {
        self.0 &= !0b001_000;
        self
    }

    /// Unlocks rotation around the `X` axis.
    #[cfg(feature = "3d")]
    #[must_use]
    pub const fn unlock_rotation_x(mut self) -> Self {
        self.0 &= !0b000_100;
        self
    }

    /// Unlocks rotation around the `Y` axis.
    #[cfg(feature = "3d")]
    #[must_use]
    pub const fn unlock_rotation_y(mut self) -> Self {
        self.0 &= !0b000_010;
        self
    }

    /// Unlocks rotation around the `Z` axis.
    #[cfg(feature = "3d")]
    #[must_use]
    pub const fn unlock_rotation_z(mut self) -> Self {
        self.0 &= !0b000_001;
        self
    }

    /// Unlocks all rotation.
    #[cfg(feature = "2d")]
    #[must_use]
    pub const fn unlock_rotation(mut self) -> Self {
        self.0 &= !0b000_001;
        self
    }

    /// Returns true if translation is locked along the `X` axis.
    pub const fn is_translation_x_locked(&self) -> bool {
        (self.0 & 0b100_000) != 0
    }

    /// Returns true if translation is locked along the `X` axis.
    pub const fn is_translation_y_locked(&self) -> bool {
        (self.0 & 0b010_000) != 0
    }

    /// Returns true if translation is locked along the `X` axis.
    #[cfg(feature = "3d")]
    pub const fn is_translation_z_locked(&self) -> bool {
        (self.0 & 0b001_000) != 0
    }

    /// Returns true if all translation is locked.
    #[cfg(feature = "2d")]
    pub const fn is_translation_locked(&self) -> bool {
        (self.0 & 0b110_000) == 0b110_000
    }

    /// Returns true if all translation is locked.
    #[cfg(feature = "3d")]
    pub const fn is_translation_locked(&self) -> bool {
        (self.0 & 0b111_000) == 0b111_000
    }

    /// Returns true if rotation is locked around the `X` axis.
    #[cfg(feature = "3d")]
    pub const fn is_rotation_x_locked(&self) -> bool {
        (self.0 & 0b000_100) != 0
    }

    /// Returns true if rotation is locked around the `Y` axis.
    #[cfg(feature = "3d")]
    pub const fn is_rotation_y_locked(&self) -> bool {
        (self.0 & 0b000_010) != 0
    }

    /// Returns true if rotation is locked around the `Z` axis.
    #[cfg(feature = "3d")]
    pub const fn is_rotation_z_locked(&self) -> bool {
        (self.0 & 0b000_001) != 0
    }

    /// Returns true if all rotation is locked.
    #[cfg(feature = "2d")]
    pub const fn is_rotation_locked(&self) -> bool {
        (self.0 & 0b000_001) != 0
    }

    /// Returns true if all rotation is locked.
    #[cfg(feature = "3d")]
    pub const fn is_rotation_locked(&self) -> bool {
        (self.0 & 0b000_111) == 0b000_111
    }

    /// Sets translational axes of the given vector to zero based on the [`LockedAxes`] configuration.
    pub(crate) fn apply_to_vec(&self, mut vector: Vector) -> Vector {
        if self.is_translation_x_locked() {
            vector.x = 0.0;
        }
        if self.is_translation_y_locked() {
            vector.y = 0.0;
        }
        #[cfg(feature = "3d")]
        if self.is_translation_z_locked() {
            vector.z = 0.0;
        }
        vector
    }

    /// Sets the given angular inertia to zero if rotational axes are locked.
    #[cfg(feature = "2d")]
    pub(crate) fn apply_to_angular_inertia(
        &self,
        angular_inertia: impl Into<ComputedAngularInertia>,
    ) -> ComputedAngularInertia {
        let mut angular_inertia = angular_inertia.into();
        let angular_inertia_mut = angular_inertia.inverse_mut();
        if self.is_rotation_locked() {
            *angular_inertia_mut = 0.0;
        }
        angular_inertia
    }

    /// Sets axes of the given angular inertia to zero based on the [`LockedAxes`] configuration.
    #[cfg(feature = "3d")]
    pub(crate) fn apply_to_angular_inertia(
        &self,
        angular_inertia: impl Into<ComputedAngularInertia>,
    ) -> ComputedAngularInertia {
        let mut angular_inertia = angular_inertia.into();
        let angular_inertia_mut = angular_inertia.inverse_mut();
        if self.is_rotation_x_locked() {
            angular_inertia_mut.m00 = 0.0;
            angular_inertia_mut.m01 = 0.0;
            angular_inertia_mut.m02 = 0.0;
        }
        if self.is_rotation_y_locked() {
            angular_inertia_mut.m11 = 0.0;
            angular_inertia_mut.m01 = 0.0;
            angular_inertia_mut.m12 = 0.0;
        }
        if self.is_rotation_z_locked() {
            angular_inertia_mut.m22 = 0.0;
            angular_inertia_mut.m02 = 0.0;
            angular_inertia_mut.m12 = 0.0;
        }
        angular_inertia
    }

    /// Sets the given angular velocity to zero if rotational axes are locked.
    #[cfg(feature = "2d")]
    pub(crate) fn apply_to_angular_velocity(&self, mut angular_velocity: Scalar) -> Scalar {
        if self.is_rotation_locked() {
            angular_velocity = 0.0;
        }
        angular_velocity
    }

    /// Sets axes of the given angular velocity to zero based on the [`LockedAxes`] configuration.
    #[cfg(feature = "3d")]
    pub(crate) fn apply_to_angular_velocity(&self, mut angular_velocity: Vector) -> Vector {
        if self.is_rotation_x_locked() {
            angular_velocity.x = 0.0;
        }
        if self.is_rotation_y_locked() {
            angular_velocity.y = 0.0;
        }
        if self.is_rotation_z_locked() {
            angular_velocity.z = 0.0;
        }
        angular_velocity
    }
}
