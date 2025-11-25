//! Efficient rigid body definitions used by the performance-critical solver.
//!
//! This helps improve memory locality and makes random access faster for the constraint solver.
//!
//! This includes the following types:
//!
//! - [`SolverBody`]: The body state used by the solver.
//! - [`SolverBodyInertia`]: The inertial properties of a body used by the solver.

mod plugin;

pub use plugin::SolverBodyPlugin;

use bevy::prelude::*;

use super::{Rotation, Vector};
use crate::{SymmetricTensor, math::Scalar, prelude::LockedAxes};
#[cfg(feature = "3d")]
use crate::{math::Quaternion, prelude::ComputedAngularInertia};

// The `SolverBody` layout is inspired by `b2BodyState` in Box2D v3.

/// Optimized rigid body state that the solver operates on,
/// designed to improve memory locality and performance.
///
/// Only awake dynamic bodies and kinematic bodies have an associated solver body,
/// stored as a component on the body entity. Static bodies and sleeping dynamic bodies
/// do not move, so they instead use a "dummy state" with [`SolverBody::default()`].
///
/// # Representation
///
/// The solver doesn't have access to the position or rotation of static or sleeping bodies,
/// which is a problem when computing constraint anchors. To work around this, we have two options:
///
/// - **Option 1**: Use delta positions and rotations. This requires preparing
///   base anchors and other necessary positional data in world space,
///   and computing the updated anchors during substeps.
/// - **Option 2**: Use full positions and rotations. This requires storing
///   anchors in world space for static bodies and sleeping bodies,
///   and in local space for dynamic bodies.
///
/// Avian uses **Option 1**, because:
///
/// - Using delta positions reduces round-off error when bodies are far from the origin.
/// - Mixing world space and local space values depending on the body type would be
///   quite confusing and error-prone, and would possibly require more branching.
///
/// In addition to the delta position and rotation, we also store the linear and angular velocities
/// and some bitflags. This all fits in 32 bytes in 2D or 56 bytes in 3D with the `f32` feature.
///
/// The 2D data layout has been designed to support fast conversion to and from
/// wide SIMD types via scatter/gather operations in the future when SIMD optimizations
/// are implemented.
// TODO: Is there a better layout for 3D?
#[derive(Component, Clone, Debug, Default, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug)]
pub struct SolverBody {
    /// The linear velocity of the body.
    ///
    /// 8 bytes in 2D and 12 bytes in 3D with the `f32` feature.
    pub linear_velocity: Vector,
    /// The angular velocity of the body.
    ///
    /// 4 bytes in 2D and 12 bytes in 3D with the `f32` feature.
    #[cfg(feature = "2d")]
    pub angular_velocity: Scalar,
    /// The angular velocity of the body.
    ///
    /// 8 bytes in 2D and 12 bytes in 3D with the `f32` feature.
    #[cfg(feature = "3d")]
    pub angular_velocity: Vector,
    /// The change in position of the body.
    ///
    /// Stored as a delta to avoid round-off error when far from the origin.
    ///
    /// 8 bytes in 2D and 12 bytes in 3D with the `f32` feature.
    pub delta_position: Vector,
    /// The change in rotation of the body.
    ///
    /// Stored as a delta because the rotation of static bodies cannot be accessed
    /// in the solver, but they have a known delta rotation of zero.
    ///
    /// 8 bytes in 2D and 16 bytes in 3D with the `f32` feature.
    pub delta_rotation: Rotation,
    /// Flags for the body.
    ///
    /// 4 bytes.
    pub flags: SolverBodyFlags,
}

impl SolverBody {
    /// A dummy [`SolverBody`] for static bodies.
    pub const DUMMY: Self = Self {
        linear_velocity: Vector::ZERO,
        #[cfg(feature = "2d")]
        angular_velocity: 0.0,
        #[cfg(feature = "3d")]
        angular_velocity: Vector::ZERO,
        delta_position: Vector::ZERO,
        delta_rotation: Rotation::IDENTITY,
        flags: SolverBodyFlags::empty(),
    };

    /// Computes the velocity at the given `point` relative to the center of the body.
    pub fn velocity_at_point(&self, point: Vector) -> Vector {
        #[cfg(feature = "2d")]
        {
            self.linear_velocity + self.angular_velocity * point.perp()
        }
        #[cfg(feature = "3d")]
        {
            self.linear_velocity + self.angular_velocity.cross(point)
        }
    }

    /// Returns `true` if gyroscopic motion is enabled for this body.
    pub fn is_gyroscopic(&self) -> bool {
        self.flags.contains(SolverBodyFlags::GYROSCOPIC_MOTION)
    }
}

/// Flags for [`SolverBody`].
#[repr(transparent)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct SolverBodyFlags(u32);

bitflags::bitflags! {
    impl SolverBodyFlags: u32 {
        /// Set if translation along the `X` axis is locked.
        const TRANSLATION_X_LOCKED = 0b100_000;
        /// Set if translation along the `Y` axis is locked.
        const TRANSLATION_Y_LOCKED = 0b010_000;
        /// Set if translation along the `Z` axis is locked.
        const TRANSLATION_Z_LOCKED = 0b001_000;
        /// Set if rotation around the `X` axis is locked.
        const ROTATION_X_LOCKED = 0b000_100;
        /// Set if rotation around the `Y` axis is locked.
        const ROTATION_Y_LOCKED = 0b000_010;
        /// Set if rotation around the `Z` axis is locked.
        const ROTATION_Z_LOCKED = 0b000_001;
        /// Set if all translational axes are locked.
        const TRANSLATION_LOCKED = Self::TRANSLATION_X_LOCKED.bits() | Self::TRANSLATION_Y_LOCKED.bits() | Self::TRANSLATION_Z_LOCKED.bits();
        /// Set if all rotational axes are locked.
        const ROTATION_LOCKED = Self::ROTATION_X_LOCKED.bits() | Self::ROTATION_Y_LOCKED.bits() | Self::ROTATION_Z_LOCKED.bits();
        /// Set if all translational and rotational axes are locked.
        const ALL_LOCKED = Self::TRANSLATION_LOCKED.bits() | Self::ROTATION_LOCKED.bits();
        /// Set if the body is kinematic. Otherwise, it is dynamic.
        const IS_KINEMATIC = 1 << 6;
        /// Set if gyroscopic motion is enabled.
        const GYROSCOPIC_MOTION = 1 << 7;
    }
}

impl SolverBodyFlags {
    /// Returns the [`LockedAxes`] of the body.
    pub fn locked_axes(&self) -> LockedAxes {
        LockedAxes::from_bits(self.0 as u8)
    }

    /// Returns `true` if the body is dynamic.
    pub fn is_dynamic(&self) -> bool {
        !self.contains(SolverBodyFlags::IS_KINEMATIC)
    }

    /// Returns `true` if the body is kinematic.
    pub fn is_kinematic(&self) -> bool {
        self.contains(SolverBodyFlags::IS_KINEMATIC)
    }
}

/*
Box2D v3 stores mass and angular inertia in constraint data.
For 2D, this is just 2 floats or 8 bytes for each body in each constraint.

However, we also support 3D and locking translational axes, so our worst case
would be *9* floats for each body, 3 for the effective mass vector
and 6 for the symmetric 3x3 inertia tensor. Storing 36 bytes
for each body in each constraint would be quite wasteful.

Instead, we store a separate `SolverBodyInertia` struct for each `SolverBody`.
The struct is optimized for memory locality and size.

In 2D, we store the effective inertial properties directly:

- Effective inverse mass (8 bytes)
- Effective inverse angular inertia (4 bytes)
- Flags (4 bytes)

for a total of 16 bytes.

In 3D, we instead compute the effective versions on the fly:

- Inverse mass (4 bytes)
- Inverse angular inertia (36 bytes, matrix with 9 floats)
- Flags (4 bytes)

for a total of 44 bytes. This will be 32 bytes in the future
if/when we switch to a symmetric 3x3 matrix representation.

The API abstracts over this difference in representation to reduce complexity.
*/

/// The inertial properties of a [`SolverBody`].
///
/// This includes the effective inverse mass and angular inertia,
/// and flags indicating whether the body is static or has locked axes.
///
/// 16 bytes in 2D and 32 bytes in 3D with the `f32` feature.
#[derive(Component, Clone, Debug, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug)]
pub struct SolverBodyInertia {
    /// The effective inverse mass of the body,
    /// taking into account any locked axes.
    ///
    /// 8 bytes with the `f32` feature.
    #[cfg(feature = "2d")]
    effective_inv_mass: Vector,

    /// The inverse mass of the body.
    ///
    /// 4 bytes with the `f32` feature.
    #[cfg(feature = "3d")]
    inv_mass: Scalar,

    /// The effective inverse angular inertia of the body,
    /// taking into account any locked axes.
    ///
    /// 4 bytes with the `f32` feature.
    #[cfg(feature = "2d")]
    effective_inv_angular_inertia: SymmetricTensor,

    /// The world-space inverse angular inertia of the body.
    ///
    /// 32 bytes with the `f32` feature.
    #[cfg(feature = "3d")]
    effective_inv_angular_inertia: SymmetricTensor,

    /// The [dominance] of the body.
    ///
    /// If the [`Dominance`] component is not specified, the default of `0` is returned for dynamic bodies.
    /// For static and kinematic bodies, `i8::MAX + 1` (`128`) is always returned instead.
    ///
    /// 2 bytes.
    ///
    /// [dominance]: crate::dynamics::rigid_body::Dominance
    /// [`Dominance`]: crate::dynamics::rigid_body::Dominance
    dominance: i16,

    /// Flags indicating the inertial properties of the body,
    /// like locked axes and whether the body is static.
    ///
    /// 2 bytes.
    flags: InertiaFlags,
}

impl SolverBodyInertia {
    /// A dummy [`SolverBodyInertia`] for static bodies.
    pub const DUMMY: Self = Self {
        #[cfg(feature = "2d")]
        effective_inv_mass: Vector::ZERO,
        #[cfg(feature = "3d")]
        inv_mass: 0.0,
        #[cfg(feature = "2d")]
        effective_inv_angular_inertia: 0.0,
        #[cfg(feature = "3d")]
        effective_inv_angular_inertia: SymmetricTensor::ZERO,
        dominance: i8::MAX as i16 + 1,
        flags: InertiaFlags::STATIC,
    };
}

impl Default for SolverBodyInertia {
    fn default() -> Self {
        Self::DUMMY
    }
}

/// Flags indicating the inertial properties of a body.
#[repr(transparent)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct InertiaFlags(u16);

bitflags::bitflags! {
    impl InertiaFlags: u16 {
        /// Set if translation along the `X` axis is locked.
        const TRANSLATION_X_LOCKED = 0b100_000;
        /// Set if translation along the `Y` axis is locked.
        const TRANSLATION_Y_LOCKED = 0b010_000;
        /// Set if translation along the `Z` axis is locked.
        const TRANSLATION_Z_LOCKED = 0b001_000;
        /// Set if rotation around the `X` axis is locked.
        const ROTATION_X_LOCKED = 0b000_100;
        /// Set if rotation around the `Y` axis is locked.
        const ROTATION_Y_LOCKED = 0b000_010;
        /// Set if rotation around the `Z` axis is locked.
        const ROTATION_Z_LOCKED = 0b000_001;
        /// Set if all translational axes are locked.
        const TRANSLATION_LOCKED = Self::TRANSLATION_X_LOCKED.bits() | Self::TRANSLATION_Y_LOCKED.bits() | Self::TRANSLATION_Z_LOCKED.bits();
        /// Set if all rotational axes are locked.
        const ROTATION_LOCKED = Self::ROTATION_X_LOCKED.bits() | Self::ROTATION_Y_LOCKED.bits() | Self::ROTATION_Z_LOCKED.bits();
        /// Set if all translational and rotational axes are locked.
        const ALL_LOCKED = Self::TRANSLATION_LOCKED.bits() | Self::ROTATION_LOCKED.bits();
        /// Set if the body has infinite mass.
        const INFINITE_MASS = 1 << 6;
        /// Set if the body has infinite inertia.
        const INFINITE_ANGULAR_INERTIA = 1 << 7;
        /// Set if the body is static.
        const STATIC = Self::INFINITE_MASS.bits() | Self::INFINITE_ANGULAR_INERTIA.bits();
    }
}

impl InertiaFlags {
    /// Returns the [`LockedAxes`] of the body.
    pub fn locked_axes(&self) -> LockedAxes {
        LockedAxes::from_bits(self.0 as u8)
    }
}

impl SolverBodyInertia {
    /// Creates a new [`SolverBodyInertia`] with the given mass, angular inertia,
    /// and locked axes.
    #[inline]
    #[cfg(feature = "2d")]
    pub fn new(
        inv_mass: Scalar,
        inv_inertia: SymmetricTensor,
        locked_axes: LockedAxes,
        dominance: i8,
        is_dynamic: bool,
    ) -> Self {
        let mut effective_inv_mass = Vector::splat(inv_mass);
        let mut effective_inv_angular_inertia = inv_inertia;
        let mut flags = InertiaFlags(locked_axes.to_bits() as u16);

        if inv_mass == 0.0 {
            flags |= InertiaFlags::INFINITE_MASS;
        }
        if inv_inertia == 0.0 {
            flags |= InertiaFlags::INFINITE_ANGULAR_INERTIA;
        }

        if locked_axes.is_translation_x_locked() {
            effective_inv_mass.x = 0.0;
        }
        if locked_axes.is_translation_y_locked() {
            effective_inv_mass.y = 0.0;
        }
        if locked_axes.is_rotation_locked() {
            effective_inv_angular_inertia = 0.0;
        }

        Self {
            effective_inv_mass,
            effective_inv_angular_inertia,
            dominance: if is_dynamic {
                dominance as i16
            } else {
                i8::MAX as i16 + 1
            },
            flags: InertiaFlags(flags.0),
        }
    }

    /// Creates a new [`SolverBodyInertia`] with the given mass, angular inertia,
    /// and locked axes.
    #[inline]
    #[cfg(feature = "3d")]
    pub fn new(
        inv_mass: Scalar,
        inv_inertia: SymmetricTensor,
        locked_axes: LockedAxes,
        dominance: i8,
        is_dynamic: bool,
    ) -> Self {
        let mut effective_inv_angular_inertia = inv_inertia;
        let mut flags = InertiaFlags(locked_axes.to_bits() as u16);

        if inv_mass == 0.0 {
            flags |= InertiaFlags::INFINITE_MASS;
        }
        if inv_inertia == SymmetricTensor::ZERO {
            flags |= InertiaFlags::INFINITE_ANGULAR_INERTIA;
        }

        if locked_axes.is_rotation_x_locked() {
            effective_inv_angular_inertia.m00 = 0.0;
            effective_inv_angular_inertia.m01 = 0.0;
            effective_inv_angular_inertia.m02 = 0.0;
        }

        if locked_axes.is_rotation_y_locked() {
            effective_inv_angular_inertia.m01 = 0.0;
            effective_inv_angular_inertia.m11 = 0.0;
            effective_inv_angular_inertia.m12 = 0.0;
        }

        if locked_axes.is_rotation_z_locked() {
            effective_inv_angular_inertia.m02 = 0.0;
            effective_inv_angular_inertia.m12 = 0.0;
            effective_inv_angular_inertia.m22 = 0.0;
        }

        Self {
            inv_mass,
            effective_inv_angular_inertia,
            dominance: if is_dynamic {
                dominance as i16
            } else {
                i8::MAX as i16 + 1
            },
            flags: InertiaFlags(flags.0),
        }
    }

    /// Returns the effective inverse mass of the body,
    /// taking into account any locked axes.
    #[inline]
    #[cfg(feature = "2d")]
    pub fn effective_inv_mass(&self) -> Vector {
        self.effective_inv_mass
    }

    /// Returns the effective inverse mass of the body,
    /// taking into account any locked axes.
    #[inline]
    #[cfg(feature = "3d")]
    pub fn effective_inv_mass(&self) -> Vector {
        let mut inv_mass = Vector::splat(self.inv_mass);

        if self.flags.contains(InertiaFlags::TRANSLATION_X_LOCKED) {
            inv_mass.x = 0.0;
        }
        if self.flags.contains(InertiaFlags::TRANSLATION_Y_LOCKED) {
            inv_mass.y = 0.0;
        }
        if self.flags.contains(InertiaFlags::TRANSLATION_Z_LOCKED) {
            inv_mass.z = 0.0;
        }

        inv_mass
    }

    /// Returns the effective inverse angular inertia of the body,
    /// taking into account any locked axes.
    #[inline]
    #[cfg(feature = "2d")]
    pub fn effective_inv_angular_inertia(&self) -> SymmetricTensor {
        self.effective_inv_angular_inertia
    }

    /// Returns the effective inverse angular inertia of the body in world space,
    /// taking into account any locked axes.
    #[inline]
    #[cfg(feature = "3d")]
    pub fn effective_inv_angular_inertia(&self) -> SymmetricTensor {
        self.effective_inv_angular_inertia
    }

    /// Updates the effective inverse angular inertia of the body in world space,
    /// taking into account any locked axes.
    #[inline]
    #[cfg(feature = "3d")]
    pub fn update_effective_inv_angular_inertia(
        &mut self,
        computed_angular_inertia: &ComputedAngularInertia,
        rotation: Quaternion,
    ) {
        let locked_axes = self.flags.locked_axes();
        let mut effective_inv_angular_inertia =
            computed_angular_inertia.rotated(rotation).inverse();

        if locked_axes.is_rotation_x_locked() {
            effective_inv_angular_inertia.m00 = 0.0;
            effective_inv_angular_inertia.m01 = 0.0;
            effective_inv_angular_inertia.m02 = 0.0;
        }

        if locked_axes.is_rotation_y_locked() {
            effective_inv_angular_inertia.m11 = 0.0;
            effective_inv_angular_inertia.m01 = 0.0;
            effective_inv_angular_inertia.m12 = 0.0;
        }

        if locked_axes.is_rotation_z_locked() {
            effective_inv_angular_inertia.m22 = 0.0;
            effective_inv_angular_inertia.m02 = 0.0;
            effective_inv_angular_inertia.m12 = 0.0;
        }

        self.effective_inv_angular_inertia = effective_inv_angular_inertia;
    }

    /// Returns the [dominance] of the body.
    ///
    /// If the [`Dominance`] component is not specified, the default of `0` is returned for dynamic bodies.
    /// For static and kinematic bodies, `i8::MAX + 1` (`128`) is always returned instead.
    ///
    /// [dominance]: crate::dynamics::rigid_body::Dominance
    /// [`Dominance`]: crate::dynamics::rigid_body::Dominance
    #[inline]
    pub fn dominance(&self) -> i16 {
        self.dominance
    }

    /// Returns the [`InertiaFlags`] of the body.
    #[inline]
    pub fn flags(&self) -> InertiaFlags {
        self.flags
    }
}
