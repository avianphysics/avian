//! Efficient rigid body definitions used by the performance-critical solver.
//!
//! This helps improve memory locality and makes random access faster for the constraint solver.
//!
//! This includes the following types:
//!
//! - [`SolverBody`]: The body state used by the solver.
//! - [`SolverBodyInertia`]: The inertial properties of a body used by the solver.
//! - [`SolverBodyIndex`]: A component storing the index of a body in the [`SolverBodies`] resource.
//! - [`SolverBodies`]: A resource storing [`SolverBody`]s contiguously for awake, active bodies.

mod plugin;

pub use plugin::SolverBodyPlugin;

use core::marker::PhantomData;

use bevy::prelude::*;

use super::Rot;
#[cfg(feature = "3d")]
use crate::prelude::ComputedAngularInertia;
use crate::{SymmetricTensor, math::Vector, prelude::LockedAxes};

// The `SolverBody` layout is inspired by `b2BodyState` in Box2D v3.

/// Optimized rigid body state that the solver operates on,
/// designed to improve memory locality and performance.
///
/// Only awake dynamic bodies and kinematic bodies have an associated solver body,
/// stored contiguously in the [`SolverBodies`] resource and indexed by a [`SolverBodyIndex`]
/// component on the body entity. Static bodies and sleeping dynamic bodies do not move,
/// so they instead use a "dummy state" with [`SolverBody::default()`].
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
/// and some bitflags. This all fits in 32 bytes in 2D or 56 bytes in 3D.
///
/// The 2D data layout has been designed to support fast conversion to and from
/// wide SIMD types via scatter/gather operations in the future when SIMD optimizations
/// are implemented.
// TODO: Is there a better layout for 3D?
#[derive(Clone, Debug, Default, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug)]
pub struct SolverBody {
    /// The linear velocity of the body.
    ///
    /// 8 bytes in 2D and 12 bytes in 3D.
    pub linear_velocity: Vector,
    /// The angular velocity of the body.
    ///
    /// 4 bytes in 2D and 12 bytes in 3D.
    #[cfg(feature = "2d")]
    pub angular_velocity: f32,
    /// The angular velocity of the body.
    ///
    /// 8 bytes in 2D and 12 bytes in 3D.
    #[cfg(feature = "3d")]
    pub angular_velocity: Vector,
    /// The change in position of the body.
    ///
    /// Stored as a delta to avoid round-off error when far from the origin.
    ///
    /// 8 bytes in 2D and 12 bytes in 3D.
    pub delta_position: Vector,
    /// The change in rotation of the body.
    ///
    /// Stored as a delta because the rotation of static bodies cannot be accessed
    /// in the solver, but they have a known delta rotation of zero.
    ///
    /// 8 bytes in 2D and 16 bytes in 3D.
    pub delta_rotation: Rot,
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
        delta_rotation: Rot::IDENTITY,
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
        /// Set if the body has a custom position integration implementation.
        const CUSTOM_POSITION_INTEGRATION = 1 << 8;
        /// Set during the continuous collision stage if the body moved fast enough
        /// to be treated as a "fast body" and have its motion swept. Transient.
        const IS_FAST = 1 << 9;
        /// Set during the continuous collision stage if the body's motion was stopped
        /// at a time of impact. Transient.
        const HAD_TIME_OF_IMPACT = 1 << 10;
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
/// 16 bytes in 2D and 32 bytes in 3D.
#[derive(Clone, Debug, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug)]
pub struct SolverBodyInertia {
    /// The effective inverse mass of the body,
    /// taking into account any locked axes.
    ///
    /// 8 bytes.
    #[cfg(feature = "2d")]
    effective_inv_mass: Vector,

    /// The inverse mass of the body.
    ///
    /// 4 bytes.
    #[cfg(feature = "3d")]
    inv_mass: f32,

    /// The effective inverse angular inertia of the body,
    /// taking into account any locked axes.
    ///
    /// 4 bytes.
    #[cfg(feature = "2d")]
    effective_inv_angular_inertia: SymmetricTensor,

    /// The world-space inverse angular inertia of the body.
    ///
    /// 32 bytes.
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
        inv_mass: f32,
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
        inv_mass: f32,
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
        rotation: Quat,
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

/// A component storing the index of an entity's [`SolverBody`] in the [`SolverBodies`] resource.
///
/// Only awake dynamic and kinematic bodies have an associated solver body.
/// Static bodies and sleeping dynamic bodies do not have a solver body, and instead
/// use a "dummy state" with [`SolverBody::default()`] and [`SolverBodyInertia::default()`].
///
/// This component is added, removed, and updated automatically by the solver plugin,
/// and should not be modified by users.
#[derive(Component, Clone, Copy, Debug, Deref, PartialEq, Eq, PartialOrd, Ord, Hash, Reflect)]
#[reflect(Component, Debug, PartialEq, Hash)]
pub struct SolverBodyIndex(pub u32);

impl SolverBodyIndex {
    /// An invalid index used to indicate that a body has no associated [`SolverBody`],
    /// such as a static body. The solver substitutes a dummy state in this case.
    pub const INVALID: Self = Self(u32::MAX);

    /// Returns `true` if the index refers to a valid [`SolverBody`] in the [`SolverBodies`] resource.
    #[inline]
    pub fn is_valid(&self) -> bool {
        self.0 != u32::MAX
    }
}

impl Default for SolverBodyIndex {
    fn default() -> Self {
        Self::INVALID
    }
}

/// A resource storing [solver bodies](SolverBody) and their [inertial properties](SolverBodyInertia)
/// contiguously for awake, active rigid bodies.
///
/// Each entity with a solver body stores an index into this resource in a [`SolverBodyIndex`] component.
#[derive(Resource, Default)]
pub struct SolverBodies {
    // TODO: Use `UniqueEntityVec`.
    entities: Vec<Entity>,
    bodies: Vec<SolverBody>,
    inertias: Vec<SolverBodyInertia>,
}

impl SolverBodies {
    /// Creates a new empty collection of solver bodies.
    #[inline]
    pub const fn new() -> Self {
        Self {
            entities: Vec::new(),
            bodies: Vec::new(),
            inertias: Vec::new(),
        }
    }

    /// Returns the number of solver bodies.
    #[inline]
    pub fn len(&self) -> usize {
        self.bodies.len()
    }

    /// Returns `true` if there are no solver bodies.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.bodies.is_empty()
    }

    /// Returns a slice of the entities that have solver bodies.
    #[inline]
    pub fn entities(&self) -> &[Entity] {
        &self.entities
    }

    /// Returns a slice of the solver bodies.
    #[inline]
    pub fn bodies(&self) -> &[SolverBody] {
        &self.bodies
    }

    /// Returns a mutable slice of the solver bodies.
    #[inline]
    pub fn bodies_mut(&mut self) -> &mut [SolverBody] {
        &mut self.bodies
    }

    /// Returns a slice of the solver body inertias.
    #[inline]
    pub fn inertias(&self) -> &[SolverBodyInertia] {
        &self.inertias
    }

    /// Returns a mutable slice of the solver body inertias.
    #[inline]
    pub fn inertias_mut(&mut self) -> &mut [SolverBodyInertia] {
        &mut self.inertias
    }

    /// Returns `true` if the given [`SolverBodyIndex`] refers to a body in this collection.
    #[inline]
    pub fn contains_index(&self, index: SolverBodyIndex) -> bool {
        (index.0 as usize) < self.bodies.len()
    }

    /// Returns `true` if the given entity has a solver body in this collection.
    #[inline]
    pub fn contains_entity(&self, entity: Entity) -> bool {
        self.entities.contains(&entity)
    }

    /// Returns the [`Entity`] associated with the given solver body index, if any.
    #[inline]
    pub fn get_entity(&self, index: SolverBodyIndex) -> Option<Entity> {
        self.entities.get(index.0 as usize).copied()
    }

    /// Returns a reference to the [`SolverBody`] with the given index, if it exists.
    #[inline]
    pub fn get(&self, index: SolverBodyIndex) -> Option<&SolverBody> {
        self.bodies.get(index.0 as usize)
    }

    /// Returns a mutable reference to the [`SolverBody`] with the given index, if it exists.
    #[inline]
    pub fn get_mut(&mut self, index: SolverBodyIndex) -> Option<&mut SolverBody> {
        self.bodies.get_mut(index.0 as usize)
    }

    /// Returns a reference to the [`SolverBodyInertia`] with the given index, if it exists.
    #[inline]
    pub fn get_inertia(&self, index: SolverBodyIndex) -> Option<&SolverBodyInertia> {
        self.inertias.get(index.0 as usize)
    }

    /// Returns a mutable reference to the [`SolverBodyInertia`] with the given index, if it exists.
    #[inline]
    pub fn get_inertia_mut(&mut self, index: SolverBodyIndex) -> Option<&mut SolverBodyInertia> {
        self.inertias.get_mut(index.0 as usize)
    }

    /// Returns an iterator over mutable references to the solver bodies.
    #[inline]
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut SolverBody> {
        self.bodies.iter_mut()
    }

    /// Adds a new solver body for the given entity, returning its [`SolverBodyIndex`].
    #[inline]
    pub fn push(
        &mut self,
        entity: Entity,
        body: SolverBody,
        inertia: SolverBodyInertia,
    ) -> SolverBodyIndex {
        let index = SolverBodyIndex(self.bodies.len() as u32);
        self.entities.push(entity);
        self.bodies.push(body);
        self.inertias.push(inertia);
        index
    }

    /// Removes the solver body with the given index, swapping in the last body to fill the gap.
    ///
    /// Returns the entity whose body was moved into `index`, so that its [`SolverBodyIndex`]
    /// component can be updated. Returns `None` if the removed body was the last one.
    ///
    /// # Panics
    ///
    /// Panics if the index is out of bounds.
    #[inline]
    pub fn swap_remove(&mut self, index: SolverBodyIndex) -> Option<Entity> {
        self.entities.swap_remove(index.0 as usize);
        self.bodies.swap_remove(index.0 as usize);
        self.inertias.swap_remove(index.0 as usize);
        // TODO: Do this differently
        // If the removed body was not the last one, a body was swapped into its slot.
        self.entities.get(index.0 as usize).copied()
    }

    /// Clears all solver bodies.
    #[inline]
    pub fn clear(&mut self) {
        self.entities.clear();
        self.bodies.clear();
        self.inertias.clear();
    }

    /// Returns a [`SolverBodiesAccess`] that allows obtaining mutable references to solver bodies
    /// from parallel closures via raw pointers.
    ///
    /// This allows the solver to hand out mutable references to disjoint bodies concurrently,
    /// which is sound as long as the caller ensures that no two closures access the same body
    /// at the same time. This is guaranteed by constraint graph coloring.
    #[inline]
    pub fn access(&mut self) -> SolverBodiesAccess<'_> {
        SolverBodiesAccess {
            bodies: self.bodies.as_mut_ptr(),
            inertias: self.inertias.as_mut_ptr(),
            len: self.bodies.len(),
            _marker: PhantomData,
        }
    }
}

/// Mutable disjoint access to the bodies and inertias of a [`SolverBodies`] resource
/// for use inside the parallel solver.
///
/// The caller is responsible for ensuring that no two closures access the same body
/// at the same time. This is guaranteed by constraint graph coloring and each
/// [`SolverBodyIndex`] being unique to a single body.
pub struct SolverBodiesAccess<'a> {
    bodies: *mut SolverBody,
    inertias: *mut SolverBodyInertia,
    len: usize,
    _marker: PhantomData<&'a mut SolverBodies>,
}

// SAFETY: The caller is responsible for only accessing disjoint indices concurrently.
unsafe impl Send for SolverBodiesAccess<'_> {}
// SAFETY: The caller is responsible for only accessing disjoint indices concurrently.
unsafe impl Sync for SolverBodiesAccess<'_> {}

impl SolverBodiesAccess<'_> {
    /// Returns the number of solver bodies.
    #[inline]
    pub fn len(&self) -> usize {
        self.len
    }

    /// Returns `true` if there are no solver bodies.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Returns a mutable reference to the [`SolverBody`] with the given index.
    ///
    /// # Safety
    ///
    /// The index must be in bounds, and no other reference to the same body may be held concurrently.
    #[inline]
    #[expect(clippy::mut_from_ref)]
    pub unsafe fn body_unchecked_mut(&self, index: SolverBodyIndex) -> &mut SolverBody {
        debug_assert!((index.0 as usize) < self.len);
        unsafe { &mut *self.bodies.add(index.0 as usize) }
    }

    /// Returns a mutable reference to the [`SolverBodyInertia`] with the given index.
    ///
    /// # Safety
    ///
    /// The index must be in bounds, and no other reference to the same inertia may be held concurrently.
    #[inline]
    #[expect(clippy::mut_from_ref)]
    pub unsafe fn inertia_unchecked_mut(&self, index: SolverBodyIndex) -> &mut SolverBodyInertia {
        debug_assert!((index.0 as usize) < self.len);
        unsafe { &mut *self.inertias.add(index.0 as usize) }
    }

    /// Returns mutable body references and inertia references for the two given indices.
    ///
    /// An [`SolverBodyIndex::INVALID`] index yields `None` for that body, in which case the caller
    /// should substitute [`SolverBody::DUMMY`] and [`SolverBodyInertia::DUMMY`].
    ///
    /// # Safety
    ///
    /// If both indices are valid, they must be different. Valid indices must be in bounds,
    /// and no other references to the same bodies may be held concurrently.
    #[inline]
    #[expect(clippy::mut_from_ref)]
    pub unsafe fn get_pair_unchecked_mut(
        &self,
        a: SolverBodyIndex,
        b: SolverBodyIndex,
    ) -> (
        Option<(&mut SolverBody, &SolverBodyInertia)>,
        Option<(&mut SolverBody, &SolverBodyInertia)>,
    ) {
        debug_assert!(!a.is_valid() || !b.is_valid() || a != b);
        unsafe {
            let first = a.is_valid().then(|| {
                (
                    &mut *self.bodies.add(a.0 as usize),
                    &*self.inertias.add(a.0 as usize),
                )
            });
            let second = b.is_valid().then(|| {
                (
                    &mut *self.bodies.add(b.0 as usize),
                    &*self.inertias.add(b.0 as usize),
                )
            });
            (first, second)
        }
    }
}
