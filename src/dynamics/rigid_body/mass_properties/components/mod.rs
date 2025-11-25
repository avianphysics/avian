//! Mass property components.

use crate::prelude::*;
use bevy::{
    ecs::{lifecycle::HookContext, world::DeferredWorld},
    prelude::*,
};
#[cfg(feature = "3d")]
use bevy_heavy::AngularInertiaTensor;
use derive_more::From;
#[cfg(feature = "3d")]
use glam_matrix_extras::{MatConversionError, SymmetricMat3};

mod collider;
pub use collider::*;

mod computed;
pub use computed::*;

/// An error returned for an invalid mass.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MassError {
    /// The mass is negative.
    Negative,
    /// The mass is NaN.
    NaN,
}

/// The [mass] of an entity, representing resistance to linear acceleration.
/// A higher mass requires more force for the same acceleration.
///
/// If [`Mass`] is not present, but the entity has a [`Collider`], its [`ColliderMassProperties`]
/// computed based on the shape and [`ColliderDensity`] will be used instead.
///
/// The [`Mass`] component does *not* take into account the masses of child entities, and it is never modified
/// by the engine. The total mass of a dynamic [rigid body] that *does* consider child entities and colliders
/// is stored in the [`ComputedMass`] component. It is updated automatically when mass properties are changed,
/// or when colliders are added or removed.
///
/// A total mass of zero is a special case, and is interpreted as infinite mass, meaning the rigid body
/// will not be affected by any forces.
///
/// [mass]: https://en.wikipedia.org/wiki/Mass
/// [rigid body]: RigidBody
///
/// # Usage
///
/// The [`Mass`] component can be used to define the mass of a [rigid body] entity or its descendants:
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// commands.spawn((
///     RigidBody::Dynamic,
///     Collider::capsule(0.5, 1.5),
///     Mass(5.0),
/// ));
/// # }
/// ```
///
/// If no [`Mass`] is present, the [`ComputedMass`] will be computed from the collider
/// based on its shape and [`ColliderDensity`].
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// // Note: `ColliderDensity` is optional, and defaults to `1.0` if not present.
/// commands.spawn((
///     RigidBody::Dynamic,
///     Collider::capsule(0.5, 1.5),
///     ColliderDensity(2.0),
/// ));
/// # }
/// ```
///
/// If the rigid body has child colliders, their masses will be added to the total [`ComputedMass`].
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// // Total mass: 10.0 + 5.0 = 15.0
/// commands.spawn((
///     RigidBody::Dynamic,
///     Collider::capsule(0.5, 1.5),
///     Mass(10.0),
/// ))
#[cfg_attr(
    feature = "2d",
    doc = ".with_child((Collider::circle(1.0), Mass(5.0)));"
)]
#[cfg_attr(
    feature = "3d",
    doc = ".with_child((Collider::sphere(1.0), Mass(5.0)));"
)]
/// # }
/// ```
///
/// To prevent masses of child entities from contributing to the total [`ComputedMass`],
/// add the [`NoAutoMass`] component. This can be useful when full control over mass is desired.
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// // Total mass: 10.0
/// commands.spawn((
///     RigidBody::Dynamic,
///     Collider::capsule(0.5, 1.5),
///     Mass(10.0),
///     NoAutoMass,
/// ))
#[cfg_attr(
    feature = "2d",
    doc = ".with_child((Collider::circle(1.0), Mass(5.0)));"
)]
#[cfg_attr(
    feature = "3d",
    doc = ".with_child((Collider::sphere(1.0), Mass(5.0)));"
)]
/// # }
/// ```
///
/// # Mass Updates
///
/// The [`Mass`] component is never modified by the engine, so you can safely update it at any time.
///
/// The total [`ComputedMass`] is updated automatically whenever the mass properties of a [rigid body]
/// or its descendants change, or when colliders are added or removed. This update is triggered by adding
/// the [`RecomputeMassProperties`] component, which is removed after the update is performed
/// in [`MassPropertySystems::UpdateComputedMassProperties`](super::MassPropertySystems::UpdateComputedMassProperties).
///
/// To immediately perform a manual update of the total mass properties for a specific rigid body entity,
/// you can call [`MassPropertyHelper::update_mass_properties`] in a system.
///
/// # Related Types
///
/// - [`ComputedMass`] stores the total mass of a dynamic [rigid body] that considers child entities and colliders.
/// - [`NoAutoMass`] disables masses of child entities being taken into account for the [`ComputedMass`].
/// - [`AngularInertia`] is the rotational equivalent of mass, representing resistance to angular acceleration.
/// - [`CenterOfMass`] is the local point where the mass is concentrated. Applying forces at this point produces no torque.
/// - [`MassPropertiesBundle`] is a bundle containing mass properties.
/// - [`MassPropertyHelper`] is a [`SystemParam`] with utilities for computing and updating mass properties.
///
/// [`SystemParam`]: bevy::ecs::system::SystemParam
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct Mass(pub f32);

impl Mass {
    /// A mass of `0.0`.
    pub const ZERO: Self = Self(0.0);

    /// Computes the [`Mass`] of the given shape using the given density.
    ///
    /// ```
    #[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
    #[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
    /// # use bevy::prelude::*;
    /// #
    /// // Compute the mass from a collider with a density of `2.0`.
    #[cfg_attr(
        feature = "2d",
        doc = "let mass = Mass::from_shape(&Collider::circle(1.0), 2.0);"
    )]
    #[cfg_attr(
        feature = "3d",
        doc = "let mass = Mass::from_shape(&Collider::sphere(1.0), 2.0);"
    )]
    ///
    /// // Bevy's primitive shapes can also be used.
    #[cfg_attr(
        feature = "2d",
        doc = "let mass = Mass::from_shape(&Circle::new(1.0), 2.0);"
    )]
    #[cfg_attr(
        feature = "3d",
        doc = "let mass = Mass::from_shape(&Sphere::new(1.0), 2.0);"
    )]
    /// ```
    #[inline]
    pub fn from_shape<T: ComputeMassProperties>(shape: &T, density: f32) -> Self {
        Self(shape.mass(density))
    }
}

// TODO: Add errors for asymmetric and non-positive definite matrices in 3D.
/// An error returned for an invalid angular inertia.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum AngularInertiaError {
    /// The angular inertia is negative.
    Negative,
    /// The angular inertia is NaN.
    NaN,
}

/// The [angular inertia] of an entity, representing resistance to angular acceleration.
/// A higher angular inertia requires more torque for the same acceleration.
///
/// If [`AngularInertia`] is not present, but the entity has a [`Collider`], its [`ColliderMassProperties`]
/// computed based on the shape and will be used instead. Angular inertia scales with mass,
/// so a higher [`Mass`] or [`ColliderDensity`] will result in higher inertia.
///
/// The [`AngularInertia`] component does *not* take into account the inertia of child entities, and it is never modified
/// by the engine. The total angular inertia of a dynamic [rigid body] that *does* consider child entities and colliders
/// is stored in the [`ComputedAngularInertia`] component. It is updated automatically when mass properties are changed,
/// or when colliders are added or removed.
///
/// A total angular inertia of zero is a special case, and is interpreted as infinite inertia, meaning the rigid body
/// will not be affected by any torque.
///
/// [angular inertia]: https://en.wikipedia.org/wiki/Moment_of_inertia
/// [rigid body]: RigidBody
///
/// # Usage
///
/// The [`AngularInertia`] component can be used to define the angular inertia of a [rigid body]
/// entity or its descendants:
///
/// ```
/// # use avian2d::prelude::*;
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// commands.spawn((
///     RigidBody::Dynamic,
///     Collider::capsule(0.5, 1.5),
///     AngularInertia(2.0),
/// ));
/// # }
/// ```
///
/// If no [`AngularInertia`] is present, the [`ComputedAngularInertia`] will be computed from the collider
/// based on its mass and shape.
///
/// ```
/// # use avian2d::prelude::*;
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// // Note: `ColliderDensity` is optional, and defaults to `1.0` if not present.
/// commands.spawn((
///     RigidBody::Dynamic,
///     Collider::capsule(0.5, 1.5),
///     ColliderDensity(2.0),
/// ));
/// # }
/// ```
///
/// If the rigid body has child colliders, their angular inertia will be added to the total [`ComputedAngularInertia`].
///
/// ```
/// # use avian2d::prelude::*;
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// // Total angular inertia: 3.0 + 2.0 = 5.0
/// commands.spawn((
///     RigidBody::Dynamic,
///     Collider::capsule(0.5, 1.5),
///     AngularInertia(3.0),
/// ))
/// .with_child((Collider::circle(1.0), AngularInertia(2.0)));
/// # }
/// ```
///
/// To prevent angular inertia of child entities from contributing to the total [`ComputedAngularInertia`],
/// add the [`NoAutoAngularInertia`] component. This can be useful when full control over inertia is desired.
///
/// ```
/// # use avian2d::prelude::*;
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// // Total angular inertia: 3.0
/// commands.spawn((
///     RigidBody::Dynamic,
///     Collider::capsule(0.5, 1.5),
///     AngularInertia(3.0),
///     NoAutoAngularInertia,
/// ))
/// .with_child((Collider::circle(1.0), AngularInertia(2.0)));
/// # }
/// ```
///
/// # Angular Inertia Updates
///
/// The [`AngularInertia`] component is never modified by the engine, so you can safely update it at any time.
///
/// The total [`ComputedAngularInertia`] is updated automatically whenever the mass properties of a [rigid body]
/// or its descendants change, or when colliders are added or removed. This update is triggered by adding
/// the [`RecomputeMassProperties`] component, which is removed after the update is performed
/// in [`MassPropertySystems::UpdateComputedMassProperties`](super::MassPropertySystems::UpdateComputedMassProperties).
///
/// To immediately perform a manual update of the total mass properties for a specific rigid body entity,
/// you can call [`MassPropertyHelper::update_mass_properties`] in a system.
///
/// # Related Types
///
/// - [`ComputedAngularInertia`] stores the total angular inertia of a dynamic [rigid body] that considers child entities and colliders.
/// - [`NoAutoAngularInertia`] disables the mass properties of child entities being taken into account for the [`ComputedAngularInertia`].
/// - [`Mass`] is the linear equivalent of angular inertia, representing resistance to linear acceleration.
/// - [`CenterOfMass`] is the local point where the mass is concentrated. Applying forces at this point produces no torque.
/// - [`MassPropertiesBundle`] is a bundle containing mass properties.
/// - [`MassPropertyHelper`] is a [`SystemParam`] with utilities for computing and updating mass properties.
///
/// [`SystemParam`]: bevy::ecs::system::SystemParam
#[cfg(feature = "2d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
#[doc(alias = "MomentOfInertia")]
pub struct AngularInertia(pub f32);

#[cfg(feature = "2d")]
impl AngularInertia {
    /// An angular inertia of `0.0`.
    pub const ZERO: Self = Self(0.0);

    /// Computes the [`AngularInertia`] of the given shape using the given mass.
    ///
    /// ```
    /// # use avian2d::prelude::*;
    /// # use bevy::prelude::*;
    /// #
    /// // Compute the angular inertia from a collider with a mass of `2.0`.
    /// let inertia = AngularInertia::from_shape(&Collider::circle(1.0), 2.0);
    ///
    /// // Bevy's primitive shapes can also be used.
    /// let inertia = AngularInertia::from_shape(&Circle::new(1.0), 2.0);
    /// ```
    #[inline]
    pub fn from_shape<T: ComputeMassProperties>(shape: &T, mass: f32) -> Self {
        Self(shape.angular_inertia(mass))
    }

    /// Computes the angular inertia shifted by the given offset, taking into account the given mass.
    #[inline]
    pub fn shifted(&self, mass: f32, offset: Vec2) -> f32 {
        if mass > 0.0 && mass.is_finite() && offset != Vec2::ZERO {
            self.0 + offset.length_squared() * mass
        } else {
            self.0
        }
    }
}

/// The local [angular inertia] of an entity, representing resistance to angular acceleration.
/// A higher angular inertia requires more torque for the same acceleration.
///
/// If [`AngularInertia`] is not present, but the entity has a [`Collider`], its [`ColliderMassProperties`]
/// computed based on the shape and [mass] will be used instead. Angular inertia scales with mass,
/// so a higher [`Mass`] or [`ColliderDensity`] will result in higher inertia.
///
/// The [`AngularInertia`] component does *not* take into account the inertia of child entities, and it is never modified
/// by the engine. The total angular inertia of a dynamic [rigid body] that *does* consider child entities and colliders
/// is stored in the [`ComputedAngularInertia`] component. It is updated automatically when mass properties are changed,
/// or when colliders are added or removed.
///
/// A total angular inertia of zero is a special case, and is interpreted as infinite inertia, meaning the rigid body
/// will not be affected by any torques.
///
/// See the [module-level documentation] for more general information on angular inertia.
///
/// [angular inertia]: https://en.wikipedia.org/wiki/Moment_of_inertia
/// [rigid body]: RigidBody
/// [mass]: super#mass
/// [module-level documentation]: super#angular-inertia
///
/// # Usage
///
/// [`AngularInertia`] is defined by the moment of inertia along the *principal axes* (a [`Vec3`])
/// defined by the *local inertial frame* (a [`Quat`]). Most entities will have a local inertial frame
/// of [`Quat::IDENTITY`], which means that the principal axes are aligned with the entity's local axes.
///
/// ```
/// # use avian3d::prelude::*;
/// # use bevy::prelude::*;
/// #
/// # #[cfg(feature = "f32")]
/// # fn setup(mut commands: Commands) {
/// // Principal angular inertia: `2.0` for the local X and Z axes, and `5.0` for the Y axis.
/// let inertia1 = AngularInertia::new(Vec3::new(2.0, 5.0, 2.0));
///
/// // A local inertial frame rotated by 90 degrees about the X axis.
/// let inertia2 = AngularInertia::new_with_local_frame(Vec3::new(2.0, 5.0, 2.0), Quat::from_rotation_x(PI / 2.0));
/// # }
/// # #[cfg(not(feature = "f32"))]
/// # fn main() {}
///
/// ```
///
/// [`AngularInertia`] can also be created from a symmetric 3x3 matrix known as the *[angular inertia tensor]*.
/// It summarizes all moments of inertia of an object in a single matrix, and can be used to perform
/// computations with angular inertia, but is often not constructed manually.
///
/// [angular inertia tensor]: AngularInertiaTensor
///
/// ```
/// # use avian3d::prelude::*;
/// # use bevy::prelude::*;
/// #
/// # #[cfg(feature = "f32")]
/// # fn setup(mut commands: Commands) {
/// // For simplicity, we use the same principal angular inertia as before, with an identity local frame.
/// let inertia1 = AngularInertia::from_tensor(Mat3::from_diagonal(Vec3::new(2.0, 5.0, 2.0)));
///
/// // The angular inertia tensor can be retrieved back from the principal angular inertia.
/// let tensor = inertia1.tensor();
/// # }
/// # #[cfg(not(feature = "f32"))]
/// # fn main() {}
/// ```
///
/// The [`AngularInertia`] component can be used to define the angular inertia of a [rigid body]
/// entity or its descendants:
///
/// ```
/// # use avian3d::prelude::*;
/// # use bevy::prelude::*;
/// #
/// # #[cfg(feature = "f32")]
/// # fn setup(mut commands: Commands) {
/// commands.spawn((
///     RigidBody::Dynamic,
///     Collider::capsule(0.5, 1.5),
///     AngularInertia::new(Vec3::new(2.0, 5.0, 2.0)),
/// ));
/// # }
/// # #[cfg(not(feature = "f32"))]
/// # fn main() {}
/// ```
///
/// If no [`AngularInertia`] is present, the [`ComputedAngularInertia`] will be computed from the collider
/// based on its mass and shape.
///
/// ```
/// # use avian3d::prelude::*;
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// // Note: `ColliderDensity` is optional, and defaults to `1.0` if not present.
/// commands.spawn((
///     RigidBody::Dynamic,
///     Collider::capsule(0.5, 1.5),
///     ColliderDensity(2.0),
/// ));
/// # }
/// ```
///
/// If the rigid body has child colliders, their angular inertia will be added to the total [`ComputedAngularInertia`].
/// Here both entities are at the origin, but offsets relative to the center of mass would also impact the total inertia.
///
/// ```
/// # use avian3d::prelude::*;
/// # use bevy::prelude::*;
/// #
/// # #[cfg(feature = "f32")]
/// # fn setup(mut commands: Commands) {
/// // Total angular inertia: [2.0, 5.0, 2.0] + [1.0, 2.0, 1.0] = [3.0, 7.0, 3.0]
/// commands.spawn((
///     RigidBody::Dynamic,
///     Collider::capsule(0.5, 1.5),
///     AngularInertia::new(Vec3::new(2.0, 5.0, 2.0)),
/// ))
/// .with_child((Collider::sphere(1.0), AngularInertia::new(Vec3::new(1.0, 2.0, 1.0))));
/// # }
/// # #[cfg(not(feature = "f32"))]
/// # fn main() {}
/// ```
///
/// To prevent angular inertia of child entities from contributing to the total [`ComputedAngularInertia`],
/// add the [`NoAutoAngularInertia`] component. This can be useful when full control over inertia is desired.
///
/// ```
/// # use avian3d::prelude::*;
/// # use bevy::prelude::*;
/// #
/// # #[cfg(feature = "f32")]
/// # fn setup(mut commands: Commands) {
/// // Total angular inertia: [2.0, 5.0, 2.0]
/// commands.spawn((
///     RigidBody::Dynamic,
///     Collider::capsule(0.5, 1.5),
///     AngularInertia::new(Vec3::new(2.0, 5.0, 2.0)),
///     NoAutoAngularInertia,
/// ))
/// .with_child((Collider::sphere(1.0), AngularInertia::new(Vec3::new(1.0, 2.0, 1.0))));
/// # }
/// # #[cfg(not(feature = "f32"))]
/// # fn main() {}
/// ```
///
/// # Angular Inertia Updates
///
/// The [`AngularInertia`] component is never modified by the engine, so you can safely update it at any time.
///
/// The total [`ComputedAngularInertia`] is updated automatically whenever the mass properties of a [rigid body]
/// or its descendants change, or when colliders are added or removed. This update is triggered by adding
/// the [`RecomputeMassProperties`] component, which is removed after the update is performed
/// in [`MassPropertySystems::UpdateComputedMassProperties`](super::MassPropertySystems::UpdateComputedMassProperties).
///
/// To immediately perform a manual update of the total mass properties for a specific rigid body entity,
/// you can call [`MassPropertyHelper::update_mass_properties`] in a system.
///
/// # Related Types
///
/// - [`ComputedAngularInertia`] stores the total angular inertia of a dynamic [rigid body] that considers child entities and colliders.
/// - [`NoAutoAngularInertia`] disables the mass properties of child entities being taken into account for the [`ComputedAngularInertia`].
/// - [`AngularInertiaTensor`] is the symmetric 3x3 matrix representation of angular inertia.
/// - [`Mass`] is the linear equivalent of angular inertia, representing resistance to linear acceleration.
/// - [`CenterOfMass`] is the local point where the mass is concentrated. Applying forces at this point produces no torque.
/// - [`MassPropertiesBundle`] is a bundle containing mass properties.
/// - [`MassPropertyHelper`] is a [`SystemParam`] with utilities for computing and updating mass properties.
///
/// [`SystemParam`]: bevy::ecs::system::SystemParam
#[cfg(feature = "3d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
#[doc(alias = "MomentOfInertia")]
pub struct AngularInertia {
    /// The principal angular inertia, representing resistance to angular acceleration
    /// about the local coordinate axes defined by the `local_frame`.
    pub principal: Vec3,
    /// The orientation of the local inertial frame.
    pub local_frame: Quat,
}

#[cfg(feature = "3d")]
impl AngularInertia {
    /// An angular inertia of `0.0` for all axes, with an identity local frame.
    pub const ZERO: Self = Self {
        principal: Vec3::ZERO,
        local_frame: Quat::IDENTITY,
    };

    /// Creates a new [`AngularInertia`] from the given principal angular inertia.
    ///
    /// The principal angular inertia represents resistance to angular acceleration
    /// about the local coordinate axes.
    ///
    /// To specify the orientation of the local inertial frame, consider using [`AngularInertia::new_with_local_frame`].
    ///
    /// # Panics
    ///
    /// Panics if any component of the principal angular inertia is negative or NaN when `debug_assertions` are enabled.
    #[inline]
    #[doc(alias = "from_principal_angular_inertia")]
    pub fn new(principal_angular_inertia: Vec3) -> Self {
        debug_assert!(
            principal_angular_inertia.cmpge(Vec3::ZERO).all()
                && !principal_angular_inertia.is_nan(),
            "principal angular inertia must be positive or zero for all axes"
        );

        Self {
            principal: principal_angular_inertia,
            local_frame: Quat::IDENTITY,
        }
    }

    /// Tries to create a new [`AngularInertia`] from the given principal angular inertia.
    ///
    /// The principal angular inertia represents resistance to angular acceleration
    /// about the local coordinate axes. To specify the orientation of the local inertial frame,
    /// consider using [`AngularInertia::try_new_with_local_frame`].
    ///
    /// # Errors
    ///
    /// Returns [`Err(AngularInertiaError)`](AngularInertiaError) if any component of the principal angular inertia is negative or NaN.
    #[inline]
    pub fn try_new(principal_angular_inertia: Vec3) -> Result<Self, AngularInertiaError> {
        if principal_angular_inertia.is_nan() {
            Err(AngularInertiaError::NaN)
        } else if !principal_angular_inertia.cmpge(Vec3::ZERO).all() {
            Err(AngularInertiaError::Negative)
        } else {
            Ok(Self {
                principal: principal_angular_inertia,
                local_frame: Quat::IDENTITY,
            })
        }
    }

    /// Creates a new [`AngularInertia`] from the given principal angular inertia
    /// and the orientation of the local inertial frame.
    ///
    /// The principal angular inertia represents resistance to angular acceleration
    /// about the local coordinate axes defined by the given `local_frame`.
    ///
    /// # Panics
    ///
    /// Panics if any component of the principal angular inertia is negative or NaN when `debug_assertions` are enabled.
    #[inline]
    #[doc(alias = "from_principal_angular_inertia_with_local_frame")]
    pub fn new_with_local_frame(principal_angular_inertia: Vec3, local_frame: Quat) -> Self {
        debug_assert!(
            principal_angular_inertia.cmpge(Vec3::ZERO).all()
                && !principal_angular_inertia.is_nan(),
            "principal angular inertia must be positive or zero for all axes"
        );

        Self {
            principal: principal_angular_inertia,
            local_frame,
        }
    }

    /// Tries to create a new [`AngularInertia`] from the given principal angular inertia
    /// and the orientation of the local inertial frame.
    ///
    /// The principal angular inertia represents resistance to angular acceleration
    /// about the local coordinate axes defined by the given `local_frame`.
    ///
    /// # Errors
    ///
    /// Returns [`Err(AngularInertiaError)`](AngularInertiaError) if any component of the principal angular inertia is negative or NaN.
    #[inline]
    pub fn try_new_with_local_frame(
        principal_angular_inertia: Vec3,
        local_frame: Quat,
    ) -> Result<Self, AngularInertiaError> {
        if principal_angular_inertia.is_nan() {
            Err(AngularInertiaError::NaN)
        } else if !principal_angular_inertia.cmpge(Vec3::ZERO).all() {
            Err(AngularInertiaError::Negative)
        } else {
            Ok(Self {
                principal: principal_angular_inertia,
                local_frame,
            })
        }
    }

    /// Creates a new [`AngularInertia`] from the given [angular inertia tensor].
    ///
    /// The tensor should be symmetric and positive definite.
    ///
    /// [angular inertia tensor]: AngularInertiaTensor
    #[inline]
    #[doc(alias = "from_mat3")]
    pub fn from_tensor(tensor: impl Into<AngularInertiaTensor>) -> Self {
        let tensor = tensor.into();
        let (principal, local_frame) = tensor.principal_angular_inertia_with_local_frame();

        Self {
            principal,
            local_frame,
        }
    }

    /// Creates a new [`AngularInertiaTensor`] from the given angular inertia [tensor]
    /// represented as a [`SymmetricMat3`].
    ///
    /// The tensor should be [positive-semidefinite], but this is *not* checked.
    ///
    /// [tensor]: https://en.wikipedia.org/wiki/Moment_of_inertia#Inertia_tensor
    /// [positive-semidefinite]: https://en.wikipedia.org/wiki/Definite_matrix
    #[inline]
    #[must_use]
    #[doc(alias = "from_tensor")]
    pub fn from_symmetric_mat3(mat: SymmetricMat3) -> Self {
        Self::from_tensor(AngularInertiaTensor::from_symmetric_mat3(mat))
    }

    /// Tries to create a new [`AngularInertiaTensor`] from the given angular inertia [tensor]
    /// represented as a [`Mat3`].
    ///
    /// The tensor should be [positive-semidefinite], but this is *not* checked.
    ///
    /// [tensor]: https://en.wikipedia.org/wiki/Moment_of_inertia#Inertia_tensor
    /// [positive-semidefinite]: https://en.wikipedia.org/wiki/Definite_matrix
    ///
    /// # Errors
    ///
    /// Returns a [`MatConversionError`] if the given matrix is not symmetric.
    #[inline]
    pub fn try_from_mat3(mat: Mat3) -> Result<Self, MatConversionError> {
        SymmetricMat3::try_from_mat3(mat).map(Self::from_tensor)
    }

    /// Creates a new [`AngularInertiaTensor`] from the given angular inertia [tensor]
    /// represented as a [`Mat3`].
    ///
    /// Only the lower left triangle of the matrix is used. No check is performed to ensure
    /// that the given matrix is truly symmetric or [positive-semidefinite].
    ///
    /// [tensor]: https://en.wikipedia.org/wiki/Moment_of_inertia#Inertia_tensor
    /// [positive-semidefinite]: https://en.wikipedia.org/wiki/Definite_matrix
    #[inline]
    #[must_use]
    pub fn from_mat3_unchecked(mat: Mat3) -> Self {
        Self::from_tensor(SymmetricMat3::from_mat3_unchecked(mat))
    }

    /// Computes the [`AngularInertia`] of the given shape using the given mass.
    ///
    /// ```
    /// # use avian3d::prelude::*;
    /// # use bevy::prelude::*;
    /// #
    /// // Compute the angular inertia from collider with a mass of `2.0`.
    /// let inertia = AngularInertia::from_shape(&Collider::sphere(1.0), 2.0);
    ///
    /// // Bevy's primitive shapes can also be used.
    /// let inertia = AngularInertia::from_shape(&Sphere::new(1.0), 2.0);
    /// ```
    #[inline]
    pub fn from_shape<T: ComputeMassProperties>(shape: &T, mass: f32) -> Self {
        let principal = shape.principal_angular_inertia(mass);
        let local_frame = shape.local_inertial_frame();
        Self::new_with_local_frame(principal, local_frame)
    }

    /// Returns the [`AngularInertiaTensor`] represented by this principal [`AngularInertia`]
    /// and local inertial frame.
    #[inline]
    pub fn tensor(self) -> AngularInertiaTensor {
        AngularInertiaTensor::new_with_local_frame(self.principal, self.local_frame)
    }

    /// Returns `true` if the principal angular inertia and inertial local frame are neither infinite nor NaN.
    #[inline]
    pub fn is_finite(self) -> bool {
        self.principal.is_finite() && self.local_frame.is_finite()
    }

    /// Returns `true` if the principal angular inertia or inertial local frame is NaN.
    #[inline]
    pub fn is_nan(self) -> bool {
        self.principal.is_nan() || self.local_frame.is_nan()
    }
}

#[cfg(feature = "3d")]
impl TryFrom<Mat3> for AngularInertia {
    type Error = MatConversionError;

    fn try_from(mat: Mat3) -> Result<Self, Self::Error> {
        Self::try_from_mat3(mat)
    }
}

#[cfg(feature = "3d")]
impl From<SymmetricMat3> for AngularInertia {
    fn from(tensor: SymmetricMat3) -> Self {
        Self::from_tensor(tensor)
    }
}

#[cfg(feature = "3d")]
impl From<AngularInertiaTensor> for AngularInertia {
    fn from(tensor: AngularInertiaTensor) -> Self {
        Self::from_tensor(tensor)
    }
}

#[cfg(feature = "3d")]
impl From<AngularInertia> for AngularInertiaTensor {
    fn from(inertia: AngularInertia) -> Self {
        inertia.tensor()
    }
}

/// The local [center of mass] of an entity, representing the average position of mass
/// in the object. Applying forces at this point produces no torque.
///
/// If [`CenterOfMass`] is not present, but the entity has a [`Collider`], its [`ColliderMassProperties`]
/// computed based on the shape will be used instead.
///
/// The [`CenterOfMass`] component does *not* take into account child entities, and it is never modified
/// by the engine. The total center of mass of a dynamic [rigid body] that *does* consider child entities and colliders
/// is stored in the [`ComputedCenterOfMass`] component. It is updated automatically when mass properties are changed,
/// or when colliders are added or removed.
///
/// [center of mass]: https://en.wikipedia.org/wiki/Center_of_mass
/// [rigid body]: RigidBody
///
/// # Usage
///
/// The [`CenterOfMass`] component can be used to define the center of mass of a [rigid body] entity or its descendants:
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// // An offset of `-0.5` along the Y axis.
/// commands.spawn((
///     RigidBody::Dynamic,
///     Collider::capsule(0.5, 1.5),
#[cfg_attr(feature = "2d", doc = "    CenterOfMass::new(0.0, -0.5),")]
#[cfg_attr(feature = "3d", doc = "    CenterOfMass::new(0.0, -0.5, 0.0),")]
/// ));
/// # }
/// ```
///
/// If no [`CenterOfMass`] is present, the [`ComputedCenterOfMass`] will be computed from the collider
/// based on its shape.
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// // For a capsule, the center of mass is at the local origin.
/// commands.spawn((RigidBody::Dynamic, Collider::capsule(0.5, 1.5)));
/// # }
/// ```
///
/// If the rigid body has child colliders, they will contribute to the total [`ComputedCenterOfMass`]
/// based on weighted average of their global centers of mass.
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
#[cfg_attr(
    feature = "2d",
    doc = "// Total center of mass: (10.0 * [0.0, -0.5] + 5.0 * [0.0, 4.0]) / (10.0 + 5.0) = [0.0, 1.0]"
)]
#[cfg_attr(
    feature = "3d",
    doc = "// Total center of mass: (10.0 * [0.0, -0.5, 0.0] + 5.0 * [0.0, 4.0, 0.0]) / (10.0 + 5.0) = [0.0, 1.0, 0.0]"
)]
/// commands.spawn((
///     RigidBody::Dynamic,
///     Collider::capsule(0.5, 1.5),
///     Mass(10.0),
#[cfg_attr(feature = "2d", doc = "    CenterOfMass::new(0.0, -0.5),")]
#[cfg_attr(feature = "3d", doc = "    CenterOfMass::new(0.0, -0.5, 0.0),")]
///     Transform::default(),
/// ))
/// .with_child((
#[cfg_attr(feature = "2d", doc = "    Collider::circle(1.0),")]
#[cfg_attr(feature = "3d", doc = "    Collider::sphere(1.0),")]
///     Mass(5.0),
///     Transform::from_xyz(0.0, 4.0, 0.0),
/// ));
/// # }
/// ```
///
/// To prevent the centers of mass of child entities from contributing to the total [`ComputedCenterOfMass`],
/// add the [`NoAutoCenterOfMass`], component. This can be useful when full control over the center of mass is desired.
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
#[cfg_attr(feature = "2d", doc = "// Total center of mass: [0.0, -0.5]")]
#[cfg_attr(feature = "3d", doc = "// Total center of mass: [0.0, -0.5, 0.0]")]
/// commands.spawn((
///     RigidBody::Dynamic,
///     Collider::capsule(0.5, 1.5),
#[cfg_attr(feature = "2d", doc = "    CenterOfMass::new(0.0, -0.5),")]
#[cfg_attr(feature = "3d", doc = "    CenterOfMass::new(0.0, -0.5, 0.0),")]
///     Transform::default(),
/// ))
/// .with_child((
#[cfg_attr(feature = "2d", doc = "    Collider::circle(1.0),")]
#[cfg_attr(feature = "3d", doc = "    Collider::sphere(1.0),")]
///     Mass(5.0),
///     Transform::from_xyz(0.0, 4.0, 0.0),
/// ));
/// # }
/// ```
///
/// # Center of Mass Updates
///
/// The [`CenterOfMass`] component is never modified by the engine, so you can safely update it at any time.
///
/// The total [`ComputedCenterOfMass`] is updated automatically whenever the mass properties of a [rigid body]
/// or its descendants change, or when colliders are added, removed, or transformed. This update is triggered by adding
/// the [`RecomputeMassProperties`] component, which is removed after the update is performed
/// in [`MassPropertySystems::UpdateComputedMassProperties`](super::MassPropertySystems::UpdateComputedMassProperties).
///
/// To immediately perform a manual update of the total mass properties for a specific rigid body entity,
/// you can call [`MassPropertyHelper::update_mass_properties`] in a system.
///
/// # Related Types
///
/// - [`ComputedCenterOfMass`] stores the total center of mass of a dynamic [rigid body] that considers child entities and colliders.
/// - [`NoAutoCenterOfMass`] disables the centers of mass of child entities being taken into account for the [`ComputedCenterOfMass`].
/// - [`Mass`] represents resistance to linear acceleration.
/// - [`AngularInertia`] is the rotational equivalent of mass, representing resistance to angular acceleration.
/// - [`MassPropertiesBundle`] is a bundle containing mass properties.
/// - [`MassPropertyHelper`] is a [`SystemParam`] with utilities for computing and updating mass properties.
///
/// [`SystemParam`]: bevy::ecs::system::SystemParam
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct CenterOfMass(pub VectorF32);

impl CenterOfMass {
    /// A center of mass set at the local origin.
    pub const ZERO: Self = Self(VectorF32::ZERO);

    /// Creates a new [`CenterOfMass`] at the given local position.
    #[inline]
    #[cfg(feature = "2d")]
    pub const fn new(x: f32, y: f32) -> Self {
        Self(Vec2::new(x, y))
    }

    /// Creates a new [`CenterOfMass`] at the given local position.
    #[inline]
    #[cfg(feature = "3d")]
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self(Vec3::new(x, y, z))
    }

    /// Computes the [`CenterOfMass`] of the given shape.
    ///
    /// ```
    #[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
    #[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
    /// # use bevy::prelude::*;
    /// #
    /// // Compute the center of mass from a collider.
    #[cfg_attr(
        feature = "2d",
        doc = "let center_of_mass = CenterOfMass::from_shape(&Collider::circle(1.0));"
    )]
    #[cfg_attr(
        feature = "3d",
        doc = "let center_of_mass = CenterOfMass::from_shape(&Collider::sphere(1.0));"
    )]
    ///
    /// // Bevy's primitive shapes can also be used.
    #[cfg_attr(
        feature = "2d",
        doc = "let center_of_mass = CenterOfMass::from_shape(&Circle::new(1.0));"
    )]
    #[cfg_attr(
        feature = "3d",
        doc = "let center_of_mass = CenterOfMass::from_shape(&Sphere::new(1.0));"
    )]
    /// ```
    #[inline]
    pub fn from_shape<T: ComputeMassProperties>(shape: &T) -> Self {
        Self(shape.center_of_mass())
    }
}

/// A marker component that prevents descendants or attached colliders
/// from contributing to the total [`ComputedMass`] of a [rigid body].
///
/// Only the [`Mass`] component of the rigid body entity itself will be considered.
/// This is useful when full control over mass is desired.
///
/// [rigid body]: RigidBody
#[derive(Reflect, Clone, Copy, Component, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
#[require(RecomputeMassProperties)]
#[component(on_remove = on_remove_no_auto_mass_property)]
pub struct NoAutoMass;

/// A marker component that prevents descendants or attached colliders
/// from contributing to the total [`ComputedAngularInertia`] of a [rigid body].
///
/// Only the [`AngularInertia`] component of the rigid body entity itself will be considered.
/// This is useful when full control over inertia is desired.
///
/// [rigid body]: RigidBody
#[derive(Reflect, Clone, Copy, Component, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
#[require(RecomputeMassProperties)]
#[component(on_remove = on_remove_no_auto_mass_property)]
pub struct NoAutoAngularInertia;

/// A marker component that prevents descendants or attached colliders
/// from contributing to the total [`ComputedCenterOfMass`] of a [rigid body].
///
/// Only the [`CenterOfMass`] component of the rigid body entity itself will be considered.
/// This is useful when full control over the center of mass is desired.
///
/// [rigid body]: RigidBody
#[derive(Reflect, Clone, Copy, Component, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
#[require(RecomputeMassProperties)]
#[component(on_remove = on_remove_no_auto_mass_property)]
pub struct NoAutoCenterOfMass;

/// Triggers the recomputation of mass properties for rigid bodies when automatic computation is re-enabled.
fn on_remove_no_auto_mass_property(mut world: DeferredWorld, ctx: HookContext) {
    if let Ok(mut entity_commands) = world.commands().get_entity(ctx.entity) {
        entity_commands.try_insert(RecomputeMassProperties);
    }
}

/// A marker component that forces the recomputation of [`ComputedMass`], [`ComputedAngularInertia`]
/// and [`ComputedCenterOfMass`] for a [rigid body].
///
/// This is added automatically when the mass properties of a [rigid body] change, or when colliders are added or removed,
/// and is removed after the recomputation in [`MassPropertySystems::UpdateComputedMassProperties`].
///
/// [rigid body]: RigidBody
/// [`MassPropertySystems::UpdateComputedMassProperties`]: super::MassPropertySystems::UpdateComputedMassProperties
#[derive(Reflect, Clone, Copy, Component, Debug, Default, PartialEq)]
#[component(storage = "SparseSet")]
pub struct RecomputeMassProperties;

/// A bundle containing [mass properties].
///
/// [mass properties]: super
#[allow(missing_docs)]
#[derive(Bundle, Debug, Default, Clone, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct MassPropertiesBundle {
    pub mass: Mass,
    pub angular_inertia: AngularInertia,
    pub center_of_mass: CenterOfMass,
}

impl MassPropertiesBundle {
    /// Computes the mass properties for a [`Collider`] based on its shape and a given density.
    ///
    /// ```
    #[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
    #[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
    /// # use bevy::prelude::*;
    /// #
    /// # fn setup(mut commands: Commands) {
    /// // Compute mass properties from a collider with a density of `2.0`.
    /// commands.spawn((
    ///     RigidBody::Dynamic,
    #[cfg_attr(
        feature = "2d",
        doc = "    MassPropertiesBundle::from_shape(&Collider::circle(0.5), 2.0),"
    )]
    #[cfg_attr(
        feature = "3d",
        doc = "    MassPropertiesBundle::from_shape(&Collider::sphere(0.5), 2.0),"
    )]
    /// ));
    ///
    /// // Bevy's primitive shapes can also be used.
    /// commands.spawn((
    ///     RigidBody::Dynamic,
    #[cfg_attr(
        feature = "2d",
        doc = "    MassPropertiesBundle::from_shape(&Circle::new(0.5), 2.0),"
    )]
    #[cfg_attr(
        feature = "3d",
        doc = "    MassPropertiesBundle::from_shape(&Sphere::new(0.5), 2.0),"
    )]
    /// ));
    /// # }
    /// ```
    #[inline]
    pub fn from_shape<T: ComputeMassProperties>(shape: &T, density: f32) -> Self {
        shape.mass_properties(density).to_bundle()
    }
}

#[cfg(test)]
mod tests {
    #[cfg(feature = "3d")]
    use super::*;
    #[cfg(feature = "3d")]
    use approx::assert_relative_eq;

    #[test]
    #[cfg(feature = "3d")]
    fn angular_inertia_creation() {
        use bevy_heavy::AngularInertiaTensor;

        let angular_inertia = AngularInertia::new(Vec3::new(10.0, 20.0, 30.0));
        assert_eq!(angular_inertia.principal, Vec3::new(10.0, 20.0, 30.0));
        assert_eq!(angular_inertia.local_frame, Quat::IDENTITY);
        assert_relative_eq!(
            angular_inertia.tensor(),
            AngularInertiaTensor::new(Vec3::new(10.0, 20.0, 30.0))
        );
    }

    #[test]
    #[should_panic]
    #[cfg(feature = "3d")]
    fn negative_angular_inertia_panics() {
        AngularInertia::new(Vec3::new(-1.0, 2.0, 3.0));
    }

    #[test]
    #[cfg(feature = "3d")]
    fn negative_angular_inertia_error() {
        assert_eq!(
            AngularInertia::try_new(Vec3::new(-1.0, 2.0, 3.0)),
            Err(AngularInertiaError::Negative),
            "negative angular inertia should return an error"
        );
    }

    #[test]
    #[cfg(feature = "3d")]
    fn nan_angular_inertia_error() {
        assert_eq!(
            AngularInertia::try_new(Vec3::new(f32::NAN, 2.0, 3.0)),
            Err(AngularInertiaError::NaN),
            "NaN angular inertia should return an error"
        );
    }
}
