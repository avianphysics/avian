use super::super::MassProperties;
use crate::prelude::*;
use bevy::prelude::*;
use derive_more::derive::From;

/// The density of a [`Collider`], used for computing [`ColliderMassProperties`].
/// Defaults to `1.0`.
///
/// If the entity has the [`Mass`] component, it will be used instead of the collider's mass.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// // Spawn a body with a collider that has a density of `2.5`.
/// fn setup(mut commands: Commands) {
///     commands.spawn((
///         RigidBody::Dynamic,
#[cfg_attr(feature = "2d", doc = "        Collider::circle(0.5),")]
#[cfg_attr(feature = "3d", doc = "        Collider::sphere(0.5),")]
///         ColliderDensity(2.5),
///     ));
/// }
/// ```
#[derive(Reflect, Clone, Copy, Component, Debug, Deref, DerefMut, PartialEq, PartialOrd, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
pub struct ColliderDensity(pub f32);

impl Default for ColliderDensity {
    fn default() -> Self {
        Self(1.0)
    }
}

impl ColliderDensity {
    /// A density of `0.0`, resulting in a collider with no mass.
    pub const ZERO: Self = Self(0.0);
}

/// A read-only component for the mass properties of a [`Collider`].
/// Computed automatically from the collider's shape and [`ColliderDensity`].
///
/// If the entity has the [`Mass`], [`AngularInertia`], or [`CenterOfMass`] components,
/// they will be used instead when updating the associated rigid body's [`ComputedMass`],
/// [`ComputedAngularInertia`], and [`ComputedCenterOfMass`] components respectively.
///
/// # Example
///
/// ```no_run
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// fn setup(mut commands: Commands) {
#[cfg_attr(
    feature = "2d",
    doc = "    commands.spawn((RigidBody::Dynamic, Collider::circle(0.5)));"
)]
#[cfg_attr(
    feature = "3d",
    doc = "    commands.spawn((RigidBody::Dynamic, Collider::sphere(0.5)));"
)]
/// }
///
/// fn print_collider_masses(query: Query<&ColliderMassProperties>) {
///     for mass_properties in &query {
///         println!("{}", mass_properties.mass);
///     }
/// }
/// ```
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
pub struct ColliderMassProperties(MassProperties);

impl ColliderMassProperties {
    /// The collider has no mass.
    pub const ZERO: Self = Self(MassProperties::ZERO);

    /// Computes mass properties from a given shape and density.
    ///
    /// Because [`ColliderMassProperties`] is intended to be read-only, adding this as a component manually
    /// has no effect. The mass properties will be recomputed using the [`ColliderDensity`].
    #[inline]
    pub fn from_shape<T: ComputeMassProperties>(shape: &T, density: f32) -> Self {
        Self(shape.mass_properties(density))
    }
}
