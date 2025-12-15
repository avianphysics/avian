//! Physics interpolation and extrapolation for rigid bodies.
//!
//! See [`PhysicsInterpolationPlugin`].

use bevy::{ecs::query::QueryData, prelude::*};
use bevy_transform_interpolation::{VelocitySource, prelude::*};

#[expect(deprecated)]
pub use bevy_transform_interpolation::{
    TransformEasingSet, TransformEasingSystems,
    prelude::{
        NoRotationEasing, NoScaleEasing, NoTransformEasing, NoTranslationEasing,
        RotationExtrapolation, RotationHermiteEasing, RotationInterpolation, ScaleInterpolation,
        TransformExtrapolation, TransformHermiteEasing, TransformInterpolation,
        TranslationExtrapolation, TranslationHermiteEasing, TranslationInterpolation,
    },
};

use crate::prelude::*;

/// A plugin for [`Transform`] interpolation and extrapolation for rigid bodies.
///
/// # Overview
///
/// To make behavior deterministic and independent of frame rate, Avian runs physics at a fixed timestep
/// in [`FixedPostUpdate`] by default. However, when this timestep doesn't match the display frame rate,
/// movement can appear choppy, especially on displays with high refresh rates.
///
/// The conventional solution is to ease transforms in between physics ticks to smooth out the visual result.
/// This can be done using either interpolation or extrapolation.
///
/// ## Interpolation
///
/// [`Transform`] interpolation computes a `Transform` that is somewhere in between the current position
/// and the position from the previous physics tick. This produces smooth and accurate movement.
///
/// The downside of interpolation is that it causes rendering to be slightly behind the physics simulation.
/// This can make movement feel slightly delayed, but this is rarely noticeable unless using a very small
/// physics tick rate.
///
/// ## Extrapolation
///
/// [`Transform`] extrapolation computes a `Transform` that is somewhere in between the current position
/// and a future position predicted based on velocity. This produces movement that looks smooth and feels
/// very responsive.
///
/// The downside of extrapolation is that it can be less accurate. When the prediction is wrong, the rendered
/// positions may jump to correct the mispredictions. This can be noticeable when the entity changes direction
/// or speed rapidly.
///
/// Extrapolation is primarily inteded for cases where low latency and high responsiveness are crucial for gameplay,
/// such as first-person shooters and racing games. For most other games, interpolation is often the better choice.
///
/// # Usage
///
/// The [`PhysicsInterpolationPlugin`] is included in the [`PhysicsPlugins`] by default,
/// so most apps don't need to add it manually.
///
/// [`Transform`] interpolation and extrapolation can be enabled for individual entities
/// using the [`TransformInterpolation`] and [`TransformExtrapolation`] components respectively:
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// fn setup(mut commands: Commands) {
///     // Enable interpolation for this rigid body.
///     commands.spawn((
///         RigidBody::Dynamic,
///         Transform::default(),
///         TransformInterpolation,
///     ));
///
///     // Enable extrapolation for this rigid body.
///     commands.spawn((
///         RigidBody::Dynamic,
///         Transform::default(),
///         TransformExtrapolation,
///     ));
/// }
/// ```
///
/// Now, any changes made to the [`Transform`] of the entity in [`FixedPreUpdate`], [`FixedUpdate`],
/// or [`FixedPostUpdate`] will automatically be smoothed in between fixed timesteps.
///
/// Transform properties can also be interpolated individually by adding the [`TranslationInterpolation`],
/// [`RotationInterpolation`], and [`ScaleInterpolation`] components, and similarly for extrapolation.
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// fn setup(mut commands: Commands) {
///     // Only interpolate translation.
///     commands.spawn((Transform::default(), TranslationInterpolation));
///     
///     // Only interpolate rotation.
///     commands.spawn((Transform::default(), RotationInterpolation));
///     
///     // Only interpolate scale.
///     commands.spawn((Transform::default(), ScaleInterpolation));
///     
///     // Mix and match!
///     // Extrapolate translation and interpolate rotation.
///     commands.spawn((
///         Transform::default(),
///         TranslationExtrapolation,
///         RotationInterpolation,
///     ));
/// }
/// ```
///
/// If you want *all* rigid bodies to be interpolated or extrapolated by default, you can use
/// [`PhysicsInterpolationPlugin::interpolate_all()`] or [`PhysicsInterpolationPlugin::extrapolate_all()`]:
///
/// ```no_run
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// fn main() {
///    App::new()
///       .add_plugins(PhysicsPlugins::default().set(PhysicsInterpolationPlugin::interpolate_all()))
///       // ...
///       .run();
/// }
/// ```
///
/// When interpolation or extrapolation is enabled for all entities by default, you can still opt out of it
/// for individual entities by adding the [`NoTransformEasing`] component, or the individual
/// [`NoTranslationEasing`], [`NoRotationEasing`], and [`NoScaleEasing`] components.
///
/// Note that changing [`Transform`] manually in any schedule that *doesn't* use a fixed timestep is also supported,
/// but it is equivalent to teleporting, and disables interpolation for the entity for the remainder of that fixed timestep.
///
/// ## Hermite Interpolation
///
/// By default, *linear interpolation* (`lerp`) is used for easing translation and scale,
/// and *spherical linear interpolation* (`slerp`) is used for easing rotation.
/// This is computationally efficient and works well for most cases.
///
/// However, linear interpolation doesn't consider velocity, which can make trajectories look less smooth
/// at low tick rates. Very high angular velocities (ex: for car wheels or fan blades) can be especially problematic,
/// as `slerp` always takes the shortest path between two rotations, which can sometimes cause entities to rotate
/// in the opposite direction.
///
/// Unlike linear interpolation, *Hermite interpolation* uses both position and velocity information
/// to estimate the trajectories of entities, producing smoother results. To enable it for interpolation
/// or extrapolation, add the [`TransformHermiteEasing`] component or the individual [`TranslationHermiteEasing`]
/// and [`RotationHermiteEasing`] components:
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// fn setup(mut commands: Commands) {
///     // Enable Hermite interpolation for this rigid body.
///     commands.spawn((
///         RigidBody::Dynamic,
///         Transform::default(),
///         TransformInterpolation,
///         TransformHermiteEasing,
///     ));
/// }
/// ```
///
/// Hermite interpolation is more expensive than linear interpolation, so it is generally recommended
/// to only use it when it produces noticeable benefits. For most cases, linear interpolation should be sufficient.
///
/// Note that scale interpolation is always linear, and does not support Hermite interpolation.
///
/// # General Interpolation or Extrapolation
///
/// Avian uses [`bevy_transform_interpolation`] for interpolation and extrapolation.
/// It is not limited to physics entities, so it is actually possible to use the components
/// shown here for interpolating the [`Transform`] of *any* entity!
///
/// Refer to the [`bevy_transform_interpolation`] documentation for more information on how to use it.
#[derive(Debug, Default)]
pub struct PhysicsInterpolationPlugin {
    interpolate_translation_all: bool,
    interpolate_rotation_all: bool,
    extrapolate_translation_all: bool,
    extrapolate_rotation_all: bool,
}

impl PhysicsInterpolationPlugin {
    /// Enables interpolation of translation and rotation for all rigid bodies.
    ///
    /// This can be overridden for individual entities by adding the [`NoTransformEasing`] component,
    /// or the individual [`NoTranslationEasing`] and [`NoRotationEasing`] components.
    pub const fn interpolate_all() -> Self {
        Self {
            interpolate_translation_all: true,
            interpolate_rotation_all: true,
            extrapolate_translation_all: false,
            extrapolate_rotation_all: false,
        }
    }

    /// Enables interpolation of translation for all rigid bodies.
    ///
    /// This can be overridden for individual entities by adding the [`NoTranslationEasing`] component.
    pub const fn interpolate_translation_all() -> Self {
        Self {
            interpolate_translation_all: true,
            interpolate_rotation_all: false,
            extrapolate_translation_all: false,
            extrapolate_rotation_all: false,
        }
    }

    /// Enables interpolation of rotation for all rigid bodies.
    ///
    /// This can be overridden for individual entities by adding the [`NoRotationEasing`] component.
    pub const fn interpolate_rotation_all() -> Self {
        Self {
            interpolate_translation_all: false,
            interpolate_rotation_all: true,
            extrapolate_translation_all: false,
            extrapolate_rotation_all: false,
        }
    }

    /// Enables extrapolation of translation and rotation for all rigid bodies.
    ///
    /// This can be overridden for individual entities by adding the [`NoTransformEasing`] component,
    /// or the individual [`NoTranslationEasing`] and [`NoRotationEasing`] components.
    pub const fn extrapolate_all() -> Self {
        Self {
            interpolate_translation_all: false,
            interpolate_rotation_all: false,
            extrapolate_translation_all: true,
            extrapolate_rotation_all: true,
        }
    }

    /// Enables extrapolation of translation for all rigid bodies.
    ///
    /// This can be overridden for individual entities by adding the [`NoTranslationEasing`] component.
    pub const fn extrapolate_translation_all() -> Self {
        Self {
            interpolate_translation_all: false,
            interpolate_rotation_all: false,
            extrapolate_translation_all: true,
            extrapolate_rotation_all: false,
        }
    }

    /// Enables extrapolation of rotation for all rigid bodies.
    ///
    /// This can be overridden for individual entities by adding the [`NoRotationEasing`] component.
    pub const fn extrapolate_rotation_all() -> Self {
        Self {
            interpolate_translation_all: false,
            interpolate_rotation_all: false,
            extrapolate_translation_all: false,
            extrapolate_rotation_all: true,
        }
    }
}

impl Plugin for PhysicsInterpolationPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((
            TransformInterpolationPlugin::default(),
            TransformExtrapolationPlugin::<LinVelSource, AngVelSource>::default(),
            TransformHermiteEasingPlugin::<LinVelSource, AngVelSource>::default(),
        ));

        // Make the previous velocity components required for Hermite interpolation to insert them automatically.
        app.register_required_components::<TranslationHermiteEasing, PreviousLinearVelocity>();
        app.register_required_components::<RotationHermiteEasing, PreviousAngularVelocity>();

        // Enable interpolation for all entities with a rigid body.
        if self.interpolate_translation_all {
            let _ = app.try_register_required_components::<RigidBody, TranslationInterpolation>();
        }
        if self.interpolate_rotation_all {
            let _ = app.try_register_required_components::<RigidBody, RotationInterpolation>();
        }

        // Enable extrapolation for all entities with a rigid body.
        if self.extrapolate_translation_all {
            let _ = app.try_register_required_components::<RigidBody, TranslationExtrapolation>();
        }
        if self.extrapolate_rotation_all {
            let _ = app.try_register_required_components::<RigidBody, RotationExtrapolation>();
        }

        // Update previous velocity components for Hermite interpolation.
        app.add_systems(
            PhysicsSchedule,
            update_previous_velocity.in_set(PhysicsStepSystems::First),
        );
    }
}

/// The previous linear velocity of an entity indicating its movement speed and direction during the previous frame.
#[derive(Component, Default, Deref, DerefMut)]
struct PreviousLinearVelocity(Vector);

/// The previous angular velocity of an entity indicating its rotation speed during the previous frame.
#[derive(Component, Default, Deref, DerefMut)]
struct PreviousAngularVelocity(AngularVelocity);

#[derive(QueryData)]
struct LinVelSource;

impl VelocitySource for LinVelSource {
    type Previous = PreviousLinearVelocity;
    type Current = LinearVelocity;

    fn previous(previous: &Self::Previous) -> Vec3 {
        #[cfg(feature = "2d")]
        {
            previous.f32().extend(0.0)
        }
        #[cfg(feature = "3d")]
        {
            previous.f32()
        }
    }

    fn current(current: &Self::Current) -> Vec3 {
        #[cfg(feature = "2d")]
        {
            current.0.f32().extend(0.0)
        }
        #[cfg(feature = "3d")]
        {
            current.0.f32()
        }
    }
}

#[derive(QueryData)]
struct AngVelSource;

#[allow(clippy::unnecessary_cast)]
impl VelocitySource for AngVelSource {
    type Previous = PreviousAngularVelocity;
    type Current = AngularVelocity;

    fn previous(previous: &Self::Previous) -> Vec3 {
        #[cfg(feature = "2d")]
        {
            Vec3::Z * previous.0.0 as f32
        }
        #[cfg(feature = "3d")]
        {
            previous.0.f32()
        }
    }

    fn current(current: &Self::Current) -> Vec3 {
        #[cfg(feature = "2d")]
        {
            Vec3::Z * current.0 as f32
        }
        #[cfg(feature = "3d")]
        {
            current.0.f32()
        }
    }
}

fn update_previous_velocity(
    mut lin_vel_query: Query<(&LinearVelocity, &mut PreviousLinearVelocity)>,
    mut ang_vel_query: Query<(&AngularVelocity, &mut PreviousAngularVelocity)>,
) {
    for (lin_vel, mut prev_lin_vel) in &mut lin_vel_query {
        prev_lin_vel.0 = lin_vel.0;
    }

    for (ang_vel, mut prev_ang_vel) in &mut ang_vel_query {
        prev_ang_vel.0 = *ang_vel;
    }
}
