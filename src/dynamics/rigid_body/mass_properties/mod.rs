//! Mass property functionality for [rigid bodies] and [colliders].
//!
//! # Overview
//!
//! Every dynamic rigid body has [mass], [angular inertia], and a [center of mass].
//! These mass properties determine how the rigid body responds to forces and torques.
//!
//! - **Mass**: Represents resistance to linear acceleration. A higher mass requires more force for the same acceleration.
//! - **Angular Inertia**: Represents resistance to angular acceleration. A higher angular inertia requires more torque for the same angular acceleration.
//! - **Center of Mass**: The average position of mass in the body. Applying forces at this point produces no torque.
//!
//! Static and kinematic rigid bodies have infinite mass and angular inertia,
//! and do not respond to forces or torques. Zero mass for a dynamic body is also
//! treated as a special case, and corresponds to infinite mass.
//!
//! Mass properties can be set for individual entities using the [`Mass`], [`AngularInertia`],
//! and [`CenterOfMass`] components. If they are not present, mass properties are instead computed
//! automatically from attached colliders based on their shape and [`ColliderDensity`].
//!
//! If a rigid body has child entities, their mass properties are combined to compute
//! the total mass properties of the rigid body. These are stored in the [`ComputedMass`],
//! [`ComputedAngularInertia`], and [`ComputedCenterOfMass`] components, which are updated
//! automatically when mass properties are changed, or when colliders are added or removed.
//!
//! To prevent mass properties of child entities from contributing to the total mass properties,
//! you can use the [`NoAutoMass`], [`NoAutoAngularInertia`], and [`NoAutoCenterOfMass`] marker components.
//! This can be useful when full control over mass properties is desired.
//!
//! [rigid bodies]: crate::dynamics::rigid_body::RigidBody
//! [colliders]: crate::collision::collider::Collider
//! [mass]: components::Mass
//! [angular inertia]: components::AngularInertia
//! [center of mass]: components::CenterOfMass
//!
//! ## Example
//!
//! If no mass properties are set, they are computed automatically from attached colliders
//! based on their shape and density.
//!
//! ```
#![cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
//! # use bevy::prelude::*;
//! #
//! # fn setup(mut commands: Commands) {
//! // Note: `ColliderDensity` is optional, and defaults to `1.0` if not present.
//! commands.spawn((
//!     RigidBody::Dynamic,
//!     Collider::capsule(0.5, 1.5),
//!     ColliderDensity(2.0),
//! ));
//! # }
//! ```
//!
//! If mass properties are set with the [`Mass`], [`AngularInertia`], and [`CenterOfMass`] components,
//! they override the values computed from colliders.
//!
//! ```
#![cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
//! # use bevy::prelude::*;
//! #
//! # fn setup(mut commands: Commands) {
//! // Override mass and the center of mass, but use the collider's angular inertia.
//! commands.spawn((
//!     RigidBody::Dynamic,
//!     Collider::capsule(0.5, 1.5),
//!     Mass(5.0),
#![cfg_attr(feature = "2d", doc = "    CenterOfMass::new(0.0, -0.5),")]
#![cfg_attr(feature = "3d", doc = "    CenterOfMass::new(0.0, -0.5, 0.0),")]
//! ));
//! # }
//! ```
//!
//! If the rigid body has child colliders, their mass properties will be combined for
//! the total [`ComputedMass`], [`ComputedAngularInertia`], and [`ComputedCenterOfMass`].
//!
//! ```
#![cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
//! # use bevy::prelude::*;
//! #
//! # fn setup(mut commands: Commands) {
//! // Total mass: 10.0 + 5.0 = 15.0
#![cfg_attr(
    feature = "2d",
    doc = "// Total center of mass: (10.0 * [0.0, -0.5] + 5.0 * [0.0, 4.0]) / (10.0 + 5.0) = [0.0, 1.0]"
)]
#![cfg_attr(
    feature = "3d",
    doc = "// Total center of mass: (10.0 * [0.0, -0.5, 0.0] + 5.0 * [0.0, 4.0, 0.0]) / (10.0 + 5.0) = [0.0, 1.0, 0.0]"
)]
//! commands.spawn((
//!     RigidBody::Dynamic,
//!     Collider::capsule(0.5, 1.5),
//!     Mass(10.0),
#![cfg_attr(feature = "2d", doc = "    CenterOfMass::new(0.0, -0.5),")]
#![cfg_attr(feature = "3d", doc = "    CenterOfMass::new(0.0, -0.5, 0.0),")]
//!     Transform::default(),
//! ))
//! .with_child((
#![cfg_attr(feature = "2d", doc = "    Collider::circle(1.0),")]
#![cfg_attr(feature = "3d", doc = "    Collider::sphere(1.0),")]
//!     Mass(5.0),
//!     Transform::from_xyz(0.0, 4.0, 0.0),
//! ));
//! # }
//! ```
//!
//! To prevent child entities from contributing to the total mass properties, use the [`NoAutoMass`],
//! [`NoAutoAngularInertia`], and [`NoAutoCenterOfMass`] marker components.
//!
//! ```
#![cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
//! # use bevy::prelude::*;
//! #
//! # fn setup(mut commands: Commands) {
//! // Total mass: 10.0
#![cfg_attr(feature = "2d", doc = "// Total center of mass: [0.0, -0.5]")]
#![cfg_attr(feature = "3d", doc = "// Total center of mass: [0.0, -0.5, 0.0]")]
//! commands.spawn((
//!     RigidBody::Dynamic,
//!     Collider::capsule(0.5, 1.5),
//!     Mass(10.0),
#![cfg_attr(feature = "2d", doc = "    CenterOfMass::new(0.0, -0.5),")]
#![cfg_attr(feature = "3d", doc = "    CenterOfMass::new(0.0, -0.5, 0.0),")]
//!     NoAutoMass,
//!     NoAutoCenterOfMass,
//!     Transform::default(),
//! ))
//! .with_child((
#![cfg_attr(feature = "2d", doc = "    Collider::circle(1.0),")]
#![cfg_attr(feature = "3d", doc = "    Collider::sphere(1.0),")]
//!     Mass(5.0),
//!     Transform::from_xyz(0.0, 4.0, 0.0),
//! ));
//! # }
//! ```
//!
//! # Computing Mass Properties for Shapes
//!
//! Mass properties of colliders and Bevy's primitive shapes can be computed using methods
//! provided by the [`ComputeMassProperties2d`] and [`ComputeMassProperties3d`] traits.
//!
//! ```
#![cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
//! # use bevy::prelude::*;
//! #
//! #
//! // Compute mass properties for a capsule collider with a density of `2.0`.
//! let capsule = Collider::capsule(0.5, 1.5);
//! let mass_properties = capsule.mass_properties(2.0);
//!
//! // Compute individual mass properties for a `Circle`.
//! let circle = Circle::new(1.0);
//! let mass = circle.mass(2.0);
//! let angular_inertia = circle.angular_inertia(mass);
//! let center_of_mass = circle.center_of_mass();
//! ```
//!
//! Similarly, shapes can be used to construct the [`Mass`], [`AngularInertia`],
//! and [`CenterOfMass`] components, or the [`MassPropertiesBundle`].
//!
//! ```
#![cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
//! # use bevy::prelude::*;
//! #
//! # fn setup(mut commands: Commands) {
//! // Construct individual mass properties from a collider.
#![cfg_attr(feature = "2d", doc = "let shape = Collider::circle(0.5);")]
#![cfg_attr(feature = "3d", doc = "let shape = Collider::sphere(0.5);")]
//! commands.spawn((
//!     RigidBody::Dynamic,
//!     Mass::from_shape(&shape, 2.0),
//!     AngularInertia::from_shape(&shape, 1.5),
//!     CenterOfMass::from_shape(&shape),
//! ));
//!
//! // Construct a `MassPropertiesBundle` from a primitive shape.
#![cfg_attr(feature = "2d", doc = "let shape = Circle::new(0.5);")]
#![cfg_attr(feature = "3d", doc = "let shape = Sphere::new(0.5);")]
//! commands.spawn((RigidBody::Dynamic, MassPropertiesBundle::from_shape(&shape, 2.0)));
//! # }
//! ```
//!
//! This mass property computation functionality is provided by the [`bevy_heavy`] crate.
//!
//! # Mass Property Helper
//!
//! [`MassPropertyHelper`] is a [`SystemParam`](bevy::ecs::system::SystemParam) that provides convenient helper methods
//! that can be used to modify or compute mass properties for individual entities and hierarchies at runtime.
//!
//! For example, [`MassPropertyHelper::total_mass_properties`] computes the total mass properties of an entity,
//! taking into account the mass properties of descendants and colliders.

use crate::physics_transform::PhysicsTransformSystems;
use crate::prelude::*;
use bevy::{
    ecs::{intern::Interned, schedule::ScheduleLabel},
    prelude::*,
};

pub mod components;
use components::RecomputeMassProperties;

mod system_param;
pub use system_param::MassPropertyHelper;

/// Mass property computation with `bevy_heavy`, re-exported for your convenience.
pub use bevy_heavy;

#[cfg(feature = "2d")]
pub(crate) use bevy_heavy::{
    ComputeMassProperties2d as ComputeMassProperties, MassProperties2d as MassProperties,
};
#[cfg(feature = "3d")]
pub(crate) use bevy_heavy::{
    ComputeMassProperties3d as ComputeMassProperties, MassProperties3d as MassProperties,
};

/// An extension trait for [`MassProperties`].
pub trait MassPropertiesExt {
    /// Converts the [`MassProperties`] to a [`MassPropertiesBundle`]
    /// containing the [`Mass`], [`AngularInertia`], and [`CenterOfMass`] components.
    fn to_bundle(&self) -> MassPropertiesBundle;
}

impl MassPropertiesExt for MassProperties {
    fn to_bundle(&self) -> MassPropertiesBundle {
        #[cfg(feature = "2d")]
        let angular_inertia = AngularInertia(self.angular_inertia);
        #[cfg(feature = "3d")]
        let angular_inertia = AngularInertia::new_with_local_frame(
            self.principal_angular_inertia.f32(),
            self.local_inertial_frame.f32(),
        );

        MassPropertiesBundle {
            mass: Mass(self.mass),
            angular_inertia,
            center_of_mass: CenterOfMass(self.center_of_mass),
        }
    }
}

/// A plugin for managing [mass properties] of rigid bodies.
///
/// - Updates the [`ComputedMass`], [`ComputedAngularInertia`], and [`ComputedCenterOfMass`] components
///   for rigid bodies when their mass properties are changed, or when colliders are added or removed.
/// - Logs warnings when dynamic bodies have invalid [`Mass`] or [`AngularInertia`].
///
/// [mass properties]: crate::dynamics::rigid_body::mass_properties
pub struct MassPropertyPlugin {
    schedule: Interned<dyn ScheduleLabel>,
}

impl MassPropertyPlugin {
    /// Creates a [`MassPropertyPlugin`] with the schedule that is used for running the [`PhysicsSchedule`].
    ///
    /// The default schedule is `FixedPostUpdate`.
    pub fn new(schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: schedule.intern(),
        }
    }
}

impl Default for MassPropertyPlugin {
    fn default() -> Self {
        Self::new(FixedPostUpdate)
    }
}

impl Plugin for MassPropertyPlugin {
    fn build(&self, app: &mut App) {
        // TODO: We probably don't need this since we have the observer.
        // Force mass property computation for new rigid bodies.
        app.register_required_components::<RigidBody, RecomputeMassProperties>();

        // Compute mass properties for new rigid bodies at spawn.
        app.add_observer(
            |trigger: On<Add, RigidBody>, mut mass_helper: MassPropertyHelper| {
                mass_helper.update_mass_properties(trigger.entity);
            },
        );

        // Update the mass properties of rigid bodies when colliders added or removed.
        // TODO: Avoid duplicating work with the above observer.
        app.add_observer(
            |trigger: On<Insert, RigidBodyColliders>, mut mass_helper: MassPropertyHelper| {
                mass_helper.update_mass_properties(trigger.entity);
            },
        );

        // Configure system sets for mass property computation.
        app.configure_sets(
            self.schedule,
            (
                MassPropertySystems::UpdateColliderMassProperties,
                MassPropertySystems::QueueRecomputation,
                MassPropertySystems::UpdateComputedMassProperties,
            )
                .chain()
                .in_set(PhysicsSystems::Prepare)
                .after(PhysicsTransformSystems::TransformToPosition),
        );

        // Queue mass property recomputation when mass properties are changed.
        app.add_systems(
            self.schedule,
            (
                queue_mass_recomputation_on_mass_change,
                queue_mass_recomputation_on_collider_mass_change,
            )
                .in_set(MassPropertySystems::QueueRecomputation),
        );

        // Update mass properties for entities with the `RecomputeMassProperties` component.
        app.add_systems(
            self.schedule,
            (update_mass_properties, warn_invalid_mass)
                .chain()
                .in_set(MassPropertySystems::UpdateComputedMassProperties),
        );
    }
}

/// A system set for logic related to updating mass properties.
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum MassPropertySystems {
    /// Update [`ColliderMassProperties`] for colliders.
    UpdateColliderMassProperties,
    /// Adds the [`RecomputeMassProperties`] component to entities with changed mass properties.
    QueueRecomputation,
    /// Update [`ComputedMass`], [`ComputedAngularInertia`], and [`ComputedCenterOfMass`]
    /// for entities with the [`RecomputeMassProperties`] component. The component is removed after updating.
    UpdateComputedMassProperties,
}

/// A query filter for entities with [`ComputedMass`], [`ComputedAngularInertia`], or [`ComputedCenterOfMass`].
pub type WithComputedMassProperty = Or<(
    With<ComputedMass>,
    With<ComputedAngularInertia>,
    With<ComputedCenterOfMass>,
)>;

/// A query filter for entities with changed [`Mass`], [`AngularInertia`], or [`CenterOfMass`].
pub type MassPropertyChanged = Or<(
    Changed<Mass>,
    Changed<AngularInertia>,
    Changed<CenterOfMass>,
)>;

/// Queues mass property recomputation for rigid bodies when their [`Mass`], [`AngularInertia`],
/// or [`CenterOfMass`] components are changed.
///
/// Colliders attached to rigid bodies are excluded, as they are handled by
/// [`queue_mass_recomputation_on_collider_mass_change`].
fn queue_mass_recomputation_on_mass_change(
    mut commands: Commands,
    mut query: Query<
        Entity,
        (
            WithComputedMassProperty,
            Without<ColliderOf>,
            MassPropertyChanged,
        ),
    >,
) {
    for entity in &mut query {
        commands.entity(entity).insert(RecomputeMassProperties);
    }
}

/// Queues mass property recomputation for rigid bodies when the [`ColliderMassProperties`],
/// [`Mass`], [`AngularInertia`], or [`CenterOfMass`] components of their colliders are changed.
fn queue_mass_recomputation_on_collider_mass_change(
    mut commands: Commands,
    mut query: Query<
        &ColliderOf,
        Or<(
            Changed<ColliderMassProperties>,
            Changed<ColliderTransform>,
            MassPropertyChanged,
        )>,
    >,
) {
    for &ColliderOf { body } in &mut query {
        if let Ok(mut entity_commands) = commands.get_entity(body) {
            entity_commands.insert(RecomputeMassProperties);
        }
    }
}

fn update_mass_properties(
    mut commands: Commands,
    query: Query<Entity, With<RecomputeMassProperties>>,
    mut mass_helper: MassPropertyHelper,
) {
    // TODO: Parallelize mass property updates.
    for entity in query.iter() {
        mass_helper.update_mass_properties(entity);
        commands.entity(entity).remove::<RecomputeMassProperties>();
    }
}

#[cfg(feature = "default-collider")]
type ShouldWarn = (
    Without<ColliderConstructor>,
    Without<ColliderConstructorHierarchy>,
);

#[cfg(not(feature = "default-collider"))]
type ShouldWarn = ();

/// Logs warnings when dynamic bodies have invalid [`Mass`] or [`AngularInertia`].
fn warn_invalid_mass(
    mut bodies: Query<
        (
            Entity,
            &RigidBody,
            Ref<ComputedMass>,
            Ref<ComputedAngularInertia>,
        ),
        (
            Or<(Changed<ComputedMass>, Changed<ComputedAngularInertia>)>,
            ShouldWarn,
        ),
    >,
) {
    for (entity, rb, mass, inertia) in &mut bodies {
        let is_mass_valid = mass.is_finite();
        #[cfg(feature = "2d")]
        let is_inertia_valid = inertia.is_finite();
        #[cfg(feature = "3d")]
        let is_inertia_valid = inertia.is_finite();

        // Warn about dynamic bodies with no mass or inertia
        if rb.is_dynamic() && !(is_mass_valid && is_inertia_valid) {
            warn!(
                "Dynamic rigid body {:?} has no mass or inertia. This can cause NaN values. Consider adding a `MassPropertiesBundle` or a `Collider` with mass.",
                entity
            );
        }
    }
}

#[cfg(test)]
#[cfg(all(feature = "2d", feature = "default-collider"))]
#[allow(clippy::unnecessary_cast)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;

    fn create_app() -> App {
        let mut app = App::new();
        app.add_plugins((MinimalPlugins, PhysicsPlugins::default(), TransformPlugin));
        app
    }

    fn get_computed_mass_properties(
        world: &mut World,
        entity: Entity,
    ) -> (
        &ComputedMass,
        &ComputedAngularInertia,
        &ComputedCenterOfMass,
    ) {
        let mut query = world.query::<(
            &ComputedMass,
            &ComputedAngularInertia,
            &ComputedCenterOfMass,
        )>();
        query.get(world, entity).unwrap()
    }

    #[test]
    fn mass_properties_zero_by_default() {
        // `RigidBody`

        let mut app = create_app();

        let body_entity = app.world_mut().spawn(RigidBody::Dynamic).id();

        app.world_mut().run_schedule(FixedPostUpdate);

        let (mass, angular_inertia, center_of_mass) =
            get_computed_mass_properties(app.world_mut(), body_entity);

        assert_eq!(*mass, ComputedMass::default());
        assert_eq!(*angular_inertia, ComputedAngularInertia::default());
        assert_eq!(*center_of_mass, ComputedCenterOfMass::default());
    }

    #[test]
    fn mass_properties_rb_collider() {
        // `RigidBody`, `Collider`

        let mut app = create_app();

        let collider = Collider::circle(1.0);
        let collider_mass_props = collider.mass_properties(1.0);

        let body_entity = app.world_mut().spawn((RigidBody::Dynamic, collider)).id();

        app.world_mut().run_schedule(FixedPostUpdate);

        let (mass, angular_inertia, center_of_mass) =
            get_computed_mass_properties(app.world_mut(), body_entity);

        assert_eq!(mass.value() as f32, collider_mass_props.mass);
        assert_eq!(
            angular_inertia.value() as f32,
            collider_mass_props.angular_inertia
        );
        assert_eq!(*center_of_mass, ComputedCenterOfMass::default());
    }

    #[test]
    fn mass_properties_rb_collider_with_set_mass() {
        // `RigidBody`, `Collider`, `Mass(5.0)`

        let mut app = create_app();

        let collider = Collider::circle(1.0);
        let collider_mass_props = collider.mass_properties(1.0);

        let body_entity = app
            .world_mut()
            .spawn((RigidBody::Dynamic, collider, Mass(5.0)))
            .id();

        app.world_mut().run_schedule(FixedPostUpdate);

        let (mass, angular_inertia, center_of_mass) =
            get_computed_mass_properties(app.world_mut(), body_entity);

        assert_eq!(mass.value() as f32, 5.0);
        assert_eq!(
            angular_inertia.value() as f32,
            mass.value() as f32 * collider_mass_props.unit_angular_inertia()
        );
        assert_eq!(*center_of_mass, ComputedCenterOfMass::default());
    }

    #[test]
    fn mass_properties_rb_collider_with_set_mass_and_angular_inertia() {
        // `RigidBody`, `Collider`, `Mass(5.0)`, `AngularInertia(10.0)`

        let mut app = create_app();

        let collider = Collider::circle(1.0);

        let body_entity = app
            .world_mut()
            .spawn((
                RigidBody::Dynamic,
                collider,
                Mass(5.0),
                AngularInertia(10.0),
            ))
            .id();

        app.world_mut().run_schedule(FixedPostUpdate);

        let (mass, angular_inertia, center_of_mass) =
            get_computed_mass_properties(app.world_mut(), body_entity);

        assert_eq!(mass.value() as f32, 5.0);
        assert_eq!(angular_inertia.value() as f32, 10.0);
        assert_eq!(*center_of_mass, ComputedCenterOfMass::default());
    }

    #[test]
    fn mass_properties_rb_collider_with_set_mass_and_child_collider() {
        // `RigidBody`, `Collider`, `Mass(5.0)`
        // - `Collider`, `ColliderDensity(2.0)`

        let mut app = create_app();

        let collider = Collider::circle(1.0);
        let collider_mass_props = collider.mass_properties(2.0);

        let body_entity = app
            .world_mut()
            .spawn((RigidBody::Dynamic, collider.clone(), Mass(5.0)))
            .with_child((collider, ColliderDensity(2.0)))
            .id();

        app.world_mut().run_schedule(FixedPostUpdate);

        let (mass, angular_inertia, center_of_mass) =
            get_computed_mass_properties(app.world_mut(), body_entity);

        assert_eq!(mass.value() as f32, 5.0 + collider_mass_props.mass);
        assert_eq!(
            angular_inertia.value() as f32,
            5.0 * collider_mass_props.unit_angular_inertia() + collider_mass_props.angular_inertia
        );
        assert_eq!(*center_of_mass, ComputedCenterOfMass::default());
    }

    #[test]
    fn mass_properties_rb_collider_with_set_mass_and_child_collider_with_set_mass() {
        // `RigidBody`, `Collider`, `Mass(5.0)`
        // - `Collider`, `ColliderDensity(2.0)`, `Mass(10.0)`

        let mut app = create_app();

        let collider = Collider::circle(1.0);
        let collider_mass_props = collider.mass_properties(1.0);

        let body_entity = app
            .world_mut()
            .spawn((RigidBody::Dynamic, collider.clone(), Mass(5.0)))
            .with_child((collider, ColliderDensity(2.0), Mass(10.0)))
            .id();

        app.world_mut().run_schedule(FixedPostUpdate);

        let (mass, angular_inertia, center_of_mass) =
            get_computed_mass_properties(app.world_mut(), body_entity);

        assert_eq!(mass.value() as f32, 5.0 + 10.0);
        assert_eq!(
            angular_inertia.value() as f32,
            5.0 * collider_mass_props.unit_angular_inertia()
                + 10.0 * collider_mass_props.unit_angular_inertia()
        );
        assert_eq!(*center_of_mass, ComputedCenterOfMass::default());
    }

    #[test]
    fn mass_properties_no_auto_mass_collider_no_set_mass() {
        // `RigidBody`, `Collider`, `NoAutoMass`
        //
        // Mass properties should be zero.

        let mut app = create_app();

        let body_entity = app
            .world_mut()
            .spawn((RigidBody::Dynamic, Collider::circle(1.0), NoAutoMass))
            .id();

        app.world_mut().run_schedule(FixedPostUpdate);

        let (mass, angular_inertia, center_of_mass) =
            get_computed_mass_properties(app.world_mut(), body_entity);

        assert_eq!(*mass, ComputedMass::default());
        assert_eq!(*angular_inertia, ComputedAngularInertia::default());
        assert_eq!(*center_of_mass, ComputedCenterOfMass::default());
    }

    #[test]
    fn mass_properties_no_auto_mass_hierarchy() {
        // `RigidBody`, `Collider`, `Mass(5.0)`, `NoAutoMass`
        // - `Collider`, `ColliderDensity(2.0)`, `Mass(10.0)`

        let mut app = create_app();

        let collider = Collider::circle(1.0);
        let collider_mass_props = collider.mass_properties(1.0);

        let body_entity = app
            .world_mut()
            .spawn((RigidBody::Dynamic, collider.clone(), Mass(5.0), NoAutoMass))
            .with_child((collider, ColliderDensity(2.0), Mass(10.0)))
            .id();

        app.world_mut().run_schedule(FixedPostUpdate);

        let (mass, angular_inertia, center_of_mass) =
            get_computed_mass_properties(app.world_mut(), body_entity);

        assert_eq!(mass.value() as f32, 5.0);
        assert_eq!(
            angular_inertia.value() as f32,
            mass.value() as f32
                * (5.0 * collider_mass_props.unit_angular_inertia()
                    + 10.0 * collider_mass_props.unit_angular_inertia())
                / 15.0
        );
        assert_eq!(*center_of_mass, ComputedCenterOfMass::default());
    }

    #[test]
    fn mass_properties_add_remove_collider() {
        // `RigidBody`, `Collider`
        //
        // - Check mass properties
        // - Add child `Collider`
        // - Check mass properties
        // - Remove child `Collider`
        // - Check mass properties

        let mut app = create_app();

        let collider = Collider::circle(1.0);
        let collider_mass_props = collider.mass_properties(1.0);

        let body_entity = app.world_mut().spawn((RigidBody::Dynamic, collider)).id();

        app.world_mut().run_schedule(FixedPostUpdate);

        let (mass, angular_inertia, center_of_mass) =
            get_computed_mass_properties(app.world_mut(), body_entity);

        assert_eq!(mass.value() as f32, collider_mass_props.mass);
        assert_eq!(
            angular_inertia.value() as f32,
            collider_mass_props.angular_inertia
        );
        assert_eq!(*center_of_mass, ComputedCenterOfMass::default());

        // Add a child collider
        let child_collider = Collider::circle(2.0);
        let child_collider_mass_props = child_collider.mass_properties(1.0);

        let child_entity = app
            .world_mut()
            .spawn((ChildOf(body_entity), child_collider, ColliderDensity(1.0)))
            .id();

        app.world_mut().run_schedule(FixedPostUpdate);

        let (mass, angular_inertia, center_of_mass) =
            get_computed_mass_properties(app.world_mut(), body_entity);

        assert_eq!(
            mass.value() as f32,
            collider_mass_props.mass + child_collider_mass_props.mass
        );
        assert_eq!(
            angular_inertia.value() as f32,
            collider_mass_props.angular_inertia + child_collider_mass_props.angular_inertia
        );
        assert_eq!(*center_of_mass, ComputedCenterOfMass::default());

        // Remove the child collider
        app.world_mut().entity_mut(child_entity).despawn();

        app.world_mut().run_schedule(FixedPostUpdate);

        let (mass, angular_inertia, center_of_mass) =
            get_computed_mass_properties(app.world_mut(), body_entity);

        assert_eq!(mass.value() as f32, collider_mass_props.mass);
        assert_eq!(
            angular_inertia.value() as f32,
            collider_mass_props.angular_inertia
        );
        assert_eq!(*center_of_mass, ComputedCenterOfMass::default());
    }

    #[test]
    fn mass_properties_change_mass() {
        // `RigidBody`, `Collider`, `Mass(5.0)`
        //
        // - Check mass properties
        // - Change mass
        // - Check mass properties

        let mut app = create_app();

        let collider = Collider::circle(1.0);
        let collider_mass_props = collider.mass_properties(1.0);

        let body_entity = app
            .world_mut()
            .spawn((RigidBody::Dynamic, collider, Mass(5.0)))
            .id();

        app.world_mut().run_schedule(FixedPostUpdate);

        let (mass, angular_inertia, center_of_mass) =
            get_computed_mass_properties(app.world_mut(), body_entity);

        assert_eq!(mass.value() as f32, 5.0);
        assert_eq!(
            angular_inertia.value() as f32,
            mass.value() as f32 * collider_mass_props.unit_angular_inertia()
        );
        assert_eq!(*center_of_mass, ComputedCenterOfMass::default());

        // Change mass
        app.world_mut().entity_mut(body_entity).insert(Mass(10.0));

        app.world_mut().run_schedule(FixedPostUpdate);

        let (mass, angular_inertia, center_of_mass) =
            get_computed_mass_properties(app.world_mut(), body_entity);

        assert_eq!(mass.value() as f32, 10.0);
        assert_eq!(
            angular_inertia.value() as f32,
            mass.value() as f32 * collider_mass_props.unit_angular_inertia()
        );
        assert_eq!(*center_of_mass, ComputedCenterOfMass::default());
    }

    #[test]
    fn mass_properties_move_child_collider() {
        // `RigidBody`, `Mass(10.0)`, `CenterOfMass(0.0, -0.5)`, `Transform`
        // - `Collider`, `Mass(5.0)`, `Transform`
        //
        // - Check mass properties
        // - Move child collider
        // - Check mass properties

        let mut app = create_app();

        let body_entity = app
            .world_mut()
            .spawn((
                RigidBody::Dynamic,
                Mass(10.0),
                CenterOfMass::new(0.0, -0.5),
                Transform::default(),
            ))
            .id();

        let child_entity = app
            .world_mut()
            .spawn((
                ChildOf(body_entity),
                Collider::circle(1.0),
                Mass(5.0),
                Transform::default(),
            ))
            .id();

        app.world_mut().run_schedule(FixedPostUpdate);

        let (mass, _, center_of_mass) = get_computed_mass_properties(app.world_mut(), body_entity);

        assert_eq!(mass.value() as f32, 10.0 + 5.0);
        assert_relative_eq!(
            center_of_mass.0,
            Vector::new(0.0, -1.0 / 3.0),
            epsilon = 1.0e-6
        );

        // Move child collider
        app.world_mut()
            .entity_mut(child_entity)
            .insert(Transform::from_xyz(0.0, 4.0, 0.0));

        app.world_mut().run_schedule(FixedPostUpdate);

        let (mass, _, center_of_mass) = get_computed_mass_properties(app.world_mut(), body_entity);

        assert_eq!(mass.value(), 10.0 + 5.0);
        assert_relative_eq!(center_of_mass.0, Vector::new(0.0, 1.0));
    }

    #[test]
    fn mass_properties_no_auto_mass_add_remove() {
        // `RigidBody`, `Collider`, `Mass(5.0)`
        // - `Collider`, `Mass(10.0)`
        //
        // - Check mass properties
        // - Add `NoAutoMass`
        // - Check mass properties
        // - Remove `NoAutoMass`
        // - Check mass properties

        let mut app = create_app();

        let collider = Collider::circle(1.0);
        let collider_mass_props = collider.mass_properties(1.0);

        let body_entity = app
            .world_mut()
            .spawn((RigidBody::Dynamic, collider.clone(), Mass(5.0)))
            .with_child((collider, Mass(10.0)))
            .id();

        app.world_mut().run_schedule(FixedPostUpdate);

        let (mass, angular_inertia, center_of_mass) =
            get_computed_mass_properties(app.world_mut(), body_entity);

        assert_eq!(mass.value() as f32, 5.0 + 10.0);
        assert_eq!(
            angular_inertia.value() as f32,
            5.0 * collider_mass_props.unit_angular_inertia()
                + 10.0 * collider_mass_props.unit_angular_inertia()
        );
        assert_eq!(*center_of_mass, ComputedCenterOfMass::default());

        // Add `NoAutoMass`
        app.world_mut().entity_mut(body_entity).insert(NoAutoMass);

        app.world_mut().run_schedule(FixedPostUpdate);

        let (mass, angular_inertia, center_of_mass) =
            get_computed_mass_properties(app.world_mut(), body_entity);

        assert_eq!(mass.value() as f32, 5.0);
        assert_eq!(
            angular_inertia.value() as f32,
            5.0 * (5.0 * collider_mass_props.unit_angular_inertia()
                + 10.0 * collider_mass_props.unit_angular_inertia())
                / 15.0
        );
        assert_eq!(*center_of_mass, ComputedCenterOfMass::default());

        // Remove `NoAutoMass`
        app.world_mut()
            .entity_mut(body_entity)
            .remove::<NoAutoMass>();

        app.world_mut().run_schedule(FixedPostUpdate);

        let (mass, angular_inertia, center_of_mass) =
            get_computed_mass_properties(app.world_mut(), body_entity);

        assert_eq!(mass.value() as f32, 5.0 + 10.0);
        assert_eq!(
            angular_inertia.value() as f32,
            5.0 * collider_mass_props.unit_angular_inertia()
                + 10.0 * collider_mass_props.unit_angular_inertia()
        );
        assert_eq!(*center_of_mass, ComputedCenterOfMass::default());
    }
}
