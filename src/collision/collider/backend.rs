//! Handles generic collider backend logic, like initializing colliders and AABBs and updating related components.
//!
//! See [`ColliderBackendPlugin`].

use core::marker::PhantomData;

#[cfg(feature = "collider-from-mesh")]
use crate::collision::collider::cache::ColliderCache;
use crate::{prelude::*, prepare::PrepareSet, sync::SyncConfig};
#[cfg(all(feature = "bevy_scene", feature = "default-collider"))]
use bevy::scene::SceneInstance;
use bevy::{
    ecs::{intern::Interned, schedule::ScheduleLabel, system::SystemId},
    prelude::*,
};
use mass_properties::{components::RecomputeMassProperties, MassPropertySystems};

/// A plugin for handling generic collider backend logic.
///
/// - Initializes colliders, handles [`ColliderConstructor`] and [`ColliderConstructorHierarchy`].
/// - Updates [`ColliderAabb`]s.
/// - Updates collider scale based on `Transform` scale.
/// - Updates [`ColliderMassProperties`].
///
/// This plugin should typically be used together with the [`ColliderHierarchyPlugin`].
///
/// # Custom Collision Backends
///
/// By default, [`PhysicsPlugins`] adds this plugin for the [`Collider`] component.
/// You can also create custom collider backends by implementing the [`AnyCollider`]
/// and [`ScalableCollider`] traits for a type.
///
/// To use a custom collider backend, simply add the [`ColliderBackendPlugin`] with your collider type:
///
/// ```no_run
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
/// #
/// # type MyCollider = Collider;
///
/// fn main() {
///     App::new()
///         .add_plugins((
///             DefaultPlugins,
///             PhysicsPlugins::default(),
///             // MyCollider must implement AnyCollider and ScalableCollider.
///             ColliderBackendPlugin::<MyCollider>::default(),
///             // To enable collision detection for the collider,
///             // we also need to add the NarrowPhasePlugin for it.
///             NarrowPhasePlugin::<MyCollider>::default(),
///         ))
///         // ...your other plugins, systems and resources
///         .run();
/// }
/// ```
///
/// Assuming you have implemented the required traits correctly,
/// it should now work with the rest of the engine just like normal [`Collider`]s!
///
/// **Note**: [Spatial queries](spatial_query) are not supported for custom colliders yet.
pub struct ColliderBackendPlugin<C: ScalableCollider> {
    schedule: Interned<dyn ScheduleLabel>,
    _phantom: PhantomData<C>,
}

impl<C: ScalableCollider> ColliderBackendPlugin<C> {
    /// Creates a [`ColliderBackendPlugin`] with the schedule that is used for running the [`PhysicsSchedule`].
    ///
    /// The default schedule is `FixedPostUpdate`.
    pub fn new(schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: schedule.intern(),
            _phantom: PhantomData,
        }
    }
}

impl<C: ScalableCollider> Default for ColliderBackendPlugin<C> {
    fn default() -> Self {
        Self {
            schedule: FixedPostUpdate.intern(),
            _phantom: PhantomData,
        }
    }
}

impl<C: ScalableCollider> Plugin for ColliderBackendPlugin<C> {
    fn build(&self, app: &mut App) {
        // Register required components for the collider type.
        let _ = app.try_register_required_components::<C, ColliderMarker>();
        let _ = app.try_register_required_components::<C, ColliderAabb>();
        let _ = app.try_register_required_components::<C, CollisionLayers>();
        let _ = app.try_register_required_components::<C, ColliderDensity>();
        let _ = app.try_register_required_components::<C, ColliderMassProperties>();

        // Register the one-shot system that is run for all removed colliders.
        if !app.world().contains_resource::<ColliderRemovalSystem>() {
            let collider_removed_id = app.world_mut().register_system(collider_removed);
            app.insert_resource(ColliderRemovalSystem(collider_removed_id));
        }

        let hooks = app.world_mut().register_component_hooks::<C>();

        // Initialize missing components for colliders.
        hooks.on_add(|mut world, ctx| {
            let existing_global_transform = world
                .entity(ctx.entity)
                .get::<GlobalTransform>()
                .copied()
                .unwrap_or_default();
            let global_transform = if existing_global_transform != GlobalTransform::IDENTITY {
                // This collider was built deferred, probably via `ColliderConstructor`.
                existing_global_transform
            } else {
                // This collider *may* have been `spawn`ed directly on a new entity.
                // As such, its global transform is not yet available.
                // You may notice that this will fail if the hierarchy's scale was updated in this
                // frame. Remember that `GlobalTransform` is not updated in between fixed updates.
                // But this is fine, as `update_collider_scale` will be updated in the next fixed update anyway.
                // The reason why we care about initializing this scale here is for those users that opted out of
                // `update_collider_scale` in order to do their own interpolation, which implies that they won't touch
                // the `Transform` component before the collider is initialized, which in turn means that it will
                // always be initialized with the correct `GlobalTransform`.
                let parent_global_transform = world
                    .entity(ctx.entity)
                    .get::<ChildOf>()
                    .and_then(|&ChildOf(parent)| {
                        world.entity(parent).get::<GlobalTransform>().copied()
                    })
                    .unwrap_or_default();
                let transform = world
                    .entity(ctx.entity)
                    .get::<Transform>()
                    .copied()
                    .unwrap_or_default();
                parent_global_transform * transform
            };

            let scale = global_transform.compute_transform().scale;
            #[cfg(feature = "2d")]
            let scale = scale.xy();

            let mut entity_mut = world.entity_mut(ctx.entity);

            // Make sure the collider is initialized with the correct scale.
            // This overwrites the scale set by the constructor, but that one is
            // meant to be only changed after initialization.
            entity_mut
                .get_mut::<C>()
                .unwrap()
                .set_scale(scale.adjust_precision(), 10);

            let collider = entity_mut.get::<C>().unwrap();

            let density = entity_mut
                .get::<ColliderDensity>()
                .copied()
                .unwrap_or_default();

            let mass_properties = if entity_mut.get::<Sensor>().is_some() {
                MassProperties::ZERO
            } else {
                collider.mass_properties(density.0)
            };

            if let Some(mut collider_mass_properties) =
                entity_mut.get_mut::<ColliderMassProperties>()
            {
                *collider_mass_properties = ColliderMassProperties::from(mass_properties);
            }
        });

        // Register a component hook that removes `ColliderMarker` components
        // and updates rigid bodies when their collider is removed.
        app.world_mut()
            .register_component_hooks::<C>()
            .on_remove(|mut world, ctx| {
                // Remove the `ColliderMarker` associated with the collider.
                // TODO: If the same entity had multiple *different* types of colliders, this would
                //       get removed even if just one collider was removed. This is a very niche edge case though.
                world
                    .commands()
                    .entity(ctx.entity)
                    .try_remove::<ColliderMarker>();

                let entity_ref = world.entity_mut(ctx.entity);

                // Get the rigid body entity that the collider is attached to.
                let Some(collider_of) = entity_ref.get::<ColliderOf>().copied() else {
                    return;
                };

                // Get the ID of the one-shot system run for collider removals.
                let ColliderRemovalSystem(system_id) =
                    *world.resource::<ColliderRemovalSystem>().to_owned();

                // Handle collider removal.
                world.commands().run_system_with(system_id, collider_of);
            });

        // When the `Sensor` component is added to a collider, queue its rigid body for a mass property update.
        app.add_observer(
            |trigger: Trigger<OnAdd, Sensor>,
             mut commands: Commands,
             query: Query<(&ColliderMassProperties, &ColliderOf)>| {
                if let Ok((collider_mass_properties, &ColliderOf { body })) =
                    query.get(trigger.target())
                {
                    // If the collider mass properties are zero, there is nothing to subtract.
                    if *collider_mass_properties == ColliderMassProperties::ZERO {
                        return;
                    }

                    // Queue the rigid body for a mass property update.
                    if let Ok(mut entity_commands) = commands.get_entity(body) {
                        entity_commands.insert(RecomputeMassProperties);
                    }
                }
            },
        );

        // When the `Sensor` component is removed from a collider, update its mass properties.
        app.add_observer(
            |trigger: Trigger<OnRemove, Sensor>,
             mut collider_query: Query<(
                Ref<C>,
                &ColliderDensity,
                &mut ColliderMassProperties,
            )>| {
                if let Ok((collider, density, mut collider_mass_properties)) =
                    collider_query.get_mut(trigger.target())
                {
                    // Update collider mass props.
                    *collider_mass_properties =
                        ColliderMassProperties::from(collider.mass_properties(density.0));
                }
            },
        );

        app.add_systems(
            self.schedule,
            (
                init_transforms::<C>
                    .in_set(PrepareSet::InitTransforms)
                    .after(init_transforms::<RigidBody>),
                (
                    update_collider_scale::<C>.in_set(PrepareSet::Finalize),
                    update_collider_mass_properties::<C>
                        .in_set(MassPropertySystems::UpdateColliderMassProperties),
                )
                    .chain(),
            ),
        );

        #[cfg(feature = "default-collider")]
        app.add_systems(
            Update,
            (
                init_collider_constructors,
                init_collider_constructor_hierarchies,
            ),
        );
    }
}

/// A marker component for colliders. Inserted and removed automatically.
///
/// This is useful for filtering collider entities regardless of the [collider backend](ColliderBackendPlugin).
#[derive(Reflect, Component, Clone, Copy, Debug, Default)]
#[reflect(Component, Debug, Default)]
pub struct ColliderMarker;

/// Generates [`Collider`]s based on [`ColliderConstructor`]s.
///
/// If a [`ColliderConstructor`] requires a mesh, the system keeps running
/// until the mesh associated with the mesh handle is available.
///
/// # Panics
///
/// Panics if the [`ColliderConstructor`] requires a mesh but no mesh handle is found.
#[cfg(feature = "default-collider")]
fn init_collider_constructors(
    mut commands: Commands,
    #[cfg(feature = "collider-from-mesh")] meshes: Res<Assets<Mesh>>,
    #[cfg(feature = "collider-from-mesh")] mesh_handles: Query<&Mesh3d>,
    #[cfg(feature = "collider-from-mesh")] mut collider_cache: Option<ResMut<ColliderCache>>,
    constructors: Query<(
        Entity,
        Option<&Collider>,
        Option<&Name>,
        &ColliderConstructor,
    )>,
) {
    for (entity, existing_collider, name, constructor) in constructors.iter() {
        let name = pretty_name(name, entity);
        if existing_collider.is_some() {
            warn!(
                "Tried to add a collider to entity {name} via {constructor:#?}, \
                but that entity already holds a collider. Skipping.",
            );
            commands.entity(entity).remove::<ColliderConstructor>();
            continue;
        }
        #[cfg(feature = "collider-from-mesh")]
        let collider = if constructor.requires_mesh() {
            let mesh_handle = mesh_handles.get(entity).unwrap_or_else(|_| panic!(
                "Tried to add a collider to entity {name} via {constructor:#?} that requires a mesh, \
                but no mesh handle was found"));
            let Some(mesh) = meshes.get(mesh_handle) else {
                // Mesh required, but not loaded yet
                continue;
            };
            collider_cache
                .as_mut()
                .map(|cache| cache.get_or_insert(mesh_handle, mesh, constructor.clone()))
                .unwrap_or_else(|| Collider::try_from_constructor(constructor.clone(), Some(mesh)))
        } else {
            Collider::try_from_constructor(constructor.clone(), None)
        };

        #[cfg(not(feature = "collider-from-mesh"))]
        let collider = Collider::try_from_constructor(constructor.clone());

        if let Some(collider) = collider {
            commands.entity(entity).insert(collider);
        } else {
            error!(
                "Tried to add a collider to entity {name} via {constructor:#?}, \
                but the collider could not be generated. Skipping.",
            );
        }
        commands.entity(entity).remove::<ColliderConstructor>();
    }
}

/// Generates [`Collider`]s for descendants of entities with the [`ColliderConstructorHierarchy`] component.
///
/// If an entity has a `SceneInstance`, its collider hierarchy is only generated once the scene is ready.
#[cfg(feature = "default-collider")]
fn init_collider_constructor_hierarchies(
    mut commands: Commands,
    #[cfg(feature = "collider-from-mesh")] meshes: Res<Assets<Mesh>>,
    #[cfg(feature = "collider-from-mesh")] mesh_handles: Query<&Mesh3d>,
    #[cfg(feature = "collider-from-mesh")] mut collider_cache: Option<ResMut<ColliderCache>>,
    #[cfg(feature = "bevy_scene")] scene_spawner: Res<SceneSpawner>,
    #[cfg(feature = "bevy_scene")] scenes: Query<&SceneRoot>,
    #[cfg(feature = "bevy_scene")] scene_instances: Query<&SceneInstance>,
    collider_constructors: Query<(Entity, &ColliderConstructorHierarchy)>,
    children: Query<&Children>,
    child_query: Query<(Option<&Name>, Option<&Collider>)>,
) {
    use super::ColliderConstructorHierarchyConfig;

    for (scene_entity, collider_constructor_hierarchy) in collider_constructors.iter() {
        #[cfg(feature = "bevy_scene")]
        {
            if scenes.contains(scene_entity) {
                if let Ok(scene_instance) = scene_instances.get(scene_entity) {
                    if !scene_spawner.instance_is_ready(**scene_instance) {
                        // Wait for the scene to be ready
                        continue;
                    }
                } else {
                    // SceneInstance is added in the SpawnScene schedule, so it might not be available yet
                    continue;
                }
            }
        }

        for child_entity in children.iter_descendants(scene_entity) {
            let Ok((name, existing_collider)) = child_query.get(child_entity) else {
                continue;
            };

            let pretty_name = pretty_name(name, child_entity);

            let default_collider = || {
                Some(ColliderConstructorHierarchyConfig {
                    constructor: collider_constructor_hierarchy.default_constructor.clone(),
                    ..default()
                })
            };

            let collider_data = if let Some(name) = name {
                collider_constructor_hierarchy
                    .config
                    .get(name.as_str())
                    .cloned()
                    .unwrap_or_else(default_collider)
            } else if existing_collider.is_some() {
                warn!("Tried to add a collider to entity {pretty_name} via {collider_constructor_hierarchy:#?}, \
                        but that entity already holds a collider. Skipping. \
                        If this was intentional, add the name of the collider to overwrite to `ColliderConstructorHierarchy.config`.");
                continue;
            } else {
                default_collider()
            };

            // If the configuration is explicitly set to `None`, skip this entity.
            let Some(collider_data) = collider_data else {
                continue;
            };

            // Use the configured constructor if specified, otherwise use the default constructor.
            // If both are `None`, skip this entity.
            let Some(constructor) = collider_data
                .constructor
                .or_else(|| collider_constructor_hierarchy.default_constructor.clone())
            else {
                continue;
            };

            #[cfg(feature = "collider-from-mesh")]
            let collider = if constructor.requires_mesh() {
                let Ok(mesh_handle) = mesh_handles.get(child_entity) else {
                    // This child entity does not have a mesh, so we skip it.
                    continue;
                };
                let Some(mesh) = meshes.get(mesh_handle) else {
                    // Mesh required, but not loaded yet
                    continue;
                };
                collider_cache
                    .as_mut()
                    .map(|cache| cache.get_or_insert(mesh_handle, mesh, constructor.clone()))
                    .unwrap_or_else(|| {
                        Collider::try_from_constructor(constructor.clone(), Some(mesh))
                    })
            } else {
                Collider::try_from_constructor(constructor.clone(), None)
            };

            #[cfg(not(feature = "collider-from-mesh"))]
            let collider = Collider::try_from_constructor(constructor);

            if let Some(collider) = collider {
                commands.entity(child_entity).insert((
                    collider,
                    collider_data
                        .layers
                        .unwrap_or(collider_constructor_hierarchy.default_layers),
                    collider_data
                        .density
                        .unwrap_or(collider_constructor_hierarchy.default_density),
                ));
            } else {
                error!(
                        "Tried to add a collider to entity {pretty_name} via {collider_constructor_hierarchy:#?}, \
                        but the collider could not be generated. Skipping.",
                    );
            }
        }

        commands
            .entity(scene_entity)
            .remove::<ColliderConstructorHierarchy>();
    }
}

#[cfg(feature = "default-collider")]
fn pretty_name(name: Option<&Name>, entity: Entity) -> String {
    name.map(|n| n.to_string())
        .unwrap_or_else(|| format!("<unnamed entity {}>", entity.index()))
}

/// Updates the scale of colliders based on [`Transform`] scale.
#[allow(clippy::type_complexity)]
pub fn update_collider_scale<C: ScalableCollider>(
    mut colliders: ParamSet<(
        // Root bodies
        Query<(&Transform, &mut C), (Without<ChildOf>, Changed<Transform>)>,
        // Child colliders
        Query<(&ColliderTransform, &mut C), (With<ChildOf>, Changed<ColliderTransform>)>,
    )>,
    sync_config: Res<SyncConfig>,
) {
    if sync_config.transform_to_collider_scale {
        // Update collider scale for root bodies
        for (transform, mut collider) in &mut colliders.p0() {
            #[cfg(feature = "2d")]
            let scale = transform.scale.truncate().adjust_precision();
            #[cfg(feature = "3d")]
            let scale = transform.scale.adjust_precision();
            if scale != collider.scale() {
                // TODO: Support configurable subdivision count for shapes that
                //       can't be represented without approximations after scaling.
                collider.set_scale(scale, 10);
            }
        }
    }
    // Update collider scale for child colliders
    for (collider_transform, mut collider) in &mut colliders.p1() {
        if collider_transform.scale != collider.scale() {
            // TODO: Support configurable subdivision count for shapes that
            //       can't be represented without approximations after scaling.
            collider.set_scale(collider_transform.scale, 10);
        }
    }
}

/// A resource that stores the system ID for the system that reacts to collider removals.
#[derive(Resource)]
struct ColliderRemovalSystem(SystemId<In<ColliderOf>>);

/// Updates the mass properties of bodies and wakes bodies up when an attached collider is removed.
///
/// Takes the removed collider's entity, rigid body entity, mass properties, and transform as input.
fn collider_removed(
    In(ColliderOf { body }): In<ColliderOf>,
    mut commands: Commands,
    mut sleep_query: Query<&mut TimeSleeping>,
) {
    let Ok(mut entity_commands) = commands.get_entity(body) else {
        return;
    };

    // Queue the rigid body for mass property recomputation.
    entity_commands.insert(RecomputeMassProperties);

    if let Ok(mut time_sleeping) = sleep_query.get_mut(body) {
        // Wake up the rigid body since removing the collider could also remove active contacts.
        entity_commands.remove::<Sleeping>();
        time_sleeping.0 = 0.0;
    }
}

/// Updates the mass properties of [`Collider`].
#[allow(clippy::type_complexity)]
pub(crate) fn update_collider_mass_properties<C: AnyCollider>(
    mut query: Query<
        (Ref<C>, &ColliderDensity, &mut ColliderMassProperties),
        (Or<(Changed<C>, Changed<ColliderDensity>)>, Without<Sensor>),
    >,
) {
    for (collider, density, mut collider_mass_properties) in &mut query {
        // Update the collider's mass properties.
        *collider_mass_properties =
            ColliderMassProperties::from(collider.mass_properties(density.0));
    }
}

#[cfg(test)]
mod tests {
    #![expect(clippy::unnecessary_cast)]

    #[cfg(feature = "default-collider")]
    use super::*;

    #[test]
    #[cfg(feature = "default-collider")]
    fn sensor_mass_properties() {
        let mut app = App::new();

        app.init_schedule(PhysicsSchedule)
            .init_schedule(SubstepSchedule);

        app.add_plugins((
            PreparePlugin::new(FixedPostUpdate),
            MassPropertyPlugin::new(FixedPostUpdate),
            ColliderHierarchyPlugin,
            ColliderTransformPlugin::default(),
            ColliderBackendPlugin::<Collider>::new(FixedPostUpdate),
        ));

        let collider = Collider::capsule(0.5, 2.0);
        let mass_properties = MassPropertiesBundle::from_shape(&collider, 1.0);

        let parent = app
            .world_mut()
            .spawn((
                RigidBody::Dynamic,
                mass_properties.clone(),
                Transform::default(),
            ))
            .id();

        let child = app
            .world_mut()
            .spawn((
                collider,
                Transform::from_xyz(1.0, 0.0, 0.0),
                ChildOf(parent),
            ))
            .id();

        app.world_mut().run_schedule(FixedPostUpdate);

        assert_eq!(
            app.world()
                .entity(parent)
                .get::<ComputedMass>()
                .expect("rigid body should have mass")
                .value() as f32,
            2.0 * mass_properties.mass.0,
        );
        assert!(
            app.world()
                .entity(parent)
                .get::<ComputedCenterOfMass>()
                .expect("rigid body should have a center of mass")
                .x
                > 0.0,
        );

        // Mark the collider as a sensor. It should no longer contribute to the mass properties of the rigid body.
        let mut entity_mut = app.world_mut().entity_mut(child);
        entity_mut.insert(Sensor);

        app.world_mut().run_schedule(FixedPostUpdate);

        assert_eq!(
            app.world()
                .entity(parent)
                .get::<ComputedMass>()
                .expect("rigid body should have mass")
                .value() as f32,
            mass_properties.mass.0,
        );
        assert!(
            app.world()
                .entity(parent)
                .get::<ComputedCenterOfMass>()
                .expect("rigid body should have a center of mass")
                .x
                == 0.0,
        );

        // Remove the sensor component. The collider should contribute to the mass properties of the rigid body again.
        let mut entity_mut = app.world_mut().entity_mut(child);
        entity_mut.remove::<Sensor>();

        app.world_mut().run_schedule(FixedPostUpdate);

        assert_eq!(
            app.world()
                .entity(parent)
                .get::<ComputedMass>()
                .expect("rigid body should have mass")
                .value() as f32,
            2.0 * mass_properties.mass.0,
        );
        assert!(
            app.world()
                .entity(parent)
                .get::<ComputedCenterOfMass>()
                .expect("rigid body should have a center of mass")
                .x
                > 0.0,
        );
    }
}
