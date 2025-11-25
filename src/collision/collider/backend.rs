//! Handles generic collider backend logic, like initializing colliders and AABBs and updating related components.
//!
//! See [`ColliderBackendPlugin`].

use core::marker::PhantomData;

#[cfg(all(feature = "collider-from-mesh", feature = "default-collider"))]
use crate::collision::collider::cache::ColliderCache;
use crate::{
    collision::broad_phase::BroadPhaseSystems,
    physics_transform::{PhysicsTransformConfig, PhysicsTransformSystems, init_physics_transform},
    prelude::*,
};
#[cfg(all(feature = "bevy_scene", feature = "default-collider"))]
use bevy::scene::SceneInstance;
use bevy::{
    ecs::{intern::Interned, schedule::ScheduleLabel, system::StaticSystemParam},
    prelude::*,
};
use mass_properties::{MassPropertySystems, components::RecomputeMassProperties};

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
        let _ = app.try_register_required_components_with::<C, Position>(|| Position::PLACEHOLDER);
        let _ = app.try_register_required_components_with::<C, Rotation>(|| Rotation::PLACEHOLDER);
        let _ = app.try_register_required_components::<C, ColliderMarker>();
        let _ = app.try_register_required_components::<C, ColliderAabb>();
        let _ = app.try_register_required_components::<C, CollisionLayers>();
        let _ = app.try_register_required_components::<C, ColliderDensity>();
        let _ = app.try_register_required_components::<C, ColliderMassProperties>();

        // Make sure the necessary resources are initialized.
        app.init_resource::<PhysicsTransformConfig>();
        app.init_resource::<NarrowPhaseConfig>();
        app.init_resource::<PhysicsLengthUnit>();

        let hooks = app.world_mut().register_component_hooks::<C>();

        // Initialize missing components for colliders.
        hooks
            .on_add(|mut world, ctx| {
                // Initialize the global physics transform for the collider.
                // Avoid doing this twice for rigid bodies added at the same time.
                // TODO: The special case for rigid bodies is a bit of a hack here.
                if !world.entity(ctx.entity).contains::<RigidBody>() {
                    init_physics_transform(&mut world, &ctx);
                }
            })
            .on_insert(|mut world, ctx| {
                let scale = world
                    .entity(ctx.entity)
                    .get::<GlobalTransform>()
                    .map(|gt| gt.scale())
                    .unwrap_or_default();
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
                let Some(ColliderOf { body }) = entity_ref.get::<ColliderOf>().copied() else {
                    return;
                };

                // Queue the rigid body for a mass property update.
                world
                    .commands()
                    .entity(body)
                    .try_insert(RecomputeMassProperties);
            });

        // Initialize `ColliderAabb` for colliders.
        app.add_observer(
            |trigger: On<Add, C>,
             mut query: Query<(
                &C,
                &Position,
                &Rotation,
                Option<&CollisionMargin>,
                &mut ColliderAabb,
            )>,
             narrow_phase_config: Res<NarrowPhaseConfig>,
             length_unit: Res<PhysicsLengthUnit>,
             collider_context: StaticSystemParam<C::Context>| {
                let contact_tolerance = length_unit.0 * narrow_phase_config.contact_tolerance;
                let aabb_context = AabbContext::new(trigger.entity, &*collider_context);

                if let Ok((collider, pos, rot, collision_margin, mut aabb)) =
                    query.get_mut(trigger.entity)
                {
                    let collision_margin = collision_margin.map_or(0.0, |m| m.0);
                    *aabb = collider
                        .aabb_with_context(pos.0, *rot, aabb_context)
                        .grow(Vector::splat(contact_tolerance + collision_margin));
                }
            },
        );

        // When the `Sensor` component is added to a collider, queue its rigid body for a mass property update.
        app.add_observer(
            |trigger: On<Add, Sensor>,
             mut commands: Commands,
             query: Query<(&ColliderMassProperties, &ColliderOf)>| {
                if let Ok((collider_mass_properties, &ColliderOf { body })) =
                    query.get(trigger.entity)
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
            |trigger: On<Remove, Sensor>,
             mut collider_query: Query<(
                Ref<C>,
                &ColliderDensity,
                &mut ColliderMassProperties,
            )>| {
                if let Ok((collider, density, mut collider_mass_properties)) =
                    collider_query.get_mut(trigger.entity)
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
                update_collider_scale::<C>
                    .in_set(PhysicsSystems::Prepare)
                    .after(PhysicsTransformSystems::TransformToPosition),
                update_collider_mass_properties::<C>
                    .in_set(MassPropertySystems::UpdateColliderMassProperties),
            )
                .chain(),
        );

        let physics_schedule = app
            .get_schedule_mut(PhysicsSchedule)
            .expect("add PhysicsSchedule first");

        // Allowing ambiguities is required so that it's possible
        // to have multiple collision backends at the same time.
        physics_schedule.add_systems(
            update_aabb::<C>
                .in_set(PhysicsStepSystems::BroadPhase)
                .after(BroadPhaseSystems::First)
                .before(BroadPhaseSystems::UpdateStructures)
                .ambiguous_with_all(),
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
            commands.trigger(ColliderConstructorReady { entity })
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
                warn!(
                    "Tried to add a collider to entity {pretty_name} via {collider_constructor_hierarchy:#?}, \
                        but that entity already holds a collider. Skipping. \
                        If this was intentional, add the name of the collider to overwrite to `ColliderConstructorHierarchy.config`."
                );
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

        commands.trigger(ColliderConstructorHierarchyReady {
            entity: scene_entity,
        })
    }
}

#[cfg(feature = "default-collider")]
fn pretty_name(name: Option<&Name>, entity: Entity) -> String {
    name.map(|n| n.to_string())
        .unwrap_or_else(|| format!("<unnamed entity {}>", entity.index()))
}

/// Updates the Axis-Aligned Bounding Boxes of all colliders.
#[allow(clippy::type_complexity)]
fn update_aabb<C: AnyCollider>(
    mut colliders: Query<
        (
            Entity,
            &C,
            &mut ColliderAabb,
            &Position,
            &Rotation,
            Option<&ColliderOf>,
            Option<&CollisionMargin>,
            Option<&SpeculativeMargin>,
            Has<SweptCcd>,
            Option<&LinearVelocity>,
            Option<&AngularVelocity>,
        ),
        Or<(
            Changed<Position>,
            Changed<Rotation>,
            Changed<LinearVelocity>,
            Changed<AngularVelocity>,
            Changed<C>,
        )>,
    >,
    rb_velocities: Query<
        (
            &Position,
            &Rotation,
            &ComputedCenterOfMass,
            &LinearVelocity,
            &AngularVelocity,
        ),
        With<Children>,
    >,
    narrow_phase_config: Res<NarrowPhaseConfig>,
    length_unit: Res<PhysicsLengthUnit>,
    time: Res<Time>,
    collider_context: StaticSystemParam<C::Context>,
) {
    let delta_secs = time.delta_seconds_adjusted();
    let default_speculative_margin = length_unit.0 * narrow_phase_config.default_speculative_margin;
    let contact_tolerance = length_unit.0 * narrow_phase_config.contact_tolerance;

    for (
        entity,
        collider,
        mut aabb,
        pos,
        rot,
        collider_of,
        collision_margin,
        speculative_margin,
        has_swept_ccd,
        lin_vel,
        ang_vel,
    ) in &mut colliders
    {
        let collision_margin = collision_margin.map_or(0.0, |margin| margin.0);
        let speculative_margin = if has_swept_ccd {
            Scalar::MAX
        } else {
            speculative_margin.map_or(default_speculative_margin, |margin| margin.0)
        };

        let context = AabbContext::new(entity, &*collider_context);

        if speculative_margin <= 0.0 {
            *aabb = collider
                .aabb_with_context(pos.0, *rot, context)
                .grow(Vector::splat(contact_tolerance + collision_margin));
            continue;
        }

        // Expand the AABB based on the body's velocity and CCD speculative margin.
        let (lin_vel, ang_vel) = if let (Some(lin_vel), Some(ang_vel)) = (lin_vel, ang_vel) {
            (*lin_vel, *ang_vel)
        } else if let Some(Ok((rb_pos, rb_rot, center_of_mass, lin_vel, ang_vel))) =
            collider_of.map(|&ColliderOf { body }| rb_velocities.get(body))
        {
            // If the rigid body is rotating, off-center colliders will orbit around it,
            // which affects their linear velocities. We need to compute the linear velocity
            // at the offset position.
            // TODO: This assumes that the colliders would continue moving in the same direction,
            //       but because they are orbiting, the direction will change. We should take
            //       into account the uniform circular motion.
            let offset = pos.0 - rb_pos.0 - rb_rot * center_of_mass.0;
            #[cfg(feature = "2d")]
            let vel_at_offset =
                lin_vel.0 + Vector::new(-ang_vel.0 * offset.y, ang_vel.0 * offset.x) * 1.0;
            #[cfg(feature = "3d")]
            let vel_at_offset = lin_vel.0 + ang_vel.cross(offset);
            (LinearVelocity(vel_at_offset), *ang_vel)
        } else {
            (LinearVelocity::ZERO, AngularVelocity::ZERO)
        };

        // Current position and predicted position for next feame
        let (start_pos, start_rot) = (*pos, *rot);
        let (end_pos, end_rot) = {
            #[cfg(feature = "2d")]
            {
                (
                    pos.0
                        + (lin_vel.0 * delta_secs)
                            .clamp_length_max(speculative_margin.max(contact_tolerance)),
                    *rot * Rotation::radians(ang_vel.0 * delta_secs),
                )
            }
            #[cfg(feature = "3d")]
            {
                let end_rot =
                    Rotation(Quaternion::from_scaled_axis(ang_vel.0 * delta_secs) * rot.0)
                        .fast_renormalize();
                (
                    pos.0
                        + (lin_vel.0 * delta_secs)
                            .clamp_length_max(speculative_margin.max(contact_tolerance)),
                    end_rot,
                )
            }
        };
        // Compute swept AABB, the space that the body would occupy if it was integrated for one frame
        // TODO: Should we expand the AABB in all directions for speculative contacts?
        *aabb = collider
            .swept_aabb_with_context(start_pos.0, start_rot, end_pos, end_rot, context)
            .grow(Vector::splat(contact_tolerance + collision_margin));
    }
}

/// Updates the scale of colliders based on [`Transform`] scale.
#[allow(clippy::type_complexity)]
pub fn update_collider_scale<C: ScalableCollider>(
    mut colliders: ParamSet<(
        // Root bodies
        Query<(&Transform, &mut C), (Without<ChildOf>, Or<(Changed<Transform>, Changed<C>)>)>,
        // Child colliders
        Query<
            (&ColliderTransform, &mut C),
            (With<ChildOf>, Or<(Changed<ColliderTransform>, Changed<C>)>),
        >,
    )>,
    config: Res<PhysicsTransformConfig>,
) {
    if config.transform_to_collider_scale {
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
    #![allow(clippy::unnecessary_cast)]

    #[cfg(feature = "default-collider")]
    use super::*;

    #[test]
    #[cfg(feature = "default-collider")]
    fn sensor_mass_properties() {
        let mut app = App::new();

        app.init_schedule(PhysicsSchedule)
            .init_schedule(SubstepSchedule);

        app.add_plugins((
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
