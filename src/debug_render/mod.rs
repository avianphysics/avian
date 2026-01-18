//! Renders physics objects and properties for debugging purposes.
//!
//! See [`PhysicsDebugPlugin`].

#![allow(clippy::unnecessary_cast)]

mod configuration;
mod gizmos;

pub use configuration::*;
pub use gizmos::*;

use crate::{
    dynamics::{
        joints::EntityConstraint,
        solver::islands::{BodyIslandNode, PhysicsIslands},
    },
    prelude::*,
};
use bevy::{
    camera::visibility::VisibilitySystems,
    ecs::{
        query::Has,
        system::{StaticSystemParam, SystemParam, SystemParamItem},
    },
    prelude::*,
};

/// A plugin that renders physics objects and properties for debugging purposes.
/// It is not enabled by default and must be added manually.
///
/// Currently, the following are supported for debug rendering:
///
/// - The axes and center of mass of [rigid bodies](RigidBody)
/// - [AABBs](ColliderAabb)
/// - [Collider] wireframes
/// - Using different colors for [sleeping](Sleeping) bodies
/// - [Contacts](ContactPair)
/// - [Joints](dynamics::joints)
/// - [`RayCaster`]
/// - [`ShapeCaster`]
/// - [Simulation islands](dynamics::solver::islands)
/// - Changing the visibility of entities to only show debug rendering
///
/// By default, [AABBs](ColliderAabb) and [contacts](ContactPair) are not debug rendered.
/// You can configure the [`PhysicsGizmos`] retrieved from `GizmoConfigStore` for the global configuration
/// and the [`DebugRender`] component for entity-level configuration.
///
/// # Example
///
/// ```no_run
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// fn main() {
///     App::new()
///         .add_plugins((
///             DefaultPlugins,
///             PhysicsPlugins::default(),
///             // Enables debug rendering
///             PhysicsDebugPlugin,
///         ))
///         // Overwrite default debug rendering configuration (optional)
///         .insert_gizmo_config(
///             PhysicsGizmos {
///                 aabb_color: Some(Color::WHITE),
///                 ..default()
///             },
///             GizmoConfig::default(),
///         )
///         .run();
/// }
///
/// fn setup(mut commands: Commands) {
///     // This rigid body and its collider and AABB will get rendered
///     commands.spawn((
///         RigidBody::Dynamic,
#[cfg_attr(feature = "2d", doc = "        Collider::circle(0.5),")]
#[cfg_attr(feature = "3d", doc = "        Collider::sphere(0.5),")]
///         // Overwrite default collider color (optional)
///         DebugRender::default().with_collider_color(Color::srgb(1.0, 0.0, 0.0)),
///     ));
/// }
/// ```
#[derive(Default)]
pub struct PhysicsDebugPlugin;

impl Plugin for PhysicsDebugPlugin {
    fn build(&self, app: &mut App) {
        app.init_gizmo_group::<PhysicsGizmos>();

        let mut store = app.world_mut().resource_mut::<GizmoConfigStore>();
        let config = store.config_mut::<PhysicsGizmos>().0;
        #[cfg(feature = "2d")]
        {
            config.line.width = 2.0;
        }
        #[cfg(feature = "3d")]
        {
            config.line.width = 1.5;
        }

        app.add_systems(
            PostUpdate,
            (
                debug_render_axes,
                debug_render_aabbs,
                #[cfg(all(
                    feature = "default-collider",
                    any(feature = "parry-f32", feature = "parry-f64")
                ))]
                debug_render_colliders,
                debug_render_contacts,
                // TODO: Refactor joints to allow iterating over all of them without generics
                debug_render_constraint::<FixedJoint, 2>,
                debug_render_constraint::<PrismaticJoint, 2>,
                debug_render_constraint::<DistanceJoint, 2>,
                debug_render_constraint::<RevoluteJoint, 2>,
                #[cfg(feature = "3d")]
                debug_render_constraint::<SphericalJoint, 2>,
                debug_render_raycasts,
                #[cfg(all(
                    feature = "default-collider",
                    any(feature = "parry-f32", feature = "parry-f64")
                ))]
                debug_render_shapecasts,
                debug_render_islands.run_if(resource_exists::<PhysicsIslands>),
            )
                .after(TransformSystems::Propagate)
                .run_if(|store: Res<GizmoConfigStore>| store.config::<PhysicsGizmos>().0.enabled),
        )
        .add_systems(
            PostUpdate,
            change_mesh_visibility.before(VisibilitySystems::CalculateBounds),
        );
    }
}

#[allow(clippy::type_complexity)]
fn debug_render_axes(
    bodies: Query<(
        &GlobalTransform,
        &ComputedCenterOfMass,
        Has<Sleeping>,
        Option<&DebugRender>,
    )>,
    mut gizmos: Gizmos<PhysicsGizmos>,
    store: Res<GizmoConfigStore>,
    length_unit: Res<PhysicsLengthUnit>,
) {
    let config = store.config::<PhysicsGizmos>().1;
    for (transform, local_com, sleeping, render_config) in &bodies {
        let pos = Position::from(transform);
        let rot = Rotation::from(transform);

        // If the body is sleeping, the colors will be multiplied by the sleeping color multiplier
        if let Some(mut lengths) = render_config.map_or(config.axis_lengths, |c| c.axis_lengths) {
            lengths *= length_unit.0;

            let mul = if sleeping {
                render_config
                    .map_or(config.sleeping_color_multiplier, |c| {
                        c.sleeping_color_multiplier
                    })
                    .unwrap_or([1.0; 4])
            } else {
                [1.0; 4]
            };
            let [x_color, y_color, _z_color] = [
                Color::hsla(0.0, 1.0 * mul[1], 0.5 * mul[2], 1.0 * mul[3]),
                Color::hsla(120.0 * mul[0], 1.0 * mul[1], 0.4 * mul[2], 1.0 * mul[3]),
                Color::hsla(220.0 * mul[0], 1.0 * mul[1], 0.6 * mul[2], 1.0 * mul[3]),
            ];
            let global_com = pos.0 + rot * local_com.0;

            let x = rot * (Vector::X * lengths.x);
            gizmos.draw_line(global_com - x, global_com + x, x_color);

            let y = rot * (Vector::Y * lengths.y);
            gizmos.draw_line(global_com - y, global_com + y, y_color);

            #[cfg(feature = "3d")]
            {
                let z = rot * (Vector::Z * lengths.z);
                gizmos.draw_line(global_com - z, global_com + z, _z_color);
            }
        }
    }
}

fn debug_render_aabbs(
    aabbs: Query<(
        Entity,
        &ColliderAabb,
        Option<&ColliderOf>,
        Option<&DebugRender>,
    )>,
    sleeping: Query<(), With<Sleeping>>,
    mut gizmos: Gizmos<PhysicsGizmos>,
    store: Res<GizmoConfigStore>,
) {
    let config = store.config::<PhysicsGizmos>().1;
    #[cfg(feature = "2d")]
    for (entity, aabb, collider_rb, render_config) in &aabbs {
        if let Some(mut color) = render_config.map_or(config.aabb_color, |c| c.aabb_color) {
            let collider_rb = collider_rb.map_or(entity, |c| c.body);

            // If the body is sleeping, multiply the color by the sleeping color multiplier
            if sleeping.contains(collider_rb) {
                let hsla = Hsla::from(color).to_vec4();
                if let Some(mul) = render_config.map_or(config.sleeping_color_multiplier, |c| {
                    c.sleeping_color_multiplier
                }) {
                    color = Hsla::from_vec4(hsla * Vec4::from_array(mul)).into();
                }
            }

            gizmos.rect_2d(aabb.center().f32(), aabb.size().f32(), color);
        }
    }

    #[cfg(feature = "3d")]
    for (entity, aabb, collider_rb, render_config) in &aabbs {
        if let Some(mut color) = render_config.map_or(config.aabb_color, |c| c.aabb_color) {
            use bevy_math::bounding::Aabb3d;

            let collider_rb = collider_rb.map_or(entity, |c| c.body);

            // If the body is sleeping, multiply the color by the sleeping color multiplier
            if sleeping.contains(collider_rb) {
                let hsla = Hsla::from(color).to_vec4();
                if let Some(mul) = render_config.map_or(config.sleeping_color_multiplier, |c| {
                    c.sleeping_color_multiplier
                }) {
                    color = Hsla::from_vec4(hsla * Vec4::from_array(mul)).into();
                }
            }

            gizmos.aabb_3d(
                Aabb3d {
                    min: Vec3A::from(aabb.min.f32()),
                    max: Vec3A::from(aabb.max.f32()),
                },
                Transform::IDENTITY,
                color,
            );
        }
    }
}

#[cfg(all(
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
#[allow(clippy::type_complexity)]
fn debug_render_colliders(
    mut colliders: Query<(
        Entity,
        &Collider,
        &GlobalTransform,
        Option<&ColliderOf>,
        Option<&DebugRender>,
    )>,
    sleeping: Query<(), With<Sleeping>>,
    mut gizmos: Gizmos<PhysicsGizmos>,
    store: Res<GizmoConfigStore>,
) {
    let config = store.config::<PhysicsGizmos>().1;
    for (entity, collider, transform, collider_rb, render_config) in &mut colliders {
        let position = Position::from(transform);
        let rotation = Rotation::from(transform);
        if let Some(mut color) = render_config.map_or(config.collider_color, |c| c.collider_color) {
            let collider_rb = collider_rb.map_or(entity, |c| c.body);

            // If the body is sleeping, multiply the color by the sleeping color multiplier
            if sleeping.contains(collider_rb) {
                let hsla = Hsla::from(color).to_vec4();
                if let Some(mul) = render_config.map_or(config.sleeping_color_multiplier, |c| {
                    c.sleeping_color_multiplier
                }) {
                    color = Hsla::from_vec4(hsla * Vec4::from_array(mul)).into();
                }
            }
            gizmos.draw_collider(collider, position, rotation, color);
        }
    }
}

fn debug_render_contacts(
    collisions: Collisions,
    mut gizmos: Gizmos<PhysicsGizmos>,
    store: Res<GizmoConfigStore>,
    time: Res<Time<Substeps>>,
    length_unit: Res<PhysicsLengthUnit>,
) {
    let config = store.config::<PhysicsGizmos>().1;

    if config.contact_point_color.is_none() && config.contact_normal_color.is_none() {
        return;
    }

    for contacts in collisions.iter() {
        for manifold in contacts.manifolds.iter() {
            for contact in manifold.points.iter() {
                // Don't render contacts that aren't penetrating
                if contact.penetration <= Scalar::EPSILON {
                    continue;
                }

                // Draw contact points
                if let Some(color) = config.contact_point_color {
                    #[cfg(feature = "2d")]
                    {
                        gizmos.circle_2d(contact.point.f32(), 0.1 * length_unit.0 as f32, color);
                    }
                    #[cfg(feature = "3d")]
                    {
                        gizmos.sphere(contact.point.f32(), 0.1 * length_unit.0 as f32, color);
                    }
                }

                // Draw contact normals
                if let Some(color) = config.contact_normal_color {
                    // The length of the normal arrows
                    let length = length_unit.0
                        * match config.contact_normal_scale {
                            ContactGizmoScale::Constant(length) => length,
                            ContactGizmoScale::Scaled(scale) => {
                                scale * contact.normal_impulse
                                    / time.delta_secs_f64().adjust_precision()
                            }
                        };

                    gizmos.draw_arrow(
                        contact.point,
                        contact.point + manifold.normal * length,
                        0.1 * length_unit.0,
                        color,
                    );
                }
            }
        }
    }
}

/// A trait for rendering debug information about constraints.
///
/// Add the [`debug_render_constraint`] system to render all constraints that implement this trait.
pub trait DebugRenderConstraint<const N: usize>: EntityConstraint<N> {
    /// A [`SystemParam`] type for any additional ECS access required for rendering the constraint.
    type Context: SystemParam;

    /// Renders the debug information for the constraint.
    fn debug_render(
        &self,
        positions: [Vector; N],
        rotations: [Rotation; N],
        context: &mut SystemParamItem<Self::Context>,
        gizmos: &mut Gizmos<PhysicsGizmos>,
        config: &PhysicsGizmos,
    );
}

/// A system that renders all constraints that implement the [`DebugRenderConstraint`] trait.
pub fn debug_render_constraint<T: Component + DebugRenderConstraint<N>, const N: usize>(
    bodies: Query<&GlobalTransform>,
    constraints: Query<&T>,
    mut gizmos: Gizmos<PhysicsGizmos>,
    store: Res<GizmoConfigStore>,
    mut context: StaticSystemParam<T::Context>,
) {
    let config = store.config::<PhysicsGizmos>().1;
    for constraint in &constraints {
        if let Ok(bodies) = bodies.get_many(constraint.entities()) {
            let positions: [Vector; N] = bodies
                .iter()
                .map(|transform| Position::from(**transform).0)
                .collect::<Vec<_>>()
                .try_into()
                .unwrap();
            let rotations: [Rotation; N] = bodies
                .iter()
                .map(|transform| Rotation::from(**transform))
                .collect::<Vec<_>>()
                .try_into()
                .unwrap();

            constraint.debug_render(positions, rotations, &mut context, &mut gizmos, config);
        }
    }
}

fn debug_render_raycasts(
    query: Query<(&RayCaster, &RayHits)>,
    mut gizmos: Gizmos<PhysicsGizmos>,
    store: Res<GizmoConfigStore>,
    length_unit: Res<PhysicsLengthUnit>,
) {
    let config = store.config::<PhysicsGizmos>().1;
    for (ray, hits) in &query {
        let ray_color = config.raycast_color.unwrap_or(Color::NONE);
        let point_color = config.raycast_point_color.unwrap_or(Color::NONE);
        let normal_color = config.raycast_normal_color.unwrap_or(Color::NONE);

        gizmos.draw_raycast(
            ray.global_origin(),
            ray.global_direction(),
            // f32::MAX renders nothing, but this number seems to be fine :P
            ray.max_distance.min(1_000_000_000_000_000_000.0),
            hits.as_slice(),
            ray_color,
            point_color,
            normal_color,
            length_unit.0,
        );
    }
}

#[cfg(all(
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
fn debug_render_shapecasts(
    query: Query<(&ShapeCaster, &ShapeHits)>,
    mut gizmos: Gizmos<PhysicsGizmos>,
    store: Res<GizmoConfigStore>,
    length_unit: Res<PhysicsLengthUnit>,
) {
    let config = store.config::<PhysicsGizmos>().1;
    for (shape_caster, hits) in &query {
        let ray_color = config.shapecast_color.unwrap_or(Color::NONE);
        let shape_color = config.shapecast_shape_color.unwrap_or(Color::NONE);
        let point_color = config.shapecast_point_color.unwrap_or(Color::NONE);
        let normal_color = config.shapecast_normal_color.unwrap_or(Color::NONE);

        gizmos.draw_shapecast(
            &shape_caster.shape,
            shape_caster.global_origin(),
            shape_caster.global_shape_rotation(),
            shape_caster.global_direction(),
            // f32::MAX renders nothing, but this number seems to be fine :P
            shape_caster.max_distance.min(1_000_000_000_000_000.0),
            hits.as_slice(),
            ray_color,
            shape_color,
            point_color,
            normal_color,
            length_unit.0,
        );
    }
}

fn debug_render_islands(
    islands: Res<PhysicsIslands>,
    bodies: Query<(&RigidBodyColliders, &BodyIslandNode)>,
    aabbs: Query<&ColliderAabb>,
    mut gizmos: Gizmos<PhysicsGizmos>,
    store: Res<GizmoConfigStore>,
) {
    let config = store.config::<PhysicsGizmos>().1;

    for island in islands.iter() {
        if let Some(mut color) = config.island_color {
            // If the island is sleeping, multiply the color by the sleeping color multiplier
            if island.is_sleeping {
                let hsla = Hsla::from(color).to_vec4();
                if let Some(mul) = config.sleeping_color_multiplier {
                    color = Hsla::from_vec4(hsla * Vec4::from_array(mul)).into();
                }
            }

            // If the island is empty, skip rendering
            if island.body_count == 0 {
                continue;
            }

            let mut body = island.head_body;
            let mut aabb: Option<ColliderAabb> = None;

            // Compute the island's AABB by merging the AABBs of all bodies in the island.
            while let Some(next_body) = body {
                if let Ok((colliders, body_island)) = bodies.get(next_body) {
                    for collider in colliders.iter() {
                        if let Ok(collider_aabb) = aabbs.get(collider) {
                            aabb = Some(
                                aabb.map_or(*collider_aabb, |aabb| aabb.merged(*collider_aabb)),
                            );
                        }
                    }
                    body = body_island.next;
                } else {
                    break;
                }
            }

            let Some(aabb) = aabb else {
                continue;
            };

            // Render the island's AABB.
            #[cfg(feature = "2d")]
            {
                gizmos.rect_2d(aabb.center().f32(), aabb.size().f32(), color);
            }
            #[cfg(feature = "3d")]
            {
                use bevy_math::bounding::Aabb3d;

                gizmos.aabb_3d(
                    Aabb3d {
                        min: Vec3A::from(aabb.min.f32()),
                        max: Vec3A::from(aabb.max.f32()),
                    },
                    Transform::IDENTITY,
                    color,
                );
            }
        }
    }
}

type MeshVisibilityQueryFilter = (
    With<RigidBody>,
    Or<(Changed<DebugRender>, Without<DebugRender>)>,
);

fn change_mesh_visibility(
    mut meshes: Query<(&mut Visibility, Option<&DebugRender>), MeshVisibilityQueryFilter>,
    store: Res<GizmoConfigStore>,
) {
    let config = store.config::<PhysicsGizmos>();
    if store.is_changed() {
        for (mut visibility, render_config) in &mut meshes {
            let hide_mesh =
                config.0.enabled && render_config.map_or(config.1.hide_meshes, |c| c.hide_mesh);
            if hide_mesh {
                *visibility = Visibility::Hidden;
            } else {
                *visibility = Visibility::Visible;
            }
        }
    }
}
