//! Renders physics objects and properties for debugging purposes.
//!
//! See [`PhysicsDebugPlugin`].

#![allow(clippy::unnecessary_cast)]

mod configuration;
mod gizmos;

pub use configuration::*;
pub use gizmos::*;

use crate::{dynamics::solver::xpbd::EntityConstraint, prelude::*};
use bevy::{
    ecs::{intern::Interned, query::Has, schedule::ScheduleLabel},
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
/// - [Joints](dynamics::solver::joints)
/// - [`RayCaster`]
/// - [`ShapeCaster`]
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
///             PhysicsDebugPlugin::default(),
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
pub struct PhysicsDebugPlugin {
    schedule: Interned<dyn ScheduleLabel>,
}

impl PhysicsDebugPlugin {
    /// Creates a [`PhysicsDebugPlugin`] with the schedule that is used for running the [`PhysicsSchedule`].
    ///
    /// The default schedule is `FixedPostUpdate`.
    pub fn new(schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: schedule.intern(),
        }
    }
}

impl Default for PhysicsDebugPlugin {
    fn default() -> Self {
        Self::new(PostUpdate)
    }
}

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

        app.register_type::<PhysicsGizmos>()
            .register_type::<DebugRender>()
            .add_systems(
                self.schedule,
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
                    debug_render_joints::<FixedJoint>,
                    debug_render_joints::<PrismaticJoint>,
                    debug_render_joints::<DistanceJoint>,
                    debug_render_joints::<RevoluteJoint>,
                    #[cfg(feature = "3d")]
                    debug_render_joints::<SphericalJoint>,
                    debug_render_raycasts,
                    #[cfg(all(
                        feature = "default-collider",
                        any(feature = "parry-f32", feature = "parry-f64")
                    ))]
                    debug_render_shapecasts,
                )
                    .after(PhysicsSet::StepSimulation)
                    .run_if(|store: Res<GizmoConfigStore>| {
                        store.config::<PhysicsGizmos>().0.enabled
                    }),
            )
            .add_systems(
                self.schedule,
                change_mesh_visibility.after(PhysicsSet::StepSimulation),
            );
    }
}

#[allow(clippy::type_complexity)]
fn debug_render_axes(
    bodies: Query<(
        &Position,
        &Rotation,
        &ComputedCenterOfMass,
        Has<Sleeping>,
        Option<&DebugRender>,
    )>,
    mut gizmos: Gizmos<PhysicsGizmos>,
    store: Res<GizmoConfigStore>,
    length_unit: Res<PhysicsLengthUnit>,
) {
    let config = store.config::<PhysicsGizmos>().1;
    for (pos, rot, local_com, sleeping, render_config) in &bodies {
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

            gizmos.cuboid(
                Transform::from_scale(Vector::from(aabb.size()).extend(0.0).f32())
                    .with_translation(Vector::from(aabb.center()).extend(0.0).f32()),
                color,
            );
        }
    }

    #[cfg(feature = "3d")]
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

            gizmos.cuboid(
                Transform::from_scale(Vector::from(aabb.size()).f32())
                    .with_translation(Vector::from(aabb.center()).f32()),
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
        &Position,
        &Rotation,
        Option<&ColliderOf>,
        Option<&DebugRender>,
    )>,
    sleeping: Query<(), With<Sleeping>>,
    mut gizmos: Gizmos<PhysicsGizmos>,
    store: Res<GizmoConfigStore>,
) {
    let config = store.config::<PhysicsGizmos>().1;
    for (entity, collider, position, rotation, collider_rb, render_config) in &mut colliders {
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
            gizmos.draw_collider(collider, *position, *rotation, color);
        }
    }
}

fn debug_render_contacts(
    colliders: Query<(&Position, &Rotation)>,
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
        let Ok((position1, rotation1)) = colliders.get(contacts.collider1) else {
            continue;
        };
        let Ok((position2, rotation2)) = colliders.get(contacts.collider2) else {
            continue;
        };

        for manifold in contacts.manifolds.iter() {
            for contact in manifold.points.iter() {
                let p1 = contact.global_point1(position1, rotation1);
                let p2 = contact.global_point2(position2, rotation2);

                // Don't render contacts that aren't penetrating
                if contact.penetration <= Scalar::EPSILON {
                    continue;
                }

                // Draw contact points
                if let Some(color) = config.contact_point_color {
                    #[cfg(feature = "2d")]
                    {
                        gizmos.circle_2d(p1.f32(), 0.1 * length_unit.0 as f32, color);
                        gizmos.circle_2d(p2.f32(), 0.1 * length_unit.0 as f32, color);
                    }
                    #[cfg(feature = "3d")]
                    {
                        gizmos.sphere(p1.f32(), 0.1 * length_unit.0 as f32, color);
                        gizmos.sphere(p2.f32(), 0.1 * length_unit.0 as f32, color);
                    }
                }

                // Draw contact normals
                if let Some(color) = config.contact_normal_color {
                    // Use dimmer color for second normal
                    let color_dim = color.mix(&Color::BLACK, 0.5);

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
                        p1,
                        p1 + manifold.normal * length,
                        0.1 * length_unit.0,
                        color,
                    );
                    gizmos.draw_arrow(
                        p2,
                        p2 - manifold.normal * length,
                        0.1 * length_unit.0,
                        color_dim,
                    );
                }
            }
        }
    }
}

fn debug_render_joints<T: Joint + EntityConstraint<2>>(
    bodies: Query<(&Position, &Rotation, Has<Sleeping>)>,
    joints: Query<(&T, Option<&DebugRender>)>,
    mut gizmos: Gizmos<PhysicsGizmos>,
    store: Res<GizmoConfigStore>,
) {
    let config = store.config::<PhysicsGizmos>().1;
    for (joint, render_config) in &joints {
        if let Ok([(pos1, rot1, sleeping1), (pos2, rot2, sleeping2)]) =
            bodies.get_many(joint.entities())
        {
            if let Some(mut anchor_color) = config.joint_anchor_color {
                // If both bodies are sleeping, multiply the color by the sleeping color multiplier
                if sleeping1 && sleeping2 {
                    let hsla = Hsla::from(anchor_color).to_vec4();
                    if let Some(mul) = render_config.map_or(config.sleeping_color_multiplier, |c| {
                        c.sleeping_color_multiplier
                    }) {
                        anchor_color = Hsla::from_vec4(hsla * Vec4::from_array(mul)).into();
                    }
                }

                gizmos.draw_line(pos1.0, pos1.0 + rot1 * joint.local_anchor_1(), anchor_color);
                gizmos.draw_line(pos2.0, pos2.0 + rot2 * joint.local_anchor_2(), anchor_color);
            }
            if let Some(mut separation_color) = config.joint_separation_color {
                // If both bodies are sleeping, multiply the color by the sleeping color multiplier
                if sleeping1 && sleeping2 {
                    let hsla = Hsla::from(separation_color).to_vec4();
                    if let Some(mul) = render_config.map_or(config.sleeping_color_multiplier, |c| {
                        c.sleeping_color_multiplier
                    }) {
                        separation_color = Hsla::from_vec4(hsla * Vec4::from_array(mul)).into();
                    }
                }

                gizmos.draw_line(
                    pos1.0 + rot1 * joint.local_anchor_1(),
                    pos2.0 + rot2 * joint.local_anchor_2(),
                    separation_color,
                );
            }
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
