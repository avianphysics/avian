//! A simple raycasting example that uses the [`RayCaster`] component.
//!
//! An alternative, more controlled approach is to use the methods of
//! the [`SpatialQuery`] system parameter.

#![allow(clippy::unnecessary_cast)]

use avian2d::{math::*, prelude::*};
use bevy::{
    color::palettes::css::{GREEN, ORANGE_RED},
    prelude::*,
};
use examples_common_2d::*;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            ExampleCommonPlugin,
            PhysicsPlugins::default(),
        ))
        .insert_resource(ClearColor(Color::srgb(0.05, 0.05, 0.1)))
        .add_systems(Update, render_rays)
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    commands.spawn(Camera2d);

    // Spawn a perimeter of circles that the ray will be cast against
    let radius = 16.0;
    for x in -4..=4 {
        for y in -4..=4 {
            if (-3..4).contains(&x) && (-3..4).contains(&y) {
                continue;
            }

            commands.spawn((
                Mesh2d(meshes.add(Circle::new(radius))),
                MeshMaterial2d(materials.add(Color::srgb(0.2, 0.7, 0.9))),
                Transform::from_xyz(x as f32 * radius * 3.0, y as f32 * radius * 3.0, 0.0),
                Collider::circle(radius),
            ));
        }
    }

    // Spawn a rotating kinematic body with a ray caster
    commands.spawn((
        RigidBody::Kinematic,
        AngularVelocity(0.2),
        RayCaster::new(RVec2::ZERO, Dir2::X),
    ));
}

// Note: The `PhysicsDebugPlugin` can also render rays, hit points, and normals.
//       This system is primarily for demonstration purposes.
fn render_rays(mut rays: Query<(&mut RayCaster, &mut RayHits)>, mut gizmos: Gizmos) {
    for (ray, hits) in &mut rays {
        // Convert to Vec3 for lines
        let origin = ray.global_origin().f32();
        let direction = ray.global_direction();

        for hit in hits.iter() {
            gizmos.line_2d(origin, origin + direction * hit.distance as f32, GREEN);
        }
        if hits.is_empty() {
            gizmos.line_2d(origin, origin + direction * 1_000_000.0, ORANGE_RED);
        }
    }
}
