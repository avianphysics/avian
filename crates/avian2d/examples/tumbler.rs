//! A spinning box with a large number of small boxes tumbling inside of it.
//! This stress tests a dynamic scene where contacts are constantly being
//! created and destroyed.
//!
//! Based on the tumbler sample from the Box2D physics engine.
//! The walls are thinner to also test tunneling prevention.

#![allow(clippy::unnecessary_cast)]

use avian2d::{math::*, prelude::*};
use bevy::prelude::*;
use examples_common_2d::ExampleCommonPlugin;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            ExampleCommonPlugin,
            PhysicsPlugins::default().with_length_unit(0.25),
        ))
        .add_systems(Startup, (spawn_tumbler, setup_camera))
        .run();
}

fn spawn_tumbler(
    mut commands: Commands,
    mut materials: ResMut<Assets<ColorMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let horizontal_wall = Rectangle::new(20.05, 0.05);
    let vertical_wall = Rectangle::new(0.05, 20.05);

    let horizontal_wall_mesh = meshes.add(horizontal_wall);
    let vertical_wall_mesh = meshes.add(vertical_wall);

    let wall_material = materials.add(Color::srgb(0.7, 0.7, 0.8));

    commands.spawn((
        RigidBody::Kinematic,
        AngularVelocity((25.0 as Scalar).to_radians()),
        Transform::default(),
        Visibility::default(),
        children![
            (
                Collider::from(horizontal_wall),
                Mesh2d(horizontal_wall_mesh.clone()),
                MeshMaterial2d(wall_material.clone()),
                Transform::from_xyz(0.0, 10.0, 0.0),
            ),
            (
                Collider::from(horizontal_wall),
                Mesh2d(horizontal_wall_mesh.clone()),
                MeshMaterial2d(wall_material.clone()),
                Transform::from_xyz(0.0, -10.0, 0.0),
            ),
            (
                Collider::from(vertical_wall),
                Mesh2d(vertical_wall_mesh.clone()),
                MeshMaterial2d(wall_material.clone()),
                Transform::from_xyz(10.0, 0.0, 0.0),
            ),
            (
                Collider::from(vertical_wall),
                Mesh2d(vertical_wall_mesh),
                MeshMaterial2d(wall_material),
                Transform::from_xyz(-10.0, 0.0, 0.0),
            ),
        ],
    ));

    let grid_count = 45;

    let rectangle = Rectangle::new(0.25, 0.25);
    let rectangle_collider = Collider::from(rectangle);
    let rectangle_mesh = meshes.add(rectangle);
    let rectangle_material = materials.add(Color::srgb(0.2, 0.7, 0.9));

    let mut y = -0.2 * grid_count as f32;
    for _ in 0..grid_count {
        let mut x = -0.2 * grid_count as f32;
        for _ in 0..grid_count {
            commands.spawn((
                RigidBody::Dynamic,
                rectangle_collider.clone(),
                Mesh2d(rectangle_mesh.clone()),
                MeshMaterial2d(rectangle_material.clone()),
                Transform::from_xyz(x, y, 0.0),
            ));
            x += 0.4;
        }
        y += 0.4;
    }
}

fn setup_camera(mut commands: Commands) {
    commands.spawn((
        Camera2d,
        Projection::Orthographic(OrthographicProjection {
            scale: 0.04,
            ..OrthographicProjection::default_2d()
        }),
    ));
}
