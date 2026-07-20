#![allow(clippy::unnecessary_cast)]

use avian2d::prelude::*;
use bevy::prelude::*;
use examples_common_2d::ExampleCommonPlugin;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            ExampleCommonPlugin,
            // Add physics plugins and specify a units-per-meter scaling factor, 1 meter = 15 pixels.
            // The unit allows the engine to tune its parameters for the scale of the world, improving stability.
            PhysicsPlugins::default().with_length_unit(15.0),
        ))
        .insert_resource(ClearColor(Color::srgb(0.05, 0.05, 0.1)))
        .insert_resource(Gravity(Vec2::NEG_Y * 1000.0))
        .add_systems(Startup, setup)
        .run();
}

// Define the collision layers
#[derive(PhysicsLayer, Default)]
enum Layer {
    #[default]
    Default,
    Blue,
    Red,
}

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<ColorMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    commands.spawn(Camera2d);

    // Spawn blue platform that belongs on the blue layer and collides with blue
    commands.spawn((
        Sprite {
            color: Color::srgb(0.2, 0.7, 0.9),
            custom_size: Some(Vec2::new(500.0, 25.0)),
            ..default()
        },
        Transform::from_xyz(0.0, -50.0, 0.0),
        RigidBody::Static,
        Collider::rectangle(500.0, 25.0),
        CollisionLayers::new([Layer::Blue], [Layer::Blue]),
    ));

    // Spawn red platform that belongs on the red layer and collides with red
    commands.spawn((
        Sprite {
            color: Color::srgb(0.9, 0.3, 0.3),
            custom_size: Some(Vec2::new(500.0, 25.0)),
            ..default()
        },
        Transform::from_xyz(0.0, -200.0, 0.0),
        RigidBody::Static,
        Collider::rectangle(500.0, 25.0),
        CollisionLayers::new([Layer::Red], [Layer::Red]),
    ));

    let marble_radius = 7.5;
    let marble_mesh = meshes.add(Circle::new(marble_radius));

    // Spawn blue marbles that belong on the blue layer and collide with blue
    let blue_material = materials.add(Color::srgb(0.2, 0.7, 0.9));
    for x in -6..6 {
        for y in 0..4 {
            commands.spawn((
                Mesh2d(marble_mesh.clone()),
                MeshMaterial2d(blue_material.clone()),
                Transform::from_xyz(
                    x as f32 * 2.5 * marble_radius,
                    y as f32 * 2.5 * marble_radius + 200.0,
                    0.0,
                ),
                RigidBody::Dynamic,
                Collider::circle(marble_radius),
                CollisionLayers::new([Layer::Blue], [Layer::Blue]),
            ));
        }
    }

    // Spawn red marbles that belong on the red layer and collide with red
    let red_material = materials.add(Color::srgb(0.9, 0.3, 0.3));
    for x in -6..6 {
        for y in -4..0 {
            commands.spawn((
                Mesh2d(marble_mesh.clone()),
                MeshMaterial2d(red_material.clone()),
                Transform::from_xyz(
                    x as f32 * 2.5 * marble_radius,
                    y as f32 * 2.5 * marble_radius + 200.0,
                    0.0,
                ),
                RigidBody::Dynamic,
                Collider::circle(marble_radius),
                CollisionLayers::new([Layer::Red], [Layer::Red]),
            ));
        }
    }
}
