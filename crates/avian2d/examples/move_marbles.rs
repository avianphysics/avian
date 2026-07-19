#![allow(clippy::unnecessary_cast)]

use avian2d::prelude::*;
use bevy::prelude::*;
use examples_common_2d::ExampleCommonPlugin;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            ExampleCommonPlugin,
            // Add physics plugins and specify a units-per-meter scaling factor, 1 meter = 20 pixels.
            // The unit allows the engine to tune its parameters for the scale of the world, improving stability.
            PhysicsPlugins::default().with_length_unit(20.0),
        ))
        .insert_resource(ClearColor(Color::srgb(0.05, 0.05, 0.1)))
        .insert_resource(Gravity(Vec2::NEG_Y * 9.81 * 100.0))
        .add_systems(Startup, setup)
        .add_systems(Update, movement)
        .run();
}

#[derive(Component)]
struct Marble;

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<ColorMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    commands.spawn(Camera2d);

    let square_sprite = Sprite {
        color: Color::srgb(0.7, 0.7, 0.8),
        custom_size: Some(Vec2::splat(50.0)),
        ..default()
    };

    // Ceiling
    commands.spawn((
        square_sprite.clone(),
        Transform::from_xyz(0.0, 50.0 * 6.0, 0.0).with_scale(Vec3::new(20.0, 1.0, 1.0)),
        RigidBody::Static,
        Collider::rectangle(50.0, 50.0),
    ));
    // Floor
    commands.spawn((
        square_sprite.clone(),
        Transform::from_xyz(0.0, -50.0 * 6.0, 0.0).with_scale(Vec3::new(20.0, 1.0, 1.0)),
        RigidBody::Static,
        Collider::rectangle(50.0, 50.0),
    ));
    // Left wall
    commands.spawn((
        square_sprite.clone(),
        Transform::from_xyz(-50.0 * 9.5, 0.0, 0.0).with_scale(Vec3::new(1.0, 11.0, 1.0)),
        RigidBody::Static,
        Collider::rectangle(50.0, 50.0),
    ));
    // Right wall
    commands.spawn((
        square_sprite,
        Transform::from_xyz(50.0 * 9.5, 0.0, 0.0).with_scale(Vec3::new(1.0, 11.0, 1.0)),
        RigidBody::Static,
        Collider::rectangle(50.0, 50.0),
    ));

    let marble_radius = 5.0;
    let marble_mesh = meshes.add(Circle::new(marble_radius));
    let marble_material = materials.add(Color::srgb(0.2, 0.7, 0.9));

    // Spawn stacks of marbles
    for x in -16..16 {
        for y in -16..16 {
            commands.spawn((
                Mesh2d(marble_mesh.clone()),
                MeshMaterial2d(marble_material.clone()),
                Transform::from_xyz(
                    x as f32 * 2.5 * marble_radius,
                    y as f32 * 2.5 * marble_radius,
                    0.0,
                ),
                RigidBody::Dynamic,
                Collider::circle(marble_radius),
                Marble,
            ));
        }
    }
}

fn movement(
    time: Res<Time>,
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mut marbles: Query<&mut LinearVelocity, With<Marble>>,
) {
    // Precision is adjusted so that the example works with
    // both the `f32` and `f64` features. Otherwise you don't need this.
    let delta_time = time.delta_secs();

    for mut linear_velocity in &mut marbles {
        if keyboard_input.any_pressed([KeyCode::KeyW, KeyCode::ArrowUp]) {
            // Use a higher acceleration for upwards movement to overcome gravity
            linear_velocity.y += 2500.0 * delta_time;
        }
        if keyboard_input.any_pressed([KeyCode::KeyS, KeyCode::ArrowDown]) {
            linear_velocity.y -= 500.0 * delta_time;
        }
        if keyboard_input.any_pressed([KeyCode::KeyA, KeyCode::ArrowLeft]) {
            linear_velocity.x -= 500.0 * delta_time;
        }
        if keyboard_input.any_pressed([KeyCode::KeyD, KeyCode::ArrowRight]) {
            linear_velocity.x += 500.0 * delta_time;
        }
    }
}
