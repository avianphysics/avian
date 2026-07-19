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
        .insert_resource(Gravity(Vec2::NEG_Y * 1000.0))
        .add_systems(Startup, setup)
        .add_systems(Update, movement)
        .run();
}

#[derive(Component)]
struct Controllable;

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

    let circle = Circle::new(7.5);
    let rectangle = Rectangle::new(15.0, 15.0);
    let capsule = Capsule2d::new(7.5, 20.0);
    let triangle = Triangle2d::new(
        Vec2::new(0.0, 10.0),
        Vec2::new(-10.0, -10.0),
        Vec2::new(10.0, -10.0),
    );

    let shapes = [
        (
            circle.collider(),
            meshes.add(circle),
            materials.add(Color::srgb(0.29, 0.33, 0.64)),
        ),
        (
            rectangle.collider(),
            meshes.add(rectangle),
            materials.add(Color::srgb(0.47, 0.58, 0.8)),
        ),
        (
            capsule.collider(),
            meshes.add(capsule),
            materials.add(Color::srgb(0.63, 0.75, 0.88)),
        ),
        (
            triangle.collider(),
            meshes.add(triangle),
            materials.add(Color::srgb(0.77, 0.87, 0.97)),
        ),
    ];

    for x in -12_i32..12 {
        for y in -8_i32..8 {
            let (collider, mesh, material) = shapes[(x + y) as usize % 4].clone();
            commands.spawn((
                Mesh2d(mesh),
                MeshMaterial2d(material),
                Transform::from_xyz(x as f32 * 25.0, y as f32 * 25.0, 0.0),
                collider,
                RigidBody::Dynamic,
                Friction::new(0.1),
                Controllable,
            ));
        }
    }
}

fn movement(
    time: Res<Time>,
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mut marbles: Query<&mut LinearVelocity, With<Controllable>>,
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
