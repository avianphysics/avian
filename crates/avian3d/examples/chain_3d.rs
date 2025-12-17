#![allow(clippy::unnecessary_cast)]

use avian3d::{math::*, prelude::*};
use bevy::prelude::*;
use examples_common_3d::ExampleCommonPlugin;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            ExampleCommonPlugin,
            PhysicsPlugins::default(),
        ))
        .insert_resource(ClearColor(Color::srgb(0.05, 0.05, 0.1)))
        .insert_resource(GlobalAmbientLight {
            brightness: 2.0,
            ..default()
        })
        .insert_resource(SubstepCount(80))
        .insert_resource(Gravity(Vector::NEG_Y * 9.81 * 2.0))
        .add_systems(Startup, (setup, ui))
        .add_systems(Update, movement)
        .run();
}

/// The acceleration used for movement.
#[derive(Component)]
struct MovementAcceleration(Scalar);

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let particle_count = 100;
    let particle_radius = 0.06;
    let particle_mesh = meshes.add(Sphere::new(particle_radius as f32).mesh().ico(5).unwrap());
    let particle_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.2, 0.7, 0.9),
        unlit: true,
        ..default()
    });

    // Spawn kinematic particle that can follow the mouse
    let mut previous_particle = commands
        .spawn((
            RigidBody::Kinematic,
            MovementAcceleration(25.0),
            Mesh3d(particle_mesh.clone()),
            MeshMaterial3d(particle_material.clone()),
        ))
        .id();

    // Spawn the rest of the particles, connecting each one to the previous one with joints
    for i in 1..particle_count {
        let current_particle = commands
            .spawn((
                RigidBody::Dynamic,
                MassPropertiesBundle::from_shape(&Sphere::new(particle_radius as f32), 1.0),
                Mesh3d(particle_mesh.clone()),
                MeshMaterial3d(particle_material.clone()),
                Transform::from_xyz(0.0, -i as f32 * particle_radius as f32 * 2.2, 0.0),
            ))
            .id();

        commands.spawn(
            SphericalJoint::new(previous_particle, current_particle)
                .with_local_anchor1(Vector::NEG_Y * particle_radius * 1.1)
                .with_local_anchor2(Vector::Y * particle_radius * 1.1)
                .with_point_compliance(0.00001),
        );

        previous_particle = current_particle;
    }

    commands.spawn((
        Camera3d::default(),
        Transform::from_translation(Vec3::new(0.0, 5.0, 20.0))
            .looking_at(Vec3::NEG_Y * 5.0, Vec3::Y),
    ));
}

fn movement(
    time: Res<Time>,
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mut query: Query<(&MovementAcceleration, &mut LinearVelocity)>,
) {
    // Precision is adjusted so that the example works with
    // both the `f32` and `f64` features. Otherwise you don't need this.
    let delta_time = time.delta_secs_f64().adjust_precision();

    for (movement_acceleration, mut linear_velocity) in &mut query {
        let up = keyboard_input.any_pressed([KeyCode::KeyW, KeyCode::ArrowUp]);
        let down = keyboard_input.any_pressed([KeyCode::KeyS, KeyCode::ArrowDown]);
        let left = keyboard_input.any_pressed([KeyCode::KeyA, KeyCode::ArrowLeft]);
        let right = keyboard_input.any_pressed([KeyCode::KeyD, KeyCode::ArrowRight]);

        let horizontal = right as i8 - left as i8;
        let vertical = down as i8 - up as i8;
        let direction =
            Vector::new(horizontal as Scalar, 0.0, vertical as Scalar).normalize_or_zero();

        // Move in input direction
        linear_velocity.x += direction.x * movement_acceleration.0 * delta_time;
        linear_velocity.z += direction.z * movement_acceleration.0 * delta_time;

        // Slow down movement
        linear_velocity.0 *= 0.9;
    }
}

fn ui(mut commands: Commands) {
    commands.spawn((
        Camera2d,
        Camera {
            order: -1,
            ..default()
        },
    ));
    commands
        .spawn((
            Node {
                width: Val::Percent(100.),
                ..default()
            },
            BackgroundColor::from(Color::srgba(0.15, 0.15, 0.15, 0.0)),
        ))
        .with_child((
            Text::new("Use Arrow Keys or WASD to Move The Chain"),
            TextFont {
                font_size: 20.0,
                ..default()
            },
            TextColor(Color::WHITE),
            Node {
                margin: UiRect {
                    left: Val::Px(5.0),
                    top: Val::Px(30.0),
                    ..default()
                },
                ..default()
            },
            // Because this is a distinct label widget and
            // not button/list item text, this is necessary
            // for accessibility to treat the text accordingly.
            Label,
        ));
}
