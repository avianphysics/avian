#![allow(clippy::unnecessary_cast)]

use avian2d::{math::*, prelude::*};
use bevy::{prelude::*, window::PrimaryWindow};
use examples_common_2d::ExampleCommonPlugin;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            ExampleCommonPlugin,
            PhysicsPlugins::default(),
        ))
        .insert_resource(ClearColor(Color::srgb(0.05, 0.05, 0.1)))
        .insert_resource(SubstepCount(50))
        .insert_resource(Gravity(Vector::NEG_Y * 1000.0))
        .add_systems(Startup, setup)
        .add_systems(Update, follow_mouse)
        .run();
}

#[derive(Component)]
struct FollowMouse;

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<ColorMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    commands.spawn(Camera2d);

    let particle_count = 100;
    let particle_radius = 1.2;
    let particle_mesh = meshes.add(Circle::new(particle_radius as f32));
    let particle_material = materials.add(Color::srgb(0.2, 0.7, 0.9));

    // Spawn kinematic particle that can follow the mouse
    let mut previous_particle = commands
        .spawn((
            RigidBody::Kinematic,
            FollowMouse,
            Mesh2d(particle_mesh.clone()),
            MeshMaterial2d(particle_material.clone()),
        ))
        .id();

    // Spawn the rest of the particles, connecting each one to the previous one with joints
    for i in 1..particle_count {
        let current_particle = commands
            .spawn((
                RigidBody::Dynamic,
                MassPropertiesBundle::from_shape(&Circle::new(particle_radius as f32), 1.0),
                Mesh2d(particle_mesh.clone()),
                MeshMaterial2d(particle_material.clone()),
                Transform::from_xyz(0.0, -i as f32 * (particle_radius as f32 * 2.0 + 1.0), 0.0),
            ))
            .id();

        commands.spawn(
            RevoluteJoint::new(previous_particle, current_particle)
                .with_local_anchor_2(Vector::Y * (particle_radius * 2.0 + 1.0))
                .with_point_compliance(0.0000001),
        );

        previous_particle = current_particle;
    }
}

fn follow_mouse(
    buttons: Res<ButtonInput<MouseButton>>,
    windows: Query<&Window, With<PrimaryWindow>>,
    camera: Query<(&Camera, &GlobalTransform)>,
    mut follower: Query<&mut Transform, With<FollowMouse>>,
) -> Result {
    if buttons.pressed(MouseButton::Left) {
        let window = windows.single()?;
        let (camera, camera_transform) = camera.single()?;
        let mut follower_position = follower.single_mut()?;

        if let Some(cursor_world_pos) = window
            .cursor_position()
            .and_then(|cursor| camera.viewport_to_world_2d(camera_transform, cursor).ok())
        {
            follower_position.translation =
                cursor_world_pos.extend(follower_position.translation.z);
        }
    }

    Ok(())
}
