use avian3d::{math::*, prelude::*};
use bevy::prelude::*;
use examples_common_3d::ExampleCommonPlugin;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            ExampleCommonPlugin,
            PhysicsPlugins::default(),
            PhysicsDebugPlugin,
        ))
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let cube_mesh = meshes.add(Cuboid::default());
    let cube_material = materials.add(Color::srgb(0.8, 0.7, 0.6));

    // Spawn a static cube and a dynamic cube that is connected to it by a distance joint.
    let static_cube = commands
        .spawn((
            Mesh3d(cube_mesh.clone()),
            MeshMaterial3d(cube_material.clone()),
            RigidBody::Static,
            Collider::cuboid(1., 1., 1.),
        ))
        .id();
    let dynamic_cube = commands
        .spawn((
            Mesh3d(cube_mesh),
            MeshMaterial3d(cube_material),
            Transform::from_xyz(-2.0, -0.5, 0.0),
            RigidBody::Dynamic,
            Collider::cuboid(1., 1., 1.),
        ))
        .id();

    // Add a distance joint to keep the cubes at a certain distance from each other.
    commands.spawn(
        DistanceJoint::new(static_cube, dynamic_cube)
            .with_local_anchor2(Vector::splat(0.5))
            .with_limits(1.5, 1.5)
            .with_compliance(1.0 / 400.0),
    );

    // Light
    commands.spawn((
        PointLight {
            intensity: 2_000_000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 4.0),
    ));

    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 0.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}
