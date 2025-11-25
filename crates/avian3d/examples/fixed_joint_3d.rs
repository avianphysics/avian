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
        .insert_resource(SubstepCount(50))
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let cube_mesh = meshes.add(Cuboid::default());
    let cube_material = materials.add(Color::srgb(0.8, 0.7, 0.6));

    // Kinematic rotating "anchor" object
    let anchor = commands
        .spawn((
            Mesh3d(cube_mesh.clone()),
            MeshMaterial3d(cube_material.clone()),
            RigidBody::Kinematic,
            AngularVelocity(Vector::Z * 1.5),
        ))
        .id();

    // Dynamic object rotating around anchor
    let object = commands
        .spawn((
            Mesh3d(cube_mesh),
            MeshMaterial3d(cube_material),
            Transform::from_xyz(1.5, 0.0, 0.0),
            RigidBody::Dynamic,
            MassPropertiesBundle::from_shape(&Cuboid::from_length(1.0), 1.0),
        ))
        .id();

    // Connect anchor and dynamic object
    commands.spawn(FixedJoint::new(anchor, object).with_local_anchor1(Vector::X * 1.5));

    // Directional light
    commands.spawn((
        DirectionalLight {
            illuminance: 2000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::default().looking_at(Vec3::new(-1.0, -2.5, -1.5), Vec3::Y),
    ));

    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_translation(Vec3::Z * 10.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}
