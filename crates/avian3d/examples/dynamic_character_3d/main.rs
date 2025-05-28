//! A basic implementation of a character controller for a dynamic rigid body.
//!
//! This showcases the following:
//!
//! - Basic directional movement and jumping
//! - Support for both keyboard and gamepad input
//! - A configurable maximum slope angle for jumping
//! - Loading a platformer environment from a glTF
//!
//! The character controller logic is contained within the `plugin` module.
//!
//! For a kinematic character controller, see the `kinematic_character_3d` example.

mod plugin;

use avian3d::{math::*, prelude::*};
use bevy::prelude::*;
use examples_common_3d::ExampleCommonPlugin;
use plugin::*;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            ExampleCommonPlugin,
            PhysicsPlugins::default(),
            CharacterControllerPlugin,
        ))
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    assets: Res<AssetServer>,
) {
    // Player
    commands.spawn((
        Mesh3d(meshes.add(Capsule3d::new(0.4, 1.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.8, 0.7, 0.6))),
        Transform::from_xyz(0.0, 1.5, 0.0),
        CharacterControllerBundle::new(Collider::capsule(0.4, 1.0)).with_movement(
            30.0,
            0.92,
            7.0,
            (30.0 as Scalar).to_radians(),
        ),
        Friction::ZERO.with_combine_rule(CoefficientCombine::Min),
        Restitution::ZERO.with_combine_rule(CoefficientCombine::Min),
        GravityScale(2.0),
    ));

    // A cube to move around
    commands.spawn((
        RigidBody::Dynamic,
        Collider::cuboid(1.0, 1.0, 1.0),
        Mesh3d(meshes.add(Cuboid::default())),
        MeshMaterial3d(materials.add(Color::srgb(0.8, 0.7, 0.6))),
        Transform::from_xyz(3.0, 2.0, 3.0),
    ));

    // Environment (see the `collider_constructors` example for creating colliders from scenes)
    commands.spawn((
        SceneRoot(assets.load("character_controller_demo.glb#Scene0")),
        Transform::from_rotation(Quat::from_rotation_y(-core::f32::consts::PI * 0.5)),
        ColliderConstructorHierarchy::new(ColliderConstructor::ConvexHullFromMesh),
        RigidBody::Static,
    ));

    // Light
    commands.spawn((
        PointLight {
            intensity: 2_000_000.0,
            range: 50.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(0.0, 15.0, 0.0),
    ));

    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-7.0, 9.5, 15.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}
