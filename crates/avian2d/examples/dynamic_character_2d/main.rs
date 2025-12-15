//! A basic implementation of a character controller for a dynamic rigid body.
//!
//! This showcases the following:
//!
//! - Basic directional movement and jumping
//! - Support for both keyboard and gamepad input
//! - A configurable maximum slope angle for jumping
//!
//! The character controller logic is contained within the `plugin` module.
//!
//! For a kinematic character controller, see the `kinematic_character_2d` example.

mod plugin;

use avian2d::{math::*, prelude::*};
use bevy::{asset::RenderAssetUsages, prelude::*, render::render_resource::PrimitiveTopology};
use examples_common_2d::ExampleCommonPlugin;
use plugin::*;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            ExampleCommonPlugin,
            // Add physics plugins and specify a units-per-meter scaling factor, 1 meter = 20 pixels.
            // The unit allows the engine to tune its parameters for the scale of the world, improving stability.
            PhysicsPlugins::default().with_length_unit(20.0),
            CharacterControllerPlugin,
        ))
        .insert_resource(ClearColor(Color::srgb(0.05, 0.05, 0.1)))
        .insert_resource(Gravity(Vector::NEG_Y * 1000.0))
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    // Player
    commands.spawn((
        Mesh2d(meshes.add(Capsule2d::new(12.5, 20.0))),
        MeshMaterial2d(materials.add(Color::srgb(0.2, 0.7, 0.9))),
        Transform::from_xyz(0.0, -100.0, 0.0),
        CharacterControllerBundle::new(Collider::capsule(12.5, 20.0)).with_movement(
            1250.0,
            5.0,
            400.0,
            (30.0 as Scalar).to_radians(),
        ),
        Friction::ZERO.with_combine_rule(CoefficientCombine::Min),
        Restitution::ZERO.with_combine_rule(CoefficientCombine::Min),
        ColliderDensity(2.0),
        GravityScale(1.5),
    ));

    // A cube to move around
    commands.spawn((
        Sprite {
            color: Color::srgb(0.0, 0.4, 0.7),
            custom_size: Some(Vec2::new(30.0, 30.0)),
            ..default()
        },
        Transform::from_xyz(50.0, -100.0, 0.0),
        RigidBody::Dynamic,
        Collider::rectangle(30.0, 30.0),
    ));

    // Platforms
    commands.spawn((
        Sprite {
            color: Color::srgb(0.7, 0.7, 0.8),
            custom_size: Some(Vec2::new(1100.0, 50.0)),
            ..default()
        },
        Transform::from_xyz(0.0, -175.0, 0.0),
        RigidBody::Static,
        Collider::rectangle(1100.0, 50.0),
    ));
    commands.spawn((
        Sprite {
            color: Color::srgb(0.7, 0.7, 0.8),
            custom_size: Some(Vec2::new(300.0, 25.0)),
            ..default()
        },
        Transform::from_xyz(175.0, -35.0, 0.0),
        RigidBody::Static,
        Collider::rectangle(300.0, 25.0),
    ));
    commands.spawn((
        Sprite {
            color: Color::srgb(0.7, 0.7, 0.8),
            custom_size: Some(Vec2::new(300.0, 25.0)),
            ..default()
        },
        Transform::from_xyz(-175.0, 0.0, 0.0),
        RigidBody::Static,
        Collider::rectangle(300.0, 25.0),
    ));
    commands.spawn((
        Sprite {
            color: Color::srgb(0.7, 0.7, 0.8),
            custom_size: Some(Vec2::new(150.0, 80.0)),
            ..default()
        },
        Transform::from_xyz(475.0, -110.0, 0.0),
        RigidBody::Static,
        Collider::rectangle(150.0, 80.0),
    ));
    commands.spawn((
        Sprite {
            color: Color::srgb(0.7, 0.7, 0.8),
            custom_size: Some(Vec2::new(150.0, 80.0)),
            ..default()
        },
        Transform::from_xyz(-475.0, -110.0, 0.0),
        RigidBody::Static,
        Collider::rectangle(150.0, 80.0),
    ));

    // Ramps

    let mut ramp_mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );

    ramp_mesh.insert_attribute(
        Mesh::ATTRIBUTE_POSITION,
        vec![[-125.0, 80.0, 0.0], [-125.0, 0.0, 0.0], [125.0, 0.0, 0.0]],
    );

    let ramp_collider = Collider::triangle(
        Vector::new(-125.0, 80.0),
        Vector::NEG_X * 125.0,
        Vector::X * 125.0,
    );

    commands.spawn((
        Mesh2d(meshes.add(ramp_mesh)),
        MeshMaterial2d(materials.add(Color::srgb(0.4, 0.4, 0.5))),
        Transform::from_xyz(-275.0, -150.0, 0.0),
        RigidBody::Static,
        ramp_collider,
    ));

    let mut ramp_mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );

    ramp_mesh.insert_attribute(
        Mesh::ATTRIBUTE_POSITION,
        vec![[20.0, -40.0, 0.0], [20.0, 40.0, 0.0], [-20.0, -40.0, 0.0]],
    );

    let ramp_collider = Collider::triangle(
        Vector::new(20.0, -40.0),
        Vector::new(20.0, 40.0),
        Vector::new(-20.0, -40.0),
    );

    commands.spawn((
        Mesh2d(meshes.add(ramp_mesh)),
        MeshMaterial2d(materials.add(Color::srgb(0.4, 0.4, 0.5))),
        Transform::from_xyz(380.0, -110.0, 0.0),
        RigidBody::Static,
        ramp_collider,
    ));

    // Camera
    commands.spawn(Camera2d);
}
