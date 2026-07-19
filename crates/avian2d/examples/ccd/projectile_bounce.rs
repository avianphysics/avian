//! Demonstrates Continuous Collision Detection (CCD) between two fast-moving
//! dynamic bodies that bounce off each other.
//!
//! Many physics engines only support CCD for fast-moving bodies colliding
//! with static bodies. Avian additionally supports CCD for dynamic-kinematic
//! pairs automatically, and opt-in CCD for dynamic-dynamic collisions.
//!
//! One problem with CCD for dynamic-dynamic pairs is that because the AABBs
//! are typically tight, a sweep query for one body would miss the other body entirely
//! if the other body is not already in the path of the sweep. Avian solves this
//! with an opt-in [`SpeculativeCcd`] component that expands the body's AABB based on
//! its velocity, allowing the sweeps of both bodies to find each other and trigger CCD.
//!
//! Because this can lead to more contact pairs being generated in the broad phase,
//! the velocity-based AABB expansion is opt-in, but it can be useful for fast-moving
//! bodies that do need to collide with each other at high speeds, such as projectiles.

use avian2d::prelude::*;
use bevy::{camera::ScalingMode, input::common_conditions::input_just_pressed, prelude::*};
use examples_common_2d::ExampleCommonPlugin;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            PhysicsPlugins::default(),
            ExampleCommonPlugin,
        ))
        .insert_resource(ClearColor(Color::srgb(0.05, 0.05, 0.1)))
        // Zero gravity, so the circles travel in straight lines until they collide.
        .insert_resource(Gravity::ZERO)
        .add_systems(Startup, (setup_camera, setup_scene))
        .add_systems(
            Update,
            reset_scene.run_if(input_just_pressed(KeyCode::KeyR)),
        )
        .run();
}

const START_DISTANCE: f32 = 9.0;
const SPEED: f32 = 150.0;
const RADIUS: f32 = 0.5;

fn setup_camera(mut commands: Commands) {
    commands.spawn((
        Camera2d,
        Projection::Orthographic(OrthographicProjection {
            scaling_mode: ScalingMode::FixedHorizontal {
                viewport_width: 24.0,
            },
            ..OrthographicProjection::default_2d()
        }),
    ));
}

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    let circle = Circle::new(RADIUS);
    let mesh = meshes.add(circle);

    let mut projectile = |position: Vec2, velocity: Vec2, color: Color| {
        (
            Name::new("Projectile"),
            Transform::from_xyz(position.x, position.y, 0.0),
            RigidBody::Dynamic,
            Collider::from(circle),
            Restitution::new(1.0).with_combine_rule(CoefficientCombine::Max),
            Friction::ZERO.with_combine_rule(CoefficientCombine::Min),
            LinearVelocity(velocity),
            // Sweep against other dynamic bodies too.
            SweptCcd::default().with_filter(CcdFilter::ALL),
            // Enable speculative CCD and therefore AABB expansion so that
            // the two projectiles can find each other.
            SpeculativeCcd::default(),
            Mesh2d(mesh.clone()),
            MeshMaterial2d(materials.add(color)),
        )
    };

    // Comes in from the left, moving right.
    commands.spawn(projectile(
        Vec2::new(-START_DISTANCE, 0.0),
        Vec2::X * SPEED,
        Color::srgb(0.2, 0.7, 0.9),
    ));

    // Comes in from the bottom, moving up.
    commands.spawn(projectile(
        Vec2::new(0.0, -START_DISTANCE),
        Vec2::Y * SPEED,
        Color::srgb(0.9, 0.3, 0.3),
    ));

    // Instruction text
    commands.spawn((
        Text::new("Press R to reset"),
        TextFont {
            font_size: FontSize::Px(20.0),
            ..default()
        },
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(10.0),
            left: Val::Px(10.0),
            ..default()
        },
    ));
}

fn reset_scene(mut commands: Commands, bodies: Query<Entity, With<RigidBody>>) {
    for entity in &bodies {
        commands.entity(entity).despawn();
    }
    commands.run_system_cached(setup_scene);
}
