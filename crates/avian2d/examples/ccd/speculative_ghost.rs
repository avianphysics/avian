//! Demonstrates that even though Avian supports Continuous Collision Detection (CCD)
//! and uses speculative contacts, it does not lead to ghost collisions at small distances.
//!
//! A common problem with speculative contacts is that they can cause "ghost collisions"
//! when a dynamic body grazes past another body at a high speed. This happens because
//! the increased contact distance catches the other body's surface, and treats it as
//! an effectively infinite contact plane, even if the surface does not actually extend
//! to intersect the body's path.
//!
//! Avian keeps the speculative contact distance very small and AABBs tight by default,
//! and primarily relies on sweep-based CCD to prevent tunneling for fast-moving bodies.
//! This avoids ghost collisions (except at *very* small distances), while still preventing
//! tunneling or deep overlaps for fast-moving bodies.
//!
//! Avian does also support opt-in speculative collision with a larger speculative
//! contact distance via the [`SpeculativeCcd`] component. If you add it, you will
//! start seeing ghost collisions at high speeds again.
//!
//! This example is based on the SpeculativeGhost sample in Box2D.

use avian2d::prelude::*;
use bevy::{camera::ScalingMode, input::common_conditions::input_just_pressed, prelude::*};
use examples_common_2d::ExampleCommonPlugin;

fn main() {
    let mut app = App::new();

    app.add_plugins((
        DefaultPlugins,
        PhysicsPlugins::default().with_length_unit(0.5),
        PhysicsDebugPlugin,
        ExampleCommonPlugin,
    ));

    app.add_systems(Startup, (setup_camera, setup_scene));
    app.add_systems(
        Update,
        reset_scene.run_if(input_just_pressed(KeyCode::KeyR)),
    );

    app.run();
}

fn setup_camera(mut commands: Commands) {
    commands.spawn((
        Camera2d,
        Projection::Orthographic(OrthographicProjection {
            scaling_mode: ScalingMode::FixedHorizontal {
                viewport_width: 12.0,
            },
            ..OrthographicProjection::default_2d()
        }),
        Transform::from_xyz(0.0, 2.5, 0.0),
    ));
}

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    // Platform
    commands.spawn((
        Name::new("Platform"),
        RigidBody::Static,
        Collider::rectangle(2.0, 0.2),
        Transform::from_xyz(0.0, 0.9, 0.0),
        Mesh2d(meshes.add(Rectangle::new(2.0, 0.2))),
        MeshMaterial2d(materials.add(Color::srgb(0.5, 0.5, 0.5))),
    ));

    // Ground
    commands.spawn((
        Name::new("Ground"),
        RigidBody::Static,
        Collider::rectangle(20.0, 0.05),
        Mesh2d(meshes.add(Rectangle::new(20.0, 0.05))),
        MeshMaterial2d(materials.add(Color::srgb(0.5, 0.5, 0.5))),
    ));

    const HZ: f32 = 64.0;

    // Dynamic body that falls right past the platform and should not collide with it
    commands.spawn((
        Name::new("Dynamic Box"),
        RigidBody::Dynamic,
        // If you uncomment this, you will see a ghost collision
        // SpeculativeCcd::default(),
        Collider::rectangle(0.5, 0.5),
        Transform::from_xyz(0.015, 2.515, 0.0),
        LinearVelocity(Vec2::new(0.1 * 3.1 * HZ, -0.1 * 3.1 * HZ)),
        GravityScale(0.0),
        Mesh2d(meshes.add(Rectangle::new(0.5, 0.5))),
        MeshMaterial2d(materials.add(Color::srgb(0.2, 0.7, 0.9))),
    ));

    // Instruction text
    commands.spawn((
        Text::new("Press R to reset the scene"),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(10.0),
            left: Val::Px(10.0),
            ..default()
        },
        TextFont {
            font_size: FontSize::Px(20.0),
            ..default()
        },
    ));
}

fn reset_scene(mut commands: Commands, query: Query<Entity, With<RigidBody>>) {
    for entity in query.iter() {
        commands.entity(entity).despawn();
    }

    commands.run_system_cached(setup_scene);
}
