//! Demonstrates Continuous Collision Detection (CCD) to prevent tunneling.
//!
//! Two forms of CCD are supported:
//!
//! 1. Speculative collision
//!     - Predicts approximate contacts before they happen.
//!     - Enabled by default for all rigid bodies.
//!     - Very efficient and relatively reliable.
//!     - Can sometimes cause ghost collisions.
//!     - Can sometimes miss collisions against objects spinning at very high speeds.
//!
//! 2. Swept CCD
//!     - Sweeps colliders from their previous positions to their current ones,
//!       and if a hit is found, moves the bodies to the time of impact.
//!     - Enabled for rigid bodies with the `SweptCcd` component.
//!     - Can prevent tunneling completely. More reliable than speculative collision.
//!     - More expensive than speculative collision.
//!     - Can cause "time loss" where bodies appear to stop for a moment
//!       because they are essentially brought back in time.
//!     - Two modes:
//!         1. Linear: Only considers translational motion.
//!         2. Non-linear: Considers both translation and rotation. More expensive.

use avian2d::{math::*, prelude::*};
use bevy::prelude::*;
use examples_common_2d::ExampleCommonPlugin;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            ExampleCommonPlugin,
            PhysicsPlugins::default(),
        ))
        .insert_resource(ClearColor(Color::srgb(0.05, 0.05, 0.1)))
        .insert_resource(Gravity(Vector::NEG_Y * 9.81 * 100.0))
        .add_systems(Startup, setup)
        .add_systems(Update, update_config)
        .add_systems(FixedUpdate, spawn_balls)
        .run();
}

#[derive(Component)]
struct SpeculativeCollisionEnabledText;

#[derive(Component)]
struct SweptCcdModeText;

fn setup(mut commands: Commands) {
    commands.spawn(Camera2d);

    // Add two kinematic bodies spinning at high speeds.
    commands.spawn((
        Sprite {
            color: Color::srgb(0.7, 0.7, 0.8),
            custom_size: Some(Vec2::new(1.0, 400.0)),
            ..default()
        },
        Transform::from_xyz(-200.0, -200.0, 0.0),
        RigidBody::Kinematic,
        AngularVelocity(25.0),
        Collider::rectangle(1.0, 400.0),
        // Enable swept CCD for this body. Considers both translational and rotational motion by default.
        // This could also be on the ball projectiles.
        SweptCcd::default(),
    ));
    commands.spawn((
        Sprite {
            color: Color::srgb(0.7, 0.7, 0.8),
            custom_size: Some(Vec2::new(1.0, 400.0)),
            ..default()
        },
        Transform::from_xyz(200.0, -200.0, 0.0),
        RigidBody::Kinematic,
        AngularVelocity(-25.0),
        Collider::rectangle(1.0, 400.0),
        // Enable swept CCD for this body. Considers both translational and rotational motion by default.
        // This could also be on the ball projectiles.
        SweptCcd::default(),
    ));

    // Setup help text.
    let font = TextFont {
        font_size: 20.0,
        ..default()
    };
    commands
        .spawn((
            Text::default(),
            Node {
                position_type: PositionType::Absolute,
                top: Val::Px(30.0),
                left: Val::Px(10.0),
                ..default()
            },
        ))
        .with_children(|children| {
            children.spawn((TextSpan::new("(1) Speculative Collision: "), font.clone()));
            children.spawn((
                TextSpan::new("On"),
                font.clone(),
                SpeculativeCollisionEnabledText,
            ));
        });
    commands
        .spawn((
            Text::default(),
            Node {
                position_type: PositionType::Absolute,
                top: Val::Px(50.0),
                left: Val::Px(10.0),
                ..default()
            },
        ))
        .with_children(|children| {
            children.spawn((TextSpan::new("(2) Swept CCD: "), font.clone()));
            children.spawn((
                TextSpan::new("Non-linear (considers both translation and rotation)"),
                font,
                SweptCcdModeText,
            ));
        });
}

/// Spawns balls moving at the spinning objects at high speeds.
fn spawn_balls(
    mut commands: Commands,
    mut materials: ResMut<Assets<ColorMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
    time: Res<Time<Physics>>,
) {
    let circle = Circle::new(2.0);

    // Compute the shooting direction.
    let (sin, cos) = (0.5 * time.elapsed_secs_f64().adjust_precision().sin() - PI / 2.0).sin_cos();
    let direction = Vector::new(cos, sin).rotate(Vector::X);

    commands.spawn((
        Mesh2d(meshes.add(circle)),
        MeshMaterial2d(materials.add(Color::srgb(0.2, 0.7, 0.9))),
        Transform::from_xyz(0.0, 350.0, 0.0),
        RigidBody::Dynamic,
        LinearVelocity(2000.0 * direction),
        Friction::ZERO.with_combine_rule(CoefficientCombine::Min),
        Collider::from(circle),
    ));
}

fn update_config(
    speculative_collision_text: Single<
        &mut TextSpan,
        (
            With<SpeculativeCollisionEnabledText>,
            Without<SweptCcdModeText>,
        ),
    >,
    swept_ccd_mode_text: Single<
        &mut TextSpan,
        (
            With<SweptCcdModeText>,
            Without<SpeculativeCollisionEnabledText>,
        ),
    >,
    keys: Res<ButtonInput<KeyCode>>,
    mut narrow_phase_config: ResMut<NarrowPhaseConfig>,
    mut ccd_bodies: Query<&mut SweptCcd>,
) {
    // Toggle speculative collision.
    // Note: This sets the default speculative margin, but it can be overridden
    //       for individual entities with the `SpeculativeMargin` component.
    if keys.just_pressed(KeyCode::Digit1) {
        let mut text = speculative_collision_text;
        if narrow_phase_config.default_speculative_margin == Scalar::MAX {
            narrow_phase_config.default_speculative_margin = 0.0;
            text.0 = "Off".to_string();
        } else {
            narrow_phase_config.default_speculative_margin = Scalar::MAX;
            text.0 = "On".to_string();
        }
    }

    // Change the sweep mode and whether swept CCD is enabled.
    if keys.just_pressed(KeyCode::Digit2) {
        let mut text = swept_ccd_mode_text;
        for mut swept_ccd in &mut ccd_bodies {
            if swept_ccd.mode == SweepMode::NonLinear && swept_ccd.include_dynamic {
                swept_ccd.mode = SweepMode::Linear;
                text.0 = "Linear (considers only translation)".to_string();
            } else if swept_ccd.mode == SweepMode::Linear {
                // Disable swept CCD for collisions against dynamic bodies.
                // To disable it completely, you should remove the component.
                swept_ccd.include_dynamic = false;

                swept_ccd.mode = SweepMode::NonLinear;
                text.0 = "Off".to_string();
            } else {
                // Enable swept CCD again.
                swept_ccd.include_dynamic = true;

                text.0 = "Non-linear (considers both translation and rotation)".to_string();
            }
        }
    }
}
