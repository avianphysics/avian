//! Demonstrates Continuous Collision Detection (CCD) to prevent tunneling
//! when fast-moving balls collide with fast-spinning thin objects.
//!
//! Three modes for swept CCD can be toggled between:
//!
//! 1. Linear: Only considers translational motion.
//! 2. Non-linear: Considers both translational and rotational motion. More expensive.
//! 3. Off: Discrete collision detection only, which can cause tunneling at high speeds.
//!
//! Speculative CCD can also be toggled, predicting contacts before they happen
//! at the risk of ghost collisions and larger collision AABBs.
//!
//! Additionally, this example demonstrates one problem with swept CCD known as "time loss".
//! To hit the balls correctly, the spinners need to stop at the first time of impact,
//! causing them to lose the rest of their rotation for that frame. This can make
//! the spinners look like they are stuttering, especially at high speeds and with
//! many objects to collide with.
//!
//! Using speculative CCD together with swept CCD can help to some extent,
//! but a proper fix would require a substepping CCD solver, which is not currently
//! implemented in Avian due to its high overhead and complexity.

use core::f32::consts::PI;

use avian2d::prelude::*;
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
        .insert_resource(Gravity(Vec2::NEG_Y * 9.81 * 100.0))
        .init_resource::<ActiveSweepMode>()
        .init_resource::<SpeculativeCcdEnabled>()
        .add_systems(Startup, setup)
        .add_systems(Update, update_config)
        .add_systems(FixedUpdate, spawn_balls)
        .run();
}

#[derive(Component)]
struct SweptCcdText;

#[derive(Component)]
struct SpeculativeCcdText;

/// The current CCD sweep mode configuration.
#[derive(Resource, Clone, Copy, Default, PartialEq)]
enum ActiveSweepMode {
    #[default]
    NonLinear,
    Linear,
    Off,
}

impl ActiveSweepMode {
    fn ccd_settings(self) -> SweptCcd {
        match self {
            ActiveSweepMode::NonLinear => SweptCcd::default().with_filter(CcdFilter::ALL),
            ActiveSweepMode::Linear => SweptCcd::default()
                .with_mode(SweepMode::Linear)
                .with_filter(CcdFilter::ALL),
            ActiveSweepMode::Off => SweptCcd::default().with_filter(CcdFilter::NONE),
        }
    }

    fn label(self) -> &'static str {
        match self {
            ActiveSweepMode::NonLinear => "Non-linear (considers both translation and rotation)",
            ActiveSweepMode::Linear => "Linear (considers only translation)",
            ActiveSweepMode::Off => "Off (discrete collision detection only)",
        }
    }
}

/// Whether speculative CCD is enabled for the bodies.
#[derive(Resource, Clone, Copy, Default, PartialEq)]
struct SpeculativeCcdEnabled(bool);

impl SpeculativeCcdEnabled {
    fn label(self) -> &'static str {
        if self.0 { "On" } else { "Off" }
    }
}

fn setup(mut commands: Commands) {
    commands.spawn(Camera2d);

    let sweep_mode = ActiveSweepMode::default();
    let swept_ccd = sweep_mode.ccd_settings();
    let speculative_ccd_enabled = SpeculativeCcdEnabled::default();

    // Add two dynamic bodies spinning at high speeds. Their translation is locked and they have a
    // high dominance, so they spin in place without being pushed around.
    commands.spawn((
        Name::new("Spinner 1"),
        Transform::from_xyz(-200.1, -200.0, 0.0),
        RigidBody::Dynamic,
        LockedAxes::TRANSLATION_LOCKED,
        AngularVelocity(25.0),
        Dominance(1),
        Collider::rectangle(1.0, 400.0),
        swept_ccd,
        Sprite {
            color: Color::srgb(0.7, 0.7, 0.8),
            custom_size: Some(Vec2::new(1.0, 400.0)),
            ..default()
        },
    ));
    commands.spawn((
        Name::new("Spinner 2"),
        Transform::from_xyz(200.1, -200.0, 0.0),
        RigidBody::Dynamic,
        LockedAxes::TRANSLATION_LOCKED,
        AngularVelocity(-25.0),
        Dominance(1),
        Collider::rectangle(1.0, 400.0),
        swept_ccd,
        Sprite {
            color: Color::srgb(0.7, 0.7, 0.8),
            custom_size: Some(Vec2::new(1.0, 400.0)),
            ..default()
        },
    ));

    // Instruction text
    let font = TextFont {
        font_size: FontSize::Px(20.0),
        ..default()
    };
    commands.spawn((
        Text::new("Press Space to toggle CCD mode\nPress S to toggle speculative CCD"),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(10.0),
            left: Val::Px(10.0),
            ..default()
        },
        font.clone(),
    ));
    commands.spawn((
        Text::default(),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(70.0),
            left: Val::Px(10.0),
            ..default()
        },
        children![
            (TextSpan::new("CCD: "), font.clone()),
            (
                TextSpan::new(sweep_mode.label()),
                font.clone(),
                SweptCcdText
            ),
        ],
    ));
    commands.spawn((
        Text::default(),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(100.0),
            left: Val::Px(10.0),
            ..default()
        },
        children![
            (TextSpan::new("Speculative CCD: "), font.clone()),
            (
                TextSpan::new(speculative_ccd_enabled.label()),
                font,
                SpeculativeCcdText
            ),
        ],
    ));
}

/// Spawns balls moving at the spinning objects at high speeds.
fn spawn_balls(
    mut commands: Commands,
    mut materials: ResMut<Assets<ColorMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
    time: Res<Time<Physics>>,
    sweep_mode: Res<ActiveSweepMode>,
    speculative: Res<SpeculativeCcdEnabled>,
) {
    let circle = Circle::new(2.0);

    // Compute the shooting direction.
    let (sin, cos) = (0.5 * time.elapsed_secs().sin() - PI / 2.0).sin_cos();
    let direction = Vec2::new(cos, sin).rotate(Vec2::X);

    let mut ball = commands.spawn((
        Name::new("Ball"),
        Mesh2d(meshes.add(circle)),
        MeshMaterial2d(materials.add(Color::srgb(0.2, 0.7, 0.9))),
        Transform::from_xyz(0.0, 350.0, 0.0),
        RigidBody::Dynamic,
        LinearVelocity(2000.0 * direction),
        Friction::ZERO.with_combine_rule(CoefficientCombine::Min),
        Collider::from(circle),
        sweep_mode.ccd_settings(),
    ));

    if speculative.0 {
        ball.insert(SpeculativeCcd::default());
    }
}

fn update_config(
    mut sweep_mode_text: Single<&mut TextSpan, (With<SweptCcdText>, Without<SpeculativeCcdText>)>,
    mut speculative_text: Single<&mut TextSpan, (With<SpeculativeCcdText>, Without<SweptCcdText>)>,
    keys: Res<ButtonInput<KeyCode>>,
    mut sweep_mode: ResMut<ActiveSweepMode>,
    mut speculative: ResMut<SpeculativeCcdEnabled>,
    bodies: Query<Entity, With<Collider>>,
    mut commands: Commands,
) {
    // Cycle the CCD sweep mode for the colliders.
    if keys.just_pressed(KeyCode::Space) {
        *sweep_mode = match *sweep_mode {
            ActiveSweepMode::NonLinear => ActiveSweepMode::Linear,
            ActiveSweepMode::Linear => ActiveSweepMode::Off,
            ActiveSweepMode::Off => ActiveSweepMode::NonLinear,
        };
        sweep_mode_text.0 = sweep_mode.label().to_string();

        // Update the existing colliders to use the new configuration.
        for entity in &bodies {
            commands.entity(entity).insert(sweep_mode.ccd_settings());
        }
    }

    // Toggle speculative CCD for the colliders.
    if keys.just_pressed(KeyCode::KeyS) {
        speculative.0 = !speculative.0;
        speculative_text.0 = speculative.label().to_string();

        // Add or remove the `SpeculativeCcd` component on the existing colliders.
        for entity in &bodies {
            if speculative.0 {
                commands.entity(entity).insert(SpeculativeCcd::default());
            } else {
                commands.entity(entity).remove::<SpeculativeCcd>();
            }
        }
    }
}
