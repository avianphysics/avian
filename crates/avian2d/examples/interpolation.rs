//! This example showcases how `Transform` interpolation or extrapolation can be used
//! to make movement appear smooth at fixed timesteps.
//!
//! To produce consistent, frame rate independent behavior, physics by default runs
//! in the `FixedPostUpdate` schedule with a fixed timestep, meaning that the time between
//! physics ticks remains constant. On some frames, physics can either not run at all or run
//! more than once to catch up to real time. This can lead to visible stutter for movement.
//!
//! `Transform` interpolation resolves this issue by updating `Transform` at every frame in between
//! physics ticks to smooth out the visual result. The interpolation is done from the previous position
//! to the current physics position, which keeps movement smooth, but has the downside of making movement
//! feel slightly delayed as the rendered result lags slightly behind the true positions.
//!
//! `Transform` extrapolation works similarly, but instead of using the previous positions, it predicts
//! the next positions based on velocity. This makes movement feel much more responsive, but can cause
//! jumpy results when the prediction is wrong, such as when the velocity of an object is suddenly altered.

use avian2d::{math::*, prelude::*};
use bevy::{
    color::palettes::{
        css::WHITE,
        tailwind::{CYAN_400, LIME_400, RED_400},
    },
    input::common_conditions::input_pressed,
    prelude::*,
};

fn main() {
    let mut app = App::new();

    // Interpolation and extrapolation functionality is enabled by the `PhysicsInterpolationPlugin`.
    // It is included in the `PhysicsPlugins` by default.
    app.add_plugins((
        DefaultPlugins,
        PhysicsPlugins::default().with_length_unit(50.0),
    ));

    // By default, interpolation must be enabled for each entity manually
    // by adding the `TransformInterpolation` component.
    //
    // It can also be enabled for all rigid bodies with `PhysicsInterpolationPlugin::interpolate_all()`:
    //
    // app.add_plugins(PhysicsPlugins::default().set(PhysicsInterpolationPlugin::interpolate_all()));

    // Set gravity.
    app.insert_resource(Gravity(Vector::NEG_Y * 900.0));

    // Set the fixed timestep to just 10 Hz for demonstration purposes.
    app.insert_resource(Time::from_hz(10.0));

    // Setup the scene and UI, and update text in `Update`.
    app.add_systems(Startup, (setup_scene, setup_balls, setup_text))
        .add_systems(
            Update,
            (
                change_timestep,
                update_timestep_text,
                // Reset the scene when the 'R' key is pressed.
                reset_balls.run_if(input_pressed(KeyCode::KeyR)),
            ),
        );

    // Run the app.
    app.run();
}

#[derive(Component)]
struct Ball;

fn setup_scene(
    mut commands: Commands,
    mut materials: ResMut<Assets<ColorMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    // Spawn a camera.
    commands.spawn(Camera2d);

    // Spawn the ground.
    commands.spawn((
        Name::new("Ground"),
        RigidBody::Static,
        Collider::rectangle(500.0, 20.0),
        Restitution::new(0.99).with_combine_rule(CoefficientCombine::Max),
        Transform::from_xyz(0.0, -300.0, 0.0),
        Mesh2d(meshes.add(Rectangle::new(500.0, 20.0))),
        MeshMaterial2d(materials.add(Color::from(WHITE))),
    ));
}

fn setup_balls(
    mut commands: Commands,
    mut materials: ResMut<Assets<ColorMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let circle = Circle::new(30.0);
    let mesh = meshes.add(circle);

    // This entity uses transform interpolation.
    commands.spawn((
        Name::new("Interpolation"),
        Ball,
        RigidBody::Dynamic,
        Collider::from(circle),
        TransformInterpolation,
        Transform::from_xyz(-100.0, 300.0, 0.0),
        Mesh2d(mesh.clone()),
        MeshMaterial2d(materials.add(Color::from(CYAN_400)).clone()),
    ));

    // This entity uses transform extrapolation.
    commands.spawn((
        Name::new("Extrapolation"),
        Ball,
        RigidBody::Dynamic,
        Collider::from(circle),
        TransformExtrapolation,
        Transform::from_xyz(0.0, 300.0, 0.0),
        Mesh2d(mesh.clone()),
        MeshMaterial2d(materials.add(Color::from(LIME_400)).clone()),
    ));

    // This entity is simulated in `FixedUpdate` without any smoothing.
    commands.spawn((
        Name::new("No Interpolation"),
        Ball,
        RigidBody::Dynamic,
        Collider::from(circle),
        Transform::from_xyz(100.0, 300.0, 0.0),
        Mesh2d(mesh.clone()),
        MeshMaterial2d(materials.add(Color::from(RED_400)).clone()),
    ));
}

/// Despawns all balls and respawns them.
fn reset_balls(mut commands: Commands, query: Query<Entity, With<Ball>>) {
    for entity in &query {
        commands.entity(entity).despawn();
    }

    commands.run_system_cached(setup_balls);
}

#[derive(Component)]
struct TimestepText;

fn setup_text(mut commands: Commands) {
    let font = TextFont {
        font_size: 20.0,
        ..default()
    };

    commands
        .spawn((
            Text::new("Fixed Hz: "),
            TextColor::from(WHITE),
            font.clone(),
            Node {
                position_type: PositionType::Absolute,
                top: Val::Px(10.0),
                left: Val::Px(10.0),
                ..default()
            },
        ))
        .with_child((TimestepText, TextSpan::default()));

    commands.spawn((
        Text::new("Change Timestep With Up/Down Arrow\nPress R to reset"),
        TextColor::from(WHITE),
        TextLayout::new_with_justify(Justify::Right),
        font.clone(),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(10.0),
            right: Val::Px(10.0),
            ..default()
        },
    ));

    commands.spawn((
        Text::new("Interpolation"),
        TextColor::from(CYAN_400),
        font.clone(),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(50.0),
            left: Val::Px(10.0),
            ..default()
        },
    ));

    commands.spawn((
        Text::new("Extrapolation"),
        TextColor::from(LIME_400),
        font.clone(),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(75.0),
            left: Val::Px(10.0),
            ..default()
        },
    ));

    commands.spawn((
        Text::new("No Interpolation"),
        TextColor::from(RED_400),
        font.clone(),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(100.0),
            left: Val::Px(10.0),
            ..default()
        },
    ));
}

/// Changes the timestep of the simulation when the up or down arrow keys are pressed.
fn change_timestep(mut time: ResMut<Time<Fixed>>, keyboard_input: Res<ButtonInput<KeyCode>>) {
    if keyboard_input.pressed(KeyCode::ArrowUp) {
        let new_timestep = (time.delta_secs_f64() * 0.975).max(1.0 / 255.0);
        time.set_timestep_seconds(new_timestep);
    }
    if keyboard_input.pressed(KeyCode::ArrowDown) {
        let new_timestep = (time.delta_secs_f64() * 1.025).min(1.0 / 5.0);
        time.set_timestep_seconds(new_timestep);
    }
}

/// Updates the text with the current timestep.
fn update_timestep_text(
    mut text: Single<&mut TextSpan, With<TimestepText>>,
    time: Res<Time<Fixed>>,
) {
    let timestep = time.timestep().as_secs_f32().recip();
    text.0 = format!("{timestep:.2}");
}
