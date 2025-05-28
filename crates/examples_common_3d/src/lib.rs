use avian3d::prelude::*;
use bevy::{
    diagnostic::FrameTimeDiagnosticsPlugin, input::common_conditions::input_just_pressed,
    prelude::*,
};

/// A plugin that adds common functionality used by examples,
/// such as physics diagnostics UI and the ability to pause and step the simulation.
pub struct ExampleCommonPlugin;

impl Plugin for ExampleCommonPlugin {
    fn build(&self, app: &mut App) {
        // Add diagnostics.
        app.add_plugins((
            PhysicsDiagnosticsPlugin,
            PhysicsDiagnosticsUiPlugin,
            FrameTimeDiagnosticsPlugin::default(),
        ));

        // Configure the default physics diagnostics UI.
        app.insert_resource(PhysicsDiagnosticsUiSettings {
            enabled: false,
            ..default()
        });

        // Spawn text instructions for keybinds.
        app.add_systems(Startup, setup_key_instructions);

        // Add systems for toggling the diagnostics UI and pausing and stepping the simulation.
        app.add_systems(
            Update,
            (
                toggle_diagnostics_ui.run_if(input_just_pressed(KeyCode::KeyU)),
                toggle_paused.run_if(input_just_pressed(KeyCode::KeyP)),
                step.run_if(physics_paused.and(input_just_pressed(KeyCode::Enter))),
            ),
        );
    }

    #[cfg(feature = "use-debug-plugin")]
    fn finish(&self, app: &mut App) {
        // Add the physics debug plugin automatically if the `use-debug-plugin` feature is enabled
        // and the plugin is not already added.
        if !app.is_plugin_added::<PhysicsDebugPlugin>() {
            app.add_plugins(PhysicsDebugPlugin::default());
        }
    }
}

fn toggle_diagnostics_ui(mut settings: ResMut<PhysicsDiagnosticsUiSettings>) {
    settings.enabled = !settings.enabled;
}

fn physics_paused(time: Res<Time<Physics>>) -> bool {
    time.is_paused()
}

fn toggle_paused(mut time: ResMut<Time<Physics>>) {
    if time.is_paused() {
        time.unpause();
    } else {
        time.pause();
    }
}

/// Advances the physics simulation by one `Time<Fixed>` time step.
fn step(mut physics_time: ResMut<Time<Physics>>, fixed_time: Res<Time<Fixed>>) {
    physics_time.advance_by(fixed_time.delta());
}

fn setup_key_instructions(mut commands: Commands) {
    commands.spawn((
        Text::new("U: Diagnostics UI | P: Pause/Unpause | Enter: Step"),
        TextFont {
            font_size: 10.0,
            ..default()
        },
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(5.0),
            right: Val::Px(5.0),
            ..default()
        },
    ));
}
