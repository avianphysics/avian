//! Debug UI for displaying [physics diagnostics](crate::diagnostics), such as how much time
//! each part of the simulation takes, and how many rigid bodies and collisions there are.
//!
//! See [`PhysicsDiagnosticsPlugin`] for more information.

use crate::dynamics::solver::constraint_graph::ConstraintGraph;
use crate::{collision::CollisionDiagnostics, dynamics::solver::SolverDiagnostics};
use crate::{diagnostics::*, prelude::*};
use bevy::color::palettes::tailwind::{GREEN_400, RED_400};
use bevy::diagnostic::{DiagnosticPath, DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy::ecs::relationship::RelatedSpawnerCommands;
use bevy::prelude::*;
use entity_counters::PhysicsEntityDiagnosticsPlugin;

const FRAME_TIME_DIAGNOSTIC: &DiagnosticPath = &FrameTimeDiagnosticsPlugin::FRAME_TIME;

/// A plugin that adds debug UI for [physics diagnostics](crate::diagnostics), such as how much time
/// each part of the simulation takes, and how many rigid bodies and collisions there are.
///
/// To customize the visibility and appearance of the UI, modify the [`PhysicsDiagnosticsUiSettings`] resource.
pub struct PhysicsDiagnosticsUiPlugin;

impl Plugin for PhysicsDiagnosticsUiPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<PhysicsDiagnosticsUiSettings>();

        app.add_systems(Startup, setup_diagnostics_ui).add_systems(
            Update,
            (
                update_diagnostics_ui_visibility,
                (
                    update_counters,
                    update_timers,
                    update_graph_color_text,
                    update_diagnostic_row_visibility,
                    update_diagnostic_group_visibility,
                )
                    .run_if(diagnostics_are_enabled),
            )
                .chain(),
        );
    }

    fn finish(&self, app: &mut App) {
        if !app.is_plugin_added::<PhysicsTotalDiagnosticsPlugin>() {
            app.add_plugins(PhysicsTotalDiagnosticsPlugin);
        }
        if !app.is_plugin_added::<PhysicsEntityDiagnosticsPlugin>() {
            app.add_plugins(PhysicsEntityDiagnosticsPlugin);
        }
    }
}

/// Settings for the [physics diagnostics](crate::diagnostics)
/// debug [UI](PhysicsDiagnosticsUiPlugin).
#[derive(Resource, Debug, Reflect)]
#[reflect(Resource, Debug)]
pub struct PhysicsDiagnosticsUiSettings {
    /// Whether the diagnostics UI is enabled.
    pub enabled: bool,
    /// Whether to show the average values of timers.
    pub show_average_times: bool,
}

impl Default for PhysicsDiagnosticsUiSettings {
    fn default() -> Self {
        Self {
            enabled: true,
            show_average_times: true,
        }
    }
}

/// A marker component for the [physics diagnostics](crate::diagnostics)
/// debug [UI](PhysicsDiagnosticsUiPlugin) node.
#[derive(Component)]
pub struct PhysicsDiagnosticsUiNode;

/// A marker component for a group of diagnostics.
#[derive(Component)]
struct DiagnosticGroup;

/// A marker component for a row representing the name and value of a diagnostic.
#[derive(Component)]
struct DiagnosticRow;

/// A marker component for the name text node of a diagnostic.
#[derive(Component)]
#[require(TextFont = diagnostic_font())]
struct PhysicsDiagnosticName;

/// A component with the [`DiagnosticPath`] of a diagnostic.
#[derive(Component)]
#[require(TextFont = diagnostic_font())]
struct PhysicsDiagnosticPath(&'static DiagnosticPath);

/// A marker component for a counter diagnostic.
#[derive(Component)]
#[require(TextFont = diagnostic_font())]
struct PhysicsDiagnosticCounter;

/// A marker component for a timer diagnostic.
#[derive(Component)]
#[require(TextFont = diagnostic_font())]
struct PhysicsDiagnosticTimer;

fn diagnostic_font() -> TextFont {
    TextFont {
        font_size: 10.0,
        ..default()
    }
}

/// A component that configures how the text color should adapt
/// based on the numerical value of a diagnostic.
///
/// The color will linearly interpolate between green and red
/// based on the `lower_bound` and `upper_bound`.
/// If the value is below `lower_bound`, the color will be green.
/// If the value is above `upper_bound`, the color will be red.
#[derive(Component, Clone, Copy)]
struct AdaptiveTextSettings {
    lower_bound: f32,
    upper_bound: f32,
}

impl AdaptiveTextSettings {
    fn new(lower_bound: f32, upper_bound: f32) -> Self {
        Self {
            lower_bound,
            upper_bound,
        }
    }
}

/// A marker component for the graph coloring text.
#[derive(Component)]
#[require(TextFont = diagnostic_font())]
struct GraphColorText;

fn setup_diagnostics_ui(mut commands: Commands, settings: Res<PhysicsDiagnosticsUiSettings>) {
    commands
        .spawn((
            Name::new("Physics Diagnostics"),
            PhysicsDiagnosticsUiNode,
            Node {
                position_type: PositionType::Absolute,
                top: Val::Px(5.0),
                left: Val::Px(5.0),
                width: Val::Px(270.0),
                padding: UiRect::all(Val::Px(10.0)),
                border_radius: BorderRadius::all(Val::Px(5.0)),
                display: if settings.enabled {
                    Display::Flex
                } else {
                    Display::None
                },
                flex_direction: FlexDirection::Column,
                row_gap: Val::Px(10.0),
                ..default()
            },
            BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.8)),
        ))
        .with_children(build_diagnostic_texts);
}

fn build_diagnostic_texts(cmd: &mut RelatedSpawnerCommands<ChildOf>) {
    // Step counter
    cmd.diagnostic_group("Step Counter")
        .with_children(|cmd| cmd.counter_text("Step Number", PhysicsTotalDiagnostics::STEP_NUMBER));

    // Graph colors
    cmd.spawn((
        Name::new("Graph Colors".to_string()),
        Node {
            display: Display::Flex,
            flex_direction: FlexDirection::Column,
            row_gap: Val::Px(1.0),
            ..default()
        },
    ))
    .with_children(|cmd| {
        cmd.spawn(Node {
            display: Display::Flex,
            justify_content: JustifyContent::SpaceBetween,
            align_items: AlignItems::Center,
            column_gap: Val::Px(20.0),
            ..default()
        })
        .with_children(|cmd| {
            cmd.spawn((PhysicsDiagnosticName, Text::new("Colors")));
            cmd.spawn((
                GraphColorText,
                Node {
                    flex_wrap: FlexWrap::Wrap,
                    ..default()
                },
                Children::default(),
            ));
        });
    });

    cmd.diagnostic_group("Counters").with_children(|cmd| {
        // Dynamic, kinematic, and static body counters
        cmd.diagnostic_row().with_children(|cmd| {
            cmd.spawn((PhysicsDiagnosticName, Text::new("Dynamic/Kinematic/Static")));
            cmd.spawn(Node::default()).with_children(|cmd| {
                cmd.spawn((
                    PhysicsDiagnosticCounter,
                    PhysicsDiagnosticPath(PhysicsEntityDiagnostics::DYNAMIC_BODY_COUNT),
                    Text::new("-"),
                ));
                cmd.spawn((Text::new("/"), diagnostic_font()));
                cmd.spawn((
                    PhysicsDiagnosticCounter,
                    PhysicsDiagnosticPath(PhysicsEntityDiagnostics::KINEMATIC_BODY_COUNT),
                    Text::new("-"),
                ));
                cmd.spawn((Text::new("/"), diagnostic_font()));
                cmd.spawn((
                    PhysicsDiagnosticCounter,
                    PhysicsDiagnosticPath(PhysicsEntityDiagnostics::STATIC_BODY_COUNT),
                    Text::new("-"),
                ));
            });
        });

        // Other counters
        cmd.counter_text("Colliders", PhysicsEntityDiagnostics::COLLIDER_COUNT);
        cmd.counter_text("Joints", PhysicsEntityDiagnostics::JOINT_COUNT);
        cmd.counter_text("Contact Pairs", CollisionDiagnostics::CONTACT_COUNT);
        cmd.counter_text(
            "Contact Constraints",
            SolverDiagnostics::CONTACT_CONSTRAINT_COUNT,
        );
    });

    // Collision detection and solver timers
    type Collision = CollisionDiagnostics;
    type Solver = SolverDiagnostics;
    let collision_timers = vec![
        ("Broad Phase", Collision::BROAD_PHASE),
        ("Narrow Phase", Collision::NARROW_PHASE),
    ];
    let solver_timers = vec![
        ("Prepare Constraints", Solver::PREPARE_CONSTRAINTS),
        ("Velocity Increments", Solver::UPDATE_VELOCITY_INCREMENTS),
        ("Integrate Velocities", Solver::INTEGRATE_VELOCITIES),
        ("Warm Start", Solver::WARM_START),
        ("Solve Constraints", Solver::SOLVE_CONSTRAINTS),
        ("Integrate Positions", Solver::INTEGRATE_POSITIONS),
        ("Relax Velocities", Solver::RELAX_VELOCITIES),
        ("Apply Restitution", Solver::APPLY_RESTITUTION),
        ("Finalize", Solver::FINALIZE),
        ("Store Impulses", Solver::STORE_IMPULSES),
        ("Swept CCD", Solver::SWEPT_CCD),
    ];
    cmd.diagnostic_group("Collision Detection and Solver")
        .with_children(|cmd| {
            cmd.timer_texts(collision_timers, AdaptiveTextSettings::new(0.0, 4.0));
            cmd.timer_texts(solver_timers, AdaptiveTextSettings::new(0.0, 4.0));
        });

    // Spatial query timers
    type Spatial = SpatialQueryDiagnostics;
    let spatial_query_timers = vec![
        ("Spatial Query BVH", Spatial::UPDATE_PIPELINE),
        ("Ray Casters", Spatial::UPDATE_RAY_CASTERS),
        ("Shape Casters", Spatial::UPDATE_SHAPE_CASTERS),
        #[cfg(feature = "bevy_picking")]
        (
            "Physics Picking",
            crate::picking::PhysicsPickingDiagnostics::UPDATE_HITS,
        ),
    ];
    cmd.diagnostic_group("Spatial Queries")
        .with_children(|cmd| {
            cmd.timer_texts(spatial_query_timers, AdaptiveTextSettings::new(0.0, 4.0));
        });

    cmd.diagnostic_group("Other").with_children(|cmd| {
        cmd.timer_text(
            "Other",
            PhysicsTotalDiagnostics::MISCELLANEOUS,
            AdaptiveTextSettings::new(0.0, 4.0),
        );
    });

    // Total times
    cmd.diagnostic_group("Total Times").with_children(|cmd| {
        cmd.timer_text(
            "Total Step",
            PhysicsTotalDiagnostics::STEP_TIME,
            AdaptiveTextSettings {
                lower_bound: 3.0,
                upper_bound: 20.0,
            },
        );
        cmd.timer_text(
            "Frame Time",
            FRAME_TIME_DIAGNOSTIC,
            AdaptiveTextSettings {
                lower_bound: 16.0,
                upper_bound: 50.0,
            },
        );
    });
}

/// An extension trait for building physics diagnostics UI.
trait CommandsExt {
    fn diagnostic_group(&mut self, name: &str) -> EntityCommands<'_>;

    fn diagnostic_row(&mut self) -> EntityCommands<'_>;

    fn counter_text(&mut self, text: &str, diagnostic_path: &'static DiagnosticPath);

    fn timer_text(
        &mut self,
        text: &str,
        diagnostic_path: &'static DiagnosticPath,
        adaptive_text: AdaptiveTextSettings,
    );

    fn timer_texts(
        &mut self,
        texts: Vec<(&str, &'static DiagnosticPath)>,
        adaptive: AdaptiveTextSettings,
    ) {
        for (text, path) in texts {
            self.timer_text(text, path, adaptive);
        }
    }
}

impl CommandsExt for RelatedSpawnerCommands<'_, ChildOf> {
    fn diagnostic_group(&mut self, name: &str) -> EntityCommands<'_> {
        self.spawn((
            DiagnosticGroup,
            Name::new(name.to_string()),
            Node {
                display: Display::Flex,
                flex_direction: FlexDirection::Column,
                row_gap: Val::Px(1.0),
                ..default()
            },
        ))
    }

    fn diagnostic_row(&mut self) -> EntityCommands<'_> {
        self.spawn((
            DiagnosticRow,
            Node {
                display: Display::Flex,
                justify_content: JustifyContent::SpaceBetween,
                column_gap: Val::Px(20.0),
                ..default()
            },
        ))
    }

    fn counter_text(&mut self, text: &str, diagnostic_path: &'static DiagnosticPath) {
        self.diagnostic_row().with_children(|child_builder| {
            child_builder.spawn((
                Name::new(text.to_string()),
                Text::new(text),
                diagnostic_font(),
            ));
            child_builder.spawn((
                PhysicsDiagnosticCounter,
                PhysicsDiagnosticPath(diagnostic_path),
                Text::new("-"),
            ));
        });
    }

    fn timer_text(
        &mut self,
        text: &str,
        diagnostic_path: &'static DiagnosticPath,
        adaptive_text: AdaptiveTextSettings,
    ) {
        self.diagnostic_row().with_children(|child_builder| {
            child_builder.spawn((
                Name::new(text.to_string()),
                Text::new(text),
                diagnostic_font(),
            ));
            child_builder.spawn((
                PhysicsDiagnosticTimer,
                PhysicsDiagnosticPath(diagnostic_path),
                adaptive_text,
                Text::new("-"),
            ));
        });
    }
}

fn update_counters(
    diagnostics: Res<DiagnosticsStore>,
    mut query: Query<
        (&mut Text, &PhysicsDiagnosticPath, &mut Node),
        With<PhysicsDiagnosticCounter>,
    >,
) {
    for (mut text, path, mut node) in &mut query {
        // Get the diagnostic.
        let Some(diagnostic) = diagnostics.get(path.0) else {
            // Hide the diagnostic if the diagnostic is not found.
            node.display = Display::None;
            continue;
        };

        // Get the measurement value.
        let Some(measurement) = diagnostic.measurement() else {
            continue;
        };

        if diagnostic.suffix.is_empty() {
            text.0 = (measurement.value as u32).to_string();
        } else {
            text.0 = format!("{} {}", measurement.value as u32, diagnostic.suffix);
        }

        // Make sure the node is visible.
        node.display = Display::Flex;
    }
}

fn update_timers(
    diagnostics: Res<DiagnosticsStore>,
    mut query: Query<
        (
            &mut Text,
            &mut TextColor,
            &AdaptiveTextSettings,
            &PhysicsDiagnosticPath,
            &mut Node,
        ),
        With<PhysicsDiagnosticTimer>,
    >,
    settings: Res<PhysicsDiagnosticsUiSettings>,
) {
    let green = LinearRgba::from(GREEN_400);
    let red = LinearRgba::from(RED_400);

    for (mut text, mut text_color, color_config, path, mut node) in &mut query {
        // Get the diagnostic.
        let Some(diagnostic) = diagnostics.get(path.0).filter(|d| d.value().is_some()) else {
            // Hide the diagnostic if the diagnostic is not found.
            node.display = Display::None;
            continue;
        };

        // Get the measurement and average values.
        let (Some(measurement), Some(average)) = (diagnostic.measurement(), diagnostic.average())
        else {
            continue;
        };

        // Update the text.
        text.0 = if settings.show_average_times {
            format!(
                "{:.2} (avg {:.2}) {}",
                measurement.value, average, diagnostic.suffix
            )
        } else {
            format!("{:.2} {}", measurement.value, diagnostic.suffix)
        };

        // Linearly interpolating between green and red, with `lower_bound` and `upper_bound`.
        let t = ((average as f32 - color_config.lower_bound)
            / (color_config.upper_bound - color_config.lower_bound))
            .min(1.0);
        text_color.0 = (green * (1.0 - t) + red * t).into();

        // Make sure the node is visible.
        node.display = Display::Flex;
    }
}

fn update_graph_color_text(
    constraint_graph: Option<Res<ConstraintGraph>>,
    children: Single<(Entity, &Children), With<GraphColorText>>,
    mut text_span_query: Query<&mut Text>,
    mut commands: Commands,
) {
    let Some(graph) = constraint_graph else {
        return;
    };

    let (entity, children) = children.into_inner();
    let children = children.iter().collect::<Vec<Entity>>();

    for (i, color) in graph.colors.iter().enumerate() {
        // Count the number of bodies in the color.
        let body_count = color.manifold_handles.len();

        // Update or spawn the text spans for the color.
        if let Some(Ok(mut text_span)) = children
            .get(i * 2)
            .map(|&child| text_span_query.get_mut(child))
        {
            text_span.0 = body_count.to_string();
        } else {
            commands.spawn((
                Text::new(body_count.to_string()),
                diagnostic_font(),
                ChildOf(entity),
            ));
            if i < graph.colors.len() - 1 {
                commands.spawn((Text::new("/"), diagnostic_font(), ChildOf(entity)));
            }
        }
    }
}

fn update_diagnostics_ui_visibility(
    settings: Res<PhysicsDiagnosticsUiSettings>,
    mut query: Query<&mut Node, With<PhysicsDiagnosticsUiNode>>,
) {
    if !settings.is_changed() {
        return;
    }

    for mut node in &mut query {
        node.display = if settings.enabled {
            Display::Flex
        } else {
            Display::None
        };
    }
}

fn update_diagnostic_row_visibility(
    mut query: Query<(Entity, &mut Node), (With<DiagnosticRow>, Without<PhysicsDiagnosticPath>)>,
    child_query: Query<&Children>,
    diagnostic_query: Query<&Node, With<PhysicsDiagnosticPath>>,
) {
    // Hide the row if all diagnostics are hidden.
    for (entity, mut node) in &mut query {
        let visible = child_query.iter_descendants(entity).any(|child| {
            diagnostic_query
                .get(child)
                .is_ok_and(|node| node.display == Display::Flex)
        });

        node.display = if visible {
            Display::Flex
        } else {
            Display::None
        };
    }
}

fn update_diagnostic_group_visibility(
    mut query: Query<(Entity, &mut Node), (With<DiagnosticGroup>, Without<DiagnosticRow>)>,
    child_query: Query<&Children>,
    row_query: Query<&Node, With<DiagnosticRow>>,
) {
    // Hide the group if all rows are hidden.
    for (entity, mut node) in &mut query {
        let visible = child_query.iter_descendants(entity).any(|child| {
            row_query
                .get(child)
                .is_ok_and(|node| node.display == Display::Flex)
        });

        node.display = if visible {
            Display::Flex
        } else {
            Display::None
        };
    }
}

/// A run condition that returns true if the diagnostics UI is enabled.
fn diagnostics_are_enabled(settings: Res<PhysicsDiagnosticsUiSettings>) -> bool {
    settings.enabled
}
