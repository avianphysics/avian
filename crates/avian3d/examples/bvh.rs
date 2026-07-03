//! Demonstrates Avian's BVH acceleration structures used for broad phase collision detection
//! and spatial queries.
//!
//! This example is primarily intended for performance testing and demonstration purposes,
//! not for practical use.
//!
//! The scene spawns a grid of colliders that move randomly each frame.
//! The size of the grid and the movement parameters can be adjusted via GUI controls.

use avian3d::{math::*, prelude::*};
use bevy::{
    color::palettes::tailwind::GRAY_400,
    feathers::{
        FeathersPlugins,
        constants::fonts::BOLD,
        controls::{FeathersCheckbox, FeathersRadio, FeathersSlider},
        dark_theme::create_dark_theme,
        theme::{ThemedText, UiTheme},
    },
    prelude::*,
    text::FontSourceTemplate,
    ui::Checked,
    ui_widgets::{
        RadioButton, RadioGroup, SliderPrecision, SliderStep, ValueChange, slider_self_update,
    },
};
use examples_common_3d::ExampleCommonPlugin;
use rand::RngExt;

fn main() {
    let mut app = App::new();

    // Add plugins relevant to the example.
    app.add_plugins((
        DefaultPlugins,
        FeathersPlugins,
        ExampleCommonPlugin,
        PhysicsDebugPlugin,
    ));

    // Add minimal physics plugins required for the example.
    // TODO: Make these more minimal and ideally use more plugin groups.
    app.add_plugins((
        PhysicsSchedulePlugin::default(),
        ColliderHierarchyPlugin,
        ColliderTransformPlugin::default(),
        ColliderBackendPlugin::<Collider>::default(),
        ColliderTreePlugin::<Collider>::default(),
        BroadPhaseCorePlugin,
        BvhBroadPhasePlugin::<()>::default(),
        PhysicsTransformPlugin::default(),
        // TODO: These are currently needed for collider tree updates, but they shouldn't be.
        SolverBodyPlugin,
        SolverSchedulePlugin,
    ));

    // Configure gizmos and initialize example settings.
    app.insert_gizmo_config(
        PhysicsGizmos {
            aabb_color: Some(GRAY_400.into()),
            collider_tree_color: Some(Color::WHITE),
            ..PhysicsGizmos::none()
        },
        GizmoConfig {
            line: GizmoLineConfig {
                width: 0.5,
                ..default()
            },
            ..default()
        },
    )
    .insert_resource(UiTheme(create_dark_theme()))
    .init_resource::<BvhExampleSettings>()
    .insert_resource(Gravity::ZERO);

    // Add systems for setting up and running the example.
    app.add_systems(Startup, (setup_scene, setup_ui))
        .add_systems(FixedUpdate, move_random);

    app.run();
}

const PARTICLE_RADIUS: f32 = 7.0;

/// Settings for the BVH example.
#[derive(Resource)]
struct BvhExampleSettings {
    x_count: usize,
    y_count: usize,
    move_fraction: f32,
    delta_fraction: f32,
}

impl Default for BvhExampleSettings {
    fn default() -> Self {
        Self {
            x_count: 100,
            y_count: 100,
            move_fraction: 0.25,
            delta_fraction: 0.1,
        }
    }
}

/// Sets up the initial scene with a grid of colliders.
fn setup_scene(mut commands: Commands, settings: Res<BvhExampleSettings>) {
    let x_count = settings.x_count as isize;
    let y_count = settings.y_count as isize;

    commands.spawn((
        Camera3d::default(),
        Projection::Orthographic(OrthographicProjection {
            scaling_mode: bevy::camera::ScalingMode::FixedVertical {
                viewport_height: 3.0 * PARTICLE_RADIUS * (y_count as f32 * 1.2),
            },
            ..OrthographicProjection::default_3d()
        }),
        Transform::from_xyz(0.0, 0.0, 30.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    for x in -x_count / 2..x_count / 2 {
        for y in -y_count / 2..y_count / 2 {
            commands.spawn((
                Transform::from_xyz(
                    (x as f32 + 0.5) * 3.0 * PARTICLE_RADIUS,
                    (y as f32 + 0.5) * 3.0 * PARTICLE_RADIUS,
                    0.0,
                ),
                Collider::sphere(PARTICLE_RADIUS.adjust_precision()),
                CollisionLayers::new(LayerMask::DEFAULT, LayerMask::NONE),
            ));
        }
    }
}

/// Clears the scene of all rigid bodies and cameras.
#[expect(clippy::type_complexity)]
fn clear_scene(mut commands: Commands, query: Query<Entity, Or<(With<Collider>, With<Camera>)>>) {
    for entity in query.iter() {
        commands.entity(entity).despawn();
    }
}

/// Moves a fraction of the colliders randomly each frame.
fn move_random(mut query: Query<&mut Transform>, settings: Res<BvhExampleSettings>) {
    if settings.move_fraction <= 0.0 || settings.delta_fraction <= 0.0 {
        return;
    }

    let mut rng = rand::rng();
    for mut transform in query.iter_mut() {
        if rng.random::<f32>() < settings.move_fraction {
            transform.translation += Vec3::new(
                rng.random_range(
                    -PARTICLE_RADIUS * settings.delta_fraction
                        ..PARTICLE_RADIUS * settings.delta_fraction,
                ),
                rng.random_range(
                    -PARTICLE_RADIUS * settings.delta_fraction
                        ..PARTICLE_RADIUS * settings.delta_fraction,
                ),
                0.0,
            );
        }
    }
}

// === UI Setup ===

#[derive(Component, Clone, Default)]
struct OptimizationModeRadio(TreeOptimizationMode);

#[derive(Component, Clone, Default)]
struct GridSizeRadio(usize);

// TODO: Change optimization settings at runtime.
fn setup_ui(mut commands: Commands, settings: Res<BvhExampleSettings>) {
    commands.spawn_scene(bsn! {
        Name("Example Settings UI")
        Node {
            position_type: PositionType::Absolute,
            bottom: Val::Px(5.0),
            right: Val::Px(5.0),
            width: Val::Px(270.0),
            padding: UiRect::all(Val::Px(10.0)),
            border_radius: BorderRadius::all(Val::Px(5.0)),
            flex_direction: FlexDirection::Column,
            row_gap: Val::Px(15.0),
        }
        BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.8))
        Children [
            (
                Node {
                    flex_direction: FlexDirection::Column,
                    row_gap: Val::Px(5.0),
                }
                Children [
                    (
                        Text("Optimization Mode")
                        TextFont {
                            font: FontSourceTemplate::Handle(BOLD),
                            font_size: FontSize::Px(14.0),
                        }
                        ThemedText
                    ),
                    (
                        Node {
                            display: Display::Flex,
                            flex_direction: FlexDirection::Column,
                            row_gap: px(5),
                        }
                        RadioGroup
                        on(
                            |value_change: On<ValueChange<Entity>>,
                             radio_buttons: Query<
                                (Entity, &OptimizationModeRadio),
                                With<RadioButton>,
                            >,
                             mut settings: ResMut<ColliderTreeOptimization>,
                             mut commands: Commands| {
                                for (entity, optimization_mode) in radio_buttons.iter() {
                                    if entity == value_change.value {
                                        commands.entity(entity).insert(Checked);
                                        if optimization_mode.0 == settings.optimization_mode {
                                            continue;
                                        }
                                        settings.optimization_mode = optimization_mode.0;
                                        commands.run_system_cached(clear_scene);
                                        commands.run_system_cached(setup_scene);
                                    } else {
                                        commands.entity(entity).remove::<Checked>();
                                    }
                                }
                            }
                        )
                        Children [
                            (
                                @FeathersRadio {
                                    @caption: bsn! {
                                        Text("Reinsert") ThemedText
                                    }
                                }
                                OptimizationModeRadio(TreeOptimizationMode::Reinsert)
                            ),
                            (
                                @FeathersRadio {
                                    @caption: bsn! {
                                        Text("Partial Rebuild") ThemedText
                                    }
                                }
                                OptimizationModeRadio(TreeOptimizationMode::PartialRebuild)
                            ),
                            (
                                @FeathersRadio {
                                    @caption: bsn! {
                                        Text("Full Rebuild") ThemedText
                                    }
                                }
                                OptimizationModeRadio(TreeOptimizationMode::FullRebuild)
                            ),
                            (
                                @FeathersRadio {
                                    @caption: bsn! {
                                        Text("Adaptive") ThemedText
                                    }
                                }
                                Checked
                                OptimizationModeRadio(TreeOptimizationMode::default())
                            ),
                        ]
                    ),
                ]
            ),
            (
                Node {
                    flex_direction: FlexDirection::Column,
                    row_gap: Val::Px(5.0),
                }
                Children [
                    (
                        Text("Grid Size")
                        TextFont {
                            font: FontSourceTemplate::Handle(BOLD),
                            font_size: FontSize::Px(14.0),
                        }
                        ThemedText
                    ),
                    (
                        Node {
                            display: Display::Flex,
                            flex_direction: FlexDirection::Column,
                            row_gap: px(5),
                        }
                        RadioGroup
                        on(
                            |value_change: On<ValueChange<Entity>>,
                             radio_buttons: Query<(Entity, &GridSizeRadio), With<RadioButton>>,
                             mut settings: ResMut<BvhExampleSettings>,
                             mut commands: Commands| {
                                for (entity, grid_size) in radio_buttons.iter() {
                                    if entity == value_change.value {
                                        commands.entity(entity).insert(Checked);
                                        if grid_size.0 == settings.x_count {
                                            continue;
                                        }
                                        settings.x_count = grid_size.0;
                                        settings.y_count = grid_size.0;
                                        commands.run_system_cached(clear_scene);
                                        commands.run_system_cached(setup_scene);
                                    } else {
                                        commands.entity(entity).remove::<Checked>();
                                    }
                                }
                            }
                        )
                        Children [
                            (
                                @FeathersRadio {
                                    @caption: bsn! { Text("10x10") ThemedText }
                                }
                                GridSizeRadio(10)
                            ),
                            (
                                @FeathersRadio {
                                    @caption: bsn! { Text("50x50") ThemedText }
                                }
                                GridSizeRadio(50)
                            ),
                            (
                                @FeathersRadio {
                                    @caption: bsn! { Text("100x100") ThemedText }
                                }
                                Checked
                                GridSizeRadio(100)
                            ),
                        ]
                    ),
                ]
            ),
            (
                Node {
                    flex_direction: FlexDirection::Column,
                    row_gap: Val::Px(5.0),
                }
                Children [
                    (
                        Text("Move Fraction")
                        TextFont {
                            font: FontSourceTemplate::Handle(BOLD),
                            font_size: FontSize::Px(14.0),
                        }
                        ThemedText
                    ),
                    (
                        @FeathersSlider {
                            @min: 0.0,
                            @max: 1.0,
                            @value: {settings.move_fraction},
                        }
                        SliderStep(0.05)
                        SliderPrecision(2)
                        on(slider_self_update)
                        on(
                            |change: On<ValueChange<f32>>,
                             mut settings: ResMut<BvhExampleSettings>| {
                                settings.move_fraction = change.value;
                            },
                        )
                    )
                ]
            ),
            (
                Node {
                    flex_direction: FlexDirection::Column,
                    row_gap: Val::Px(5.0),
                }
                Children [
                    (
                        Text("Delta Fraction")
                        TextFont {
                            font: FontSourceTemplate::Handle(BOLD),
                            font_size: FontSize::Px(14.0),
                        }
                        ThemedText
                    ),
                    (
                        @FeathersSlider {
                            @min: 0.0,
                            @max: 1.0,
                            @value: {settings.delta_fraction},
                        }
                        SliderStep(0.05)
                        SliderPrecision(2)
                        on(slider_self_update)
                        on(
                            |change: On<ValueChange<f32>>,
                             mut settings: ResMut<BvhExampleSettings>| {
                                settings.delta_fraction = change.value;
                            },
                        )
                    )
                ]
            ),
            (
                Node {
                    flex_direction: FlexDirection::Column,
                    row_gap: Val::Px(5.0),
                }
                Children [
                    (
                        Text("BVH Debug Rendering")
                        TextFont {
                            font: FontSourceTemplate::Handle(BOLD),
                            font_size: FontSize::Px(14.0),
                        }
                        ThemedText
                    ),
                    (
                        @FeathersCheckbox {
                            @caption: bsn! { Text("Draw Internal Nodes") ThemedText }
                        }
                        Checked
                        on(
                            |change: On<ValueChange<bool>>,
                             mut gizmo_store: ResMut<GizmoConfigStore>,
                             mut commands: Commands| {
                                let gizmo_config = gizmo_store.config_mut::<PhysicsGizmos>().1;
                                if change.value {
                                    gizmo_config.collider_tree_color = Some(Color::WHITE);
                                    commands.entity(change.source).insert(Checked);
                                } else {
                                    gizmo_config.collider_tree_color = None;
                                    commands.entity(change.source).remove::<Checked>();
                                }
                            },
                        )
                    ),
                    (
                        @FeathersCheckbox {
                            @caption: bsn! { Text("Draw Leaf Nodes") ThemedText }
                        }
                        Checked
                        on(
                            |change: On<ValueChange<bool>>,
                             mut gizmo_store: ResMut<GizmoConfigStore>,
                             mut commands: Commands| {
                                let gizmo_config = gizmo_store.config_mut::<PhysicsGizmos>().1;
                                if change.value {
                                    gizmo_config.aabb_color = Some(GRAY_400.into());
                                    commands.entity(change.source).insert(Checked);
                                } else {
                                    gizmo_config.aabb_color = None;
                                    commands.entity(change.source).remove::<Checked>();
                                }
                            },
                        )
                    )
                ]
            ),
        ]
    });
}
