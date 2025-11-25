//! Demonstrates using move and slide to control a kinematic character.
//!
//! This is intended more as a testbed and less as a polished usage example.

use core::f32::consts::FRAC_PI_2;

use avian3d::{math::*, prelude::*};
use bevy::{
    asset::io::web::WebAssetPlugin,
    color::palettes::tailwind,
    ecs::entity::EntityHashSet,
    gltf::GltfLoaderSettings,
    input::{common_conditions::input_just_pressed, mouse::AccumulatedMouseMotion},
    pbr::Atmosphere,
    prelude::*,
    window::{CursorGrabMode, CursorOptions},
};
use examples_common_3d::ExampleCommonPlugin;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins.set(WebAssetPlugin {
                silence_startup_warning: true,
            }),
            ExampleCommonPlugin,
            PhysicsPlugins::default(),
        ))
        .add_systems(Startup, setup)
        .add_systems(FixedUpdate, move_player)
        .add_systems(
            Update,
            (
                update_camera_transform,
                capture_cursor.run_if(input_just_pressed(MouseButton::Left)),
                release_cursor.run_if(input_just_pressed(KeyCode::Escape)),
                update_debug_text,
            ),
        )
        .run();
}

#[derive(Component, Default)]
struct Player {
    /// The velocity of the player.
    ///
    /// We cannot use [`LinearVelocity`] directly, because we control the transform manually,
    /// and don't want to also apply simulation velocity on top.
    velocity: Vec3,
    /// The entities touched during the last `move_and_slide` call. Stored for debug printing.
    touched: EntityHashSet,
}

#[derive(Component)]
struct DebugText;

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
    assets: ResMut<AssetServer>,
) {
    // Character
    let shape = Sphere::new(0.5);
    commands.spawn((
        Mesh3d(meshes.add(shape)),
        MeshMaterial3d(materials.add(Color::from(tailwind::SKY_400.with_alpha(0.6)))),
        Collider::from(shape),
        RigidBody::Kinematic,
        Player::default(),
        TransformInterpolation,
        // Not needed for move and slide to work, but we add it for debug printing
        CollidingEntities::default(),
    ));

    // Scene
    commands.spawn((
        SceneRoot(assets.load_with_settings(
            "https://github.com/avianphysics/avian_asset_files/raw/08f82a1031c4fbdf1a461600468d2a37593a804a/move_and_slide_level/move_and_slide_level.glb#Scene0",
            |settings: &mut GltfLoaderSettings| {
                settings.use_model_forward_direction = Some(true);
            },
        )),
        ColliderConstructorHierarchy::new(ColliderConstructor::TrimeshFromMesh),
        RigidBody::Static,
    )).observe(|
        _ready: On<ColliderConstructorHierarchyReady>,
        mut commands: Commands,
        mut meshes: ResMut<Assets<Mesh>>,
        mut materials: ResMut<Assets<StandardMaterial>>| {
        // Add some dynamic cubes
        for i in 0..5 {
            for j in 0..5 {
                let position = Vec3::new(i as f32 * 2.0 - 15.0, 0.0, j as f32 * 2.0 - 15.0);
                let cube = Cuboid::from_length(0.75);
                commands.spawn((
                    Name::new("Cube"),
                    Mesh3d(meshes.add(cube)),
                    MeshMaterial3d(materials.add(StandardMaterial::default())),
                    Collider::from(cube),
                    RigidBody::Dynamic,
                    Transform::from_translation(position),
                ));
            }
        }
    });

    // Light
    commands.spawn((
        DirectionalLight {
            illuminance: 6000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::default().looking_at(Vec3::new(-1.0, -3.0, -2.0), Vec3::Y),
    ));

    // Camera and atmosphere
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-5.0, 3.5, 5.5).looking_at(Vec3::ZERO, Vec3::Y),
        Atmosphere::EARTH,
        EnvironmentMapLight {
            diffuse_map: assets.load("https://github.com/avianphysics/avian_asset_files/raw/08f82a1031c4fbdf1a461600468d2a37593a804a/voortrekker_interior/voortrekker_interior_1k_diffuse.ktx2"),
            specular_map: assets.load("https://github.com/avianphysics/avian_asset_files/raw/08f82a1031c4fbdf1a461600468d2a37593a804a/voortrekker_interior/voortrekker_interior_1k_specular.ktx2"),
            intensity: 1500.0,
            ..default()
        },
        Projection::Perspective(PerspectiveProjection {
            fov: 70.0_f32.to_radians(),
            ..default()
        }),
    ));

    // Debug test
    commands.spawn((
        DebugText,
        Node {
            width: percent(100.0),
            height: percent(100.0),
            ..default()
        },
        Text::default(),
    ));
}

fn move_player(
    player: Single<(Entity, &mut Transform, &mut Player, &Collider), Without<Camera>>,
    move_and_slide: MoveAndSlide,
    time: Res<Time>,
    input: Res<ButtonInput<KeyCode>>,
    camera: Single<&Transform, With<Camera>>,
    mut gizmos: Gizmos,
) {
    let (entity, mut transform, mut player, collider) = player.into_inner();
    let mut movement_velocity = Vec3::ZERO;
    if input.pressed(KeyCode::KeyW) {
        movement_velocity += Vec3::NEG_Z
    }
    if input.pressed(KeyCode::KeyS) {
        movement_velocity += Vec3::Z
    }
    if input.pressed(KeyCode::KeyA) {
        movement_velocity += Vec3::NEG_X
    }
    if input.pressed(KeyCode::KeyD) {
        movement_velocity += Vec3::X
    }
    if input.pressed(KeyCode::Space) || input.pressed(KeyCode::KeyE) {
        movement_velocity += Vec3::Y
    }
    if input.pressed(KeyCode::ControlLeft) || input.pressed(KeyCode::KeyQ) {
        movement_velocity += Vec3::NEG_Y
    }
    movement_velocity = movement_velocity.normalize_or_zero();
    movement_velocity *= 7.0;
    if input.pressed(KeyCode::ShiftLeft) {
        movement_velocity *= 3.0;
    }
    movement_velocity = camera.rotation * movement_velocity;

    // Add velocity from last frame to preserve momentum
    movement_velocity += player.velocity;

    let current_speed = movement_velocity.length();
    if current_speed > 0.0 {
        // Apply friction
        movement_velocity = movement_velocity / current_speed
            * (current_speed - current_speed * 20.0 * time.delta_secs()).max(0.0)
    }

    player.touched.clear();

    // Perform move and slide
    let MoveAndSlideOutput {
        position,
        projected_velocity: velocity,
    } = move_and_slide.move_and_slide(
        collider,
        transform.translation.adjust_precision(),
        transform.rotation.adjust_precision(),
        movement_velocity.adjust_precision(),
        time.delta(),
        &MoveAndSlideConfig::default(),
        &SpatialQueryFilter::from_excluded_entities([entity]),
        |hit| {
            // For each collision, draw debug gizmos
            if hit.intersects() {
                gizmos.sphere(
                    Isometry3d::from_translation(transform.translation),
                    0.6,
                    tailwind::RED_600,
                );
            } else {
                gizmos.arrow(
                    hit.point.f32(),
                    (hit.point
                        + hit.normal.adjust_precision() * hit.collision_distance
                            / time.delta_secs().adjust_precision())
                    .f32(),
                    tailwind::EMERALD_400,
                );
            }
            player.touched.insert(hit.entity);
            true
        },
    );

    // Update transform and stored velocity
    transform.translation = position.f32();
    player.velocity = velocity.f32();
}

fn update_camera_transform(
    accumulated_mouse_motion: Res<AccumulatedMouseMotion>,
    player: Single<(Entity, &Transform), With<Player>>,
    mut camera: Single<&mut Transform, (With<Camera>, Without<Player>)>,
    spatial: Res<SpatialQueryPipeline>,
) {
    let (player_entity, player_transform) = player.into_inner();
    let delta = accumulated_mouse_motion.delta;

    let delta_yaw = -delta.x * 0.005;
    let delta_pitch = -delta.y * 0.005;

    let (yaw, pitch, roll) = camera.rotation.to_euler(EulerRot::YXZ);
    let yaw = yaw + delta_yaw;

    const PITCH_LIMIT: f32 = FRAC_PI_2 - 0.01;
    let pitch = (pitch + delta_pitch).clamp(-PITCH_LIMIT, PITCH_LIMIT);

    camera.rotation = Quat::from_euler(EulerRot::YXZ, yaw, pitch, roll);
    const MAX_DISTANCE: f32 = 15.0;
    camera.translation = player_transform.translation + camera.back() * MAX_DISTANCE;
    if let Some(hit) = spatial.cast_ray(
        player_transform.translation.adjust_precision(),
        camera.back(),
        MAX_DISTANCE.adjust_precision(),
        true,
        &SpatialQueryFilter::from_excluded_entities([player_entity]),
    ) {
        camera.translation = player_transform.translation
            + camera.back() * (hit.distance.val_num_f32() - 1.0).max(0.0);
    }
}

fn capture_cursor(mut cursor: Single<&mut CursorOptions>) {
    cursor.visible = false;
    cursor.grab_mode = CursorGrabMode::Locked;
}

fn release_cursor(mut cursor: Single<&mut CursorOptions>) {
    cursor.visible = true;
    cursor.grab_mode = CursorGrabMode::None;
}

fn update_debug_text(
    mut text: Single<&mut Text, With<DebugText>>,
    player: Single<(&Player, &CollidingEntities), With<Player>>,
    names: Query<NameOrEntity>,
) {
    let (player, colliding_entities) = player.into_inner();
    ***text = format!(
        "velocity: [{:.3}, {:.3}, {:.3}]\n{} intersections (goal is 0): {:#?}\n{} touched: {:#?}",
        player.velocity.x,
        player.velocity.y,
        player.velocity.z,
        colliding_entities.len(),
        names
            .iter_many(colliding_entities.iter())
            .map(|name| name
                .name
                .map(|n| format!("{} ({})", name.entity, n))
                .unwrap_or_else(|| format!("{}", name.entity)))
            .collect::<Vec<_>>(),
        player.touched.len(),
        names
            .iter_many(player.touched.iter())
            .map(|name| name
                .name
                .map(|n| format!("{} ({})", name.entity, n))
                .unwrap_or_else(|| format!("{}", name.entity)))
            .collect::<Vec<_>>()
    );
}
