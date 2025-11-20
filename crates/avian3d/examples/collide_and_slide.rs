use avian3d::{math::FRAC_PI_2, prelude::*};
use bevy::{
    asset::io::web::WebAssetPlugin,
    color::palettes::tailwind,
    gltf::GltfLoaderSettings,
    input::mouse::AccumulatedMouseMotion,
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
                capture_cursor,
                exit_game,
                update_debug_text,
            ),
        )
        .run();
}

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
        // Not needed for collide-and-slide to work, but we add it for debug printing
        CollidingEntities::default(),
    ));

    // Scene
    commands.spawn((
        SceneRoot(assets.load_with_settings(
            "https://github.com/janhohenheim/avian_asset_files/raw/refs/heads/collide_and_slide/collide_and_slide_level/collide_and_slide_level.glb#Scene0",
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

    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-5.0, 3.5, 5.5).looking_at(Vec3::ZERO, Vec3::Y),
        Atmosphere::EARTH,
        EnvironmentMapLight {
            diffuse_map: assets.load("https://github.com/janhohenheim/avian_asset_files/raw/refs/heads/collide_and_slide/voortrekker_interior/voortrekker_interior_1k_diffuse.ktx2"),
            specular_map: assets.load("https://github.com/janhohenheim/avian_asset_files/raw/refs/heads/collide_and_slide/voortrekker_interior/voortrekker_interior_1k_specular.ktx2"),
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

#[derive(Component)]
struct DebugText;

#[derive(Component, Default)]
struct Player {
    internal_velocity: Vec3,
    touched: Vec<Entity>,
}

fn move_player(
    player: Single<(Entity, &mut Transform, &mut Player, &Collider), Without<Camera>>,
    collide_and_slide: CollideAndSlide,
    time: Res<Time>,
    input: Res<ButtonInput<KeyCode>>,
    camera: Single<&Transform, With<Camera>>,
    mut gizmos: Gizmos,
) {
    let (entity, mut transform, mut player, collider) = player.into_inner();
    let mut wish_velocity = Vec3::ZERO;
    if input.pressed(KeyCode::KeyW) {
        wish_velocity += Vec3::NEG_Z
    }
    if input.pressed(KeyCode::KeyS) {
        wish_velocity += Vec3::Z
    }
    if input.pressed(KeyCode::KeyA) {
        wish_velocity += Vec3::NEG_X
    }
    if input.pressed(KeyCode::KeyD) {
        wish_velocity += Vec3::X
    }
    if input.pressed(KeyCode::Space) || input.pressed(KeyCode::KeyE) {
        wish_velocity += Vec3::Y
    }
    if input.pressed(KeyCode::ControlLeft) || input.pressed(KeyCode::KeyQ) {
        wish_velocity += Vec3::NEG_Y
    }
    wish_velocity = wish_velocity.normalize_or_zero();
    wish_velocity *= 7.0;
    if input.pressed(KeyCode::ShiftLeft) {
        wish_velocity *= 3.0;
    }
    wish_velocity = camera.rotation * wish_velocity;
    // preserve momentum
    wish_velocity += player.internal_velocity;
    let current_speed = wish_velocity.length();
    if current_speed > 0.0 {
        // apply friction
        wish_velocity = wish_velocity / current_speed
            * (current_speed - current_speed * 20.0 * time.delta_secs()).max(0.0)
    }

    player.touched.clear();
    let CollideAndSlideResult {
        position,
        internal_velocity,
    } = collide_and_slide.collide_and_slide(
        collider,
        transform.rotation,
        transform.translation,
        wish_velocity,
        &CollideAndSlideConfig::default(),
        &SpatialQueryFilter::from_excluded_entities([entity]),
        |hit| {
            if hit.hit.distance == 0.0 {
                gizmos.sphere(Isometry3d::IDENTITY, 0.6, tailwind::RED_600);
            } else {
                gizmos.arrow(
                    hit.hit.point1,
                    hit.hit.point1 + hit.hit.normal1 * hit.hit.distance / time.delta_secs(),
                    tailwind::EMERALD_400,
                );
            }
            player.touched.push(hit.hit.entity);
            true
        },
    );
    transform.translation = position;
    player.internal_velocity = internal_velocity;
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
        player_transform.translation,
        camera.back(),
        MAX_DISTANCE,
        true,
        &SpatialQueryFilter::from_excluded_entities([player_entity]),
    ) {
        camera.translation =
            player_transform.translation + camera.back() * (hit.distance - 1.0).max(0.0);
    }
}

fn capture_cursor(mut cursor: Single<&mut CursorOptions>) {
    cursor.visible = false;
    cursor.grab_mode = CursorGrabMode::Locked;
}

fn exit_game(input: Res<ButtonInput<KeyCode>>, mut app_exit: MessageWriter<AppExit>) {
    if input.just_pressed(KeyCode::Escape) {
        app_exit.write(AppExit::Success);
    }
}

fn update_debug_text(
    mut text: Single<&mut Text, With<DebugText>>,
    player: Single<(&Player, &LinearVelocity, &CollidingEntities), With<Player>>,
    names: Query<NameOrEntity>,
) {
    let (player, velocity, colliding_entities) = player.into_inner();
    ***text = format!(
        "Velocity: [{:.3}, {:.3}, {:.3}]\nMomentum: [{:.3}, {:.3}, {:.3}]\n{} intersections (goal is 0): {:#?}\n{} touched: {:#?}",
        velocity.x,
        velocity.y,
        velocity.z,
        player.internal_velocity.x,
        player.internal_velocity.y,
        player.internal_velocity.z,
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
