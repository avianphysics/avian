use avian2d::{
    math::{AdjustPrecision as _, AsF32, Vector},
    prelude::*,
};
use bevy::{
    asset::RenderAssetUsages, color::palettes::tailwind, ecs::entity::EntityHashSet,
    mesh::PrimitiveTopology, prelude::*,
};
use examples_common_2d::ExampleCommonPlugin;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            ExampleCommonPlugin,
            PhysicsPlugins::default(),
        ))
        .add_systems(Startup, setup)
        .add_systems(FixedUpdate, move_player)
        .add_systems(Update, update_debug_text)
        .run();
}

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<ColorMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    // Character
    let shape = Circle::new(30.0);
    commands.spawn((
        Mesh2d(meshes.add(shape)),
        MeshMaterial2d(materials.add(Color::from(tailwind::SKY_400.with_alpha(0.6)))),
        Collider::from(shape),
        RigidBody::Kinematic,
        Player::default(),
        TransformInterpolation,
        // Not needed for collide-and-slide to work, but we add it for debug printing
        CollidingEntities::default(),
    ));

    // A cube to move around
    commands.spawn((
        Sprite {
            color: Color::srgb(0.0, 0.4, 0.7),
            custom_size: Some(Vec2::new(30.0, 30.0)),
            ..default()
        },
        Transform::from_xyz(50.0, -100.0, 0.0),
        RigidBody::Dynamic,
        Collider::rectangle(30.0, 30.0),
    ));

    // Platforms
    commands.spawn((
        Sprite {
            color: Color::srgb(0.7, 0.7, 0.8),
            custom_size: Some(Vec2::new(1100.0, 50.0)),
            ..default()
        },
        Transform::from_xyz(0.0, -175.0, 0.0),
        RigidBody::Static,
        Collider::rectangle(1100.0, 50.0),
    ));
    commands.spawn((
        Sprite {
            color: Color::srgb(0.7, 0.7, 0.8),
            custom_size: Some(Vec2::new(300.0, 25.0)),
            ..default()
        },
        Transform::from_xyz(175.0, -35.0, 0.0),
        RigidBody::Static,
        Collider::rectangle(300.0, 25.0),
    ));
    commands.spawn((
        Sprite {
            color: Color::srgb(0.7, 0.7, 0.8),
            custom_size: Some(Vec2::new(300.0, 25.0)),
            ..default()
        },
        Transform::from_xyz(-175.0, 0.0, 0.0),
        RigidBody::Static,
        Collider::rectangle(300.0, 25.0),
    ));
    commands.spawn((
        Sprite {
            color: Color::srgb(0.7, 0.7, 0.8),
            custom_size: Some(Vec2::new(150.0, 80.0)),
            ..default()
        },
        Transform::from_xyz(475.0, -110.0, 0.0),
        RigidBody::Static,
        Collider::rectangle(150.0, 80.0),
    ));
    commands.spawn((
        Sprite {
            color: Color::srgb(0.7, 0.7, 0.8),
            custom_size: Some(Vec2::new(150.0, 80.0)),
            ..default()
        },
        Transform::from_xyz(-475.0, -110.0, 0.0),
        RigidBody::Static,
        Collider::rectangle(150.0, 80.0),
    ));

    // Ramps

    let mut ramp_mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );

    ramp_mesh.insert_attribute(
        Mesh::ATTRIBUTE_POSITION,
        vec![[-125.0, 80.0, 0.0], [-125.0, 0.0, 0.0], [125.0, 0.0, 0.0]],
    );

    let ramp_collider = Collider::triangle(
        Vector::new(-125.0, 80.0),
        Vector::NEG_X * 125.0,
        Vector::X * 125.0,
    );

    commands.spawn((
        Mesh2d(meshes.add(ramp_mesh)),
        MeshMaterial2d(materials.add(Color::srgb(0.4, 0.4, 0.5))),
        Transform::from_xyz(-275.0, -150.0, 0.0),
        RigidBody::Static,
        ramp_collider,
    ));

    let mut ramp_mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );

    ramp_mesh.insert_attribute(
        Mesh::ATTRIBUTE_POSITION,
        vec![[20.0, -40.0, 0.0], [20.0, 40.0, 0.0], [-20.0, -40.0, 0.0]],
    );

    let ramp_collider = Collider::triangle(
        Vector::new(20.0, -40.0),
        Vector::new(20.0, 40.0),
        Vector::new(-20.0, -40.0),
    );

    commands.spawn((
        Mesh2d(meshes.add(ramp_mesh)),
        MeshMaterial2d(materials.add(Color::srgb(0.4, 0.4, 0.5))),
        Transform::from_xyz(380.0, -110.0, 0.0),
        RigidBody::Static,
        ramp_collider,
    ));

    // Camera
    commands.spawn(Camera2d);

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
    internal_velocity: Vector,
    touched: EntityHashSet,
}

fn move_player(
    player: Single<(Entity, &mut Transform, &mut Player, &Collider), Without<Camera>>,
    move_and_slide: MoveAndSlide,
    time: Res<Time>,
    input: Res<ButtonInput<KeyCode>>,
    mut gizmos: Gizmos,
) {
    let (entity, mut transform, mut player, collider) = player.into_inner();
    let mut wish_velocity = Vector::ZERO;
    if input.pressed(KeyCode::KeyW) {
        wish_velocity += Vector::Y
    }
    if input.pressed(KeyCode::KeyS) {
        wish_velocity += Vector::NEG_Y
    }
    if input.pressed(KeyCode::KeyA) {
        wish_velocity += Vector::NEG_X
    }
    if input.pressed(KeyCode::KeyD) {
        wish_velocity += Vector::X
    }
    wish_velocity = wish_velocity.normalize_or_zero();
    wish_velocity *= 100.0;
    if input.pressed(KeyCode::ShiftLeft) {
        wish_velocity *= 2.0;
    }
    // preserve momentum
    wish_velocity += player.internal_velocity;
    let current_speed = wish_velocity.length();
    if current_speed > 0.0 {
        // apply friction
        wish_velocity = wish_velocity / current_speed
            * (current_speed - current_speed * 20.0 * time.delta_secs().adjust_precision()).max(0.0)
    }

    player.touched.clear();
    let MoveAndSlideOutput {
        position,
        clipped_velocity: internal_velocity,
    } = move_and_slide.move_and_slide(
        collider,
        transform.translation.xy().adjust_precision(),
        transform
            .rotation
            .to_euler(EulerRot::XYZ)
            .2
            .adjust_precision(),
        wish_velocity,
        &MoveAndSlideConfig::default(),
        &SpatialQueryFilter::from_excluded_entities([entity]),
        |hit| {
            if hit.intersects {
                gizmos.circle_2d(
                    Isometry2d::from_translation(transform.translation.xy()),
                    33.0,
                    tailwind::RED_600,
                );
            } else {
                gizmos.arrow_2d(
                    hit.point1.f32(),
                    (hit.point1
                        + hit.normal1 * hit.collision_distance
                            / time.delta_secs().adjust_precision())
                    .f32(),
                    tailwind::EMERALD_400,
                );
            }
            player.touched.insert(hit.entity);
            true
        },
    );
    transform.translation = position.extend(0.0).f32();
    player.internal_velocity = internal_velocity;
}

fn update_debug_text(
    mut text: Single<&mut Text, With<DebugText>>,
    player: Single<(&Player, &CollidingEntities), With<Player>>,
    names: Query<NameOrEntity>,
) {
    let (player, colliding_entities) = player.into_inner();
    ***text = format!(
        "velocity: [{:.3}, {:.3}]\n{} intersections (goal is 0): {:#?}\n{} touched: {:#?}",
        player.internal_velocity.x,
        player.internal_velocity.y,
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
