//! Demonstrates using move and slide to control a kinematic character.
//!
//! This is intended more as a testbed and less as a polished usage example.

use avian2d::{math::*, prelude::*};
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
            PhysicsPlugins::default()
                // Specify a units-per-meter scaling factor, 1 meter = 50 pixels.
                // The unit allows the engine to tune its parameters for the scale of the world, improving stability.
                .with_length_unit(50.0),
        ))
        .add_systems(Startup, setup)
        .add_systems(FixedUpdate, (player_movement, run_move_and_slide).chain())
        .add_systems(Update, update_debug_text)
        .run();
}

#[derive(Component)]
struct Player;

/// The entities touched during the last `move_and_slide` call. Stored for debug printing.
#[derive(Component, Default, Deref, DerefMut)]
struct TouchedEntities(EntityHashSet);

#[derive(Component)]
struct DebugText;

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<ColorMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    // Character
    let shape = Circle::new(30.0);
    commands.spawn((
        Player,
        Mesh2d(meshes.add(shape)),
        MeshMaterial2d(materials.add(Color::from(tailwind::SKY_400.with_alpha(0.6)))),
        Collider::from(shape),
        RigidBody::Kinematic,
        TransformInterpolation,
        // We want to control position updates manually using move and slide.
        CustomPositionIntegration,
        // Store touched and colliding entities for debug printing.
        TouchedEntities::default(),
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

/// System to handle player movement and friction.
///
/// This only updates velocity. The actual movement is handled by the `run_move_and_slide` system.
fn player_movement(
    mut query: Query<&mut LinearVelocity, With<Player>>,
    time: Res<Time>,
    input: Res<ButtonInput<KeyCode>>,
) {
    for mut lin_vel in &mut query {
        // Determine movement velocity from input
        let mut movement_velocity = Vec2::ZERO;
        if input.pressed(KeyCode::KeyW) {
            movement_velocity += Vec2::Y
        }
        if input.pressed(KeyCode::KeyS) {
            movement_velocity += Vec2::NEG_Y
        }
        if input.pressed(KeyCode::KeyA) {
            movement_velocity += Vec2::NEG_X
        }
        if input.pressed(KeyCode::KeyD) {
            movement_velocity += Vec2::X
        }
        movement_velocity = movement_velocity.normalize_or_zero();
        movement_velocity *= 100.0;
        if input.pressed(KeyCode::ShiftLeft) {
            movement_velocity *= 2.0;
        }

        // Add to current velocity
        lin_vel.0 += movement_velocity.adjust_precision();

        let current_speed = lin_vel.length();
        if current_speed > 0.0 {
            // Apply friction
            lin_vel.0 = lin_vel.0 / current_speed
                * (current_speed - current_speed * 20.0 * time.delta_secs().adjust_precision())
                    .max(0.0)
        }
    }
}

/// System to run the move and slide algorithm, updating the player's transform and velocity.
///
/// This replaces Avian's default "position integration" that moves kinematic bodies based on their
/// velocity without any collision handling.
fn run_move_and_slide(
    mut query: Query<
        (
            Entity,
            &mut Transform,
            &mut LinearVelocity,
            &mut TouchedEntities,
            &Collider,
        ),
        With<Player>,
    >,
    move_and_slide: MoveAndSlide,
    time: Res<Time>,
    mut gizmos: Gizmos,
) {
    for (entity, mut transform, mut lin_vel, mut touched, collider) in &mut query {
        touched.clear();

        // Perform move and slide
        let MoveAndSlideOutput {
            position,
            projected_velocity,
        } = move_and_slide.move_and_slide(
            collider,
            transform.translation.xy().adjust_precision(),
            transform
                .rotation
                .to_euler(EulerRot::XYZ)
                .2
                .adjust_precision(),
            lin_vel.0,
            time.delta(),
            &MoveAndSlideConfig::default(),
            &SpatialQueryFilter::from_excluded_entities([entity]),
            |hit| {
                // For each collision, draw debug gizmos
                if hit.intersects() {
                    gizmos.circle_2d(transform.translation.xy(), 33.0, tailwind::RED_600);
                } else {
                    gizmos.arrow_2d(
                        hit.point.f32(),
                        (hit.point
                            + hit.normal.adjust_precision() * hit.collision_distance
                                / time.delta_secs().adjust_precision())
                        .f32(),
                        tailwind::EMERALD_400,
                    );
                }
                touched.insert(hit.entity);
                MoveAndSlideHitResponse::Accept
            },
        );

        // Update transform and velocity
        transform.translation = position.extend(0.0).f32();
        lin_vel.0 = projected_velocity;
    }
}

fn update_debug_text(
    mut text: Single<&mut Text, With<DebugText>>,
    player: Single<(&LinearVelocity, &TouchedEntities, &CollidingEntities), With<Player>>,
    names: Query<NameOrEntity>,
) {
    let (lin_vel, touched, colliding_entities) = player.into_inner();
    ***text = format!(
        "velocity: [{:.3}, {:.3}]\n{} intersections (goal is 0): {:#?}\n{} touched: {:#?}",
        lin_vel.x,
        lin_vel.y,
        colliding_entities.len(),
        names
            .iter_many(colliding_entities.iter())
            .map(|name| name
                .name
                .map(|n| format!("{} ({})", name.entity, n))
                .unwrap_or_else(|| format!("{}", name.entity)))
            .collect::<Vec<_>>(),
        touched.len(),
        names
            .iter_many(touched.iter())
            .map(|name| name
                .name
                .map(|n| format!("{} ({})", name.entity, n))
                .unwrap_or_else(|| format!("{}", name.entity)))
            .collect::<Vec<_>>()
    );
}
