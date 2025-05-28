//! An example providing a visual demonstration of the 2D cross-platform determinism test
//! in `src/tests/determinism_2d`.
//!
//! This scene is designed to produce a chaotic result engaging:
//!
//! - the contact solver
//! - speculative collision
//! - joints and joint limits
//!
//! Once the simulation has run for a while, a transform hash is computed.
//! The determinism test compares this to the expected value for every PR on multiple platforms using GitHub Actions.
//! Every time simulation behavior changes, the expected hash must be updated.
//!
//! This test is based on the `FallingHinges` test in the Box2D physics engine:
//! <https://github.com/erincatto/box2d/blob/90c2781f64775085035655661d5fe6542bf0fbd5/samples/sample_determinism.cpp>

use avian2d::{
    math::{AdjustPrecision, Scalar, Vector, PI},
    prelude::*,
};
use bevy::{
    color::palettes::tailwind::CYAN_400, ecs::system::SystemParam,
    input::common_conditions::input_just_pressed, prelude::*, render::camera::ScalingMode,
};
use bytemuck::{Pod, Zeroable};

// How many steps to record the hash for.
const STEP_COUNT: usize = 500;

const ROWS: u32 = 30;
const COLUMNS: u32 = 4;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            PhysicsPlugins::default()
                .with_length_unit(0.5)
                .with_collision_hooks::<PhysicsHooks>(),
            PhysicsDebugPlugin::default(),
        ))
        .init_resource::<Step>()
        .add_systems(Startup, (setup_scene, setup_ui))
        .add_systems(FixedUpdate, update_hash)
        .add_systems(
            PreUpdate,
            // Reset the scene when the R key is pressed.
            (clear_scene, setup_scene)
                .chain()
                .run_if(input_just_pressed(KeyCode::KeyR)),
        )
        .run();
}

#[derive(Resource, Default, Deref, DerefMut)]
struct Step(usize);

fn setup_scene(
    mut commands: Commands,
    mut materials: ResMut<Assets<ColorMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    commands.spawn((
        Camera2d,
        Projection::from(OrthographicProjection {
            scaling_mode: ScalingMode::FixedHorizontal {
                viewport_width: 40.0,
            },
            ..OrthographicProjection::default_2d()
        }),
        Transform::from_xyz(0.0, 7.5, 0.0),
    ));

    let ground_shape = Rectangle::new(40.0, 2.0);
    commands.spawn((
        Name::new("Ground"),
        RigidBody::Static,
        Collider::from(ground_shape),
        Mesh2d(meshes.add(ground_shape)),
        MeshMaterial2d(materials.add(Color::WHITE)),
        Transform::from_xyz(0.0, -1.0, 0.0),
    ));

    let half_size = 0.25;
    let square_shape = Rectangle::new(2.0 * half_size, 2.0 * half_size);
    let square_collider = Collider::from(square_shape);
    let square_mesh = meshes.add(square_shape);
    let square_material = materials.add(Color::from(CYAN_400));

    let offset = 0.4 * half_size;
    let delta_x = 10.0 * half_size;
    let x_root = -0.5 * delta_x * (COLUMNS as f32 - 1.0);

    for col in 0..COLUMNS {
        let x = x_root + col as f32 * delta_x;

        let mut prev_entity = None;

        for row in 0..ROWS {
            let entity = commands
                .spawn((
                    Name::new("Square ({col}, {row})"),
                    RigidBody::Dynamic,
                    square_collider.clone(),
                    Mesh2d(square_mesh.clone()),
                    MeshMaterial2d(square_material.clone()),
                    Transform::from_xyz(
                        x + offset * row as f32,
                        half_size + 2.0 * half_size * row as f32,
                        0.0,
                    )
                    .with_rotation(Quat::from_rotation_z(0.1 * row as f32 - 1.0)),
                ))
                .id();

            if row & 1 == 0 {
                prev_entity = Some(entity);
            } else {
                commands.spawn((
                    Name::new(format!(
                        "Revolute Joint ({}, {})",
                        prev_entity.unwrap(),
                        entity
                    )),
                    RevoluteJoint::new(prev_entity.unwrap(), entity)
                        .with_angle_limits(-0.1 * PI, 0.2 * PI)
                        .with_point_compliance(0.0001)
                        .with_local_anchor_1(Vec2::splat(half_size).adjust_precision())
                        .with_local_anchor_2(Vec2::new(offset, -half_size).adjust_precision()),
                ));
                prev_entity = None;
            }
        }
    }
}

#[derive(Component)]
struct StepText;

#[derive(Component)]
struct HashText;

fn setup_ui(mut commands: Commands) {
    let font = TextFont {
        font_size: 20.0,
        ..default()
    };

    commands
        .spawn((
            Text::new("Step: "),
            font.clone(),
            Node {
                position_type: PositionType::Absolute,
                top: Val::Px(5.0),
                left: Val::Px(5.0),
                ..default()
            },
        ))
        .with_child((TextSpan::new("0"), font.clone(), StepText));

    commands
        .spawn((
            Text::new("Hash: "),
            font.clone(),
            Node {
                position_type: PositionType::Absolute,
                top: Val::Px(30.0),
                left: Val::Px(5.0),
                ..default()
            },
        ))
        .with_child((TextSpan::default(), font.clone(), HashText));

    commands.spawn((
        Text::new("Press R to reset scene"),
        font.clone(),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(5.0),
            right: Val::Px(5.0),
            ..default()
        },
    ));
}

#[derive(SystemParam)]
pub struct PhysicsHooks<'w, 's> {
    joints: Query<'w, 's, &'static RevoluteJoint>,
}

impl CollisionHooks for PhysicsHooks<'_, '_> {
    fn filter_pairs(&self, collider1: Entity, collider2: Entity, _commands: &mut Commands) -> bool {
        // Ignore the collision if the entities are connected by a joint.
        // TODO: This should be an optimized built-in feature for joints.
        self.joints
            .iter()
            .find(|joint| {
                (joint.entity1 == collider1 && joint.entity2 == collider2)
                    || (joint.entity1 == collider2 && joint.entity2 == collider1)
            })
            .is_some()
    }
}

fn clear_scene(
    mut commands: Commands,
    query: Query<
        Entity,
        Or<(
            With<RigidBody>,
            With<Collider>,
            With<RevoluteJoint>,
            With<Camera>,
        )>,
    >,
    mut step: ResMut<Step>,
) {
    step.0 = 0;
    for entity in &query {
        commands.entity(entity).despawn();
    }
}

#[derive(Pod, Zeroable, Clone, Copy)]
#[repr(C)]
struct Isometry {
    translation: Vector,
    rotation: Scalar,
}

fn update_hash(
    transforms: Query<(&Position, &Rotation), With<RigidBody>>,
    mut step_text: Single<&mut TextSpan, With<StepText>>,
    mut hash_text: Single<&mut TextSpan, (With<HashText>, Without<StepText>)>,
    mut step: ResMut<Step>,
) {
    step_text.0 = step.to_string();
    step.0 += 1;

    if step.0 > STEP_COUNT {
        return;
    }

    let mut hash = 5381;
    for (position, rotation) in &transforms {
        let isometry = Isometry {
            translation: position.0,
            rotation: rotation.as_radians(),
        };
        hash = djb2_hash(hash, bytemuck::bytes_of(&isometry));
    }

    if step.0 == STEP_COUNT {
        hash_text.0 = format!("0x{:x} (step {})", hash, step.0);
    } else {
        hash_text.0 = format!("0x{:x}", hash);
    }
}

fn djb2_hash(mut hash: u32, data: &[u8]) -> u32 {
    for &byte in data {
        hash = (hash << 5).wrapping_add(hash + byte as u32);
    }
    hash
}
