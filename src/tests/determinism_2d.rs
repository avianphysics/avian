//! A 2D cross-platform determinism test.
//!
//! This scene is designed to produce a chaotic result engaging:
//!
//! - the contact solver
//! - speculative collision
//! - joints and joint limits
//!
//! Once the simulation has run for a while, a transform hash is computed.
//! This is compared against the expected hash for every PR on multiple platforms using GitHub Actions.
//! Every time simulation behavior changes, the expected hash must be updated.
//!
//! This test is based on the `FallingHinges` test in the Box2D physics engine
//! <https://github.com/erincatto/box2d/blob/90c2781f64775085035655661d5fe6542bf0fbd5/samples/sample_determinism.cpp>

#![allow(clippy::doc_markdown)]

use core::time::Duration;

use crate::{
    math::{PI, Vector},
    prelude::*,
};
use bevy::{prelude::*, time::TimeUpdateStrategy};
use bytemuck::{Pod, Zeroable};

// How many steps to record the hash for.
const STEP_COUNT: usize = 500;

const ROWS: u32 = 30;
const COLUMNS: u32 = 4;

#[test]
fn cross_platform_determinism_2d() {
    let mut app = App::new();

    app.add_plugins((
        MinimalPlugins,
        TransformPlugin,
        PhysicsPlugins::default().with_length_unit(0.5),
        #[cfg(feature = "bevy_scene")]
        AssetPlugin::default(),
        #[cfg(feature = "bevy_scene")]
        bevy::scene::ScenePlugin,
    ))
    .insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f32(
        1.0 / 64.0,
    )))
    .add_systems(Startup, setup_scene);

    app.finish();

    // Run the simulation `STEP_COUNT` times.
    for _ in 0..STEP_COUNT {
        app.update();
    }

    // Compute the transform hash.
    let query = app.world_mut().query::<(&Position, &Rotation)>();
    let hash = compute_hash(app.world(), query);

    // Update this value if simulation behavior changes.
    let expected = 0xfcddc55;

    assert!(
        hash == expected,
        "\nExpected transform hash 0x{:x}, found 0x{:x} instead.\nIf changes in behavior were expected, update the hash in src/tests/determinism_2d.rs on line 61.\n",
        expected,
        hash,
    );
}

#[derive(Pod, Zeroable, Clone, Copy)]
#[repr(C)]
struct Isometry {
    translation: Vector,
    rotation: Scalar,
}

fn compute_hash(world: &World, mut query: QueryState<(&Position, &Rotation)>) -> u32 {
    let mut hash = 5381;
    for (position, rotation) in query.iter(world) {
        let isometry = Isometry {
            translation: position.0,
            rotation: rotation.as_radians(),
        };
        hash = djb2_hash(hash, bytemuck::bytes_of(&isometry));
    }
    hash
}

fn djb2_hash(mut hash: u32, data: &[u8]) -> u32 {
    for &byte in data {
        hash = (hash << 5).wrapping_add(hash + byte as u32);
    }
    hash
}

fn setup_scene(mut commands: Commands) {
    commands.spawn((
        Name::new("Ground"),
        RigidBody::Static,
        Collider::rectangle(40.0, 2.0),
        Transform::from_xyz(0.0, -1.0, 0.0),
    ));

    let half_size = 0.25;
    let square_collider = Collider::rectangle(2.0 * half_size as Scalar, 2.0 * half_size as Scalar);

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
                        .with_local_anchor1(Vec2::splat(half_size).adjust_precision())
                        .with_local_anchor2(Vec2::new(offset, -half_size).adjust_precision()),
                    JointCollisionDisabled,
                ));
                prev_entity = None;
            }
        }
    }
}
