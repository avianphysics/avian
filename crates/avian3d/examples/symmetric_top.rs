//! Gyroscopic motion is the tendency of a rotating object to maintain its axis of rotation
//! unless acted upon by an external torque. It manifests as objects with non-uniform angular
//! inertia tensors seemingly wobbling as they spin in the air or on the ground, and is
//! responsible for many rotational phenomena.
//!
//! One interesting example of gyroscopic motion is the Dzhanibekov effect, also known as the
//! "tennis racket theorem". When a rigid body with three distinct principal moments of inertia
//! is rotated about one of its principal axes, it can exhibit a sudden flip or change in
//! rotation about the other two axes. This example demonstrates this effect using a
//! tennis racket and a T-handle.
//!
//! Avian handles gyroscopic motion automatically. No special setup is required.

use avian3d::{math::*, prelude::*};
use bevy::prelude::*;
use examples_common_3d::ExampleCommonPlugin;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            ExampleCommonPlugin,
            PhysicsPlugins::default(),
        ))
        .insert_resource(Gravity::ZERO)
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (setup_angular_momentum, log_conserved_quantities).chain(),
        )
        .run();
}

/// A marker component for the "T-handle" used to demonstrate the Dzhanibekov effect.
#[derive(Component)]
struct InitialAngularMomentum(Vec3);

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let material = materials.add(Color::WHITE);

    // Spawn the T-handle.
    let bar = Cuboid::new(1.0, 5.0, 1.0);
    commands.spawn((
        Name::new("Long Bar"),
        RigidBody::Dynamic,
        InitialAngularMomentum(Vector::Y * 10.0),
        Transform::from_xyz(-4.0, 0.0 as f32, 0.0).with_rotation(Quat::from_rotation_z(0.3)),
        Collider::from(bar),
        Mesh3d(meshes.add(bar)),
        MeshMaterial3d(material.clone()),
    ));

    // Spawn the tennis racket.
    let plate = Cuboid::new(5.0, 1.0, 5.0);
    commands.spawn((
        Name::new("Flat Plate"),
        RigidBody::Dynamic,
        InitialAngularMomentum(Vector::Y * 500.0),
        Transform::from_xyz(4.0, 0.0 as f32, 0.0).with_rotation(Quat::from_rotation_z(0.3)),
        Collider::from(plate),
        Mesh3d(meshes.add(plate)),
        MeshMaterial3d(material.clone()),
    ));

    // Directional light
    commands.spawn((
        DirectionalLight {
            illuminance: 3000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::default().looking_at(Vec3::new(-1.0, -2.5, -1.5), Vec3::Y),
    ));

    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_translation(Vec3::new(0.0, 0.0, 10.0)).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

fn setup_angular_momentum(
    // new_inertia: On<Add, ComputedAngularInertia>,
    q: Query<
        (
            &InitialAngularMomentum,
            &ComputedAngularInertia,
            &Rotation,
            &mut AngularVelocity,
        ),
        Changed<ComputedAngularInertia>,
    >,
) {
    for (l, inertia, rot, mut w) in q {
        let j = inertia.inverse();
        let body_l = rot.inverse() * l.0;
        let body_w = j * body_l;
        w.0 = rot * body_w;
    }
}

fn log_conserved_quantities(
    q: Query<(&Name, &ComputedAngularInertia, &Rotation, &AngularVelocity)>,
) {
    for (name, inertia, rot, w) in q {
        let body_w = rot.inverse() * w.0;

        let inertia = inertia.tensor();
        let body_l = inertia * body_w;
        let body_l2 = body_l.length_squared();
        let j_l = body_l.dot(body_w) / body_l2;
        let body_l = body_l2.sqrt();

        info!("{name}: {j_l}; {body_l}");
    }
}
