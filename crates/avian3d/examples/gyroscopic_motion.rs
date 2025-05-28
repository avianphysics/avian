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
use core::f32::consts::FRAC_PI_2;
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
        .add_systems(FixedUpdate, log_racket_angular_momentum)
        .run();
}

/// A marker component for the "T-handle" used to demonstrate the Dzhanibekov effect.
#[derive(Component)]
struct THandle;

/// A marker component for the "tennis racket", also used to demonstrate the Dzhanibekov effect.
#[derive(Component)]
struct Racket;

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let material = materials.add(Color::WHITE);

    // Spawn the T-handle.
    let large_cylinder = Cylinder::new(0.4, 5.0);
    let small_cylinder = Cylinder::new(0.25, 1.25);
    commands.spawn((
        Name::new("T-handle"),
        THandle,
        RigidBody::Dynamic,
        AngularVelocity(Vector::Z * 10.0),
        Transform::from_xyz(-4.0, 0.0, 0.0).with_rotation(Quat::from_rotation_y(FRAC_PI_2)),
        Collider::from(large_cylinder),
        Mesh3d(meshes.add(large_cylinder)),
        MeshMaterial3d(material.clone()),
        children![(
            Transform::from_xyz(1.0, 0.0, 0.0).with_rotation(Quat::from_rotation_z(FRAC_PI_2)),
            Collider::from(small_cylinder),
            Mesh3d(meshes.add(small_cylinder)),
            MeshMaterial3d(material.clone()),
        )],
    ));

    // Spawn the tennis racket.
    let racket_handle_circular = Cylinder::new(0.25, 1.5);
    let racket_face_circular = Cylinder::new(1.5, 0.2);
    commands.spawn((
        Name::new("Tennis Racket"),
        Racket,
        RigidBody::Dynamic,
        Transform::from_xyz(4.0, 0.0, 0.0).with_rotation(Quat::from_rotation_y(0.001)),
        AngularVelocity(Vector::X * 10.0),
        Visibility::default(),
        children![
            (
                Name::new("Racket Handle"),
                Transform::from_xyz(0.0, 0.0, 1.5).with_rotation(Quat::from_rotation_x(FRAC_PI_2)),
                Collider::from(racket_handle_circular),
                Mesh3d(meshes.add(racket_handle_circular)),
                MeshMaterial3d(material.clone()),
            ),
            (
                Name::new("Racket Face"),
                Transform::from_xyz(0.0, 0.0, -1.5).with_scale(Vec3::new(1.0, 1.0, 1.5)),
                Collider::from(racket_face_circular),
                Mesh3d(meshes.add(racket_face_circular)),
                MeshMaterial3d(material.clone()),
            )
        ],
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
        Transform::from_translation(Vec3::new(10.0, 5.0, 10.0)).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

/// Logs the angular momentum of the racket to see how it changes over time.
fn log_racket_angular_momentum(
    query: Query<(&AngularVelocity, &GlobalTransform, &ComputedAngularInertia), With<Racket>>,
) {
    for (angular_velocity, global_transform, inertia) in &query {
        // Compute the up-to-date inertia tensor in world space.
        let world_inertia = inertia.rotated(global_transform.rotation().adjust_precision());

        // Compute the angular momentum.
        let angular_momentum = world_inertia.tensor() * angular_velocity.0;

        // Gyroscopic torque should conserve angular momentum and kinetic energy,
        // assuming no external torques are applied.
        //
        // However, due to the semi-implicit Euler integration scheme and the discrete nature
        // of the simulation, energy may still gradually decrease over time. This is expected
        // and typically acceptable for game physics. The more important problem to avoid is
        // more energy being introduced into the system, as that can lead to instability.
        info!("Racket angular momentum: {:.1}", angular_momentum.length());
    }
}
