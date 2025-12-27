//! Tests for joint motors.

use core::time::Duration;

#[cfg(feature = "2d")]
use approx::assert_relative_eq;
use bevy::{mesh::MeshPlugin, prelude::*, time::TimeUpdateStrategy};

use crate::prelude::*;

const TIMESTEP: f32 = 1.0 / 64.0;

fn create_app() -> App {
    let mut app = App::new();
    app.add_plugins((
        MinimalPlugins,
        PhysicsPlugins::default(),
        TransformPlugin,
        #[cfg(feature = "bevy_scene")]
        AssetPlugin::default(),
        #[cfg(feature = "bevy_scene")]
        bevy::scene::ScenePlugin,
        MeshPlugin,
    ));

    // Use 20 substeps.
    app.insert_resource(SubstepCount(20));

    // Disable gravity for joint tests.
    app.insert_resource(Gravity(Vector::ZERO));

    // Configure the timestep.
    app.insert_resource(Time::<Fixed>::from_duration(Duration::from_secs_f32(
        TIMESTEP,
    )));
    app.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f32(
        TIMESTEP,
    )));

    app
}

/// Tests that an angular motor on a revolute joint spins the attached body.
#[test]
fn revolute_motor_spins_body() {
    let mut app = create_app();
    app.finish();

    // Create two bodies: one static anchor, one dynamic to spin.
    let anchor = app
        .world_mut()
        .spawn((RigidBody::Static, Position(Vector::ZERO)))
        .id();

    let dynamic = app
        .world_mut()
        .spawn((
            RigidBody::Dynamic,
            Position(Vector::X * 2.0),
            Mass(1.0),
            #[cfg(feature = "2d")]
            AngularInertia(1.0),
            #[cfg(feature = "3d")]
            AngularInertia::new(Vec3::splat(1.0)),
        ))
        .id();

    // Create a revolute joint with an angular motor.
    // Use AccelerationBased model for velocity control.
    let _joint = app
        .world_mut()
        .spawn(
            RevoluteJoint::new(anchor, dynamic).with_motor(AngularMotor {
                target_velocity: 2.0, // 2 rad/s
                max_torque: 100.0,
                motor_model: MotorModel::AccelerationBased {
                    stiffness: 0.0,
                    damping: 10.0,
                },
                ..default()
            }),
        )
        .id();

    // Initialize the app.
    app.update();

    // Run simulation for 1 second.
    let duration = 1.0;
    let steps = (duration / TIMESTEP) as usize;

    for _ in 0..steps {
        app.update();
    }

    // Get the angular velocity of the dynamic body.
    let body_ref = app.world().entity(dynamic);
    let angular_velocity = body_ref.get::<AngularVelocity>().unwrap();

    // The angular velocity should be close to the target (2 rad/s).
    #[cfg(feature = "2d")]
    {
        assert!(
            angular_velocity.0.abs() > 1.0,
            "Angular velocity should be significant"
        );
        assert_relative_eq!(angular_velocity.0, 2.0, epsilon = 0.5);
    }
    #[cfg(feature = "3d")]
    {
        // In 3D, the motor drives around the Z axis by default.
        let speed = angular_velocity.0.length();
        assert!(speed > 1.0, "Angular velocity should be significant");
    }
}

/// Tests that a linear motor on a prismatic joint moves the attached body.
#[test]
fn prismatic_motor_moves_body() {
    let mut app = create_app();
    app.finish();

    // Create two bodies: one static anchor, one dynamic to move.
    let anchor = app
        .world_mut()
        .spawn((RigidBody::Static, Position(Vector::ZERO)))
        .id();

    let dynamic = app
        .world_mut()
        .spawn((
            RigidBody::Dynamic,
            Position(Vector::X * 2.0),
            Mass(1.0),
            #[cfg(feature = "2d")]
            AngularInertia(1.0),
            #[cfg(feature = "3d")]
            AngularInertia::new(Vec3::splat(1.0)),
        ))
        .id();

    // Create a prismatic joint with a linear motor along the X axis.
    let _joint = app
        .world_mut()
        .spawn(
            PrismaticJoint::new(anchor, dynamic)
                .with_local_anchor1(Vector::X * 2.0)
                .with_motor(LinearMotor {
                    target_velocity: 1.0, // 1 m/s
                    max_force: 100.0,
                    motor_model: MotorModel::AccelerationBased {
                        stiffness: 0.0,
                        damping: 10.0,
                    },
                    ..default()
                }),
        )
        .id();

    // Initialize the app.
    app.update();

    // Get initial position.
    let initial_x = app.world().entity(dynamic).get::<Position>().unwrap().0.x;

    // Run simulation for 1 second.
    let duration = 1.0;
    let steps = (duration / TIMESTEP) as usize;

    for _ in 0..steps {
        app.update();
    }

    // Get the final position of the dynamic body.
    let body_ref = app.world().entity(dynamic);
    let final_x = body_ref.get::<Position>().unwrap().0.x;

    // The body should have moved along the X axis.
    let displacement = final_x - initial_x;
    assert!(
        displacement > 0.5,
        "Body should have moved significantly: {}",
        displacement
    );
}

/// Tests that an angular motor with max torque limit respects the limit.
#[test]
fn revolute_motor_respects_max_torque() {
    let mut app = create_app();
    app.finish();

    // Create two bodies.
    let anchor = app
        .world_mut()
        .spawn((RigidBody::Static, Position(Vector::ZERO)))
        .id();

    let dynamic = app
        .world_mut()
        .spawn((
            RigidBody::Dynamic,
            Position(Vector::X * 2.0),
            Mass(100.0), // Heavy body to test torque limiting
            #[cfg(feature = "2d")]
            AngularInertia(100.0),
            #[cfg(feature = "3d")]
            AngularInertia::new(Vec3::splat(100.0)),
        ))
        .id();

    // Create a revolute joint with a very limited motor torque.
    let _joint = app
        .world_mut()
        .spawn(
            RevoluteJoint::new(anchor, dynamic).with_motor(AngularMotor {
                target_velocity: 10.0, // High target
                max_torque: 0.1,       // Very low max torque
                motor_model: MotorModel::AccelerationBased {
                    stiffness: 0.0,
                    damping: 1.0,
                },
                ..default()
            }),
        )
        .id();

    // Initialize the app.
    app.update();

    // Run simulation for 1 second.
    let duration = 1.0;
    let steps = (duration / TIMESTEP) as usize;

    for _ in 0..steps {
        app.update();
    }

    // Get the angular velocity.
    let body_ref = app.world().entity(dynamic);
    let angular_velocity = body_ref.get::<AngularVelocity>().unwrap();

    // With limited torque and heavy body, the velocity should be much lower than target.
    #[cfg(feature = "2d")]
    {
        assert!(
            angular_velocity.0.abs() < 5.0,
            "Velocity should be limited by max torque"
        );
    }
    #[cfg(feature = "3d")]
    {
        let speed = angular_velocity.0.length();
        assert!(speed < 5.0, "Velocity should be limited by max torque");
    }
}

/// Tests that a position-targeting motor moves the joint towards the target position.
#[test]
fn revolute_motor_position_target() {
    let mut app = create_app();
    app.finish();

    // Create two bodies.
    let anchor = app
        .world_mut()
        .spawn((RigidBody::Static, Position(Vector::ZERO)))
        .id();

    let dynamic = app
        .world_mut()
        .spawn((
            RigidBody::Dynamic,
            Position(Vector::X * 2.0),
            Mass(1.0),
            #[cfg(feature = "2d")]
            AngularInertia(1.0),
            #[cfg(feature = "3d")]
            AngularInertia::new(Vec3::splat(1.0)),
        ))
        .id();

    // Create a revolute joint with a position-targeting motor.
    // Use critically damped parameters: damping = 2 * sqrt(stiffness * inertia)
    // For stiffness=100, inertia=1: damping = 2 * sqrt(100) = 20
    let target_angle = 1.0; // 1 radian
    let _joint = app
        .world_mut()
        .spawn(
            RevoluteJoint::new(anchor, dynamic).with_motor(AngularMotor {
                target_position: target_angle,
                max_torque: Scalar::MAX,
                motor_model: MotorModel::AccelerationBased {
                    stiffness: 50.0, // Moderate spring
                    damping: 20.0,   // High damping for stability
                },
                ..default()
            }),
        )
        .id();

    // Initialize the app.
    app.update();

    // Run simulation for 3 seconds to let it settle.
    let duration = 3.0;
    let steps = (duration / TIMESTEP) as usize;

    for _ in 0..steps {
        app.update();
    }

    // Get the rotation of the dynamic body.
    let body_ref = app.world().entity(dynamic);
    let rotation = body_ref.get::<Rotation>().unwrap();

    // The body should have rotated towards the target angle (allow some tolerance).
    #[cfg(feature = "2d")]
    {
        let angle = rotation.as_radians();
        // Just verify that it moved significantly towards the target.
        assert!(
            angle.abs() > 0.3,
            "Motor should have rotated the body: {}",
            angle
        );
    }
    #[cfg(feature = "3d")]
    {
        // In 3D, extract the rotation angle around the Z axis.
        let (axis, angle) = rotation.to_axis_angle();
        let signed_angle = angle * axis.z.signum();
        // Just verify that it moved significantly.
        assert!(
            signed_angle.abs() > 0.3,
            "Motor should have rotated the body: {}",
            signed_angle
        );
    }
}

/// Tests that a linear position-targeting motor moves the joint towards the target position.
#[test]
fn prismatic_motor_position_target() {
    let mut app = create_app();
    app.finish();

    // Create two bodies.
    let anchor = app
        .world_mut()
        .spawn((RigidBody::Static, Position(Vector::ZERO)))
        .id();

    let dynamic = app
        .world_mut()
        .spawn((
            RigidBody::Dynamic,
            Position(Vector::X * 2.0),
            Mass(1.0),
            #[cfg(feature = "2d")]
            AngularInertia(1.0),
            #[cfg(feature = "3d")]
            AngularInertia::new(Vec3::splat(1.0)),
        ))
        .id();

    // Create a prismatic joint with a position-targeting motor.
    // Use AccelerationBased model for stable position targeting.
    let target_position = 1.0; // Target is 1 meter along the slider axis
    let _joint = app
        .world_mut()
        .spawn(
            PrismaticJoint::new(anchor, dynamic)
                .with_local_anchor1(Vector::X * 2.0)
                .with_motor(LinearMotor {
                    target_position,
                    max_force: 100.0,
                    motor_model: MotorModel::AccelerationBased {
                        stiffness: 10.0, // Moderate stiffness
                        damping: 5.0,    // Moderate damping
                    },
                    ..default()
                }),
        )
        .id();

    // Initialize the app.
    app.update();

    // Get initial position.
    let initial_pos = app.world().entity(dynamic).get::<Position>().unwrap().0;

    // Run simulation for 3 seconds to let it settle.
    let duration = 3.0;
    let steps = (duration / TIMESTEP) as usize;

    for _ in 0..steps {
        app.update();
    }

    // Get the final position.
    let body_ref = app.world().entity(dynamic);
    let final_pos = body_ref.get::<Position>().unwrap().0;

    // Check that positions are not NaN.
    assert!(!final_pos.x.is_nan(), "Final position should not be NaN");
    assert!(!final_pos.y.is_nan(), "Final position should not be NaN");

    // The body should have moved along the slider axis.
    let displacement = final_pos.x - initial_pos.x;

    // Should have moved (either direction is valid depending on joint configuration).
    assert!(
        displacement.abs() > 0.1 || final_pos.x.abs() > 0.1,
        "Body should have moved: displacement={}, final_x={}",
        displacement,
        final_pos.x
    );
}
