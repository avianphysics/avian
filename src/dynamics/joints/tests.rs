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

    app.insert_resource(SubstepCount(20));

    app.insert_resource(Gravity(Vector::ZERO));

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

    app.world_mut().spawn(
        RevoluteJoint::new(anchor, dynamic).with_motor(AngularMotor {
            target_velocity: 2.0,
            max_torque: 100.0,
            motor_model: MotorModel::AccelerationBased {
                stiffness: 0.0,
                damping: 10.0,
            },
            ..default()
        }),
    );

    // Initialize the app.
    app.update();

    // Run simulation for 1 second.
    let duration = 1.0;
    let steps = (duration / TIMESTEP) as usize;

    for _ in 0..steps {
        app.update();
    }

    let body_ref = app.world().entity(dynamic);
    let angular_velocity = body_ref.get::<AngularVelocity>().unwrap();

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
        let speed = angular_velocity.0.length();
        assert!(speed > 1.0, "Angular velocity should be significant");
    }
}

/// Tests that a linear motor on a prismatic joint moves the attached body.
#[test]
fn prismatic_motor_moves_body() {
    let mut app = create_app();
    app.finish();

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

    app.world_mut().spawn(
        PrismaticJoint::new(anchor, dynamic)
            .with_local_anchor1(Vector::X * 2.0)
            .with_motor(LinearMotor {
                target_velocity: 1.0,
                max_force: 100.0,
                motor_model: MotorModel::AccelerationBased {
                    stiffness: 0.0,
                    damping: 10.0,
                },
                ..default()
            }),
    );

    // Initialize the app.
    app.update();

    let initial_x = app.world().entity(dynamic).get::<Position>().unwrap().0.x;

    // Run simulation for 1 second.
    let duration = 1.0;
    let steps = (duration / TIMESTEP) as usize;

    for _ in 0..steps {
        app.update();
    }

    let body_ref = app.world().entity(dynamic);
    let final_x = body_ref.get::<Position>().unwrap().0.x;

    let displacement = final_x - initial_x;
    assert!(
        displacement > 0.5,
        "Body should have moved: {}",
        displacement
    );
}

/// Tests that an angular motor with max torque limit respects the limit.
#[test]
fn revolute_motor_respects_max_torque() {
    let mut app = create_app();
    app.finish();

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
    app.world_mut().spawn(
        RevoluteJoint::new(anchor, dynamic).with_motor(AngularMotor {
            target_velocity: 10.0,
            max_torque: 0.1,
            motor_model: MotorModel::AccelerationBased {
                stiffness: 0.0,
                damping: 1.0,
            },
            ..default()
        }),
    );

    // Initialize the app.
    app.update();

    // Run simulation for 1 second.
    let duration = 1.0;
    let steps = (duration / TIMESTEP) as usize;

    for _ in 0..steps {
        app.update();
    }

    let body_ref = app.world().entity(dynamic);
    let angular_velocity = body_ref.get::<AngularVelocity>().unwrap();

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
    let target_angle = 1.0;
    app.world_mut().spawn(
        RevoluteJoint::new(anchor, dynamic).with_motor(AngularMotor {
            target_position: target_angle,
            max_torque: Scalar::MAX,
            motor_model: MotorModel::AccelerationBased {
                stiffness: 50.0,
                damping: 20.0,
            },
            ..default()
        }),
    );

    // Initialize the app.
    app.update();

    // Run simulation for 3 seconds to let it settle.
    let duration = 3.0;
    let steps = (duration / TIMESTEP) as usize;

    for _ in 0..steps {
        app.update();
    }

    let body_ref = app.world().entity(dynamic);
    let rotation = body_ref.get::<Rotation>().unwrap();

    // The body should have rotated towards the target angle (allow some tolerance).
    #[cfg(feature = "2d")]
    {
        let angle = rotation.as_radians();
        assert!(
            angle.abs() > 0.3,
            "Motor should have rotated the body: {}",
            angle
        );
    }
    #[cfg(feature = "3d")]
    {
        let (axis, angle) = rotation.to_axis_angle();
        let signed_angle = angle * axis.z.signum();
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
    app.world_mut().spawn(
        PrismaticJoint::new(anchor, dynamic)
            .with_local_anchor1(Vector::X * 2.0)
            .with_motor(LinearMotor {
                target_position,
                max_force: 100.0,
                motor_model: MotorModel::AccelerationBased {
                    stiffness: 10.0,
                    damping: 5.0,
                },
                ..default()
            }),
    );

    // Initialize the app.
    app.update();

    let initial_pos = app.world().entity(dynamic).get::<Position>().unwrap().0;

    // Run simulation for 3 seconds to let it settle.
    let duration = 3.0;
    let steps = (duration / TIMESTEP) as usize;

    for _ in 0..steps {
        app.update();
    }

    let body_ref = app.world().entity(dynamic);
    let final_pos = body_ref.get::<Position>().unwrap().0;

    assert!(!final_pos.x.is_nan(), "Final position should not be NaN");
    assert!(!final_pos.y.is_nan(), "Final position should not be NaN");

    let displacement = final_pos.x - initial_pos.x;

    assert!(
        displacement.abs() > 0.1 || final_pos.x.abs() > 0.1,
        "Body should have moved: displacement={}, final_x={}",
        displacement,
        final_pos.x
    );
}

/// Tests that a velocity motor on a revolute joint respects angle limits.
///
/// The motor drives with constant velocity, but the joint should stop
/// when it reaches the angle limit.
#[test]
fn revolute_motor_respects_angle_limits() {
    use crate::math::PI;

    let mut app = create_app();
    app.finish();

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

    let angle_limit = PI / 4.0;

    app.world_mut().spawn(
        RevoluteJoint::new(anchor, dynamic)
            .with_angle_limits(-angle_limit, angle_limit)
            .with_motor(AngularMotor {
                target_velocity: 5.0,
                max_torque: 100.0,
                motor_model: MotorModel::AccelerationBased {
                    stiffness: 0.0,
                    damping: 10.0,
                },
                ..default()
            }),
    );

    app.update();

    // Run for 2 seconds - enough time for motor to hit the limit.
    let duration = 2.0;
    let steps = (duration / TIMESTEP) as usize;

    for _ in 0..steps {
        app.update();
    }

    let body_ref = app.world().entity(dynamic);
    let rotation = body_ref.get::<Rotation>().unwrap();

    #[cfg(feature = "2d")]
    {
        let angle = rotation.as_radians();
        assert!(
            angle <= angle_limit + 0.1,
            "Angle {} should not exceed limit {}",
            angle,
            angle_limit
        );
        assert!(
            angle > angle_limit - 0.3,
            "Angle {} should be near the limit {}",
            angle,
            angle_limit
        );
    }
    #[cfg(feature = "3d")]
    {
        let (axis, angle) = rotation.to_axis_angle();
        let signed_angle = angle * axis.z.signum();
        assert!(
            signed_angle <= angle_limit + 0.1,
            "Angle {} should not exceed limit {}",
            signed_angle,
            angle_limit
        );
        assert!(
            signed_angle > angle_limit - 0.3,
            "Angle {} should be near the limit {}",
            signed_angle,
            angle_limit
        );
    }
}

/// Tests that a velocity motor on a prismatic joint respects distance limits.
///
/// The motor drives with constant velocity, but the joint should stop
/// when it reaches the distance limit.
#[test]
fn prismatic_motor_respects_limits() {
    let mut app = create_app();
    app.finish();

    let anchor = app
        .world_mut()
        .spawn((RigidBody::Static, Position(Vector::ZERO)))
        .id();

    // Start at origin so we can measure displacement clearly.
    let dynamic = app
        .world_mut()
        .spawn((
            RigidBody::Dynamic,
            Position(Vector::ZERO),
            Mass(1.0),
            #[cfg(feature = "2d")]
            AngularInertia(1.0),
            #[cfg(feature = "3d")]
            AngularInertia::new(Vec3::splat(1.0)),
        ))
        .id();

    // Limit translation to [0, 1] meters along the slide axis.
    let distance_limit = 1.0;

    app.world_mut().spawn(
        PrismaticJoint::new(anchor, dynamic)
            .with_limits(0.0, distance_limit)
            .with_motor(LinearMotor {
                target_velocity: 5.0, // High velocity to ensure we hit the limit
                max_force: 100.0,
                motor_model: MotorModel::AccelerationBased {
                    stiffness: 0.0,
                    damping: 10.0,
                },
                ..default()
            }),
    );

    app.update();

    // Make sure the motor is not near the limit from the start.
    {
        let body_ref = app.world().entity(dynamic);
        let position = body_ref.get::<Position>().unwrap();
        assert!(
            (position.0.x - distance_limit).abs() > 0.1,
            "Displacement {} should not be near the limit {} at the start of the test",
            position.0.x,
            distance_limit
        );
    }

    // Run for 2 seconds - enough time for motor to hit the limit.
    let duration = 2.0;
    let steps = (duration / TIMESTEP) as usize;

    for _ in 0..steps {
        app.update();
    }

    let body_ref = app.world().entity(dynamic);
    let position = body_ref.get::<Position>().unwrap();

    // The displacement along the slide axis (X) should be at or near the limit.
    let displacement = position.0.x;
    assert!(
        displacement <= distance_limit + 0.001,
        "Displacement {} should not exceed limit {}",
        displacement,
        distance_limit
    );
    assert!(
        (displacement - distance_limit).abs() < 0.1,
        "Displacement {} should be near the limit {}",
        displacement,
        distance_limit
    );
}

/// Tests that `ForceBased` motor model works for revolute joints.
///
/// This is the physically accurate motor model that takes mass into account.
#[test]
fn revolute_motor_force_based() {
    let mut app = create_app();
    app.finish();

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

    // Use ForceBased motor model with velocity control.
    app.world_mut().spawn(
        RevoluteJoint::new(anchor, dynamic).with_motor(AngularMotor {
            target_velocity: 2.0,
            max_torque: 100.0,
            motor_model: MotorModel::ForceBased {
                stiffness: 0.0,
                damping: 10.0,
            },
            ..default()
        }),
    );

    app.update();

    let body_ref = app.world().entity(dynamic);
    let angular_velocity = body_ref.get::<AngularVelocity>().unwrap();
    #[cfg(feature = "2d")]
    let initial_speed = angular_velocity.0.abs();
    #[cfg(feature = "3d")]
    let initial_speed = angular_velocity.0.length();

    assert!(
        initial_speed.abs() < 0.001,
        "ForceBased motor should be initiall still, speed: {}",
        initial_speed
    );

    // Run simulation for 1 second.
    let duration = 1.0;
    let steps = (duration / TIMESTEP) as usize;

    for _ in 0..steps {
        app.update();
    }

    let body_ref = app.world().entity(dynamic);
    let angular_velocity = body_ref.get::<AngularVelocity>().unwrap();

    // The body should have gained angular velocity.
    #[cfg(feature = "2d")]
    {
        assert!(
            angular_velocity.0.abs() > 0.5,
            "ForceBased motor should spin the body: {}",
            angular_velocity.0
        );
    }
    #[cfg(feature = "3d")]
    {
        let speed = angular_velocity.0.length();
        assert!(
            speed > 0.5,
            "ForceBased motor should spin the body: {}",
            speed
        );
    }
}

/// Tests that the default `SpringDamper` motor model works.
///
/// `SpringDamper` is unconditionally stable and uses `frequency`/`damping_ratio`.
#[test]
fn revolute_motor_spring_damper() {
    let mut app = create_app();
    app.finish();

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

    let target_angle = 1.0;
    app.world_mut().spawn(
        RevoluteJoint::new(anchor, dynamic).with_motor(AngularMotor {
            target_position: target_angle,
            max_torque: Scalar::MAX,
            motor_model: MotorModel::SpringDamper {
                frequency: 2.0,
                damping_ratio: 1.0,
            },
            ..default()
        }),
    );

    app.update();

    // TODO Assert the body is initially not near the target.

    // Run simulation for 3 seconds to let the spring settle.
    let duration = 3.0;
    let steps = (duration / TIMESTEP) as usize;

    for _ in 0..steps {
        app.update();
    }

    let body_ref = app.world().entity(dynamic);
    let rotation = body_ref.get::<Rotation>().unwrap();

    // The body should have rotated towards the target.
    #[cfg(feature = "2d")]
    {
        let angle = rotation.as_radians();
        assert!(
            angle.abs() > 0.3,
            "SpringDamper motor should rotate towards target: {}",
            angle
        );
    }
    #[cfg(feature = "3d")]
    {
        let (axis, angle) = rotation.to_axis_angle();
        let signed_angle = angle * axis.z.signum();
        assert!(
            signed_angle.abs() > 0.3,
            "SpringDamper motor should rotate towards target: {}",
            signed_angle
        );
    }
}

/// Tests that a motor with both velocity and position targeting works.
///
/// Combined spring-damper behavior: position targeting with velocity damping.
#[test]
fn revolute_motor_combined_position_velocity() {
    let mut app = create_app();
    app.finish();

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

    // Motor with both position and velocity targeting.
    // The velocity adds a constant offset to the spring behavior.
    let target_angle = 0.5;
    app.world_mut().spawn(
        RevoluteJoint::new(anchor, dynamic).with_motor(AngularMotor {
            enabled: true,
            target_position: target_angle,
            target_velocity: 0.5,
            max_torque: 100.0,
            motor_model: MotorModel::AccelerationBased {
                stiffness: 30.0,
                damping: 15.0,
            },
        }),
    );

    app.update();

    // Run for 2 seconds.
    let duration = 2.0;
    let steps = (duration / TIMESTEP) as usize;

    for _ in 0..steps {
        app.update();
    }

    let body_ref = app.world().entity(dynamic);
    let rotation = body_ref.get::<Rotation>().unwrap();

    // The body should have rotated in the positive direction.
    #[cfg(feature = "2d")]
    {
        let angle = rotation.as_radians();
        assert!(
            angle > 0.2,
            "Combined motor should rotate body positively: {}",
            angle
        );
    }
    #[cfg(feature = "3d")]
    {
        let (axis, angle) = rotation.to_axis_angle();
        let signed_angle = angle * axis.z.signum();
        assert!(
            signed_angle > 0.2,
            "Combined motor should rotate body positively: {}",
            signed_angle
        );
    }
}

/// Tests that a prismatic motor with both velocity and position targeting works.
#[test]
fn prismatic_motor_combined_position_velocity() {
    let mut app = create_app();
    app.finish();

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

    // Motor with both position and velocity targeting.
    app.world_mut().spawn(
        PrismaticJoint::new(anchor, dynamic)
            .with_local_anchor1(Vector::X * 2.0)
            .with_motor(LinearMotor {
                enabled: true,
                target_position: 1.0,
                target_velocity: 0.5,
                max_force: 100.0,
                motor_model: MotorModel::AccelerationBased {
                    stiffness: 20.0,
                    damping: 10.0,
                },
            }),
    );

    app.update();

    let initial_x = app.world().entity(dynamic).get::<Position>().unwrap().0.x;

    // Run for 2 seconds.
    let duration = 2.0;
    let steps = (duration / TIMESTEP) as usize;

    for _ in 0..steps {
        app.update();
    }

    let body_ref = app.world().entity(dynamic);
    let final_x = body_ref.get::<Position>().unwrap().0.x;

    // The body should have moved.
    let displacement = final_x - initial_x;
    assert!(
        displacement.abs() > 0.1,
        "Combined motor should move the body: {}",
        displacement
    );
}
