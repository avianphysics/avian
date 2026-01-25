//! Demonstrates motor joints in 2D.
//!
//! - Left side: Revolute joint with velocity-controlled angular motor (spinning wheel)
//! - Center: Revolute joint with position-controlled angular motor (servo)
//! - Right side: Prismatic joint with linear motor (piston)
//!
//! Controls:
//! - Arrow Up/Down: Adjust left motor target velocity
//! - A/D: Adjust center motor target angle
//! - W/S: Adjust right motor target position
//! - Space: Toggle motors on/off

use avian2d::{math::*, prelude::*};
use bevy::prelude::*;
use examples_common_2d::ExampleCommonPlugin;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            ExampleCommonPlugin,
            PhysicsPlugins::default(),
        ))
        .insert_resource(ClearColor(Color::srgb(0.05, 0.05, 0.1)))
        .insert_resource(Gravity(Vector::NEG_Y * 1000.0))
        .add_systems(Startup, setup)
        .add_systems(Update, (control_motors, update_ui))
        .run();
}

#[derive(Component)]
struct VelocityMotorJoint;

#[derive(Component)]
struct PositionMotorJoint;

#[derive(Component)]
struct PrismaticMotorJoint;

#[derive(Component)]
struct UiText;

fn setup(mut commands: Commands) {
    commands.spawn(Camera2d);

    // === Velocity-Controlled Revolute Joint (left side) ===
    // Static anchor for the wheel
    let velocity_anchor = commands
        .spawn((
            Sprite {
                color: Color::srgb(0.5, 0.5, 0.5),
                custom_size: Some(Vec2::splat(20.0)),
                ..default()
            },
            Transform::from_xyz(-200.0, 0.0, 0.0),
            RigidBody::Static,
        ))
        .id();

    // Spinning wheel
    let velocity_wheel = commands
        .spawn((
            Sprite {
                color: Color::srgb(0.9, 0.3, 0.3),
                custom_size: Some(Vec2::splat(80.0)),
                ..default()
            },
            Transform::from_xyz(-200.0, 0.0, 0.0),
            RigidBody::Dynamic,
            Mass(1.0),
            AngularInertia(1.0),
            SleepingDisabled, // Prevent sleeping so motor can always control it
        ))
        .id();

    // Revolute joint with velocity-controlled motor
    // Default anchors are at body centers (Vector::ZERO)
    commands.spawn((
        RevoluteJoint::new(velocity_anchor, velocity_wheel).with_motor(AngularMotor {
            target_velocity: 5.0,
            max_torque: 1000.0,
            motor_model: MotorModel::AccelerationBased {
                stiffness: 0.0,
                damping: 1.0,
            },
            ..default()
        }),
        VelocityMotorJoint,
    ));

    // === Position-Controlled Revolute Joint (center) ===
    // Static anchor for the servo
    let position_anchor = commands
        .spawn((
            Sprite {
                color: Color::srgb(0.5, 0.5, 0.5),
                custom_size: Some(Vec2::splat(20.0)),
                ..default()
            },
            Transform::from_xyz(0.0, 0.0, 0.0),
            RigidBody::Static,
        ))
        .id();

    // Servo arm - also positioned at anchor, rotates around its center
    let servo_arm = commands
        .spawn((
            Sprite {
                color: Color::srgb(0.3, 0.5, 0.9),
                custom_size: Some(Vec2::new(100.0, 20.0)),
                ..default()
            },
            Transform::from_xyz(0.0, 0.0, 0.0),
            RigidBody::Dynamic,
            Mass(1.0),
            AngularInertia(1.0),
            SleepingDisabled, // Prevent sleeping so motor can always control it
        ))
        .id();

    // Revolute joint with position-controlled motor (servo behavior)
    //
    // Using spring parameters (frequency, damping_ratio) for stable behavior.
    // This provides predictable spring-damper dynamics across different configurations.
    // - frequency: 5 Hz = fairly stiff spring
    // - damping_ratio: 1.0 = critically damped (fastest approach without overshoot)
    commands.spawn((
        RevoluteJoint::new(position_anchor, servo_arm).with_motor(
            AngularMotor::new(MotorModel::SpringDamper {
                frequency: 5.0,
                damping_ratio: 1.0,
            })
            .with_target_position(0.0)
            .with_max_torque(Scalar::MAX),
        ),
        PositionMotorJoint,
    ));

    // === Prismatic Joint with Linear Motor (right side) ===
    let piston_base_sprite = Sprite {
        color: Color::srgb(0.5, 0.5, 0.5),
        custom_size: Some(Vec2::new(40.0, 200.0)),
        ..default()
    };

    let piston_sprite = Sprite {
        color: Color::srgb(0.3, 0.9, 0.3),
        custom_size: Some(Vec2::new(60.0, 40.0)),
        ..default()
    };

    // Static base for the piston
    let piston_base = commands
        .spawn((
            piston_base_sprite,
            Transform::from_xyz(200.0, 0.0, 0.0),
            RigidBody::Static,
            Position(Vector::new(200.0, 0.0)),
        ))
        .id();

    // Moving piston
    let piston = commands
        .spawn((
            piston_sprite,
            Transform::from_xyz(200.0, 0.0, 0.0),
            RigidBody::Dynamic,
            Mass(1.0),
            AngularInertia(1.0),
            SleepingDisabled, // Prevent sleeping so motor can always control it
            Position(Vector::new(200.0, 0.0)),
        ))
        .id();

    // frequency = 20 Hz, damping_ratio = 1.0 (critically damped)
    commands.spawn((
        PrismaticJoint::new(piston_base, piston)
            .with_slider_axis(Vector::Y)
            .with_limits(-100.0, 100.0)
            .with_motor(
                LinearMotor::new(MotorModel::SpringDamper {
                    frequency: 20.0,
                    damping_ratio: 1.0,
                })
                .with_target_position(50.0)
                .with_max_force(Scalar::MAX),
            ),
        PrismaticMotorJoint,
    ));

    commands.spawn((
        Text::new("Motor Joints Demo\n\nArrow Up/Down: Velocity motor speed\nA/D: Position motor angle\nW/S: Prismatic motor position\nSpace: Reset motors\n\nVelocity: 5.0 rad/s\nPosition: 0.00 rad\nPrismatic: 50.0 units"),
        TextFont {
            font_size: 18.0,
            ..default()
        },
        TextColor(Color::WHITE),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(10.0),
            left: Val::Px(10.0),
            ..default()
        },
        UiText,
    ));
}

fn control_motors(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut velocity_motors: Single<&mut RevoluteJoint, With<VelocityMotorJoint>>,
    mut position_motors: Single<
        &mut RevoluteJoint,
        (With<PositionMotorJoint>, Without<VelocityMotorJoint>),
    >,
    mut prismatic_motors: Single<&mut PrismaticJoint, With<PrismaticMotorJoint>>,
) {
    // Velocity-controlled revolute joint motor
    if keyboard.just_pressed(KeyCode::ArrowUp) {
        velocity_motors.motor.target_velocity += 1.0;
    }
    if keyboard.just_pressed(KeyCode::ArrowDown) {
        velocity_motors.motor.target_velocity -= 1.0;
    }

    // Position-controlled revolute joint motor
    if keyboard.just_pressed(KeyCode::KeyA) {
        position_motors.motor.target_position += 0.5;
    }
    if keyboard.just_pressed(KeyCode::KeyD) {
        position_motors.motor.target_position -= 0.5;
    }

    // Position-controlled prismatic joint motor
    if keyboard.just_pressed(KeyCode::KeyW) {
        prismatic_motors.motor.target_position += 25.0;
    }
    if keyboard.just_pressed(KeyCode::KeyS) {
        prismatic_motors.motor.target_position -= 25.0;
    }

    // Toggle motors on/off
    if keyboard.just_pressed(KeyCode::Space) {
        velocity_motors.motor.enabled = !velocity_motors.motor.enabled;
        position_motors.motor.enabled = !position_motors.motor.enabled;
        prismatic_motors.motor.enabled = !prismatic_motors.motor.enabled;
    }
}

fn update_ui(
    velocity_motor: Single<&RevoluteJoint, With<VelocityMotorJoint>>,
    position_motor: Single<&RevoluteJoint, With<PositionMotorJoint>>,
    prismatic_motor: Single<&PrismaticJoint, With<PrismaticMotorJoint>>,
    mut ui_text: Single<&mut Text, With<UiText>>,
) {
    ui_text.0 = format!(
        "Motor Joints Demo\n\n\
             Arrow Up/Down: Velocity motor speed\n\
             A/D: Position motor angle\n\
             W/S: Prismatic motor position\n\
             Space: Toggle motors\n\n\
             Velocity: {:.1} rad/s\n\
             Position: {:.2} rad\n\
             Prismatic: {:.1} units\n\
             Enabled: {}",
        velocity_motor.motor.target_velocity,
        position_motor.motor.target_position,
        prismatic_motor.motor.target_position,
        // We can pick any of the motors here since they are toggled together
        velocity_motor.motor.enabled
    );
}
