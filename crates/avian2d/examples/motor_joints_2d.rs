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
        .insert_resource(SubstepCount(50))
        .insert_resource(Gravity(Vector::ZERO)) // No gravity for clearer motor demo
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
        RevoluteJoint::new(velocity_anchor, velocity_wheel),
        AngularJointMotor {
            target_velocity: 5.0,
            max_torque: 1000.0,
            motor_model: MotorModel::AccelerationBased {
                stiffness: 0.0,
                damping: 1.0,
            },
            ..default()
        },
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
    // Using spring parameters (frequency, damping_ratio) for timestep-independent behavior.
    // This provides predictable spring-damper dynamics regardless of substep count.
    // - frequency: 5 Hz = fairly stiff spring
    // - damping_ratio: 1.0 = critically damped (fastest approach without overshoot)
    commands.spawn((
        RevoluteJoint::new(position_anchor, servo_arm),
        AngularJointMotor::new(0.0)
            .with_spring_damper(5.0, 1.0)
            .with_target_position_value(0.0)
            .with_max_torque(Scalar::MAX),
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
        PrismaticJoint::new(piston_base, piston).with_slider_axis(Vector::Y),
        LinearJointMotor::new(0.0)
            .with_spring_damper(20.0, 1.0)
            .with_target_position_value(50.0)
            .with_max_force(Scalar::MAX),
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
    mut velocity_motors: Query<&mut AngularJointMotor, With<VelocityMotorJoint>>,
    mut position_motors: Query<
        &mut AngularJointMotor,
        (With<PositionMotorJoint>, Without<VelocityMotorJoint>),
    >,
    mut prismatic_motors: Query<&mut LinearJointMotor, With<PrismaticMotorJoint>>,
) {
    for mut motor in velocity_motors.iter_mut() {
        if keyboard.just_pressed(KeyCode::ArrowUp) {
            motor.target_velocity += 1.0;
        }
        if keyboard.just_pressed(KeyCode::ArrowDown) {
            motor.target_velocity -= 1.0;
        }
        if keyboard.just_pressed(KeyCode::Space) {
            if motor.target_velocity != 0.0 {
                motor.target_velocity = 0.0;
            } else {
                motor.target_velocity = 5.0;
            }
        }
    }

    for mut motor in position_motors.iter_mut() {
        if keyboard.just_pressed(KeyCode::KeyA) {
            motor.target_position += 0.5;
        }
        if keyboard.just_pressed(KeyCode::KeyD) {
            motor.target_position -= 0.5;
        }
        if keyboard.just_pressed(KeyCode::Space) {
            motor.target_position = 0.0;
        }
    }

    for mut motor in prismatic_motors.iter_mut() {
        if keyboard.just_pressed(KeyCode::KeyW) {
            motor.target_position += 25.0;
        }
        if keyboard.just_pressed(KeyCode::KeyS) {
            motor.target_position -= 25.0;
        }
        if keyboard.just_pressed(KeyCode::Space) {
            if motor.target_position != 0.0 {
                motor.target_position = 0.0;
            } else {
                motor.target_position = 50.0;
            }
        }
    }
}

fn update_ui(
    velocity_motors: Query<&AngularJointMotor, With<VelocityMotorJoint>>,
    position_motors: Query<&AngularJointMotor, With<PositionMotorJoint>>,
    prismatic_motors: Query<&LinearJointMotor, With<PrismaticMotorJoint>>,
    mut ui_text: Query<&mut Text, With<UiText>>,
) {
    let velocity_target = velocity_motors
        .iter()
        .next()
        .map(|m| m.target_velocity)
        .unwrap_or(0.0);
    let position_target = position_motors
        .iter()
        .next()
        .map(|m| m.target_position)
        .unwrap_or(0.0);
    let prismatic_pos = prismatic_motors
        .iter()
        .next()
        .map(|m| m.target_position)
        .unwrap_or(0.0);

    for mut text in ui_text.iter_mut() {
        text.0 = format!(
            "Motor Joints Demo\n\n\
             Arrow Up/Down: Velocity motor speed\n\
             A/D: Position motor angle\n\
             W/S: Prismatic motor position\n\
             Space: Reset motors\n\n\
             Velocity: {:.1} rad/s\n\
             Position: {:.2} rad\n\
             Prismatic: {:.1} units",
            velocity_target, position_target, prismatic_pos
        );
    }
}
