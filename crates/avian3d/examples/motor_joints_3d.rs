//! Demonstrates motor joints in 3D.
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

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let anchor_mesh = meshes.add(Cuboid::from_size(Vec3::splat(0.3)));
    let anchor_material = materials.add(Color::srgb(0.5, 0.5, 0.5));

    let wheel_mesh = meshes.add(Cylinder::new(1.0, 0.3));
    let wheel_material = materials.add(Color::srgb(0.9, 0.3, 0.3));

    let arm_mesh = meshes.add(Cuboid::from_size(Vec3::new(2.0, 0.3, 0.3)));
    let arm_material = materials.add(Color::srgb(0.3, 0.5, 0.9));

    let piston_base_mesh = meshes.add(Cuboid::from_size(Vec3::new(0.5, 3.0, 0.5)));
    let piston_base_material = materials.add(Color::srgb(0.5, 0.5, 0.5));

    let piston_mesh = meshes.add(Cuboid::from_size(Vec3::new(0.8, 0.5, 0.8)));
    let piston_material = materials.add(Color::srgb(0.3, 0.9, 0.3));

    // === Velocity-Controlled Revolute Joint (left side) ===
    // Static anchor for the wheel
    let velocity_anchor = commands
        .spawn((
            Mesh3d(anchor_mesh.clone()),
            MeshMaterial3d(anchor_material.clone()),
            Transform::from_xyz(-3.0, 0.0, 0.0),
            RigidBody::Static,
        ))
        .id();

    // Spinning wheel
    let velocity_wheel = commands
        .spawn((
            Mesh3d(wheel_mesh),
            MeshMaterial3d(wheel_material),
            Transform::from_xyz(-3.0, 0.0, 0.0),
            RigidBody::Dynamic,
            MassPropertiesBundle::from_shape(&Cylinder::new(1.0, 0.3), 1.0),
            SleepingDisabled, // Prevent sleeping so motor can always control it
        ))
        .id();

    // Revolute joint with velocity-controlled motor
    commands.spawn((
        RevoluteJoint::new(velocity_anchor, velocity_wheel).with_hinge_axis(Vector::Z),
        AngularJointMotor {
            target_velocity: 5.0,
            damping: 1.0,
            max_torque: 1000.0,
            motor_model: MotorModel::AccelerationBased,
            ..default()
        },
        VelocityMotorJoint,
    ));

    // === Position-Controlled Revolute Joint (center) ===
    // Static anchor for the servo
    let position_anchor = commands
        .spawn((
            Mesh3d(anchor_mesh.clone()),
            MeshMaterial3d(anchor_material.clone()),
            Transform::from_xyz(0.0, 0.0, 0.0),
            RigidBody::Static,
        ))
        .id();

    // Servo arm - rotates around its center
    let servo_arm = commands
        .spawn((
            Mesh3d(arm_mesh),
            MeshMaterial3d(arm_material),
            Transform::from_xyz(0.0, 0.0, 0.0),
            RigidBody::Dynamic,
            MassPropertiesBundle::from_shape(&Cuboid::from_size(Vec3::new(2.0, 0.3, 0.3)), 1.0),
            SleepingDisabled, // Prevent sleeping so motor can always control it
        ))
        .id();

    // Revolute joint with position-controlled motor (servo behavior)
    commands.spawn((
        RevoluteJoint::new(position_anchor, servo_arm).with_hinge_axis(Vector::Z),
        AngularJointMotor::new(0.0)
            .with_spring_parameters(5.0, 1.0)
            .with_target_position_value(0.0)
            .with_max_torque(Scalar::MAX),
        PositionMotorJoint,
    ));

    // === Prismatic Joint with Linear Motor (right side) ===
    // Static base for the piston
    let piston_base = commands
        .spawn((
            Mesh3d(piston_base_mesh),
            MeshMaterial3d(piston_base_material),
            Transform::from_xyz(3.0, 0.0, 0.0),
            RigidBody::Static,
        ))
        .id();

    // Moving piston
    let piston = commands
        .spawn((
            Mesh3d(piston_mesh),
            MeshMaterial3d(piston_material),
            Transform::from_xyz(3.0, 0.0, 0.0),
            RigidBody::Dynamic,
            MassPropertiesBundle::from_shape(&Cuboid::from_size(Vec3::new(0.8, 0.5, 0.8)), 1.0),
            SleepingDisabled, // Prevent sleeping so motor can always control it
        ))
        .id();

    // Prismatic joint with linear motor
    commands.spawn((
        PrismaticJoint::new(piston_base, piston).with_slider_axis(Vector::Y),
        LinearJointMotor::new(0.0)
            .with_spring_parameters(20.0, 1.0)
            .with_target_position_value(1.0)
            .with_max_force(Scalar::MAX),
        PrismaticMotorJoint,
    ));

    // UI text
    commands.spawn((
        Text::new("Motor Joints Demo\n\nArrow Up/Down: Velocity motor speed\nA/D: Position motor angle\nW/S: Prismatic motor position\nSpace: Reset motors\n\nVelocity: 5.0 rad/s\nPosition: 0.00 rad\nPrismatic: 1.0 units"),
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

    // Directional light
    commands.spawn((
        DirectionalLight {
            illuminance: 2000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::default().looking_at(Vec3::new(-1.0, -2.5, -1.5), Vec3::Y),
    ));

    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_translation(Vec3::new(0.0, 2.0, 10.0)).looking_at(Vec3::ZERO, Vec3::Y),
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
            motor.target_position += 0.5;
        }
        if keyboard.just_pressed(KeyCode::KeyS) {
            motor.target_position -= 0.5;
        }
        if keyboard.just_pressed(KeyCode::Space) {
            if motor.target_position != 0.0 {
                motor.target_position = 0.0;
            } else {
                motor.target_position = 1.0;
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
