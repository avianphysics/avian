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
        .insert_resource(Gravity(Vector::NEG_Y * 1000.0))
        .add_systems(Startup, setup)
        .run();
}

fn setup(mut commands: Commands) {
    commands.spawn(Camera2d);

    let square_sprite = Sprite {
        color: Color::srgb(0.2, 0.7, 0.9),
        custom_size: Some(Vec2::splat(50.0)),
        ..default()
    };

    let anchor = commands
        .spawn((square_sprite.clone(), RigidBody::Kinematic))
        .id();

    let object = commands
        .spawn((
            square_sprite,
            Transform::from_xyz(100.0, 0.0, 0.0),
            RigidBody::Dynamic,
            MassPropertiesBundle::from_shape(&Rectangle::from_length(50.0), 1.0),
        ))
        .id();

    commands.spawn((
        DistanceJoint::new(anchor, object)
            .with_limits(100.0, 100.0)
            .with_compliance(0.00000001),
        JointDamping {
            linear: 0.1,
            angular: 1.0,
        },
    ));
}
