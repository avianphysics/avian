use avian3d::{math::*, prelude::*};
use bevy::{color::palettes::tailwind::GRAY_400, prelude::*};
use examples_common_3d::ExampleCommonPlugin;
use rand::Rng;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            ExampleCommonPlugin,
            PhysicsPlugins::default()
                .with_length_unit(10.0)
                .build()
                .disable::<SpatialQueryPlugin>(),
            PhysicsDebugPlugin,
        ))
        .insert_gizmo_config(
            PhysicsGizmos::none().with_aabb_color(GRAY_400.into()),
            GizmoConfig {
                line: GizmoLineConfig {
                    width: 0.5,
                    ..default()
                },
                ..default()
            },
        )
        .insert_resource(Gravity::ZERO)
        .add_systems(Startup, setup)
        .add_systems(FixedUpdate, move_random)
        .run();
}

fn setup(mut commands: Commands) {
    let x_count = 100;
    let y_count = 100;
    let particle_radius = 7.0;

    commands.spawn((
        Camera3d::default(),
        Projection::Orthographic(OrthographicProjection {
            scaling_mode: bevy::camera::ScalingMode::FixedVertical {
                viewport_height: 3.0 * particle_radius * (y_count as f32 * 1.1),
            },
            ..OrthographicProjection::default_3d()
        }),
        Transform::from_xyz(0.0, 0.0, 30.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    for x in -x_count / 2..x_count / 2 {
        for y in -y_count / 2..y_count / 2 {
            commands.spawn((
                Transform::from_xyz(
                    x as f32 * 3.0 * particle_radius,
                    y as f32 * 3.0 * particle_radius,
                    0.0,
                ),
                RigidBody::Dynamic,
                SleepingDisabled,
                Collider::sphere(particle_radius.adjust_precision()),
                CollisionLayers::new(LayerMask::DEFAULT, LayerMask::NONE),
            ));
        }
    }
}

fn move_random(mut query: Query<&mut Position>) {
    let mut rng = rand::rng();
    for mut position in query.iter_mut() {
        if rng.random::<f32>() < 0.25 {
            position.0 += Vec3::new(
                rng.random_range(-1.0..1.0),
                rng.random_range(-1.0..1.0),
                0.0,
            );
        }
    }
}
