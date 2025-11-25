use crate::prelude::*;
#[cfg(all(
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
use approx::assert_relative_eq;
use bevy::{
    ecs::schedule::{LogLevel, ScheduleBuildSettings, ScheduleLabel},
    prelude::*,
    time::TimeUpdateStrategy,
};
use core::time::Duration;

#[cfg(all(feature = "2d", feature = "enhanced-determinism"))]
mod determinism_2d;

fn create_app() -> App {
    let mut app = App::new();

    app.add_plugins((
        MinimalPlugins,
        TransformPlugin,
        PhysicsPlugins::default(),
        bevy::asset::AssetPlugin::default(),
        #[cfg(all(feature = "collider-from-mesh", feature = "default-collider"))]
        bevy::mesh::MeshPlugin,
        #[cfg(feature = "bevy_scene")]
        bevy::scene::ScenePlugin,
    ))
    .insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f32(
        1.0 / 60.0,
    )));

    app.finish();

    app
}

fn tick_app(app: &mut App, timestep: f64) {
    let strategy = TimeUpdateStrategy::ManualDuration(Duration::from_secs_f64(timestep));

    if let Some(mut update_strategy) = app.world_mut().get_resource_mut::<TimeUpdateStrategy>() {
        *update_strategy = strategy;
    } else {
        app.insert_resource(strategy);
    }

    app.update();
}

#[cfg(all(feature = "3d", feature = "default-collider"))]
fn setup_cubes_simulation(mut commands: Commands) {
    let mut next_id = 0;
    // copied from "cubes" example
    let floor_size = Vector::new(80.0, 1.0, 80.0);
    commands.spawn((
        RigidBody::Static,
        Position(Vector::NEG_Y),
        Collider::cuboid(floor_size.x, floor_size.y, floor_size.z),
    ));

    let radius = 1.0;
    let count_x = 4;
    let count_y = 4;
    let count_z = 4;
    for y in 0..count_y {
        for x in 0..count_x {
            for z in 0..count_z {
                let pos = Vector::new(
                    (x as Scalar - count_x as Scalar * 0.5) * 2.1 * radius,
                    10.0 * radius * y as Scalar,
                    (z as Scalar - count_z as Scalar * 0.5) * 2.1 * radius,
                );
                commands.spawn((
                    Transform::default(),
                    RigidBody::Dynamic,
                    Position(pos + Vector::Y * 5.0),
                    Collider::cuboid(radius * 2.0, radius * 2.0, radius * 2.0),
                    Id(next_id),
                ));
                next_id += 1;
            }
        }
    }
}

#[test]
fn it_loads_plugin_without_errors() -> Result<(), Box<dyn core::error::Error>> {
    let mut app = create_app();

    for _ in 0..500 {
        tick_app(&mut app, 1.0 / 60.0);
    }

    Ok(())
}

#[test]
#[cfg(all(
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
fn body_with_velocity_moves() {
    let mut app = create_app();

    app.insert_resource(Gravity::ZERO);

    app.add_systems(Startup, |mut commands: Commands| {
        // move right at 1 unit per second
        commands.spawn((
            Transform::default(),
            RigidBody::Dynamic,
            LinearVelocity(Vector::X),
            #[cfg(feature = "2d")]
            MassPropertiesBundle::from_shape(&Circle::new(0.5), 1.0),
            #[cfg(feature = "3d")]
            MassPropertiesBundle::from_shape(&Sphere::new(0.5), 1.0),
        ));
    });

    // Run startup systems
    app.update();

    const UPDATES: usize = 500;

    for _ in 0..UPDATES {
        tick_app(&mut app, 1.0 / 60.0);
    }

    let mut app_query = app.world_mut().query::<(&Transform, &RigidBody)>();

    let (transform, _body) = app_query.single(app.world()).unwrap();

    assert_relative_eq!(transform.translation.y, 0.);
    assert_relative_eq!(transform.translation.z, 0.);

    // make sure we end up in the expected position
    assert_relative_eq!(
        transform.translation.x,
        1. * UPDATES as f32 * 1. / 60.,
        epsilon = 0.03 // allow some leeway, as we might be one frame off
    );
}

#[derive(Component, Clone, Copy, Debug, PartialEq, PartialOrd, Eq, Ord)]
#[cfg(all(feature = "3d", feature = "default-collider"))]
struct Id(usize);

#[cfg(all(feature = "3d", feature = "default-collider"))]
#[test]
fn cubes_simulation_is_locally_deterministic() {
    use itertools::Itertools;

    fn run_cubes() -> Vec<(Id, Transform)> {
        let mut app = create_app();

        app.add_systems(Startup, setup_cubes_simulation);

        // Run startup systems
        app.update();

        const SECONDS: usize = 5;
        const UPDATES: usize = 60 * SECONDS;

        for _ in 0..UPDATES {
            tick_app(&mut app, 1.0 / 60.0);
        }

        let mut app_query = app.world_mut().query::<(&Id, &Transform)>();

        let mut bodies: Vec<(Id, Transform)> = app_query
            .iter(app.world())
            .map(|(id, transform)| (*id, *transform))
            .collect();
        bodies.sort_by_key(|b| b.0);
        bodies
    }

    // run simulation and check that results are equal each time
    for (a, b) in (0..4).map(|_| run_cubes()).tuple_windows() {
        assert_eq!(a, b);
    }
}

#[test]
fn no_ambiguity_errors() {
    #[derive(ScheduleLabel, Clone, Debug, PartialEq, Eq, Hash)]
    struct DeterministicSchedule;

    App::new()
        .add_plugins((
            MinimalPlugins,
            PhysicsPlugins::new(DeterministicSchedule)
                .build()
                .disable::<ColliderHierarchyPlugin>(),
            bevy::asset::AssetPlugin::default(),
            #[cfg(feature = "bevy_scene")]
            bevy::scene::ScenePlugin,
            #[cfg(all(feature = "collider-from-mesh", feature = "default-collider"))]
            bevy::mesh::MeshPlugin,
        ))
        .edit_schedule(DeterministicSchedule, |s| {
            s.set_build_settings(ScheduleBuildSettings {
                ambiguity_detection: LogLevel::Error,
                ..default()
            });
        })
        .add_systems(Update, |w: &mut World| {
            w.run_schedule(DeterministicSchedule);
        })
        .update();
}
