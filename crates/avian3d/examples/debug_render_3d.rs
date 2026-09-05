use std::time::Duration;

use avian3d::{math::RVector, prelude::*};
use bevy::prelude::*;

fn main() {
    App::default()
        .add_plugins((
            DefaultPlugins,
            PhysicsPlugins::default(),
            PhysicsDebugPlugin,
        ))
        .add_systems(Startup, scene.spawn())
        .insert_resource(Cycle::new(Duration::from_millis(500)))
        .add_systems(Update, (cast, move_wall))
        .run();
}

fn scene() -> impl SceneList {
    bsn_list![
        (
            Camera3d
            template_value(Transform::from_xyz(10.0, 10.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y))
        ),
        (
            Mesh3d(asset_value(Cuboid::new(8.0, 1.0, 1.0)))
            MeshMaterial3d::<StandardMaterial>(asset_value(Color::WHITE))
            Collider::cuboid(8.0, 1.0, 1.0)
            Transform::from_xyz(0.0, 0.0, -2.0)
            MoveLeft
        ),
    ]
}

#[derive(Resource, Deref, DerefMut)]
struct Cycle(Timer);

impl Cycle {
    pub fn new(duration: Duration) -> Self {
        Self(Timer::new(duration, TimerMode::Repeating))
    }
}

fn cast(space: SpatialQuery, mut cycle: ResMut<Cycle>, time: Res<Time>) {
    if cycle.tick(time.delta()).just_finished() {
        let filter = SpatialQueryFilter::default();
        let shape = Collider::sphere(0.5);

        space.cast_ray(RVector::ZERO, Dir3::NEG_Z, 100.0, false, &filter);
        space.cast_shape(
            &shape,
            RVector::new(1.0, 0.0, 0.0),
            Quat::IDENTITY,
            Dir3::NEG_Z,
            &Default::default(),
            &filter,
        );
        space.project_point(RVector::new(-1.0, 0.0, 0.0), false, &filter);
        space.shape_intersections(
            &shape,
            RVector::new(-2.0, 0.0, -1.25),
            Quat::IDENTITY,
            &filter,
        );
    }
}

#[derive(Resource, Deref, DerefMut, Clone, Copy, Default)]
struct MoveLeft(pub bool);

fn move_wall(mut wall: Single<(&mut Transform, &mut MoveLeft)>, time: Res<Time>) {
    let (transform, move_left) = &mut *wall;
    let delta = 1.0 * time.delta_secs();

    if ***move_left {
        transform.translation.x -= delta;
    } else {
        transform.translation.x += delta;
    }

    if transform.translation.x <= -5.0 {
        ***move_left = false;
    } else if transform.translation.x >= 5.0 {
        ***move_left = true;
    }
}
