use core::time::Duration;

use avian3d::math::*;
use avian3d::prelude::*;
use benches_common_3d::bench_app;
use bevy::prelude::*;
use criterion::{criterion_group, criterion_main, Criterion};

fn setup_cubes(app: &mut App, size: u32) {
    app.insert_resource(SubstepCount(8));
    app.add_systems(Startup, move |mut commands: Commands| {
        commands.spawn((
            RigidBody::Static,
            Position(-2.0 * Vector::Z),
            Collider::cuboid(100.0, 1.0, 100.0),
        ));

        for x in 0..size {
            for z in 0..size {
                commands.spawn((
                    RigidBody::Dynamic,
                    Position(Vector::new(x as Scalar, 2.0, z as Scalar)),
                    Collider::cuboid(1.0, 1.0, 1.0),
                ));
            }
        }
    });
}

fn criterion_benchmark(c: &mut Criterion) {
    c.bench_function("cubes 3x3, 30 steps", |b| {
        bench_app(b, 30, |app| setup_cubes(app, 3))
    });

    c.bench_function("cubes 5x5, 30 steps", |b| {
        bench_app(b, 30, |app| setup_cubes(app, 5))
    });

    c.bench_function("cubes 10x10, 30 steps", |b| {
        bench_app(b, 30, |app| setup_cubes(app, 10))
    });
}

criterion_group!(
    name = benches;
    config = Criterion::default().measurement_time(Duration::from_secs(20));
    targets = criterion_benchmark
);
criterion_main!(benches);
