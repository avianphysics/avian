use core::time::Duration;

use avian2d::math::*;
use avian2d::prelude::*;
use benches_common_2d::bench_app;
use bevy::prelude::*;
use criterion::{criterion_group, criterion_main, Criterion};

fn setup(app: &mut App, base_count: u32) {
    app.insert_resource(SubstepCount(8));
    app.add_systems(Startup, move |mut commands: Commands| {
        // Ground
        commands.spawn((
            RigidBody::Static,
            Collider::rectangle(800.0, 40.0),
            Position::from_xy(0.0, -20.0),
        ));

        let h = 0.5;
        let box_size = 2.0 * h;
        let collider = Collider::rectangle(box_size, box_size);
        let shift = h;
        for i in 0..base_count {
            let y = (2.0 * i as Scalar + 1.0) * shift * 0.99;

            for j in i..base_count {
                let x = (i as Scalar + 1.0) * shift + 2.0 * (j - i) as Scalar * shift
                    - h * base_count as Scalar;

                commands.spawn((
                    RigidBody::Dynamic,
                    collider.clone(),
                    Position::from_xy(x, y),
                ));
            }
        }
    });
}

fn criterion_benchmark(c: &mut Criterion) {
    c.bench_function("pyramid with base of 20 boxes, 5 steps", |b| {
        bench_app(b, 5, |app| setup(app, 20))
    });

    c.bench_function("pyramid with base of 40 boxes, 5 steps", |b| {
        bench_app(b, 5, |app| setup(app, 40))
    });

    c.bench_function("pyramid with base of 60 boxes, 5 steps", |b| {
        bench_app(b, 5, |app| setup(app, 60))
    });

    c.bench_function("pyramid with base of 80 boxes, 5 steps", |b| {
        bench_app(b, 5, |app| setup(app, 80))
    });
}

criterion_group!(
    name = benches;
    config = Criterion::default().measurement_time(Duration::from_secs(10));
    targets = criterion_benchmark
);
criterion_main!(benches);
