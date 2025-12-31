use bevy_math::{Dir3, Vec3};
use core::hint::black_box;
use criterion::{BenchmarkId, Criterion, PlotConfiguration, criterion_group, criterion_main};

use avian3d::{
    character_controller::move_and_slide::{
        project_velocity, project_velocity_bruteforce, test::QuasiRandomDirection,
    },
    math::PI,
};

fn bench_velocity_projection(c: &mut Criterion) {
    let mut group = c.benchmark_group("Velocity Projection");
    let normals = &[
        Dir3::Z,
        Dir3::from_xyz(2.0, 0.0, 1.0).unwrap(),
        Dir3::from_xyz(-2.0, 0.0, 1.0).unwrap(),
        Dir3::from_xyz(0.0, 2.0, 1.0).unwrap(),
        Dir3::from_xyz(0.0, -2.0, 1.0).unwrap(),
        Dir3::from_xyz(1.5, 1.5, 1.0).unwrap(),
        Dir3::from_xyz(1.5, -1.5, 1.0).unwrap(),
        Dir3::from_xyz(-1.5, 1.5, 1.0).unwrap(),
        Dir3::from_xyz(-1.5, -1.5, 1.0).unwrap(),
        Dir3::from_xyz(1.0, 1.75, 1.0).unwrap(),
        Dir3::from_xyz(1.0, -1.75, 1.0).unwrap(),
        Dir3::from_xyz(-1.0, 1.75, 1.0).unwrap(),
        Dir3::from_xyz(-1.0, -1.75, 1.0).unwrap(),
        Dir3::from_xyz(1.75, 1.0, 1.0).unwrap(),
        Dir3::from_xyz(1.75, -1.0, 1.0).unwrap(),
        Dir3::from_xyz(-1.75, 1.0, 1.0).unwrap(),
        Dir3::from_xyz(-1.75, -1.0, 1.0).unwrap(),
    ];

    // Performance is not the same for every input velocity,
    // so ensure we're sampling the sphere evenly.
    let mut velocities = QuasiRandomDirection::default();

    for n in 1..=normals.len() {
        velocities.reset();
        group.bench_with_input(
            BenchmarkId::new("brute-force", n),
            &normals[..n],
            |b, norms| {
                b.iter_batched(
                    || velocities.next().unwrap(),
                    |v| project_velocity_bruteforce(black_box(v), black_box(norms)),
                    criterion::BatchSize::SmallInput,
                )
            },
        );
        velocities.reset();
        group.bench_with_input(BenchmarkId::new("gjk", n), &normals[..n], |b, norms| {
            b.iter_batched(
                || velocities.next().unwrap(),
                |v| project_velocity(black_box(v), black_box(norms)),
                criterion::BatchSize::SmallInput,
            )
        });
    }
    group
        .plot_config(PlotConfiguration::default().summary_scale(criterion::AxisScale::Logarithmic));
}

criterion_group!(benches, bench_velocity_projection);
criterion_main!(benches);
