use bevy_math::{Dir3, Vec3};
use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};
use std::hint::black_box;

use avian3d::{character_controller::velocity_project::*, math::PI};

fn bench_velocity_projection(c: &mut Criterion) {
    let mut group = c.benchmark_group("Velocity Projection");
    let normals = &[
        Dir3::Z,
        Dir3::from_xyz(2.0, 0.0, 1.0).unwrap(),
        Dir3::from_xyz(-2.0, 0.0, 1.0).unwrap(),
        Dir3::from_xyz(0.0, 2.0, 1.0).unwrap(),
        Dir3::from_xyz(0.0, -2.0, 1.0).unwrap(),
        Dir3::from_xyz(1.5, 1.5, 1.0).unwrap(),
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
    // so ensure we're sampling the sphere evenly
    let mut velocities = QuasiRandomDirection::default();

    for n in 0..normals.len() {
        velocities.reset();
        group.bench_with_input(
            BenchmarkId::new("old", n + 1),
            &normals[..=n],
            |b, norms| {
                b.iter_batched(
                    || velocities.next().unwrap(),
                    |v| project_velocity_old(black_box(v), black_box(norms)),
                    criterion::BatchSize::SmallInput,
                )
            },
        );
        velocities.reset();
        group.bench_with_input(
            BenchmarkId::new("new", n + 1),
            &normals[..=n],
            |b, norms| {
                b.iter_batched(
                    || velocities.next().unwrap(),
                    |v| project_velocity_new(black_box(v), black_box(norms)),
                    criterion::BatchSize::SmallInput,
                )
            },
        );
    }
}

criterion_group!(benches, bench_velocity_projection);
criterion_main!(benches);

#[derive(Default)]
struct QuasiRandomDirection {
    i: f32,
    j: f32,
}

const PLASTIC: f32 = 1.32471795724475;
const INV_PLASTIC: f32 = 1.0 / PLASTIC;
const INV_PLASTIC_SQ: f32 = INV_PLASTIC * INV_PLASTIC;

impl QuasiRandomDirection {
    pub fn reset(&mut self) {
        *self = Self::default()
    }
}
impl Iterator for QuasiRandomDirection {
    type Item = Vec3;

    fn next(&mut self) -> Option<Self::Item> {
        let z = 2.0 * self.i - 1.0;
        let rho = (1.0 - z * z).sqrt();
        let phi = 2.0 * PI * self.j;
        let x = rho * phi.cos();
        let y = rho * phi.sin();
        self.i = (self.i + INV_PLASTIC) % 1.0;
        self.j = (self.j + INV_PLASTIC_SQ) % 1.0;
        Some(Vec3::new(x, y, z))
    }
}
