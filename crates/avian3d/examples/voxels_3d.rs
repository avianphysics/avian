//! This example demonstrates creating voxelized colliders from a set of points.
//!
//! Note that voxel colliders can also be created from meshes using methods such as
//! [`Collider::voxelized_trimesh_from_mesh`], or by using [`ColliderConstructor::VoxelizedTrimeshFromMesh`].

#![allow(clippy::unnecessary_cast)]

use avian3d::{
    math::{AsF32, PI, Scalar, Vector},
    prelude::*,
};
use bevy::{
    asset::RenderAssetUsages,
    camera::Exposure,
    core_pipeline::tonemapping::Tonemapping,
    mesh::{Indices, PrimitiveTopology},
    pbr::{Atmosphere, ScatteringMedium},
    prelude::*,
};
use examples_common_3d::ExampleCommonPlugin;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            ExampleCommonPlugin,
            PhysicsPlugins::default(),
        ))
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut scattering_media: ResMut<Assets<ScatteringMedium>>,
) {
    // Create a voxelized bowl-like surface by sampling points.
    let mut points = vec![];
    let voxel_size = Vector::splat(0.25);
    let n = 200;
    for i in 0..n {
        for j in 0..n {
            let x = i as Scalar;
            let z = j as Scalar;
            let y = -40.0
                * ((x / n as Scalar * PI).sin() * (z / n as Scalar * PI).sin()).clamp(0.1, 0.9);
            points.push(voxel_size * Vector::new(x, y, z));
        }
    }

    let collider = Collider::voxels_from_points(voxel_size, &points);

    // Compute the mesh for rendering.
    let (vertices, indices) = collider.shape().as_voxels().unwrap().to_trimesh();
    let vertices: Vec<[f32; 3]> = vertices
        .iter()
        .map(|v| [v.x as f32, v.y as f32, v.z as f32])
        .collect();
    let indices: Vec<u32> = indices.into_iter().flatten().collect();
    let mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    )
    .with_inserted_indices(Indices::U32(indices.clone()))
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, vertices)
    .with_duplicated_vertices()
    .with_computed_flat_normals();

    // Spawn the voxel surface.
    commands.spawn((
        RigidBody::Static,
        collider,
        Friction::new(0.2),
        Mesh3d(meshes.add(mesh)),
        MeshMaterial3d(materials.add(Color::srgb(0.4, 0.6, 0.4))),
        Transform::from_translation(
            Vector::new(
                -n as Scalar / 2.0 * voxel_size.x,
                -5.0 * voxel_size.y,
                -n as Scalar / 2.0 * voxel_size.z,
            )
            .f32(),
        ),
    ));

    // Spawn a grid of various dynamic primitives that will fall onto the surface.
    let sphere = Sphere::new(0.5);
    let cuboid = Cuboid::from_length(1.0);
    let capsule = Capsule3d::new(0.35, 1.0);
    let cylinder = Cylinder::new(0.5, 1.0);

    let shapes = [
        (
            sphere.collider(),
            meshes.add(sphere),
            materials.add(Color::srgb(0.29, 0.33, 0.64)),
        ),
        (
            cuboid.collider(),
            meshes.add(cuboid),
            materials.add(Color::srgb(0.47, 0.58, 0.8)),
        ),
        (
            capsule.collider(),
            meshes.add(capsule),
            materials.add(Color::srgb(0.63, 0.75, 0.88)),
        ),
        (
            cylinder.collider(),
            meshes.add(cylinder),
            materials.add(Color::srgb(0.77, 0.87, 0.97)),
        ),
    ];

    for x in -12_i32..12 {
        for z in -12_i32..12 {
            // Skip the center.
            if x.abs() < 9 && z.abs() < 9 {
                continue;
            }

            let (collider, mesh, material) = shapes[(x + z) as usize % 4].clone();
            commands.spawn((
                RigidBody::Dynamic,
                collider,
                Mesh3d(mesh),
                MeshMaterial3d(material),
                Transform::from_xyz(x as f32 * 1.4, 3.5, z as f32 * 1.4)
                    .with_rotation(Quat::from_rotation_z(0.1)),
                Friction::new(0.2),
            ));
        }
    }

    // Directional light
    commands.spawn((
        DirectionalLight {
            illuminance: 14e4,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(1.0, 2.0, 2.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Camera and atmosphere
    commands.spawn((
        Camera3d::default(),
        Atmosphere::earthlike(scattering_media.add(ScatteringMedium::default())),
        AmbientLight {
            brightness: 4000.0,
            color: Color::WHITE,
            ..default()
        },
        Exposure::SUNLIGHT,
        Tonemapping::AcesFitted,
        Transform::from_xyz(13.0, 1.0, 26.0).looking_at(Vec3::new(0.0, -8.0, 0.0), Vec3::Y),
    ));
}
