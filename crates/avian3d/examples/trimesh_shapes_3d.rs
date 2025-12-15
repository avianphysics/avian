//! This example is a version of Bevy's `3d_shapes` example that uses trimesh colliders for the shapes.
//!
//! You could also use convex decomposition to generate compound shapes from Bevy meshes.
//! The decomposition algorithm can be slow, but the generated colliders are often faster and more robust
//! than trimesh colliders.

use avian3d::prelude::*;
use bevy::{
    asset::RenderAssetUsages,
    color::palettes::css::SILVER,
    prelude::*,
    render::render_resource::{Extent3d, TextureDimension, TextureFormat},
};
use examples_common_3d::ExampleCommonPlugin;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins.set(ImagePlugin::default_nearest()),
            ExampleCommonPlugin,
            PhysicsPlugins::default(),
        ))
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut images: ResMut<Assets<Image>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let debug_material = materials.add(StandardMaterial {
        base_color_texture: Some(images.add(uv_debug_texture())),
        ..default()
    });

    let shapes: [Mesh; 5] = [
        Cuboid::default().into(),
        Capsule3d::default().into(),
        Torus::default().into(),
        Cylinder::default().into(),
        Sphere::new(0.5).into(),
    ];

    let num_shapes = shapes.len();

    // Spawn shapes
    for (i, shape) in shapes.into_iter().enumerate() {
        commands.spawn((
            RigidBody::Dynamic,
            Collider::trimesh_from_mesh(&shape).unwrap(),
            Mesh3d(meshes.add(shape)),
            MeshMaterial3d(debug_material.clone()),
            Transform::from_xyz(
                -10.0 / 2.0 + i as f32 / (num_shapes - 1) as f32 * 10.0,
                2.0,
                0.0,
            )
            .with_rotation(Quat::from_rotation_x(0.4)),
        ));
    }

    // Ground plane
    commands.spawn((
        RigidBody::Static,
        Collider::cuboid(50.0, 0.1, 50.0),
        Mesh3d(meshes.add(Plane3d::default().mesh().size(50.0, 50.0))),
        MeshMaterial3d(materials.add(Color::from(SILVER))),
        Transform::from_xyz(0.0, -1.0, 0.0),
    ));

    // Light
    commands.spawn((
        PointLight {
            intensity: 8_000_000.0,
            range: 100.,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(8.0, 16.0, 8.0),
    ));

    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 6., 12.0).looking_at(Vec3::new(0., 1., 0.), Vec3::Y),
    ));
}

/// Creates a colorful test pattern
fn uv_debug_texture() -> Image {
    const TEXTURE_SIZE: usize = 8;

    let mut palette: [u8; 32] = [
        255, 102, 159, 255, 255, 159, 102, 255, 236, 255, 102, 255, 121, 255, 102, 255, 102, 255,
        198, 255, 102, 198, 255, 255, 121, 102, 255, 255, 236, 102, 255, 255,
    ];

    let mut texture_data = [0; TEXTURE_SIZE * TEXTURE_SIZE * 4];
    for y in 0..TEXTURE_SIZE {
        let offset = TEXTURE_SIZE * y * 4;
        texture_data[offset..(offset + TEXTURE_SIZE * 4)].copy_from_slice(&palette);
        palette.rotate_right(4);
    }

    Image::new_fill(
        Extent3d {
            width: TEXTURE_SIZE as u32,
            height: TEXTURE_SIZE as u32,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        &texture_data,
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::default(),
    )
}
