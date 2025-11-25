//! An example showcasing how to create colliders for meshes and scenes
//! using `ColliderConstructor` and `ColliderConstructorHierarchy` respectively.

use avian3d::prelude::*;
use bevy::prelude::*;
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
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
    assets: ResMut<AssetServer>,
) {
    // Spawn ground and generate a collider for the mesh using ColliderConstructor
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(8.0, 8.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.3, 0.5, 0.3))),
        ColliderConstructor::TrimeshFromMesh,
        RigidBody::Static,
    ));

    // Spawn Ferris the crab and generate colliders for the scene using ColliderConstructorHierarchy
    commands.spawn((
        // The model was made by RayMarch, licenced under CC0-1.0, and can be found here:
        // https://github.com/RayMarch/ferris3d
        SceneRoot(assets.load("ferris.glb#Scene0")),
        Transform::from_xyz(0.0, 1.0, 0.0).with_scale(Vec3::splat(2.0)),
        // Create colliders using convex decomposition.
        // This takes longer than creating a trimesh or convex hull collider,
        // but is more performant for collision detection.
        ColliderConstructorHierarchy::new(ColliderConstructor::ConvexDecompositionFromMesh)
            // Make the arms heavier to make it easier to stand upright
            // Note: Bevy uses the format `MeshName.MaterialName` for the `Name` of glTF mesh primitives.
            .with_density_for_name("armL_mesh.ferris_material", 3.0)
            .with_density_for_name("armR_mesh.ferris_material", 3.0),
        RigidBody::Dynamic,
    ));

    // Light
    commands.spawn((
        PointLight {
            intensity: 1_000_000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(2.0, 8.0, 2.0),
    ));

    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-5.0, 3.5, 5.5).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}
