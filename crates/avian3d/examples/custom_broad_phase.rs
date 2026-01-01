use avian3d::{dynamics::solver::joint_graph::JointGraph, math::*, prelude::*};
use bevy::prelude::*;
use examples_common_3d::ExampleCommonPlugin;

fn main() {
    let mut app = App::new();

    app.add_plugins((DefaultPlugins, ExampleCommonPlugin));

    // Add `PhysicsPlugins` and replace the default broad phase with our custom broad phase.
    app.add_plugins(
        PhysicsPlugins::default()
            .build()
            .disable::<BvhBroadPhasePlugin>()
            .add(BruteForceBroadPhasePlugin),
    );

    app.add_systems(Startup, setup).run();
}

// Modified from Bevy's 3d_scene example, a cube falling to the ground with velocity
fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Plane
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(8.0, 8.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.3, 0.5, 0.3))),
        RigidBody::Static,
        Collider::cuboid(8.0, 0.002, 8.0),
    ));
    // Cube
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::default())),
        MeshMaterial3d(materials.add(Color::srgb(0.8, 0.7, 0.6))),
        Transform::from_xyz(0.0, 4.0, 0.0),
        RigidBody::Dynamic,
        AngularVelocity(Vector::new(2.5, 3.4, 1.6)),
        Collider::cuboid(1.0, 1.0, 1.0),
    ));
    // Light
    commands.spawn((
        PointLight {
            intensity: 2_000_000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 4.0),
    ));
    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-4.0, 6.5, 8.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

/// Finds pairs of entities with overlapping `ColliderAabb`s and creates contact pairs for them in the `ContactGraph`.
/// The narrow phasoe will then process these contact pairs and compute contact data.
///
// A brute force algorithm is used for simplicity.
pub struct BruteForceBroadPhasePlugin;

impl Plugin for BruteForceBroadPhasePlugin {
    fn build(&self, app: &mut App) {
        // Add the broad phase system into the broad phase set.
        app.add_systems(
            PhysicsSchedule,
            collect_collision_pairs.in_set(BroadPhaseSystems::CollectCollisions),
        );
    }
}

fn collect_collision_pairs(
    colliders: Query<(Entity, &ColliderAabb, &ColliderOf)>,
    bodies: Query<&RigidBody>,
    mut contact_graph: ResMut<ContactGraph>,
    joint_graph: Res<JointGraph>,
) {
    // Loop through all entity combinations and create contact pairs for overlapping AABBs.
    for [
        (collider1, aabb1, collider_of1),
        (collider2, aabb2, collider_of2),
    ] in colliders.iter_combinations()
    {
        // Get the rigid bodies of the colliders.
        let Ok(rb1) = bodies.get(collider_of1.body) else {
            continue;
        };
        let Ok(rb2) = bodies.get(collider_of2.body) else {
            continue;
        };

        // Skip pairs where both bodies are non-dynamic.
        if !rb1.is_dynamic() && !rb2.is_dynamic() {
            continue;
        }

        // Check if the AABBs of the colliders intersect.
        if !aabb1.intersects(aabb2) {
            continue;
        }

        // Optional: Check if a joint disables contacts between the two bodies.
        if joint_graph
            .joints_between(collider_of1.body, collider_of2.body)
            .any(|edge| edge.collision_disabled)
        {
            continue;
        }

        // Create a contact in the contact graph.
        let mut contact_edge = ContactEdge::new(collider1, collider2);
        contact_edge.body1 = Some(collider_of1.body);
        contact_edge.body2 = Some(collider_of2.body);
        contact_graph.add_edge(contact_edge);
    }
}
