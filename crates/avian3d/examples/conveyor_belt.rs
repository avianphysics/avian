//! Demonstrates how to use `CollisionHooks::modify_contacts`
//! and `tangent_velocity` to simulate conveyor belts.

use avian3d::{math::*, prelude::*};
use bevy::{
    ecs::system::{SystemParam, lifetimeless::Read},
    prelude::*,
};
use examples_common_3d::ExampleCommonPlugin;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            ExampleCommonPlugin,
            // Add our collision hooks to modify contacts for conveyor belts.
            PhysicsPlugins::default().with_collision_hooks::<ConveyorHooks>(),
        ))
        .add_systems(Startup, setup)
        .run();
}

// Enable contact modification for conveyor belts with the `ActiveCollisionHooks` component.
// Here we use required components, but you could also add it manually.
#[derive(Component)]
#[require(ActiveCollisionHooks::MODIFY_CONTACTS)]
struct ConveyorBelt {
    local_direction: Vec3,
    speed: f32,
}

// Define a custom `SystemParam` for our collision hooks.
// It can have read-only access to queries, resources, and other system parameters.
#[derive(SystemParam)]
struct ConveyorHooks<'w, 's> {
    conveyor_query: Query<'w, 's, (Read<ConveyorBelt>, Read<GlobalTransform>)>,
}

// Implement the `CollisionHooks` trait for our custom system parameter.
impl CollisionHooks for ConveyorHooks<'_, '_> {
    fn modify_contacts(&self, contacts: &mut ContactPair, _commands: &mut Commands) -> bool {
        // Get the conveyor belt and its global transform.
        // We don't know which entity is the conveyor belt, if any, so we need to check both.
        // This also affects the sign used for the conveyor belt's speed to apply it in the correct direction.
        let (Ok((conveyor_belt, global_transform)), sign) = self
            .conveyor_query
            .get(contacts.collider1)
            .map_or((self.conveyor_query.get(contacts.collider2), 1.0), |q| {
                (Ok(q), -1.0)
            })
        else {
            // If neither entity is a conveyor belt, return `true` early
            // to accept the contact pair without any modifications.
            return true;
        };

        // Calculate the conveyor belt's direction in world space.
        let direction = global_transform.rotation() * conveyor_belt.local_direction;

        // Iterate over all contact surfaces between the conveyor belt and the other collider,
        // and apply a relative velocity to simulate the movement of the conveyor belt's surface.
        for manifold in contacts.manifolds.iter_mut() {
            let tangent_velocity = sign * conveyor_belt.speed * direction;
            manifold.tangent_velocity = tangent_velocity.adjust_precision();
        }

        // Return `true` to accept the contact pair.
        true
    }
}

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let long_conveyor = Cuboid::new(18.0, 0.1, 6.0);
    let short_conveyor = Cuboid::new(14.0, 0.1, 6.0);

    let long_conveyor_mesh = meshes.add(long_conveyor);
    let short_conveyor_mesh = meshes.add(short_conveyor);

    let long_conveyor_material = materials.add(Color::srgb(0.3, 0.3, 0.3));
    let short_conveyor_material = materials.add(Color::srgb(0.2, 0.2, 0.2));

    // Spawn four conveyor belts.
    commands.spawn((
        RigidBody::Static,
        Collider::from(long_conveyor),
        Friction::new(1.0),
        ConveyorBelt {
            local_direction: Vec3::X,
            speed: 6.0,
        },
        Transform::from_xyz(-3.0, -0.25, 7.0)
            .with_rotation(Quat::from_rotation_z(2_f32.to_radians())),
        Mesh3d(long_conveyor_mesh.clone()),
        MeshMaterial3d(long_conveyor_material.clone()),
    ));
    commands.spawn((
        RigidBody::Static,
        Collider::from(long_conveyor),
        Friction::new(1.0),
        ConveyorBelt {
            local_direction: Vec3::NEG_X,
            speed: 6.0,
        },
        Transform::from_xyz(3.0, -0.25, -7.0)
            .with_rotation(Quat::from_rotation_z(-2_f32.to_radians())),
        Mesh3d(long_conveyor_mesh),
        MeshMaterial3d(long_conveyor_material.clone()),
    ));
    commands.spawn((
        RigidBody::Static,
        Collider::from(short_conveyor),
        Friction::new(1.0),
        ConveyorBelt {
            local_direction: Vec3::X,
            speed: 3.0,
        },
        Transform::from_xyz(9.0, -0.25, 3.0).with_rotation(
            Quat::from_rotation_y(90_f32.to_radians()) * Quat::from_rotation_z(2_f32.to_radians()),
        ),
        Mesh3d(short_conveyor_mesh.clone()),
        MeshMaterial3d(short_conveyor_material.clone()),
    ));
    commands.spawn((
        RigidBody::Static,
        Collider::from(short_conveyor),
        Friction::new(1.0),
        ConveyorBelt {
            local_direction: Vec3::NEG_X,
            speed: 3.0,
        },
        Transform::from_xyz(-9.0, -0.25, -3.0).with_rotation(
            Quat::from_rotation_y(90_f32.to_radians()) * Quat::from_rotation_z(-2_f32.to_radians()),
        ),
        Mesh3d(short_conveyor_mesh),
        MeshMaterial3d(short_conveyor_material),
    ));

    // Spawn cube stacks on top of one of the conveyor belts.
    let cuboid_mesh = meshes.add(Cuboid::default());
    let cuboid_material = materials.add(Color::srgb(0.2, 0.7, 0.9));
    for x in -2..2 {
        for y in 0..3 {
            for z in -2..2 {
                let position = Vec3::new(x as f32 + 10.0, y as f32 + 1.0, z as f32);
                commands.spawn((
                    RigidBody::Dynamic,
                    // This small margin just helps prevent hitting internal edges
                    // while sliding from one conveyor to another.
                    CollisionMargin(0.01),
                    Collider::cuboid(0.98, 0.98, 0.98),
                    Transform::from_translation(position),
                    Mesh3d(cuboid_mesh.clone()),
                    MeshMaterial3d(cuboid_material.clone()),
                ));
            }
        }
    }

    // Directional light
    commands.spawn((
        DirectionalLight {
            illuminance: 5000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::default().looking_at(Vec3::new(-1.0, -2.5, -1.5), Vec3::Y),
    ));

    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_translation(Vec3::new(20.0, 10.0, 20.0)).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}
