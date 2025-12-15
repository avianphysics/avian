#![allow(clippy::unnecessary_cast)]

use avian3d::{math::*, prelude::*};
use bevy::{color::palettes::css::RED, light::NotShadowReceiver, prelude::*};
use examples_common_3d::ExampleCommonPlugin;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            ExampleCommonPlugin,
            PhysicsPlugins::default(),
        ))
        .insert_resource(ClearColor(Color::srgb(0.05, 0.05, 0.1)))
        .add_systems(Startup, setup)
        .add_systems(Update, (movement, reset_colors, raycast).chain())
        .run();
}

/// The acceleration used for movement.
#[derive(Component)]
struct MovementAcceleration(Scalar);

#[derive(Component)]
struct RayIndicator;

/// If to be ignored by raycast
#[derive(Component)]
struct OutOfGlass(bool);

const CUBE_COLOR: Color = Color::srgba(0.2, 0.7, 0.9, 1.0);
const CUBE_COLOR_GLASS: Color = Color::srgba(0.2, 0.7, 0.9, 0.5);

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let cube_mesh = meshes.add(Cuboid::default());

    // Ground
    commands.spawn((
        Mesh3d(cube_mesh.clone()),
        MeshMaterial3d(materials.add(Color::srgb(0.7, 0.7, 0.8))),
        Transform::from_xyz(0.0, -2.0, 0.0).with_scale(Vec3::new(100.0, 1.0, 100.0)),
        RigidBody::Static,
        Collider::cuboid(1.0, 1.0, 1.0),
    ));

    let cube_size = 2.0;

    // Spawn cube stacks
    for x in -1..2 {
        for y in -1..2 {
            for z in -1..2 {
                let position = Vec3::new(x as f32, y as f32 + 5.0, z as f32) * (cube_size + 0.05);
                let material: StandardMaterial = if x == -1 {
                    CUBE_COLOR_GLASS.into()
                } else {
                    CUBE_COLOR.into()
                };
                commands.spawn((
                    Mesh3d(cube_mesh.clone()),
                    MeshMaterial3d(materials.add(material.clone())),
                    Transform::from_translation(position).with_scale(Vec3::splat(cube_size as f32)),
                    RigidBody::Dynamic,
                    Collider::cuboid(1.0, 1.0, 1.0),
                    MovementAcceleration(10.0),
                    OutOfGlass(x == -1),
                ));
            }
        }
    }

    // raycast indicator
    commands.spawn((
        Mesh3d(cube_mesh.clone()),
        MeshMaterial3d(materials.add(Color::srgb(1.0, 0.0, 0.0))),
        Transform::from_xyz(-500.0, 2.0, 0.0).with_scale(Vec3::new(1000.0, 0.1, 0.1)),
        RayIndicator,
        NotShadowReceiver,
    ));

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
        Transform::from_translation(Vec3::new(0.0, 12.0, 40.0)).looking_at(Vec3::Y * 5.0, Vec3::Y),
    ));
}

fn movement(
    time: Res<Time>,
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mut query: Query<(&MovementAcceleration, &mut LinearVelocity)>,
) {
    // Precision is adjusted so that the example works with
    // both the `f32` and `f64` features. Otherwise you don't need this.
    let delta_time = time.delta_secs_f64().adjust_precision();

    for (movement_acceleration, mut linear_velocity) in &mut query {
        let up = keyboard_input.any_pressed([KeyCode::KeyW, KeyCode::ArrowUp]);
        let down = keyboard_input.any_pressed([KeyCode::KeyS, KeyCode::ArrowDown]);
        let left = keyboard_input.any_pressed([KeyCode::KeyA, KeyCode::ArrowLeft]);
        let right = keyboard_input.any_pressed([KeyCode::KeyD, KeyCode::ArrowRight]);

        let horizontal = right as i8 - left as i8;
        let vertical = down as i8 - up as i8;
        let direction =
            Vector::new(horizontal as Scalar, 0.0, vertical as Scalar).normalize_or_zero();

        // Move in input direction
        if direction != Vector::ZERO {
            linear_velocity.x += direction.x * movement_acceleration.0 * delta_time;
            linear_velocity.z += direction.z * movement_acceleration.0 * delta_time;
        }
    }
}

fn reset_colors(
    mut materials: ResMut<Assets<StandardMaterial>>,
    cubes: Query<(&MeshMaterial3d<StandardMaterial>, &OutOfGlass)>,
) {
    for (material_handle, out_of_glass) in cubes.iter() {
        if let Some(material) = materials.get_mut(material_handle) {
            if out_of_glass.0 {
                material.base_color = CUBE_COLOR_GLASS;
            } else {
                material.base_color = CUBE_COLOR;
            }
        }
    }
}

fn raycast(
    query: SpatialQuery,
    mut materials: ResMut<Assets<StandardMaterial>>,
    cubes: Query<(&MeshMaterial3d<StandardMaterial>, &OutOfGlass)>,
    mut indicator_transform: Single<&mut Transform, With<RayIndicator>>,
) {
    let origin = Vector::new(-200.0, 2.0, 0.0);
    let direction = Dir3::X;
    let filter = SpatialQueryFilter::default();

    if let Some(ray_hit_data) =
        query.cast_ray_predicate(origin, direction, Scalar::MAX, true, &filter, &|entity| {
            // Only look at cubes not made out of glass.
            if let Ok((_, out_of_glass)) = cubes.get(entity) {
                return !out_of_glass.0;
            }
            true
        })
    {
        // Set the color of the hit object to red.
        if let Ok((material_handle, _)) = cubes.get(ray_hit_data.entity)
            && let Some(material) = materials.get_mut(material_handle)
        {
            material.base_color = RED.into();
        }

        // Set the length of the ray indicator to look more like a laser,
        let contact_point = (origin + direction.adjust_precision() * ray_hit_data.distance).x;
        let target_scale = 1000.0 + contact_point * 2.0;
        indicator_transform.scale.x = target_scale as f32;
    } else {
        indicator_transform.scale.x = 2000.0;
    }
}
