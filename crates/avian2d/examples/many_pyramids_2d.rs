use avian2d::prelude::*;
use bevy::{camera::ScalingMode, prelude::*};
use examples_common_2d::ExampleCommonPlugin;

fn main() {
    let mut app = App::new();

    app.add_plugins((
        DefaultPlugins,
        PhysicsPlugins::default(),
        ExampleCommonPlugin,
    ));

    app.add_systems(Startup, setup);

    app.run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    let base_count = 10;
    let h = 0.5;
    let row_count = 10;
    let column_count = 10;

    let ground_delta_y = 2.0 * h * (base_count + 1) as f32;
    let ground_width = 2.0 * h * column_count as f32 * (base_count + 1) as f32;

    // Ground
    let ground_shape = Rectangle::new(ground_width, 0.01);
    let collider = Collider::from(ground_shape);
    let mesh = meshes.add(ground_shape);
    let material = materials.add(Color::srgb(0.3, 0.5, 0.3));
    for i in 0..row_count {
        commands.spawn((
            RigidBody::Static,
            collider.clone(),
            Transform::from_xyz(0.0, i as f32 * ground_delta_y, 0.0),
            Mesh2d(mesh.clone()),
            MeshMaterial2d(material.clone()),
        ));
    }

    let base_width = 2.0 * h * base_count as f32;

    for i in 0..row_count {
        let base_y = i as f32 * ground_delta_y;
        for j in 0..column_count {
            let center_x = -ground_width / 2.0 + j as f32 * (base_width + 2.0 * h) + h;
            spawn_small_pyramid(
                &mut commands,
                &mut meshes,
                &mut materials,
                base_count,
                h,
                center_x,
                base_y,
            );
        }
    }

    commands.spawn((
        Camera2d,
        Projection::Orthographic(OrthographicProjection {
            scaling_mode: ScalingMode::FixedVertical {
                viewport_height: row_count as f32 * ground_delta_y * 1.1,
            },
            ..OrthographicProjection::default_2d()
        }),
        Transform::from_xyz(0.0, row_count as f32 * ground_delta_y / 2.0, 0.0),
    ));
}

/// Spawns a small pyramid structure at the specified position.
fn spawn_small_pyramid(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<ColorMaterial>>,
    base_count: usize,
    h: f32,
    center_x: f32,
    base_y: f32,
) {
    let box_size = 2.0 * h;
    let rectangle = Rectangle::from_length(box_size);
    let collider = Collider::from(rectangle);
    let mesh = meshes.add(rectangle);
    let material = materials.add(Color::srgb(0.2, 0.7, 0.9));

    for i in 0..base_count {
        let y = (2 * i + 1) as f32 * h + base_y;

        for j in i..base_count {
            let x = (i + 1) as f32 * h + 2.0 * (j - i) as f32 * h + center_x - 0.5;

            commands.spawn((
                RigidBody::Dynamic,
                collider.clone(),
                Transform::from_xyz(x, y, 0.0),
                Mesh2d(mesh.clone()),
                MeshMaterial2d(material.clone()),
            ));
        }
    }
}
