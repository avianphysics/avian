use avian3d::prelude::*;
use bevy::prelude::*;

use super::Benchmark3dPlugins;

pub fn create_bench(base_count: usize, row_count: usize, column_count: usize) -> App {
    let mut app = App::new();
    app.add_plugins((Benchmark3dPlugins, PhysicsPlugins::default()));
    app.add_systems(Startup, move |commands: Commands| {
        setup(commands, base_count, row_count, column_count);
    });
    app
}

fn setup(mut commands: Commands, base_count: usize, row_count: usize, column_count: usize) {
    let h = 0.5;
    let ground_delta_y = 2.0 * h * (base_count + 1) as f32;
    let ground_width = 2.0 * h * column_count as f32 * (base_count + 1) as f32;

    // Ground
    for i in 0..row_count {
        commands.spawn((
            RigidBody::Static,
            Collider::cuboid(ground_width, 0.01, ground_width),
            Transform::from_xyz(0.0, i as f32 * ground_delta_y, 0.0),
        ));
    }

    let base_width = 2.0 * h * base_count as f32;

    for i in 0..row_count {
        let base_y = i as f32 * ground_delta_y;
        for j in 0..column_count {
            let center_x = -ground_width / 2.0 + j as f32 * (base_width + 2.0 * h) + h;
            spawn_small_pyramid(&mut commands, base_count, h, center_x, base_y);
        }
    }
}

/// Spawns a small pyramid structure at the specified position.
fn spawn_small_pyramid(
    commands: &mut Commands,
    base_count: usize,
    h: f32,
    center_x: f32,
    base_y: f32,
) {
    let box_size = 2.0 * h;
    let collider = Collider::cuboid(box_size, box_size, box_size);

    for i in 0..base_count {
        let y = (2 * i + 1) as f32 * h + base_y;

        for j in i..base_count {
            let x = (i + 1) as f32 * h + 2.0 * (j - i) as f32 * h + center_x - 0.5;

            commands.spawn((
                RigidBody::Dynamic,
                collider.clone(),
                Transform::from_xyz(x, y, 0.0),
            ));
        }
    }
}
