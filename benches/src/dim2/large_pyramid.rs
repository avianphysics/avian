use avian2d::{math::Scalar, prelude::*};
use bevy::prelude::*;

use super::Benchmark2dPlugins;

pub fn create_bench(base_count: usize) -> App {
    let mut app = App::new();
    app.add_plugins((Benchmark2dPlugins, PhysicsPlugins::default()));
    app.add_systems(Startup, move |commands: Commands| {
        setup(commands, base_count)
    });
    app
}

fn setup(mut commands: Commands, base_count: usize) {
    // Ground
    commands.spawn((
        RigidBody::Static,
        Collider::rectangle(800.0, 40.0),
        Transform::from_xyz(0.0, -20.0, 0.0),
    ));

    let h = 0.5;
    let box_size = 2.0 * h;
    let collider = Collider::rectangle(box_size as Scalar, box_size as Scalar);
    let shift = h;
    for i in 0..base_count {
        let y = (2.0 * i as f32 + 1.0) * shift * 0.99;

        for j in i..base_count {
            let x = (i as f32 + 1.0) * shift + 2.0 * (j - i) as f32 * shift - h * base_count as f32;

            commands.spawn((
                RigidBody::Dynamic,
                collider.clone(),
                Transform::from_xyz(x, y, 0.0),
            ));
        }
    }
}
