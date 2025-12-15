use core::f32::consts::FRAC_PI_2;
use core::time::Duration;

use approx::assert_relative_eq;
use bevy::{mesh::MeshPlugin, prelude::*, time::TimeUpdateStrategy};
use bevy_math::FloatPow;

use crate::prelude::*;

const TIMESTEP: f32 = 1.0 / 64.0;

fn create_app() -> App {
    let mut app = App::new();
    app.add_plugins((
        MinimalPlugins,
        PhysicsPlugins::default(),
        TransformPlugin,
        #[cfg(feature = "bevy_scene")]
        AssetPlugin::default(),
        #[cfg(feature = "bevy_scene")]
        bevy::scene::ScenePlugin,
        MeshPlugin,
    ));

    // Use 20 substeps.
    app.insert_resource(SubstepCount(20));

    // Use a gravity of 9.81 m/s².
    app.insert_resource(Gravity(Vector::NEG_Y * 9.81));

    // Configure the timestep.
    app.insert_resource(Time::<Fixed>::from_duration(Duration::from_secs_f32(
        TIMESTEP,
    )));
    app.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f32(
        TIMESTEP,
    )));

    app
}

fn spawn_body(app: &mut App, mass: f32, angular_inertia: f32) -> EntityWorldMut<'_> {
    app.world_mut().spawn((
        RigidBody::Dynamic,
        Mass(mass),
        #[cfg(feature = "2d")]
        AngularInertia(angular_inertia),
        #[cfg(feature = "3d")]
        AngularInertia::new(Vec3::splat(angular_inertia)),
    ))
}

#[test]
fn apply_force() {
    let mut app = create_app();
    app.finish();

    // Continuously apply a force of 9.81 N in the positive Y direction.
    app.add_systems(FixedUpdate, |mut query: Query<Forces>| {
        for mut forces in query.iter_mut() {
            forces.apply_force(Vector::Y * 9.81);
        }
    });

    let body_one_kg = spawn_body(&mut app, 1.0, 1.0).id();
    let body_half_kg = spawn_body(&mut app, 0.5, 0.5).id();

    // Step by TIMESTEP seconds for 5 seconds.
    let duration = 5.0;
    let steps = (duration / TIMESTEP) as usize;

    // Initialize the app.
    app.update();

    for _ in 0..steps {
        app.update();
    }

    // Get the body after the simulation.
    let body_one_kg_ref = app.world().entity(body_one_kg);
    let body_half_kg_ref = app.world().entity(body_half_kg);

    // The 1 kg body should not have moved.
    assert_relative_eq!(
        body_one_kg_ref.get::<Transform>().unwrap().translation.y,
        0.0,
        epsilon = 1e-6
    );

    // The 0.5 kg body should have moved by 1/2 * a * Δt².
    assert_relative_eq!(
        body_half_kg_ref.get::<Transform>().unwrap().translation.y,
        0.5 * 9.81 * duration.squared(),
        epsilon = 0.05
    );
}

#[test]
fn apply_local_force() {
    let mut app = create_app();
    app.finish();

    // Continuously apply a force of 9.81 N in the local positive Y direction.
    app.add_systems(FixedUpdate, |mut query: Query<Forces>| {
        for mut forces in query.iter_mut() {
            forces.apply_local_force(Vector::Y * 9.81);
        }
    });

    // Rotate the 0.5 kg body by 90 degrees about the Z axis.
    let body_one_kg = spawn_body(&mut app, 1.0, 1.0).id();
    let body_half_kg = spawn_body(&mut app, 0.5, 0.5)
        .insert(Transform::from_rotation(Quat::from_rotation_z(FRAC_PI_2)))
        .id();

    // Step by TIMESTEP seconds for 5 seconds.
    let duration = 5.0;
    let steps = (duration / TIMESTEP) as usize;

    // Initialize the app.
    app.update();

    for _ in 0..steps {
        app.update();
    }

    // Get the body after the simulation.
    let body_one_kg_ref = app.world().entity(body_one_kg);
    let body_half_kg_ref = app.world().entity(body_half_kg);

    // The 1 kg body should not have moved.
    assert_relative_eq!(
        body_one_kg_ref.get::<Transform>().unwrap().translation.y,
        0.0,
        epsilon = 1e-6
    );

    // The 0.5 kg body should have moved by 1/2 * a * Δt² in the negative Y direction,
    // and by a ⋅ Δt² in the negative X direction.
    assert_relative_eq!(
        body_half_kg_ref.get::<Transform>().unwrap().translation,
        vec3(
            -9.81 * duration.squared(),
            -0.5 * 9.81 * duration.squared(),
            0.0
        ),
        epsilon = 0.05
    );
}

#[test]
fn apply_linear_impulse() {
    let mut app = create_app();
    app.finish();

    // Continuously apply a linear impulse of 9.81 kg⋅m/s multiplied by the timestep in the positive Y direction.
    app.add_systems(FixedUpdate, |mut query: Query<Forces>| {
        for mut forces in query.iter_mut() {
            forces.apply_linear_impulse(Vector::Y * 9.81 * TIMESTEP as Scalar);
        }
    });

    let body_one_kg = spawn_body(&mut app, 1.0, 1.0).id();
    let body_half_kg = spawn_body(&mut app, 0.5, 0.5).id();

    // Step by TIMESTEP seconds for 5 seconds.
    let duration = 5.0;
    let steps = (duration / TIMESTEP) as usize;

    // Initialize the app.
    app.update();

    for _ in 0..steps {
        app.update();
    }

    // Get the body after the simulation.
    let body_one_kg_ref = app.world().entity(body_one_kg);
    let body_half_kg_ref = app.world().entity(body_half_kg);

    // The 1 kg body should not have moved.
    assert_relative_eq!(
        body_one_kg_ref.get::<Transform>().unwrap().translation.y,
        0.0,
        epsilon = 0.5
    );

    // The 0.5 kg body should have moved by 1/2 * a * Δt².
    assert_relative_eq!(
        body_half_kg_ref.get::<Transform>().unwrap().translation.y,
        0.5 * 9.81 * duration.squared(),
        epsilon = 1.0
    );
}

#[test]
fn apply_local_linear_impulse() {
    let mut app = create_app();
    app.finish();

    // Continuously apply a linear impulse of 9.81 m/s² multiplied by the timestep in the local positive Y direction.
    app.add_systems(FixedUpdate, |mut query: Query<Forces>| {
        for mut forces in query.iter_mut() {
            forces.apply_local_linear_impulse(Vector::Y * 9.81 * TIMESTEP as Scalar);
        }
    });

    // Rotate the 0.5 kg body by 90 degrees about the Z axis.
    let body_one_kg = spawn_body(&mut app, 1.0, 1.0).id();
    let body_half_kg = spawn_body(&mut app, 0.5, 0.5)
        .insert(Transform::from_rotation(Quat::from_rotation_z(FRAC_PI_2)))
        .id();

    // Step by TIMESTEP seconds for 5 seconds.
    let duration = 5.0;
    let steps = (duration / TIMESTEP) as usize;

    // Initialize the app.
    app.update();

    for _ in 0..steps {
        app.update();
    }

    // Get the body after the simulation.
    let body_one_kg_ref = app.world().entity(body_one_kg);
    let body_half_kg_ref = app.world().entity(body_half_kg);

    // The 1 kg body should not have moved.
    assert_relative_eq!(
        body_one_kg_ref.get::<Transform>().unwrap().translation.y,
        0.0,
        epsilon = 0.5
    );

    // The 0.5 kg body should have moved by 1/2 * a * Δt² in the negative Y direction,
    // and by a ⋅ Δt² in the negative X direction.
    assert_relative_eq!(
        body_half_kg_ref.get::<Transform>().unwrap().translation,
        vec3(
            -9.81 * duration.squared(),
            -0.5 * 9.81 * duration.squared(),
            0.0
        ),
        epsilon = 2.0
    );
}

#[test]
fn apply_linear_acceleration() {
    let mut app = create_app();
    app.finish();

    // Continuously apply a linear acceleration of 9.81 m/s² in the positive Y direction.
    app.add_systems(FixedUpdate, |mut query: Query<Forces>| {
        for mut forces in query.iter_mut() {
            forces.apply_linear_acceleration(Vector::Y * 9.81);
        }
    });

    let body_one_kg = spawn_body(&mut app, 1.0, 1.0).id();
    let body_half_kg = spawn_body(&mut app, 0.5, 0.5).id();

    // Step by TIMESTEP seconds for 5 seconds.
    let duration = 5.0;
    let steps = (duration / TIMESTEP) as usize;

    // Initialize the app.
    app.update();

    for _ in 0..steps {
        app.update();
    }

    // Get the body after the simulation.
    let body_one_kg_ref = app.world().entity(body_one_kg);
    let body_half_kg_ref = app.world().entity(body_half_kg);

    // The 1 kg body should not have moved.
    assert_relative_eq!(
        body_one_kg_ref.get::<Transform>().unwrap().translation.y,
        0.0,
        epsilon = 1e-6
    );

    // The 0.5 kg body should not have moved.
    assert_relative_eq!(
        body_half_kg_ref.get::<Transform>().unwrap().translation.y,
        0.0,
        epsilon = 1e-6
    );
}

#[test]
fn apply_local_linear_acceleration() {
    let mut app = create_app();
    app.finish();

    // Continuously apply a linear acceleration of 9.81 m/s² in the local positive Y direction.
    app.add_systems(FixedUpdate, |mut query: Query<Forces>| {
        for mut forces in query.iter_mut() {
            forces.apply_local_linear_acceleration(Vector::Y * 9.81);
        }
    });

    // Rotate the 0.5 kg body by 90 degrees about the Z axis.
    let body_one_kg = spawn_body(&mut app, 1.0, 1.0).id();
    let body_half_kg = spawn_body(&mut app, 0.5, 0.5)
        .insert(Transform::from_rotation(Quat::from_rotation_z(FRAC_PI_2)))
        .id();

    // Step by TIMESTEP seconds for 5 seconds.
    let duration = 5.0;
    let steps = (duration / TIMESTEP) as usize;

    // Initialize the app.
    app.update();

    for _ in 0..steps {
        app.update();
    }

    // Get the body after the simulation.
    let body_one_kg_ref = app.world().entity(body_one_kg);
    let body_half_kg_ref = app.world().entity(body_half_kg);

    // The 1 kg body should not have moved.
    assert_relative_eq!(
        body_one_kg_ref.get::<Transform>().unwrap().translation.y,
        0.0,
        epsilon = 1e-6
    );

    // The 0.5 kg body should have moved by 1/2 * a * Δt² in both the negative X and Y directions.
    assert_relative_eq!(
        body_half_kg_ref.get::<Transform>().unwrap().translation,
        vec3(
            -0.5 * 9.81 * duration.squared(),
            -0.5 * 9.81 * duration.squared(),
            0.0
        ),
        epsilon = 0.05
    );
}

#[test]
fn apply_torque() {
    let mut app = create_app();
    app.finish();

    // Continuously apply a torque of 1.5 N⋅m about the Z axis.
    app.add_systems(FixedUpdate, |mut query: Query<Forces>| {
        for mut forces in query.iter_mut() {
            #[cfg(feature = "2d")]
            forces.apply_torque(1.5);
            #[cfg(feature = "3d")]
            forces.apply_torque(Vector::Z * 1.5);
        }
    });

    let body_one_kg = spawn_body(&mut app, 1.0, 1.0).id();
    let body_half_kg = spawn_body(&mut app, 0.5, 0.5).id();

    // Step by TIMESTEP seconds for 5 seconds.
    let duration = 1.5;
    let steps = (duration / TIMESTEP) as usize;

    // Initialize the app.
    app.update();

    for _ in 0..steps {
        app.update();
    }

    // Get the body after the simulation.
    let body_one_kg_ref = app.world().entity(body_one_kg);
    let body_half_kg_ref = app.world().entity(body_half_kg);

    // The 1 kg body should have rotated by 1/2 * a * Δt² radians about the Z axis.
    let diff = body_one_kg_ref
        .get::<Transform>()
        .unwrap()
        .rotation
        .angle_between(Quat::from_rotation_z(0.5 * 1.5 * duration.squared()));
    assert!(diff < 0.1, "angle difference {diff} is not less than 0.1");

    // The 0.5 kg body should have rotated by a ⋅ Δt² radians about the Z axis.
    let diff = body_half_kg_ref
        .get::<Transform>()
        .unwrap()
        .rotation
        .angle_between(Quat::from_rotation_z(1.5 * duration.squared()));
    assert!(diff < 0.15, "angle difference {diff} is not less than 0.15");
}

#[test]
#[cfg(feature = "3d")]
fn apply_local_torque() {
    let mut app = create_app();
    app.finish();

    // Continuously apply a torque of 1.5 N⋅m about the local Z axis.
    app.add_systems(FixedUpdate, |mut query: Query<Forces>| {
        for mut forces in query.iter_mut() {
            forces.apply_local_torque(Vector::Z * 1.5);
        }
    });

    // Rotate the 0.5 kg body by 90 degrees about the Y axis.
    let body_one_kg = spawn_body(&mut app, 1.0, 1.0).id();
    let body_half_kg = spawn_body(&mut app, 0.5, 0.5)
        .insert(Transform::from_rotation(Quat::from_rotation_y(FRAC_PI_2)))
        .id();

    // Step by TIMESTEP seconds for 5 seconds.
    let duration = 5.0;
    let steps = (duration / TIMESTEP) as usize;

    // Initialize the app.
    app.update();

    for _ in 0..steps {
        app.update();
    }

    // Get the body after the simulation.
    let body_one_kg_ref = app.world().entity(body_one_kg);
    let body_half_kg_ref = app.world().entity(body_half_kg);

    // The 1 kg body should have rotated by 1/2 * a * Δt² radians about the local Z axis.
    let diff = body_one_kg_ref
        .get::<Transform>()
        .unwrap()
        .rotation
        .angle_between(Quat::from_rotation_z(0.5 * 1.5 * duration.squared()));
    assert!(diff < 0.1, "angle difference {diff} is not less than 0.1");

    // The 0.5 kg body should have rotated by a ⋅ Δt² radians about the local Z axis.
    let diff = body_half_kg_ref
        .get::<Transform>()
        .unwrap()
        .rotation
        .angle_between(
            Quat::from_rotation_y(FRAC_PI_2) * Quat::from_rotation_z(1.5 * duration.squared()),
        );
    assert!(diff < 0.15, "angle difference {diff} is not less than 0.15");
}

#[test]
fn apply_angular_impulse() {
    let mut app = create_app();
    app.finish();

    // Continuously apply an angular impulse of 1.5 kg⋅m²/s multiplied by the timestep about the Z axis.
    app.add_systems(FixedUpdate, |mut query: Query<Forces>| {
        for mut forces in query.iter_mut() {
            #[cfg(feature = "2d")]
            forces.apply_angular_impulse(1.5 * TIMESTEP as Scalar);
            #[cfg(feature = "3d")]
            forces.apply_angular_impulse(Vector::Z * 1.5 * TIMESTEP as Scalar);
        }
    });

    let body_one_kg = spawn_body(&mut app, 1.0, 1.0).id();
    let body_half_kg = spawn_body(&mut app, 0.5, 0.5).id();

    // Step by TIMESTEP seconds for 5 seconds.
    let duration = 1.5;
    let steps = (duration / TIMESTEP) as usize;

    // Initialize the app.
    app.update();

    for _ in 0..steps {
        app.update();
    }

    // Get the body after the simulation.
    let body_one_kg_ref = app.world().entity(body_one_kg);
    let body_half_kg_ref = app.world().entity(body_half_kg);

    // The 1 kg body should have rotated by 1/2 * a * Δt² radians about the Z axis.
    let diff = body_one_kg_ref
        .get::<Transform>()
        .unwrap()
        .rotation
        .angle_between(Quat::from_rotation_z(0.5 * 1.5 * duration.squared()));
    assert!(diff < 0.1, "angle difference {diff} is not less than 0.1");

    // The 0.5 kg body should have rotated by a ⋅ Δt² radians about the Z axis.
    let diff = body_half_kg_ref
        .get::<Transform>()
        .unwrap()
        .rotation
        .angle_between(Quat::from_rotation_z(1.5 * duration.squared()));
    assert!(diff < 0.15, "angle difference {diff} is not less than 0.15");
}

#[test]
#[cfg(feature = "3d")]
fn apply_local_angular_impulse() {
    let mut app = create_app();
    app.finish();

    // Continuously apply an angular impulse of 1.5 kg⋅m²/s multiplied by the timestep about the local Z axis.
    app.add_systems(FixedUpdate, |mut query: Query<Forces>| {
        for mut forces in query.iter_mut() {
            forces.apply_local_angular_impulse(Vector::Z * 1.5 * TIMESTEP as Scalar);
        }
    });

    // Rotate the 0.5 kg body by 90 degrees about the Y axis.
    let body_one_kg = spawn_body(&mut app, 1.0, 1.0).id();
    let body_half_kg = spawn_body(&mut app, 0.5, 0.5)
        .insert(Transform::from_rotation(Quat::from_rotation_y(FRAC_PI_2)))
        .id();

    // Step by TIMESTEP seconds for 5 seconds.
    let duration = 5.0;
    let steps = (duration / TIMESTEP) as usize;

    // Initialize the app.
    app.update();

    for _ in 0..steps {
        app.update();
    }

    // Get the body after the simulation.
    let body_one_kg_ref = app.world().entity(body_one_kg);
    let body_half_kg_ref = app.world().entity(body_half_kg);

    // The 1 kg body should have rotated by 1/2 * a * Δt² radians about the local Z axis.
    let diff = body_one_kg_ref
        .get::<Transform>()
        .unwrap()
        .rotation
        .angle_between(Quat::from_rotation_z(0.5 * 1.5 * duration.squared()));
    assert!(diff < 0.1, "angle difference {diff} is not less than 0.1");

    // The 0.5 kg body should have rotated by a ⋅ Δt² radians about the local Z axis.
    let diff = body_half_kg_ref
        .get::<Transform>()
        .unwrap()
        .rotation
        .angle_between(
            Quat::from_rotation_y(FRAC_PI_2) * Quat::from_rotation_z(1.5 * duration.squared()),
        );
    assert!(diff < 0.15, "angle difference {diff} is not less than 0.15");
}

#[test]
fn apply_angular_acceleration() {
    let mut app = create_app();
    app.finish();

    // Continuously apply a torque of 1.5 rad/s² about the Z axis.
    app.add_systems(FixedUpdate, |mut query: Query<Forces>| {
        for mut forces in query.iter_mut() {
            #[cfg(feature = "2d")]
            forces.apply_angular_acceleration(1.5);
            #[cfg(feature = "3d")]
            forces.apply_angular_acceleration(Vector::Z * 1.5);
        }
    });

    let body_one_kg = spawn_body(&mut app, 1.0, 1.0).id();
    let body_half_kg = spawn_body(&mut app, 0.5, 0.5).id();

    // Step by TIMESTEP seconds for 5 seconds.
    let duration = 1.5;
    let steps = (duration / TIMESTEP) as usize;

    // Initialize the app.
    app.update();

    for _ in 0..steps {
        app.update();
    }

    // Get the body after the simulation.
    let body_one_kg_ref = app.world().entity(body_one_kg);
    let body_half_kg_ref = app.world().entity(body_half_kg);

    // The 1 kg body should have rotated by 1/2 * a * Δt² radians about the Z axis.
    let diff = body_one_kg_ref
        .get::<Transform>()
        .unwrap()
        .rotation
        .angle_between(Quat::from_rotation_z(0.5 * 1.5 * duration.squared()));
    assert!(diff < 0.1, "angle difference {diff} is not less than 0.1");

    // The 0.5 kg body should have rotated by 1/2 * a * Δt² radians about the Z axis.
    let diff = body_half_kg_ref
        .get::<Transform>()
        .unwrap()
        .rotation
        .angle_between(Quat::from_rotation_z(0.5 * 1.5 * duration.squared()));
    assert!(diff < 0.1, "angle difference {diff} is not less than 0.1");
}

#[test]
#[cfg(feature = "3d")]
fn apply_local_angular_acceleration() {
    let mut app = create_app();
    app.finish();

    // Continuously apply a torque of 1.5 rad/s² about the local Z axis.
    app.add_systems(FixedUpdate, |mut query: Query<Forces>| {
        for mut forces in query.iter_mut() {
            forces.apply_local_angular_acceleration(Vector::Z * 1.5);
        }
    });

    // Rotate the 0.5 kg body by 90 degrees about the Y axis.
    let body_one_kg = spawn_body(&mut app, 1.0, 1.0).id();
    let body_half_kg = spawn_body(&mut app, 0.5, 0.5)
        .insert(Transform::from_rotation(Quat::from_rotation_y(FRAC_PI_2)))
        .id();

    // Step by TIMESTEP seconds for 5 seconds.
    let duration = 5.0;
    let steps = (duration / TIMESTEP) as usize;

    // Initialize the app.
    app.update();

    for _ in 0..steps {
        app.update();
    }

    // Get the body after the simulation.
    let body_one_kg_ref = app.world().entity(body_one_kg);
    let body_half_kg_ref = app.world().entity(body_half_kg);

    // The 1 kg body should have rotated by 1/2 * a * Δt² radians about the local Z axis.
    let diff = body_one_kg_ref
        .get::<Transform>()
        .unwrap()
        .rotation
        .angle_between(Quat::from_rotation_z(0.5 * 1.5 * duration.squared()));
    assert!(diff < 0.1, "angle difference {diff} is not less than 0.1");

    // The 0.5 kg body should have rotated by 1/2 * a * Δt² radians about the local Z axis.
    let diff = body_half_kg_ref
        .get::<Transform>()
        .unwrap()
        .rotation
        .angle_between(
            Quat::from_rotation_y(FRAC_PI_2)
                * Quat::from_rotation_z(0.5 * 1.5 * duration.squared()),
        );
    assert!(diff < 0.1, "angle difference {diff} is not less than 0.1");
}
