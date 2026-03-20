use avian2d::{math::*, prelude::*};
use bevy::prelude::*;
use examples_common_2d::ExampleCommonPlugin;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            ExampleCommonPlugin,
            PhysicsPlugins::default(),
            PhysicsPickingPlugin,
        ))
        .insert_resource(ClearColor(Color::srgb(0.05, 0.05, 0.1)))
        .insert_resource(Gravity(Vector::NEG_Y))
        .add_systems(Startup, setup)
        .add_systems(Update, move_camera)
        .run();
}

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<ColorMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    // 2D camera with a perspective projection; completely unheard of!
    commands.spawn((
        Camera2d,
        Projection::Perspective(PerspectiveProjection::default()),
        Transform::from_xyz(0.0, 0.0, 1.0), // We have to start slightly away from the Z axis, since now everything is drawing at Z = 0
    ));

    let square_sprite = Sprite {
        color: Color::srgb(0.7, 0.7, 0.8),
        custom_size: Some(Vec2::splat(0.05)),
        ..default()
    };

    // Ceiling
    commands.spawn((
        square_sprite.clone(),
        Transform::from_xyz(0.0, 0.05 * 6.0, 0.0).with_scale(Vec3::new(20.0, 1.0, 1.0)),
        RigidBody::Static,
        Collider::rectangle(0.05, 0.05),
    ));
    // Floor
    commands.spawn((
        square_sprite.clone(),
        Transform::from_xyz(0.0, -0.05 * 6.0, 0.0).with_scale(Vec3::new(20.0, 1.0, 1.0)),
        RigidBody::Static,
        Collider::rectangle(0.05, 0.05),
    ));
    // Left wall
    commands.spawn((
        square_sprite.clone(),
        Transform::from_xyz(-0.05 * 9.5, 0.0, 0.0).with_scale(Vec3::new(1.0, 11.0, 1.0)),
        RigidBody::Static,
        Collider::rectangle(0.05, 0.05),
    ));
    // Right wall
    commands.spawn((
        square_sprite,
        Transform::from_xyz(0.05 * 9.5, 0.0, 0.0).with_scale(Vec3::new(1.0, 11.0, 1.0)),
        RigidBody::Static,
        Collider::rectangle(0.05, 0.05),
    ));

    for flip in [-1.0, 1.0] {
        // Static anchor for the churner
        let velocity_anchor = commands
            .spawn((
                Sprite {
                    color: Color::srgb(0.5, 0.5, 0.5),
                    custom_size: Some(Vec2::splat(0.01)),
                    ..default()
                },
                Transform::from_xyz(-0.3 * flip, -0.15, 0.0),
                RigidBody::Static,
            ))
            .id();

        // Spinning churner
        let velocity_wheel = commands
            .spawn((
                Sprite {
                    color: Color::srgb(0.9, 0.3, 0.3),
                    custom_size: Some(vec2(0.24, 0.02)),
                    ..default()
                },
                Transform::from_xyz(-0.3 * flip, -0.15, 0.0),
                RigidBody::Dynamic,
                Mass(1.0),
                AngularInertia(1.0),
                SleepingDisabled, // Prevent sleeping so motor can always control it
                Collider::rectangle(0.24, 0.02),
            ))
            .id();

        // Revolute joint with velocity-controlled motor
        // Default anchors are at body centers (Vector::ZERO)
        commands.spawn((
            RevoluteJoint::new(velocity_anchor, velocity_wheel).with_motor(AngularMotor {
                target_velocity: 5.0 * flip.adjust_precision(),
                max_torque: 1000.0,
                motor_model: MotorModel::AccelerationBased {
                    stiffness: 0.0,
                    damping: 1.0,
                },
                ..default()
            }),
        ));
    }

    let circle = Circle::new(0.0075);
    let collider = circle.collider();
    let mesh = meshes.add(circle);

    let ball_color = Color::srgb(0.29, 0.33, 0.64);

    for x in -12i32..=12 {
        for y in -1_i32..=8 {
            commands
                .spawn((
                    Mesh2d(mesh.clone()),
                    MeshMaterial2d(materials.add(ball_color)),
                    Transform::from_xyz(x as f32 * 0.025, y as f32 * 0.025, 0.0),
                    collider.clone(),
                    RigidBody::Dynamic,
                    Friction::new(0.1),
                ))
                .observe(
                    |over: On<Pointer<Over>>,
                     mut materials: ResMut<Assets<ColorMaterial>>,
                     balls: Query<&MeshMaterial2d<ColorMaterial>>| {
                        materials
                            .get_mut(balls.get(over.entity).unwrap())
                            .unwrap()
                            .color = Color::WHITE;
                    },
                )
                .observe(
                    move |out: On<Pointer<Out>>,
                          mut materials: ResMut<Assets<ColorMaterial>>,
                          balls: Query<&MeshMaterial2d<ColorMaterial>>| {
                        materials
                            .get_mut(balls.get(out.entity).unwrap())
                            .unwrap()
                            .color = ball_color;
                    },
                );
        }
    }
}

// TODO: if anyone would like to improve this example, a freecam could be fun to mess around with :D
fn move_camera(
    mut camera: Single<&mut Transform, With<Camera>>,
    input: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
) {
    const MOVE_SCALE: f32 = 1.0;

    let mut delta = Vec3::ZERO;
    if input.pressed(KeyCode::KeyW) {
        delta.z -= time.delta_secs() * MOVE_SCALE;
    }
    if input.pressed(KeyCode::KeyS) {
        delta.z += time.delta_secs() * MOVE_SCALE;
    }
    if input.pressed(KeyCode::KeyA) {
        delta.x -= time.delta_secs() * MOVE_SCALE;
    }
    if input.pressed(KeyCode::KeyD) {
        delta.x += time.delta_secs() * MOVE_SCALE;
    }
    if input.pressed(KeyCode::ShiftLeft) {
        delta.y -= time.delta_secs() * MOVE_SCALE;
    }
    if input.pressed(KeyCode::Space) {
        delta.y += time.delta_secs() * MOVE_SCALE;
    }
    camera.translation += delta;
}
