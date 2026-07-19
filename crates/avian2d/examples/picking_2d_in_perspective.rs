use avian2d::prelude::*;
use bevy::{camera::ScalingMode, prelude::*};
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
        .insert_resource(Gravity(Vec2::NEG_Y))
        .add_systems(Startup, setup)
        .add_systems(Update, control_camera_and_plane)
        .run();
}

#[derive(Component)]
#[component(immutable)]
struct PickingPlane;

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

    // For example purposes, define a custom picking plane that can be moved with the E and Q keys.
    // Using a picking plane like this is not required, but the Z coordinates of 2D colliders
    // should match `PhysicsPickingSettings::z_plane` for them to be picked at the correct XY coordinates.
    commands
        .spawn((PickingPlane, Visibility::Visible, Transform::default()))
        .with_children(|child_spawner_commands| {
            let square_sprite = Sprite {
                color: Color::srgb(0.7, 0.7, 0.8),
                custom_size: Some(Vec2::splat(0.05)),
                ..default()
            };

            // Ceiling
            child_spawner_commands.spawn((
                square_sprite.clone(),
                Transform::from_xyz(0.0, 0.05 * 6.0, 0.0).with_scale(Vec3::new(20.0, 1.0, 1.0)),
                RigidBody::Static,
                Collider::rectangle(0.05, 0.05),
            ));
            // Floor
            child_spawner_commands.spawn((
                square_sprite.clone(),
                Transform::from_xyz(0.0, -0.05 * 6.0, 0.0).with_scale(Vec3::new(20.0, 1.0, 1.0)),
                RigidBody::Static,
                Collider::rectangle(0.05, 0.05),
            ));
            // Left wall
            child_spawner_commands.spawn((
                square_sprite.clone(),
                Transform::from_xyz(-0.05 * 9.5, 0.0, 0.0).with_scale(Vec3::new(1.0, 11.0, 1.0)),
                RigidBody::Static,
                Collider::rectangle(0.05, 0.05),
            ));
            // Right wall
            child_spawner_commands.spawn((
                square_sprite,
                Transform::from_xyz(0.05 * 9.5, 0.0, 0.0).with_scale(Vec3::new(1.0, 11.0, 1.0)),
                RigidBody::Static,
                Collider::rectangle(0.05, 0.05),
            ));

            for flip in [-1.0, 1.0] {
                // Static anchor for the churner
                let velocity_anchor = child_spawner_commands
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
                let velocity_wheel = child_spawner_commands
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
                // Default anchors are at body centers
                child_spawner_commands.spawn((RevoluteJoint::new(velocity_anchor, velocity_wheel)
                    .with_motor(AngularMotor {
                        target_velocity: 5.0 * flip,
                        max_torque: 1000.0,
                        motor_model: MotorModel::AccelerationBased {
                            stiffness: 0.0,
                            damping: 1.0,
                        },
                        ..default()
                    }),));
            }

            let circle = Circle::new(0.0075);
            let collider = circle.collider();
            let mesh = meshes.add(circle);

            let ball_color = Color::srgb(0.29, 0.33, 0.64);

            for x in -12i32..=12 {
                for y in -1_i32..=8 {
                    child_spawner_commands
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
        });
}

// TODO: if anyone would like to improve this example, a freecam could be fun to mess around with :D
fn control_camera_and_plane(
    mut camera: Single<(&mut Transform, &mut Projection), With<Camera>>,
    mut picking_plane: Single<&mut Transform, (With<PickingPlane>, Without<Camera>)>,
    mut physics_picking_settings: ResMut<PhysicsPickingSettings>,
    input: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
) {
    const CAMERA_MOVE_SCALE: f32 = 1.0;
    const CAMERA_ROTATE_SCALE: f32 = 1.0;
    const PICKING_MOVE_SCALE: f32 = 1.0;

    let mut linear_delta = Vec3::ZERO;
    if input.pressed(KeyCode::KeyW) {
        linear_delta.z -= CAMERA_MOVE_SCALE;
    }
    if input.pressed(KeyCode::KeyS) {
        linear_delta.z += CAMERA_MOVE_SCALE;
    }
    if input.pressed(KeyCode::KeyA) {
        linear_delta.x -= CAMERA_MOVE_SCALE;
    }
    if input.pressed(KeyCode::KeyD) {
        linear_delta.x += CAMERA_MOVE_SCALE;
    }
    if input.pressed(KeyCode::ShiftLeft) {
        linear_delta.y -= CAMERA_MOVE_SCALE;
    }
    if input.pressed(KeyCode::Space) {
        linear_delta.y += CAMERA_MOVE_SCALE;
    }
    let camera_rotation = camera.0.rotation;
    camera.0.translation += camera_rotation * (time.delta_secs() * linear_delta);

    let mut angular_delta = 0.0;
    if input.pressed(KeyCode::ArrowRight) {
        angular_delta -= CAMERA_ROTATE_SCALE;
    }
    if input.pressed(KeyCode::ArrowLeft) {
        angular_delta += CAMERA_ROTATE_SCALE;
    }
    camera.0.rotate_y(time.delta_secs() * angular_delta);

    let mut picking_plane_delta = 0.0;
    if input.pressed(KeyCode::KeyE) {
        picking_plane_delta += PICKING_MOVE_SCALE
    }
    if input.pressed(KeyCode::KeyQ) {
        picking_plane_delta -= PICKING_MOVE_SCALE;
    }
    picking_plane_delta *= time.delta_secs();

    physics_picking_settings.z_plane += picking_plane_delta;
    picking_plane.translation.z += picking_plane_delta;

    if input.just_pressed(KeyCode::KeyR) {
        *camera.1 = if let Projection::Perspective(_) = *camera.1 {
            Projection::Orthographic(OrthographicProjection {
                scaling_mode: ScalingMode::AutoMin {
                    min_width: 1.0,
                    min_height: 1.0,
                },
                ..OrthographicProjection::default_3d()
            })
        } else {
            Projection::Perspective(PerspectiveProjection::default())
        }
    }
}
