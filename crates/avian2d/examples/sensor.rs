use avian2d::{math::*, prelude::*};
use bevy::prelude::*;
use examples_common_2d::ExampleCommonPlugin;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            ExampleCommonPlugin,
            // Add physics plugins and specify a units-per-meter scaling factor, 1 meter = 20 pixels.
            // The unit allows the engine to tune its parameters for the scale of the world, improving stability.
            PhysicsPlugins::default().with_length_unit(20.0),
        ))
        .insert_resource(ClearColor(Color::srgb(0.05, 0.05, 0.1)))
        .insert_resource(Gravity(Vector::ZERO))
        .add_message::<MovementAction>()
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                keyboard_input1,
                keyboard_input2,
                movement.run_if(has_movement), // don't mutably access the character if there is no movement
                apply_movement_damping,
                apply_pressure_plate_colour,
                update_velocity_text,
                log_events,
            )
                .chain(),
        )
        .run();
}

#[derive(Component, Default, Reflect)]
struct Character;

#[derive(Component, Default, Reflect)]
struct PressurePlate;

#[derive(Component, Default, Reflect)]
struct CharacterVelocityText;

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    commands.spawn((
        Name::new("Character"),
        Mesh2d(meshes.add(Capsule2d::new(12.5, 20.0))),
        MeshMaterial2d(materials.add(Color::srgb(0.2, 0.7, 0.9))),
        Transform::from_xyz(0.0, -100.0, 1.0),
        Character,
        RigidBody::Dynamic,
        Collider::capsule(12.5, 20.0),
    ));

    commands.spawn((
        Name::new("Pressure Plate"),
        Sprite {
            color: Color::WHITE,
            custom_size: Some(Vec2::new(100.0, 100.0)),
            ..default()
        },
        Transform::from_xyz(0.0, 150.0, 0.0),
        PressurePlate,
        Sensor,
        RigidBody::Static,
        Collider::rectangle(100.0, 100.0),
        // Enable collision events for this entity.
        CollisionEventsEnabled,
        // Read entities colliding with this entity.
        CollidingEntities::default(),
    ));

    commands.spawn((
        Name::new("Character Velocity Text"),
        Text::new("Velocity: "),
        TextFont {
            font_size: 16.0,
            ..default()
        },
        Node {
            position_type: PositionType::Absolute,
            bottom: Val::Px(5.0),
            left: Val::Px(5.0),
            ..default()
        },
        CharacterVelocityText,
    ));

    commands.spawn(Camera2d);
}

#[derive(Message, Debug, Reflect)]
pub enum MovementAction {
    Velocity(Vector),
    Offset(Vector),
    Stop,
}

// use velocity
fn keyboard_input1(
    mut movement_writer: MessageWriter<MovementAction>,
    keyboard_input: Res<ButtonInput<KeyCode>>,
    time: Res<Time<Physics>>,
) {
    if time.is_paused() {
        return;
    }
    let space = keyboard_input.pressed(KeyCode::Space);
    if space {
        movement_writer.write(MovementAction::Stop);
        return;
    }

    let left = keyboard_input.any_pressed([KeyCode::KeyA]);
    let right = keyboard_input.any_pressed([KeyCode::KeyD]);
    let up = keyboard_input.any_pressed([KeyCode::KeyW]);
    let down = keyboard_input.any_pressed([KeyCode::KeyS]);
    let horizontal = right as i8 - left as i8;
    let vertical = up as i8 - down as i8;
    let direction = Vector::new(horizontal as Scalar, vertical as Scalar);
    if direction != Vector::ZERO {
        movement_writer.write(MovementAction::Velocity(direction));
    }
}
// use position offset
fn keyboard_input2(
    mut movement_writer: MessageWriter<MovementAction>,
    keyboard_input: Res<ButtonInput<KeyCode>>,
    time: Res<Time<Physics>>,
) {
    if time.is_paused() {
        return;
    }
    let left = keyboard_input.any_pressed([KeyCode::ArrowLeft]);
    let right = keyboard_input.any_pressed([KeyCode::ArrowRight]);
    let up = keyboard_input.any_pressed([KeyCode::ArrowUp]);
    let down = keyboard_input.any_pressed([KeyCode::ArrowDown]);
    let horizontal = right as i8 - left as i8;
    let vertical = up as i8 - down as i8;
    let direction = Vector::new(horizontal as Scalar, vertical as Scalar);
    if direction != Vector::ZERO {
        movement_writer.write(MovementAction::Offset(direction));
    }
}

fn has_movement(mut reader: MessageReader<MovementAction>) -> bool {
    reader.read().next().is_some()
}
fn movement(
    time: Res<Time>,
    mut movement_reader: MessageReader<MovementAction>,
    mut controllers: Query<(&mut LinearVelocity, &mut Position), With<Character>>,
) {
    let delta_time = time.delta_secs_f64().adjust_precision();
    for event in movement_reader.read() {
        for (mut linear_velocity, mut position) in &mut controllers {
            match event {
                MovementAction::Stop => {
                    linear_velocity.x = 0.0;
                    linear_velocity.y = 0.0;
                }
                MovementAction::Velocity(direction) => {
                    let movement_acceleration = 2000.0;
                    linear_velocity.x += direction.x * movement_acceleration * delta_time;
                    linear_velocity.y += direction.y * movement_acceleration * delta_time;
                }
                MovementAction::Offset(direction) => {
                    let speed = 100.0;
                    position.x += direction.x * speed * delta_time;
                    position.y += direction.y * speed * delta_time;
                }
            }
        }
    }
}

#[allow(clippy::type_complexity)]
fn apply_movement_damping(
    mut query: Query<
        (&mut LinearVelocity, &mut AngularVelocity),
        (With<Character>, Without<Sleeping>),
    >,
    time: Res<Time<Physics>>,
) {
    if time.is_paused() {
        return;
    }
    let damping_factor = 0.95;
    for (mut linear_velocity, mut angular_velocity) in &mut query {
        linear_velocity.x *= damping_factor;
        if linear_velocity.x.abs() < 0.001 {
            linear_velocity.x = 0.0;
        }
        linear_velocity.y *= damping_factor;
        if linear_velocity.y.abs() < 0.001 {
            linear_velocity.y = 0.0;
        }
        angular_velocity.0 *= damping_factor;
        if angular_velocity.0.abs() < 0.001 {
            angular_velocity.0 = 0.0;
        }
    }
}

fn apply_pressure_plate_colour(
    mut query: Query<(&mut Sprite, &CollidingEntities), With<PressurePlate>>,
) {
    for (mut sprite, colliding_entities) in &mut query {
        if colliding_entities.0.is_empty() {
            sprite.color = Color::srgb(0.2, 0.7, 0.9);
        } else {
            sprite.color = Color::srgb(0.9, 0.7, 0.2);
        }
    }
}

fn update_velocity_text(
    character_query: Query<(&LinearVelocity, Has<Sleeping>), With<Character>>,
    pressure_plate_query: Query<Has<Sleeping>, With<PressurePlate>>,
    mut text_query: Query<&mut Text, With<CharacterVelocityText>>,
) -> Result {
    if let (Ok((velocity, character_sleeping)), Ok(pressure_plate_sleeping)) =
        (character_query.single(), pressure_plate_query.single())
    {
        text_query.single_mut()?.0 = format!(
            "Velocity: {:.4}, {:.4}\nCharacter sleeping:{}\nPressure plate sleeping: {}",
            velocity.x, velocity.y, character_sleeping, pressure_plate_sleeping
        );
    }
    Ok(())
}

fn log_events(mut started: MessageReader<CollisionStart>, mut ended: MessageReader<CollisionEnd>) {
    // print out the started and ended events
    for event in started.read() {
        println!("CollisionStart: {event:?}");
    }
    for event in ended.read() {
        println!("CollisionEnd: {event:?}");
    }
}
