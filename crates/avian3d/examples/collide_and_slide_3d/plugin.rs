use avian3d::{math::*, prelude::*};
use bevy::prelude::*;

pub struct CharacterControllerPlugin;

impl Plugin for CharacterControllerPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<MovementAction>()
            .add_systems(
                Update,
                (keyboard_input, gamepad_input, handle_movement_actions).chain(),
            )
            .add_systems(
                PostUpdate,
                (
                    ground_detection_system,
                    gravity_system,
                    process_movement_passes,
                    update_kinematic_character_controller,
                )
                    .chain(),
            );
    }
}

#[derive(Event)]
pub enum MovementAction {
    Move(Vector2),
    Jump,
}

#[derive(Component)]
pub struct KinematicCharacterController {
    pub prev_velocity: Vector,
    pub velocity: Vector,
    pub collider: Collider,
}

impl Default for KinematicCharacterController {
    fn default() -> Self {
        Self {
            prev_velocity: Vector::ZERO,
            velocity: Vector::ZERO,
            collider: Collider::capsule(0.4, 0.8),
        }
    }
}

#[derive(Component)]
pub struct KCCGrounded {
    pub grounded: bool,
    pub prev_grounded: bool,
}

impl Default for KCCGrounded {
    fn default() -> Self {
        Self {
            grounded: true,
            prev_grounded: false,
        }
    }
}

#[derive(Component)]
pub struct KCCGravity {
    pub terminal_velocity: Scalar,
    pub acceleration_factor: Scalar,
    pub current_velocity: Vector,
    pub direction: Vector,
}

impl Default for KCCGravity {
    fn default() -> Self {
        Self {
            terminal_velocity: 53.0,
            acceleration_factor: 9.81 * 2.0,
            current_velocity: Vector::ZERO,
            direction: Vector::NEG_Y,
        }
    }
}

#[derive(Component)]
pub struct KCCSlope {
    pub max_slope_angle: Scalar,
}

impl Default for KCCSlope {
    fn default() -> Self {
        Self {
            max_slope_angle: (35.0 as Scalar).to_radians(),
        }
    }
}

#[derive(Component)]
pub struct MovementSettings {
    pub speed: Scalar,
    pub jump_force: Scalar,
}

impl Default for MovementSettings {
    fn default() -> Self {
        Self {
            speed: 10.0,
            jump_force: 7.0,
        }
    }
}

#[derive(Component)]
pub struct KCCCoyoteTime {
    pub timer: Timer,
    pub can_jump: bool,
}

impl Default for KCCCoyoteTime {
    fn default() -> Self {
        Self {
            timer: Timer::from_seconds(0.1, TimerMode::Once),
            can_jump: false,
        }
    }
}

#[derive(Bundle)]
pub struct CharacterControllerBundle {
    controller: KinematicCharacterController,
    rigid_body: RigidBody,
    grounded: KCCGrounded,
    gravity: KCCGravity,
    slope: KCCSlope,
    movement: MovementSettings,
    coyote_time: KCCCoyoteTime,
}

impl Default for CharacterControllerBundle {
    fn default() -> Self {
        Self {
            controller: KinematicCharacterController::default(),
            rigid_body: RigidBody::Kinematic,
            grounded: KCCGrounded::default(),
            gravity: KCCGravity::default(),
            slope: KCCSlope::default(),
            movement: MovementSettings::default(),
            coyote_time: KCCCoyoteTime::default(),
        }
    }
}

// Movement constants
const MAX_BUMPS: u32 = 4;
const MIN_MOVEMENT: Scalar = 0.0001;
const COLLISION_EPSILON: Scalar = 0.01;
const DEPENETRATION_EPSILON: Scalar = 0.01;

fn keyboard_input(
    mut movement_event_writer: EventWriter<MovementAction>,
    keyboard_input: Res<ButtonInput<KeyCode>>,
) {
    let up = keyboard_input.any_pressed([KeyCode::KeyW, KeyCode::ArrowUp]);
    let down = keyboard_input.any_pressed([KeyCode::KeyS, KeyCode::ArrowDown]);
    let left = keyboard_input.any_pressed([KeyCode::KeyA, KeyCode::ArrowLeft]);
    let right = keyboard_input.any_pressed([KeyCode::KeyD, KeyCode::ArrowRight]);

    let horizontal = right as i8 - left as i8;
    let vertical = up as i8 - down as i8;
    let direction = Vector2::new(horizontal as Scalar, vertical as Scalar).clamp_length_max(1.0);

    if direction != Vector2::ZERO {
        movement_event_writer.send(MovementAction::Move(direction));
    }

    if keyboard_input.just_pressed(KeyCode::Space) {
        movement_event_writer.send(MovementAction::Jump);
    }
}

fn gamepad_input(
    mut movement_event_writer: EventWriter<MovementAction>,
    gamepads: Query<&Gamepad>,
) {
    for gamepad in gamepads.iter() {
        if let (Some(x), Some(y)) = (
            gamepad.get(GamepadAxis::LeftStickX),
            gamepad.get(GamepadAxis::LeftStickY),
        ) {
            movement_event_writer.send(MovementAction::Move(
                Vector2::new(x as Scalar, y as Scalar).clamp_length_max(1.0),
            ));
        }

        if gamepad.just_pressed(GamepadButton::South) {
            movement_event_writer.send(MovementAction::Jump);
        }
    }
}

/// Processes incoming movement events and updates character controller states.
///
/// # Parameters
/// * `time` - Time resource for delta time calculations
/// * `movement_events` - Event reader for movement input events
/// * `query` - Query for character components needed for movement processing
fn handle_movement_actions(
    time: Res<Time>,
    mut movement_events: EventReader<MovementAction>,
    mut query: Query<(
        &MovementSettings,
        &mut KinematicCharacterController,
        &mut KCCGravity,
        &KCCGrounded,
        &mut KCCCoyoteTime,
    )>,
) {
    let delta = time.delta_secs_f64().adjust_precision();

    for event in movement_events.read() {
        for (settings, mut controller, mut gravity, grounded, mut coyote) in query.iter_mut() {
            match event {
                MovementAction::Move(direction) => {
                    let movement = Vector::new(
                        direction.x * settings.speed * delta,
                        0.0,
                        -direction.y * settings.speed * delta,
                    );
                    controller.velocity += movement;
                }
                MovementAction::Jump => {
                    // Jump if grounded or within coyote time
                    if grounded.grounded || coyote.can_jump {
                        gravity.current_velocity = Vector::Y * settings.jump_force;
                        coyote.can_jump = false; // Prevent double jumping during coyote time
                    }
                }
            }
        }
    }
}

/// Performs ground detection and state management for the character controllers.
///
/// # Parameters
/// * `time` - Time resource for coyote time management
/// * `query` - Query for character components needed for ground detection
/// * `spatial_query` - Spatial query system for collision detection
fn ground_detection_system(
    time: Res<Time>,
    mut query: Query<(
        &Transform,
        Entity,
        &KinematicCharacterController,
        &KCCGravity,
        &mut KCCGrounded,
        &mut KCCCoyoteTime,
    )>,
    spatial_query: SpatialQuery,
) {
    for (transform, entity, controller, gravity, mut grounded, mut coyote) in &mut query {
        grounded.prev_grounded = grounded.grounded;

        let filter = SpatialQueryFilter::default().with_excluded_entities([entity]);

        let config = ShapeCastConfig {
            max_distance: 0.1,
            target_distance: 0.0,
            compute_contact_on_penetration: true,
            ignore_origin_penetration: false,
        };

        if let Some(hit) = spatial_query.cast_shape(
            &controller.collider,
            transform.translation,
            transform.rotation,
            gravity.direction.try_into().unwrap_or(Dir3::NEG_Y),
            &config,
            &filter,
        ) {
            grounded.grounded = hit.normal1.angle_between(Vector::Y)
                < core::f64::consts::FRAC_PI_4.adjust_precision();

            if grounded.grounded {
                coyote.timer.reset();
                coyote.can_jump = true;
            }
        } else {
            grounded.grounded = false;

            if grounded.prev_grounded {
                coyote.timer.reset();
            }

            coyote.timer.tick(time.delta());
            if coyote.timer.finished() {
                coyote.can_jump = false;
            }
        }
    }
}

/// Applies gravitational forces to character controllers with clamping and grounding logic.
pub fn gravity_system(
    mut query: Query<(&KinematicCharacterController, &mut KCCGravity, &KCCGrounded)>,
    time: Res<Time>,
) {
    let dt = time.delta_secs_f64().adjust_precision();

    for (_, mut gravity, grounded) in query.iter_mut() {
        let current_speed = gravity.current_velocity.length();
        if current_speed >= gravity.terminal_velocity {
            gravity.current_velocity *= 0.99;
            continue;
        }

        // Apply reduced acceleration when grounded
        let acceleration = if grounded.grounded {
            gravity.acceleration_factor * 0.01
        } else {
            gravity.acceleration_factor
        };

        let delta_velocity = gravity.direction * acceleration * dt;
        let new_velocity = gravity.current_velocity + delta_velocity;

        let is_jumping = gravity.current_velocity.dot(gravity.direction) < 0.0;
        let term_velocity_modifier = if grounded.grounded || is_jumping {
            1.0
        } else {
            1.0
        };

        let modified_terminal_velocity = gravity.terminal_velocity * term_velocity_modifier;
        gravity.current_velocity = if new_velocity.length() > modified_terminal_velocity {
            new_velocity.normalize() * modified_terminal_velocity
        } else {
            new_velocity
        };
    }
}

/// Runs multi-pass collision detection and response for character controllers.
pub fn process_movement_passes(
    mut query: Query<(
        &mut Transform,
        Entity,
        &mut KinematicCharacterController,
        Option<&KCCSlope>,
        Option<&KCCGrounded>,
        Option<&mut KCCGravity>,
    )>,
    mut spatial_query: SpatialQuery,
    time: Res<Time>,
) {
    let delta = time.delta_secs_f64().adjust_precision();

    for (mut transform, entity, mut controller, slope, grounded, gravity) in &mut query {
        let filter = SpatialQueryFilter::default().with_excluded_entities([entity]);

        // Process horizontal movement
        let movement = collide_and_slide(
            &mut spatial_query,
            &filter,
            &controller,
            &mut transform,
            controller.velocity,
            slope,
            grounded,
            false,
        );

        controller.velocity = movement.remaining_velocity;

        // Process gravity separately if enabled
        if let Some(gravity) = gravity {
            let _movement = collide_and_slide(
                &mut spatial_query,
                &filter,
                &controller,
                &mut transform,
                gravity.current_velocity * delta,
                slope,
                grounded,
                true,
            );
        }

        // Perform depenetration
        depenetrate(
            &mut spatial_query,
            &filter,
            &controller.collider,
            &mut transform,
        );
    }
}

/// Result of a collision-aware movement step.
///
/// Contains:
/// - The actual movement vector achieved
/// - Any remaining velocity after collision
/// - The normal of the last surface hit, if any
#[derive(Debug)]
struct MovementResult {
    movement: Vector, // Not used in this example.
    remaining_velocity: Vector,
    hit_normal: Option<Vector>, // Not used in this example.
}

/// Implementation of Kasper Fauerby's Collide and Slide algorithm for 3D character movement.
/// This is not a pure 1:1 translation of the original algorithm, and makes some changes to improve stability and robustness.
///
/// # Parameters
/// * `spatial_query` - Spatial query system for collision detection
/// * `filter` - Filter configuration for collision checks
/// * `controller` - Character controller component containing collision shape
/// * `transform` - Transform component for position updates
/// * `velocity` - Current velocity vector to process
/// * `slope` - Optional slope handling configuration
/// * `floor_detection` - Optional ground detection state
/// * `is_gravity_pass` - Whether this is processing gravity movement
///
/// # Returns
/// `MovementResult` containing the movement outcome and collision information
fn collide_and_slide(
    spatial_query: &mut spatial_query::SpatialQuery,
    filter: &spatial_query::SpatialQueryFilter,
    controller: &KinematicCharacterController,
    transform: &mut Transform,
    velocity: Vector,
    slope: Option<&KCCSlope>,
    floor_detection: Option<&KCCGrounded>,
    is_gravity_pass: bool,
) -> MovementResult {
    if velocity.length_squared() < MIN_MOVEMENT {
        return MovementResult {
            movement: Vector::ZERO,
            remaining_velocity: Vector::ZERO,
            hit_normal: None,
        };
    }

    let mut total_movement = Vector::ZERO;
    let mut current_velocity = velocity;
    let mut collision_planes = Vec::with_capacity(MAX_BUMPS as usize);
    let mut last_hit_normal = None;

    for _ in 0..MAX_BUMPS {
        if current_velocity.length_squared() < MIN_MOVEMENT || current_velocity.is_nan() {
            current_velocity = Vector::ZERO;
            break;
        }

        let (velocity_dir, length) = match Dir3::new_and_length(current_velocity) {
            Ok(v) => v,
            Err(_) => break,
        };

        let config = ShapeCastConfig {
            max_distance: length,
            target_distance: 0.0,
            compute_contact_on_penetration: true,
            ignore_origin_penetration: false,
        };

        match spatial_query.cast_shape(
            &controller.collider,
            transform.translation,
            transform.rotation,
            velocity_dir,
            &config,
            filter,
        ) {
            Some(hit) => {
                let safe_distance = (hit.distance - COLLISION_EPSILON).max(0.0);
                let safe_movement = current_velocity * safe_distance;

                transform.translation += safe_movement;
                total_movement += safe_movement;
                current_velocity -= safe_movement;
                last_hit_normal = Some(hit.normal1);

                if is_gravity_pass && should_stop_on_slope(slope, floor_detection, hit.normal1) {
                    break;
                }

                current_velocity = calculate_sliding_velocity(
                    &mut collision_planes,
                    hit.normal1,
                    current_velocity,
                );
            }
            None => {
                transform.translation += current_velocity;
                total_movement += current_velocity;
                current_velocity = Vector::ZERO;
                break;
            }
        }
    }

    MovementResult {
        movement: total_movement,
        remaining_velocity: current_velocity,
        hit_normal: last_hit_normal,
    }
}

/// Evaluates whether character movement should stop based on slope characteristics.
///
/// # Parameters
/// * `slope` - Slope configuration for maximum angle
/// * `floor_detection` - Ground detection state
/// * `normal` - Surface normal vector at contact point
///
/// # Returns
/// `true` if movement should stop, `false` if sliding should continue
#[inline]
fn should_stop_on_slope(
    slope: Option<&KCCSlope>,
    floor_detection: Option<&KCCGrounded>,
    normal: Vector,
) -> bool {
    match (slope, floor_detection) {
        (Some(slope), Some(_)) => normal.angle_between(Vector::Y) < slope.max_slope_angle,
        _ => true,
    }
}

/// Computes sliding velocity vectors for collision response.
///
/// # Parameters
/// * `planes` - Accumulated collision plane normals
/// * `normal` - Current collision surface normal
/// * `velocity` - Current velocity vector
///
/// # Returns
/// New velocity vector projected along valid sliding planes
#[inline]
fn calculate_sliding_velocity(
    planes: &mut Vec<Vector>,
    normal: Vector,
    velocity: Vector,
) -> Vector {
    planes.push(normal);
    let mut result = velocity.reject_from(normal);

    if planes.len() > 1 {
        result = planes.windows(2).fold(result, |acc, plane_pair| {
            acc.project_onto(plane_pair[0].cross(plane_pair[1]))
        });
    }

    result
}

/// Resolves character penetration with static geometry.
///
/// # Parameters
/// * `spatial_query` - Spatial query system for penetration detection
/// * `filter` - Filter configuration for collision checks
/// * `collider` - Character's collision shape
/// * `transform` - Transform component to update position
fn depenetrate(
    spatial_query: &mut spatial_query::SpatialQuery,
    filter: &spatial_query::SpatialQueryFilter,
    collider: &Collider,
    transform: &mut Transform,
) {
    let config = ShapeCastConfig {
        max_distance: 0.0,
        target_distance: 0.0,
        compute_contact_on_penetration: true,
        ignore_origin_penetration: false, // We want to detect penetration at origin
    };

    let hit = spatial_query.cast_shape(
        collider,
        transform.translation,
        transform.rotation,
        Dir3::NEG_Y,
        &config,
        filter,
    );

    if let Some(hit) = hit {
        let push_out_distance = hit.distance + DEPENETRATION_EPSILON;
        transform.translation += hit.normal1 * push_out_distance;
    }
}

/// Updates velocity history for character controllers between physics steps.
pub fn update_kinematic_character_controller(mut query: Query<&mut KinematicCharacterController>) {
    for mut controller in query.iter_mut() {
        controller.prev_velocity = controller.velocity;
    }
}
