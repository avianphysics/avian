use avian3d::{math::*, prelude::*};
use bevy::{ecs::query::Has, prelude::*};

/// A plugin that implements a basic platformer kinematic character controller using move-and-slide,
/// with support for ground detection and configurable movement settings.
pub struct CharacterControllerPlugin;

impl Plugin for CharacterControllerPlugin {
    fn build(&self, app: &mut App) {
        app.add_message::<MovementAction>();

        // Collect input in `PreUpdate` so that it's processed before the physics update.
        app.add_systems(PreUpdate, (keyboard_input, gamepad_input).chain());

        // Run movement logic in `FixedUpdate` to ensure consistent behavior regardless of frame rate.
        app.add_systems(
            FixedUpdate,
            (
                update_grounded,
                apply_gravity,
                movement,
                apply_movement_damping,
                move_and_slide,
                apply_forces_to_dynamic_bodies,
            )
                .chain(),
        );
    }
}

/// A [`Message`] written for a movement input action.
#[derive(Message)]
pub enum MovementAction {
    Move(Vector2),
    Jump,
}

/// A marker component indicating that an entity is using a character controller.
///
/// This also requires the entity to have a `CustomPositionIntegration` component, which is used
/// to prevent Avian from automatically applying the character's velocity to its position,
/// since the character controller will handle movement manually using move-and-slide.
#[derive(Component)]
#[require(
    RigidBody::Kinematic,
    CustomPositionIntegration,
    // We don't want to impart speculative collision impulses in this case
    SpeculativeMargin(0.0)
)]
pub struct CharacterController;

/// Component for configuring movement settings for a character controller.
#[derive(Component)]
pub struct CharacterMovementSettings {
    /// The acceleration used for character movement.
    pub acceleration: Scalar,
    /// The damping coefficient used for slowing down movement.
    pub damping: Scalar,
    /// The strength of a jump.
    pub jump_impulse: Scalar,
    /// The gravitational acceleration used for the character.
    pub gravity: Vector,
    /// The maximum speed that gravity can accelerate the character to.
    /// This prevents the character from accelerating indefinitely while falling.
    pub terminal_velocity: Scalar,
}

impl Default for CharacterMovementSettings {
    fn default() -> Self {
        Self {
            acceleration: 50.0,
            damping: 10.0,
            jump_impulse: 7.0,
            gravity: Vector::new(0.0, -9.81 * 2.0, 0.0),
            terminal_velocity: 50.0,
        }
    }
}

/// Component for configuring ground detection for a character controller.
#[derive(Component)]
pub struct GroundDetection {
    /// The maximum angle (in radians) where a surface is considered ground/ceiling
    /// relative to the up-direction. Outside of this angle, surfaces are considered walls.
    ///
    /// **Default**: 30 degrees (π / 6 radians)
    pub max_angle: Scalar,
    /// The maximum distance for ground detection.
    pub max_distance: Scalar,
    /// The shape cast collider used for ground detection.
    pub cast_shape: Option<Collider>,
}

impl Default for GroundDetection {
    fn default() -> Self {
        Self {
            max_angle: PI / 6.0,
            max_distance: 0.2,
            cast_shape: None,
        }
    }
}

/// A marker component indicating that an entity is on a surface that is considered
/// ground, meaning the steepness is less than [`GroundDetection::max_angle`].
///
/// Characters that are grounded can jump, and do not slide down slopes.
#[derive(Component)]
#[component(storage = "SparseSet")]
pub struct Grounded;

/// A component containing information about the current collisions for a character controller.
///
/// This is used to apply forces to dynamic rigid bodies hit by the character.
#[derive(Component, Default, Deref)]
pub struct CharacterCollisions(Vec<CharacterCollision>);

/// Information about a collision between a character controller and another collider.
pub struct CharacterCollision {
    /// The collider that was hit by the character.
    pub collider: Entity,
    /// The point of contact in world space.
    pub point: Vector,
    /// The normal of the contact surface, pointing away from the character.
    pub normal: Dir3,
    /// The velocity of the character at the point of contact.
    pub character_velocity: Vector,
}

/// Sends [`MovementAction`] events based on keyboard input.
fn keyboard_input(
    mut movement_writer: MessageWriter<MovementAction>,
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
        movement_writer.write(MovementAction::Move(direction));
    }

    if keyboard_input.just_pressed(KeyCode::Space) {
        movement_writer.write(MovementAction::Jump);
    }
}

/// Sends [`MovementAction`] events based on gamepad input.
fn gamepad_input(mut movement_writer: MessageWriter<MovementAction>, gamepads: Query<&Gamepad>) {
    for gamepad in gamepads.iter() {
        if let (Some(x), Some(y)) = (
            gamepad.get(GamepadAxis::LeftStickX),
            gamepad.get(GamepadAxis::LeftStickY),
        ) {
            movement_writer.write(MovementAction::Move(
                Vector2::new(x as Scalar, y as Scalar).clamp_length_max(1.0),
            ));
        }

        if gamepad.just_pressed(GamepadButton::South) {
            movement_writer.write(MovementAction::Jump);
        }
    }
}

/// Updates the [`Grounded`] status for character controllers.
fn update_grounded(
    mut commands: Commands,
    mut query: Query<(Entity, &GroundDetection, &GlobalTransform)>,
    spatial_query: SpatialQuery,
) {
    for (entity, ground_detection, global_transform) in &mut query {
        let Some(collider) = &ground_detection.cast_shape else {
            continue;
        };

        let translation = global_transform.translation().adjust_precision();
        let rotation = global_transform.rotation().adjust_precision();

        // Cast the shape downward to check for ground
        let hit = spatial_query.cast_shape(
            collider,
            translation,
            rotation,
            global_transform.down(),
            &ShapeCastConfig::from_max_distance(ground_detection.max_distance),
            &SpatialQueryFilter::from_excluded_entities([entity]),
        );

        // The character is grounded if we hit a surface that isn't too steep
        let is_grounded = hit.is_some_and(|hit| {
            let up = global_transform.up().adjust_precision();
            (rotation * hit.normal1).angle_between(up) <= ground_detection.max_angle
        });

        // Update grounded state
        if is_grounded {
            commands.entity(entity).insert(Grounded);
        } else {
            commands.entity(entity).remove::<Grounded>();
        }
    }
}

/// Responds to [`MovementAction`] events and moves character controllers accordingly.
fn movement(
    time: Res<Time>,
    mut movement_reader: MessageReader<MovementAction>,
    mut controllers: Query<(
        &CharacterMovementSettings,
        &mut LinearVelocity,
        Has<Grounded>,
    )>,
) {
    let delta_secs = time.delta_secs_f64().adjust_precision();

    for event in movement_reader.read() {
        for (movement, mut linear_velocity, is_grounded) in &mut controllers {
            match event {
                MovementAction::Move(direction) => {
                    linear_velocity.x += direction.x * movement.acceleration * delta_secs;
                    linear_velocity.z -= direction.y * movement.acceleration * delta_secs;
                }
                MovementAction::Jump => {
                    if is_grounded {
                        linear_velocity.y = movement.jump_impulse;
                    }
                }
            }
        }
    }
}

/// Applies gravity to character controllers.
fn apply_gravity(
    time: Res<Time>,
    mut controllers: Query<(&CharacterMovementSettings, &mut LinearVelocity)>,
) {
    let delta_secs = time.delta_secs_f64().adjust_precision();

    for (movement, mut linear_velocity) in &mut controllers {
        let gravity_direction = movement.gravity.normalize_or_zero();

        let velocity_along_gravity = linear_velocity.dot(gravity_direction);
        if velocity_along_gravity > movement.terminal_velocity {
            // Don't apply more gravity if we're already at terminal velocity.
            continue;
        }

        // Calculate the new velocity after applying gravity.
        let new_velocity = linear_velocity.0 + movement.gravity * delta_secs;

        // Don't exceed terminal velocity.
        let new_velocity_along_gravity = new_velocity.dot(gravity_direction);
        if new_velocity_along_gravity < movement.terminal_velocity {
            linear_velocity.0 = new_velocity;
        } else {
            linear_velocity.0 = gravity_direction * movement.terminal_velocity;
        }
    }
}

/// Slows down movement in the XZ plane.
fn apply_movement_damping(
    mut query: Query<(&CharacterMovementSettings, &mut LinearVelocity)>,
    time: Res<Time>,
) {
    let delta_secs = time.delta_secs_f64().adjust_precision();

    for (movement, mut linear_velocity) in &mut query {
        // Approximate exponential decay. We could use `LinearDamping` for this,
        // but we don't want to dampen movement along the Y axis.
        linear_velocity.x *= 1.0 / (1.0 + delta_secs * movement.damping);
        linear_velocity.z *= 1.0 / (1.0 + delta_secs * movement.damping);
    }
}

/// Performs move-and-slide for character controllers, moving them according to their velocity
/// and sliding along any contact surfaces. Also updates the [`Grounded`] state.
///
/// For simplicity, we assume that the character is not a child entity,
/// and its collider is on the same entity as the `CharacterController`.
fn move_and_slide(
    mut query: Query<
        (
            Entity,
            Option<&GroundDetection>,
            Option<&mut CharacterCollisions>,
            &mut Transform,
            &mut LinearVelocity,
            &Collider,
        ),
        With<CharacterController>,
    >,
    move_and_slide: MoveAndSlide,
    time: Res<Time>,
) {
    for (entity, ground_detection, mut collisions, mut transform, mut lin_vel, collider) in
        &mut query
    {
        let mut hit_ground_or_ceiling = false;

        if let Some(collisions) = &mut collisions {
            // Clear previous collisions
            collisions.0.clear();
        }

        let up = transform.up().adjust_precision();

        // Perform move-and-slide
        let MoveAndSlideOutput {
            position: new_position,
            projected_velocity,
        } = move_and_slide.move_and_slide(
            collider,
            transform.translation.adjust_precision(),
            transform.rotation.adjust_precision(),
            lin_vel.0,
            time.delta(),
            &MoveAndSlideConfig::default(),
            &SpatialQueryFilter::from_excluded_entities([entity]),
            |hit| {
                // This callback is called for each surface we collide with during move-and-slide.
                // In this example, we use it to customize collision behavior for ground surfaces,
                // preventing sliding down slopes when we are grounded, and preventing climbing up steep slopes.

                let Some(ground_detection) = ground_detection else {
                    // Early out if we don't have ground detection.
                    return MoveAndSlideHitResponse::Accept;
                };

                // Determine if the surface is ground based on the angle between the up-vector and the hit normal.
                let angle = up.angle_between(hit.normal.adjust_precision());
                let is_ground = angle <= ground_detection.max_angle;
                let is_ceiling = is_ground && up.dot(hit.normal.adjust_precision()) < 0.0;

                // Decompose the original input velocity into components relative to the hit normal and the up direction,
                // to determine how much of the velocity is contributing to climbing, slipping, and unconstrained movement.
                let [horizontal_component, vertical_component] =
                    split_into_components(lin_vel.0, up);

                // Decompose the horizontal component and the current sliding velocity to determine
                // whether the character is trying to climb or slip, and whether it is actually climbing or slipping.
                let horizontal_velocity_decomposition =
                    decompose_hit_velocity(horizontal_component, *hit.normal, up);
                let decomposition = decompose_hit_velocity(*hit.velocity, *hit.normal, up);

                // An object is trying to slip if the tangential movement induced by its vertical movement
                // points downward (with a small threshold).
                let slipping_intent =
                    up.dot(horizontal_velocity_decomposition.vertical_tangent) < -0.001;

                // An object is slipping if its vertical movement points downward (with a small threshold).
                let slipping = up.dot(decomposition.vertical_tangent) < -0.001;

                // An object is trying to climb if its vertical input motion points upward.
                let climbing_intent = up.dot(vertical_component) > 0.0;

                // An object is climbing if the tangential movement induced by its vertical movement points upward.
                let climbing = up.dot(decomposition.vertical_tangent) > 0.0;

                let projected_velocity = if !is_ground && climbing && !climbing_intent {
                    // Can’t climb the slope, remove the vertical tangent motion induced by the forward motion.
                    decomposition.horizontal_tangent + decomposition.normal_part
                } else if is_ground && slipping && !slipping_intent {
                    // Prevent the vertical movement from sliding down.
                    decomposition.horizontal_tangent + decomposition.normal_part
                } else {
                    // Otherwise, allow full movement (including climbing and slipping)
                    decomposition.horizontal_tangent
                        + decomposition.vertical_tangent
                        + decomposition.normal_part
                };

                // Update the current velocity used by the algorithm.
                *hit.velocity = projected_velocity;

                if is_ground || is_ceiling {
                    // We hit a ground or ceiling surface!
                    hit_ground_or_ceiling = true;
                }

                if let Some(collisions) = &mut collisions {
                    // Record the collision for use in other systems, such as applying forces to dynamic bodies.
                    collisions.0.push(CharacterCollision {
                        collider: hit.entity,
                        point: hit.point,
                        normal: *hit.normal,
                        character_velocity: *hit.velocity,
                    });
                }

                // Accept the hit and continue the move-and-slide algorithm with the modified velocity.
                MoveAndSlideHitResponse::Accept
            },
        );

        // Update position to the final position calculated by move-and-slide.
        transform.translation = new_position.f32();

        // If we hit the ground or a ceiling, update the velocity along the up-direction
        // to prevent accumulating velocity along the ground normal when hitting slopes,
        // and to prevent sticking to ceilings when jumping.
        if hit_ground_or_ceiling {
            let up = up.adjust_precision();
            let velocity_along_up = lin_vel.dot(up);
            let new_velocity_along_up = projected_velocity.dot(up);
            lin_vel.0 += (new_velocity_along_up - velocity_along_up) * up;
        }
    }
}

/// The decomposition of a velocity vector into parts relative to a collision normal and an up-direction.
///
/// This is used for determining how much of the velocity is contributing to climbing, slipping, and unconstrained movement.
#[derive(Debug)]
struct VelocityDecomposition {
    /// The part of the velocity that is directly against the collision normal.
    normal_part: Vector,
    /// The part of the velocity that is tangent to the collision surface and perpendicular to the up-direction.
    horizontal_tangent: Vector,
    /// The part of the velocity that is tangent to the collision surface and parallel to the up-direction.
    vertical_tangent: Vector,
}

/// Decomposes a velocity vector into parts relative to a collision `normal` and an `up` direction.
fn decompose_hit_velocity(velocity: Vector, normal: Dir, up: Vector) -> VelocityDecomposition {
    let normal = normal.adjust_precision();
    let normal_part = normal * normal.dot(velocity);
    let tangent_part = velocity - normal_part;

    let horizontal_tangent_dir = normal.cross(up).normalize_or_zero();
    let horizontal_tangent = tangent_part.dot(horizontal_tangent_dir) * horizontal_tangent_dir;
    let vertical_tangent = tangent_part - horizontal_tangent;

    VelocityDecomposition {
        normal_part,
        horizontal_tangent,
        vertical_tangent,
    }
}

/// Splits a vector into horizontal and vertical components relative to a given `up` direction.
fn split_into_components(v: Vector, up: Vector) -> [Vector; 2] {
    let vertical_component = up * v.dot(up);
    let horizontal_component = v - vertical_component;
    [horizontal_component, vertical_component]
}

/// Applies forces to dynamic rigid bodies hit by character controllers based on their collisions.
fn apply_forces_to_dynamic_bodies(
    characters: Query<(&ComputedMass, &CharacterCollisions)>,
    colliders: Query<&ColliderOf>,
    mut rigid_bodies: Query<(&RigidBody, Forces)>,
) {
    for (mass, collisions) in &characters {
        let mass = mass.value();
        for collision in &collisions.0 {
            let Ok(collider_of) = colliders.get(collision.collider) else {
                continue;
            };
            let Ok((rigid_body, mut forces)) = rigid_bodies.get_mut(collider_of.body) else {
                continue;
            };
            if !rigid_body.is_dynamic() {
                continue;
            }

            let touch_dir = -collision.normal.adjust_precision();
            let relative_velocity = collision.character_velocity - forces.linear_velocity();
            let touch_velocity = touch_dir.dot(relative_velocity) * touch_dir;
            let impulse = touch_velocity * mass;

            forces.apply_linear_impulse_at_point(impulse, collision.point);
        }
    }
}
