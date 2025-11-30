//! A virtual character controller using the [`MoveAndSlide`] algorithm.
//!
//! See the [`VirtualCharacter`] and [`AutoMoveAndSlide`] components for more information.

use crate::prelude::*;
use bevy::{
    ecs::{intern::Interned, schedule::ScheduleLabel},
    prelude::*,
};

/// Marker component for virtual characters that use character controller logic.
///
/// A virtual character is an entity that does not use a [`RigidBody`](crate::prelude::RigidBody)
/// by default, but instead relies on [`MoveAndSlide`] for movement and collisions.
#[derive(Component, Default)]
#[require(AutoMoveAndSlide)]
pub struct VirtualCharacter;

/// Component that enables automatic [`MoveAndSlide`] updates for an entity.
#[derive(Component, Clone, Debug)]
#[require(LinearVelocity, Position, Rotation, CustomPositionIntegration)]
pub struct AutoMoveAndSlide {
    /// If `true`, automatic [`MoveAndSlide`] updates are performed each frame.
    pub enabled: bool,
    /// Configuration for the [`MoveAndSlide`] algorithm.
    pub config: MoveAndSlideConfig,
    /// Optional custom shape for collision detection.
    ///
    /// If `None`, the entity's existing [`Collider`] is used, if available.
    pub custom_shape: Option<Collider>,
    /// Callback function invoked on each contact surface hit during movement.
    pub on_hit: fn(MoveAndSlideHitData) -> bool,
}

impl Default for AutoMoveAndSlide {
    fn default() -> Self {
        Self::new(MoveAndSlideConfig::default(), |_| true)
    }
}

impl AutoMoveAndSlide {
    /// Creates a new [`AutoMoveAndSlide`] component with default settings.
    #[inline]
    pub const fn new(config: MoveAndSlideConfig, on_hit: fn(MoveAndSlideHitData) -> bool) -> Self {
        Self {
            enabled: true,
            config,
            custom_shape: None,
            on_hit,
        }
    }
}

/// A plugin that performs automatic [`MoveAndSlide`] updates for entities with the [`AutoMoveAndSlide`] component.
///
/// By default, the move-and-slide algorithm is run in the [`PhysicsSchedule`], within the
/// [`PhysicsStepSystems::VirtualCharacters`] system set. A different schedule can be specified using
/// [`MoveAndSlidePlugin::new`].
///
/// For full control over when and how the move-and-slide algorithm is executed,
/// consider implementing your own system that uses the [`MoveAndSlide`] system parameter directly.
pub struct MoveAndSlidePlugin {
    schedule: Interned<dyn ScheduleLabel>,
}

impl MoveAndSlidePlugin {
    /// Creates a [`MoveAndSlidePlugin`] using the given schedule for running the [`MoveAndSlide`] algorithm.
    ///
    /// The default schedule is [`PhysicsSchedule`], in [`PhysicsStepSystems::VirtualCharacters`].
    pub fn new(schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: schedule.intern(),
        }
    }
}

impl Default for MoveAndSlidePlugin {
    fn default() -> Self {
        Self::new(PhysicsSchedule)
    }
}

impl Plugin for MoveAndSlidePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            self.schedule,
            move_and_slide.in_set(PhysicsStepSystems::VirtualCharacters),
        );
    }
}

fn move_and_slide(
    mut query: Query<(
        Entity,
        &Position,
        &Rotation,
        &mut Transform,
        &mut LinearVelocity,
        Option<&Collider>,
        &AutoMoveAndSlide,
        Option<&UpDirection>,
        Option<&SlopeHandling>,
    )>,
    move_and_slide: MoveAndSlide,
    time: Res<Time>,
) {
    query.par_iter_mut().for_each(
        |(
            entity,
            position,
            rotation,
            mut transform,
            mut lin_vel,
            collider,
            mns_config,
            up,
            slopes,
        )| {
            if !mns_config.enabled {
                return;
            }

            let collider = if let Some(custom_shape) = &mns_config.custom_shape {
                custom_shape
            } else if let Some(collider) = collider {
                collider
            } else {
                // No collider available, skip movement.
                return;
            };

            let mut is_grounded = false;

            // Perform move and slide
            let MoveAndSlideOutput {
                position: new_position,
                projected_velocity,
            } = move_and_slide.move_and_slide(
                collider,
                position.0,
                (*rotation).into(),
                lin_vel.0,
                time.delta(),
                &mns_config.config,
                &SpatialQueryFilter::from_excluded_entities([entity]),
                mns_config.on_hit,
            );

            // Update position and velocity
            #[cfg(feature = "2d")]
            {
                transform.translation = new_position.f32().extend(transform.translation.z);
            }
            #[cfg(feature = "3d")]
            {
                transform.translation = new_position.f32();
            }
            lin_vel.0 = projected_velocity;
        },
    );
}

/// Component representing the up-direction for a virtual character.
///
/// Ground detection, slope handling, and stair stepping logic
/// are based on this up-direction.
///
/// By default, the up-direction is along the positive y-axis.
#[derive(Component, Clone, Copy, Debug)]
pub struct UpDirection(pub Dir);

impl Default for UpDirection {
    fn default() -> Self {
        Self(Dir::Y)
    }
}

/// Marker component indicating that a [`VirtualCharacter`] is currently grounded.
/// Added or removed automatically during character movement.
///
/// A character is considered grounded when it is in contact with at least one surface
/// where the angle between the contact normal and the [`UpDirection`] is less than
/// the [`max_ground_angle`] defined in the [`SlopeHandling`] component.
///
/// [`max_ground_angle`]: SlopeHandling::max_ground_angle
#[derive(Component, Default)]
#[component(storage = "SparseSet")]
pub struct Grounded;

/// Component for configuring slope handling behavior for [`VirtualCharacter`]s.
///
/// These settings influence how the character interacts with surfaces during movement,
/// including what angles are considered ground, walls, and ceilings, as well as
/// whether to slide along or walk on these surfaces.
#[derive(Component, Clone, Debug)]
pub struct SlopeHandling {
    /// The maximum angle (in radians) where a surface is considered ground/ceiling
    /// relative to the [`UpDirection`]. Outside this angle, surfaces are considered walls.
    ///
    /// **Default**: 45 degrees (π / 4 radians)
    pub max_ground_angle: f32,

    /// The maximum angle (in radians) where sliding along walls is allowed relative
    /// to the [`UpDirection`]. Walls steeper than this angle are considered to be vertical,
    /// and sliding along them will not affect vertical movement.
    ///
    /// The angle should be greater than [`max_ground_angle`](Self::max_ground_angle),
    /// but less than 90 degrees (π / 2 radians).
    ///
    /// **Default**: 75 degrees (5π / 12 radians)
    pub max_wall_slide_angle: f32,

    /// Whether the character should slide down along slopes when colliding with them.
    ///
    /// **Default**: `false`
    pub slide_on_ground: bool,

    /// Whether the character should slide up along ceilings when colliding with them.
    ///
    /// **Default**: `true`
    pub slide_on_ceiling: bool,
}

impl Default for SlopeHandling {
    fn default() -> Self {
        Self {
            max_ground_angle: core::f32::consts::FRAC_PI_4, // 45 degrees
            max_wall_slide_angle: 75f32.to_radians(),       // 75 degrees
            slide_on_ground: false,
            slide_on_ceiling: true,
        }
    }
}

/// Component for configuring ground snapping behavior for [`VirtualCharacter`]s.
///
/// Ground snapping forces characters to stick to the ground when the following conditions
/// are met simultaneously:
///
/// 1. At the start of the movement, the character is [`Grounded`].
/// 2. The movement has a downward component (unless [`skip_upward_movement`](Self::skip_upward_movement) is `false`).
/// 3. After movement, the character is separated from the ground by less than [`max_snap_distance`](Self::max_snap_distance).
///
/// Typical use cases for ground snapping include remaining grounded when moving down small slopes
/// or steps or when walking over uneven terrain.
///
/// Adding this component will also add the [`SlopeHandling`] component with default settings
/// if it is not already present.
#[derive(Component, Clone, Debug)]
#[require(SlopeHandling)]
pub struct GroundSnapping {
    /// The maximum distance from the ground within which snapping is performed.
    ///
    /// **Default**: `0.1`
    pub max_snap_distance: f32,

    /// If `true`, snapping is only performed when the character is moving downwards.
    ///
    /// This helps prevent snapping when jumping or getting pushed upwards.
    ///
    /// **Default**: `true`
    pub skip_upward_movement: bool,
}

impl Default for GroundSnapping {
    fn default() -> Self {
        Self {
            max_snap_distance: 0.1,
            skip_upward_movement: true,
        }
    }
}

/// Component for configuring stair stepping behavior for [`VirtualCharacter`]s.
#[derive(Component, Clone, Debug)]
pub struct StairStepping {
    /// The maximum height of a stair that can be stepped onto.
    ///
    /// Measured along the [`UpDirection`].
    ///
    /// **Default**: `0.3`
    pub max_height: f32,

    /// The minimum width of a stair that can be stepped onto.
    ///
    /// Measured perpendicular to the [`UpDirection`].
    ///
    /// **Default**: `0.2`
    pub min_width: f32,
}

impl Default for StairStepping {
    fn default() -> Self {
        Self {
            max_height: 0.3,
            min_width: 0.2,
        }
    }
}
