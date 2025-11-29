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
        &mut Position,
        &Rotation,
        &mut LinearVelocity,
        Option<&Collider>,
        &AutoMoveAndSlide,
    )>,
    move_and_slide: MoveAndSlide,
    time: Res<Time>,
) {
    query.par_iter_mut().for_each(
        |(entity, mut position, rotation, mut lin_vel, collider, mns_config)| {
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
            position.0 = new_position;
            lin_vel.0 = projected_velocity;
        },
    );
}
