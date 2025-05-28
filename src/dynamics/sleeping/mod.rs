//! Manages sleeping and waking for bodies, automatically deactivating them to save computational resources.
//!
//! See [`SleepingPlugin`].

use crate::prelude::*;
use bevy::{
    ecs::{component::Tick, system::SystemChangeTick},
    prelude::*,
};

/// Manages sleeping and waking for bodies, automatically deactivating them to save computational resources.
///
/// Bodies are marked as [`Sleeping`] when their linear and angular velocities are below the [`SleepingThreshold`]
/// for a duration indicated by [`DeactivationTime`].
///
/// Bodies are woken up when an active body or constraint interacts with them, or when gravity changes,
/// or when the body's position, rotation, velocity, or external forces are changed.
///
/// This plugin does *not* handle constraints waking up bodies. That is done by the [solver](dynamics::solver).
///
/// The sleeping systems run in [`PhysicsStepSet::Sleeping`].
pub struct SleepingPlugin;

impl Plugin for SleepingPlugin {
    fn build(&self, app: &mut App) {
        // TODO: This is only relevant for dynamic bodies. Different bodies should be distinguished with marker components.
        // Add sleep timer for all rigid bodies.
        let _ = app.try_register_required_components::<RigidBody, TimeSleeping>();

        app.init_resource::<SleepingThreshold>()
            .init_resource::<DeactivationTime>()
            .init_resource::<LastPhysicsTick>();

        let physics_schedule = app
            .get_schedule_mut(PhysicsSchedule)
            .expect("add PhysicsSchedule first");

        // TODO: Where exactly should this be in the schedule?
        physics_schedule.add_systems(
            (
                wake_on_changed,
                wake_all_sleeping_bodies.run_if(resource_changed::<Gravity>),
                mark_sleeping_bodies,
            )
                .chain()
                .after(PhysicsStepSet::First)
                .before(PhysicsStepSet::BroadPhase),
        );

        physics_schedule.add_systems(
            (|mut last_physics_tick: ResMut<LastPhysicsTick>,
              system_change_tick: SystemChangeTick| {
                last_physics_tick.0 = system_change_tick.this_run();
            })
            .after(PhysicsStepSet::Last),
        );
    }
}

/// A threshold that indicates the maximum linear and angular velocity allowed for a body to be deactivated.
///
/// Setting a negative sleeping threshold disables sleeping entirely.
///
/// See [`Sleeping`] for further information about sleeping.
#[derive(Reflect, Resource, Clone, Copy, PartialEq, PartialOrd, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Resource, PartialEq)]
pub struct SleepingThreshold {
    /// The maximum linear velocity allowed for a body to be marked as sleeping.
    ///
    /// This is implicitly scaled by the [`PhysicsLengthUnit`].
    ///
    /// Default: `0.15`
    pub linear: Scalar,
    /// The maximum angular velocity allowed for a body to be marked as sleeping.
    ///
    /// Default: `0.15`
    pub angular: Scalar,
}

impl Default for SleepingThreshold {
    fn default() -> Self {
        Self {
            linear: 0.15,
            angular: 0.15,
        }
    }
}

/// How long in seconds the linear and angular velocity of a body need to be below
/// the [`SleepingThreshold`] before the body is deactivated. Defaults to 1 second.
///
/// See [`Sleeping`] for further information about sleeping.
///
/// Default: `0.5`
#[derive(Reflect, Resource, Clone, Copy, PartialEq, PartialOrd, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Default, PartialEq)]
pub struct DeactivationTime(pub Scalar);

impl Default for DeactivationTime {
    fn default() -> Self {
        Self(0.5)
    }
}

/// A [`Command`] that wakes up a [rigid body](RigidBody) by removing the [`Sleeping`] component
/// and resetting the [`TimeSleeping`] to zero.
pub struct WakeUpBody(pub Entity);

impl Command for WakeUpBody {
    fn apply(self, world: &mut World) {
        let Ok(mut entity_mut) = world.get_entity_mut(self.0) else {
            return;
        };

        entity_mut.remove::<Sleeping>();

        if let Some(mut time_sleeping) = entity_mut.get_mut::<TimeSleeping>() {
            time_sleeping.0 = 0.0;
        };
    }
}

/// Adds the [`Sleeping`] component to bodies whose linear and anigular velocities have been
/// under the [`SleepingThreshold`] for a duration indicated by [`DeactivationTime`].
#[allow(clippy::type_complexity)]
pub fn mark_sleeping_bodies(
    mut commands: Commands,
    mut query: Query<
        (
            Entity,
            &RigidBody,
            &mut LinearVelocity,
            &mut AngularVelocity,
            &mut TimeSleeping,
        ),
        (Without<Sleeping>, Without<SleepingDisabled>),
    >,
    contact_graph: Res<ContactGraph>,
    rb_query: Query<&RigidBody>,
    deactivation_time: Res<DeactivationTime>,
    sleep_threshold: Res<SleepingThreshold>,
    length_unit: Res<PhysicsLengthUnit>,
    time: Res<Time>,
) {
    let length_unit_sq = length_unit.powi(2);
    let delta_secs = time.delta_seconds_adjusted();

    for (entity, rb, mut lin_vel, mut ang_vel, mut time_sleeping) in &mut query {
        let colliding_entities = contact_graph.collisions_with(entity).map(|c| {
            if entity == c.collider1 {
                c.collider2
            } else {
                c.collider1
            }
        });

        // Only dynamic bodies can sleep, and only if they are not
        // in contact with other dynamic bodies.
        //
        // Contacts with other types of bodies will be allowed once
        // sleeping/waking is implemented with simulation islands.
        if !rb.is_dynamic()
            || rb_query
                .iter_many(colliding_entities)
                .any(|rb| rb.is_dynamic())
        {
            continue;
        }

        let lin_vel_sq = lin_vel.length_squared();

        #[cfg(feature = "2d")]
        let ang_vel_sq = ang_vel.0.powi(2);
        #[cfg(feature = "3d")]
        let ang_vel_sq = ang_vel.0.dot(ang_vel.0);

        // Negative thresholds indicate that sleeping is disabled.
        let lin_sleeping_threshold_sq =
            length_unit_sq * sleep_threshold.linear * sleep_threshold.linear.abs();
        let ang_sleeping_threshold_sq = sleep_threshold.angular * sleep_threshold.angular.abs();

        // If linear and angular velocity are below the sleeping threshold,
        // add delta time to the time sleeping, i.e. the time that the body has remained still.
        if lin_vel_sq < lin_sleeping_threshold_sq && ang_vel_sq < ang_sleeping_threshold_sq {
            time_sleeping.0 += delta_secs;
        } else {
            time_sleeping.0 = 0.0;
        }

        // If the body has been still for long enough, set it to sleep and reset velocities.
        if time_sleeping.0 > deactivation_time.0 {
            commands.entity(entity).try_insert(Sleeping);
            *lin_vel = LinearVelocity::ZERO;
            *ang_vel = AngularVelocity::ZERO;
        }
    }
}

/// A [`Tick`] corresponding to the end of the previous run of the [`PhysicsSchedule`].
#[derive(Resource, Reflect, Default)]
#[reflect(Resource, Default)]
pub(crate) struct LastPhysicsTick(pub Tick);

/// Removes the [`Sleeping`] component from sleeping bodies when properties like
/// position, rotation, velocity and external forces are changed by the user.
#[allow(clippy::type_complexity)]
pub(crate) fn wake_on_changed(
    mut commands: Commands,
    mut query: ParamSet<(
        // These could've been changed by physics too.
        // We need to ignore non-user changes.
        Query<
            (
                Entity,
                Ref<Position>,
                Ref<Rotation>,
                Ref<LinearVelocity>,
                Ref<AngularVelocity>,
            ),
            (
                With<Sleeping>,
                Or<(
                    Changed<Position>,
                    Changed<Rotation>,
                    Changed<LinearVelocity>,
                    Changed<AngularVelocity>,
                )>,
            ),
        >,
        // These are not modified by the physics engine
        // and don't need special handling.
        Query<
            Entity,
            Or<(
                Changed<ExternalForce>,
                Changed<ExternalTorque>,
                Changed<ExternalImpulse>,
                Changed<ExternalAngularImpulse>,
                Changed<GravityScale>,
            )>,
        >,
    )>,
    last_physics_tick: Res<LastPhysicsTick>,
    system_tick: SystemChangeTick,
) {
    let this_run = system_tick.this_run();

    for (entity, pos, rot, lin_vel, ang_vel) in &query.p0() {
        if is_changed_after_tick(pos, last_physics_tick.0, this_run)
            || is_changed_after_tick(rot, last_physics_tick.0, this_run)
            || is_changed_after_tick(lin_vel, last_physics_tick.0, this_run)
            || is_changed_after_tick(ang_vel, last_physics_tick.0, this_run)
        {
            commands.queue(WakeUpBody(entity));
        }
    }

    for entity in &query.p1() {
        commands.queue(WakeUpBody(entity));
    }
}

fn is_changed_after_tick<C: Component>(component_ref: Ref<C>, tick: Tick, this_run: Tick) -> bool {
    let last_changed = component_ref.last_changed();
    component_ref.is_changed() && last_changed.is_newer_than(tick, this_run)
}

/// Removes the [`Sleeping`] component from all sleeping bodies.
/// Triggered automatically when [`Gravity`] is changed.
fn wake_all_sleeping_bodies(mut commands: Commands, bodies: Query<Entity, With<Sleeping>>) {
    for entity in &bodies {
        commands.queue(WakeUpBody(entity));
    }
}
