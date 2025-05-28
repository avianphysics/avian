//! Clocks used for tracking physics simulation time.

use crate::prelude::*;
use bevy::prelude::*;

/// The clock representing physics time, following the [`Time`] clock used by the schedule that physics runs in.
///
/// In [`FixedPostUpdate`] and other fixed schedules, this uses [`Time<Fixed>`](Fixed), while in schedules such as [`Update`],
/// a variable timestep following [`Time<Virtual>`](Virtual) is used.
///
/// [`Time<Physics>`](Physics) is automatically set as the generic [`Time`] resource for
/// the [`PhysicsSchedule`].
///
/// # Usage
///
/// ## Physics Speed
///
/// The relative speed of [`Time<Physics>`](Physics) can be configured at startup
/// using [`with_relative_speed`], or when the app is running using [`set_relative_speed`]:
///
/// ```no_run
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// fn main() {
///     App::new()
///         .add_plugins((DefaultPlugins, PhysicsPlugins::default()))
///         // Run physics at 0.5 speed
///         .insert_resource(Time::<Physics>::default().with_relative_speed(0.5))
///         .run();
/// }
/// ```
///
/// [`with_relative_speed`]: PhysicsTime::with_relative_speed
/// [`set_relative_speed`]: PhysicsTime::set_relative_speed
///
/// ## Pausing, Resuming, and Stepping Physics
///
/// [`Time<Physics>`](Physics) can be used to pause and resume the simulation:
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// fn pause(mut time: ResMut<Time<Physics>>) {
///     time.pause();
/// }
///
/// fn unpause(mut time: ResMut<Time<Physics>>) {
///     time.unpause();
/// }
/// ```
///
/// To advance the simulation by a certain amount of time instantly, you can advance the
/// [`Time<Physics>`](Physics) clock and manually run the [`PhysicsSchedule`] in an exclusive system:
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
/// use core::time::Duration;
///
/// fn run_physics(world: &mut World) {
///     // Advance the simulation by 10 steps at 120 Hz
///     for _ in 0..10 {
///         world
///             .resource_mut::<Time<Physics>>()
///             .advance_by(Duration::from_secs_f64(1.0 / 120.0));
///         world.run_schedule(PhysicsSchedule);
///     }
/// }
/// ```
///
/// ## When to Multiply by Delta Time?
///
/// Schedules like `Update` use a variable timestep, which can often cause frame rate dependent
/// behavior when moving bodies. One way to help address the issue is by multiplying by delta time.
///
/// In general, if you're doing a *continuous* operation, you should always multiply by delta time,
/// but for *instantaneous* operations it's not necessary.
///
/// Continuous operations move or accelerate bodies over time:
///
/// ```
/// # use bevy::math::Vec3;
/// #
/// # let mut position = Vec3::default();
/// # let mut velocity = Vec3::default();
/// # let mut acceleration = Vec3::default();
/// # let mut delta_time = 1.0 / 60.0;
/// #
/// // Move continuously over time
/// position += velocity * delta_time;
/// // Accelerate continuously
/// velocity += acceleration * delta_time;
/// ```
///
/// Instantaneous operations apply a singular sudden burst of velocity
/// or set it to a specific value:
///
/// ```
/// # use bevy::math::Vec3;
/// #
/// # let mut velocity = Vec3::default();
/// # let mut impulse = Vec3::default();
/// # let mut target_velocity = Vec3::default();
/// #
/// // Apply a burst of speed once (jumps, explosions and so on)
/// velocity += impulse;
/// // Set velocity to a specific value
/// velocity = target_velocity;
/// ```
///
/// For systems using a fixed timestep, using delta time is not necessary for frame rate
/// independence, but it's still recommended so that the physical units are more logical.

#[derive(Reflect, Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct Physics {
    paused: bool,
    relative_speed: f64,
}

impl Default for Physics {
    fn default() -> Self {
        Self {
            paused: false,
            relative_speed: 1.0,
        }
    }
}

/// An extension trait for [`Time<Physics>`](Physics).
pub trait PhysicsTime {
    /// Returns the speed of physics relative to your system clock as an `f32`.
    /// This is also known as "time scaling" or "time dilation" in other engines.
    ///
    /// The speed impacts the accuracy of the simulation, and large values may
    /// cause jittering or missed collisions. You can improve simulation consistency
    /// by adjusting your timestep at the cost of performance.
    fn relative_speed(&self) -> f32;

    /// Returns the speed of physics relative to your system clock as an `f64`.
    /// This is also known as "time scaling" or "time dilation" in other engines.
    ///
    /// The speed impacts the accuracy of the simulation, and large values may
    /// cause jittering or missed collisions. You can improve simulation consistency
    /// by adjusting your timestep at the cost of performance.
    fn relative_speed_f64(&self) -> f64;

    /// Sets the speed of physics relative to your system clock, given as an `f32`.
    ///
    /// For example, setting this to `2.0` will make the physics clock advance twice
    /// as fast as your system clock.
    ///
    /// The speed impacts the accuracy of the simulation, and large values may
    /// cause jittering or missed collisions. You can improve simulation consistency
    /// by adjusting your timestep at the cost of performance.
    ///
    /// # Panics
    ///
    /// Panics if `ratio` is negative or not finite.
    fn with_relative_speed(self, ratio: f32) -> Self;

    /// Sets the speed of physics relative to your system clock, given as an `f64`.
    ///
    /// For example, setting this to `2.0` will make the physics clock advance twice
    /// as fast as your system clock.
    ///
    /// The speed impacts the accuracy of the simulation, and large values may
    /// cause jittering or missed collisions. You can improve simulation consistency
    /// by adjusting your timestep at the cost of performance.
    ///
    /// # Panics
    ///
    /// Panics if `ratio` is negative or not finite.
    fn with_relative_speed_f64(self, ratio: f64) -> Self;

    /// Sets the speed of physics relative to your system clock, given as an `f32`.
    ///
    /// For example, setting this to `2.0` will make the physics clock advance twice
    /// as fast as your system clock.
    ///
    /// The speed impacts the accuracy of the simulation, and large values may
    /// cause jittering or missed collisions. You can improve simulation consistency
    /// by adjusting your timestep at the cost of performance.
    ///
    /// # Panics
    ///
    /// Panics if `ratio` is negative or not finite.
    fn set_relative_speed(&mut self, ratio: f32);

    /// Sets the speed of physics relative to your system clock, given as an `f64`.
    ///
    /// For example, setting this to `2.0` will make the physics clock advance twice
    /// as fast as your system clock.
    ///
    /// The speed impacts the accuracy of the simulation, and large values may
    /// cause jittering or missed collisions. You can improve simulation consistency
    /// by adjusting your timestep at the cost of performance.
    ///
    /// # Panics
    ///
    /// Panics if `ratio` is negative or not finite.
    fn set_relative_speed_f64(&mut self, ratio: f64);

    /// Returns `true` if the physics clock is currently paused.
    fn is_paused(&self) -> bool;

    /// Stops the clock, preventing the physics simulation from advancing until resumed.
    fn pause(&mut self);

    /// Resumes the clock if paused.
    #[doc(alias = "resume")]
    fn unpause(&mut self);
}

impl PhysicsTime for Time<Physics> {
    fn with_relative_speed(self, ratio: f32) -> Self {
        self.with_relative_speed_f64(ratio as f64)
    }

    fn with_relative_speed_f64(mut self, ratio: f64) -> Self {
        assert!(ratio.is_finite(), "tried to go infinitely fast");
        assert!(ratio >= 0.0, "tried to go back in time");
        self.context_mut().relative_speed = ratio;
        self
    }

    fn relative_speed(&self) -> f32 {
        self.relative_speed_f64() as f32
    }

    fn relative_speed_f64(&self) -> f64 {
        self.context().relative_speed
    }

    fn set_relative_speed(&mut self, ratio: f32) {
        self.set_relative_speed_f64(ratio as f64);
    }

    fn set_relative_speed_f64(&mut self, ratio: f64) {
        assert!(ratio.is_finite(), "tried to go infinitely fast");
        assert!(ratio >= 0.0, "tried to go back in time");
        self.context_mut().relative_speed = ratio;
    }

    fn pause(&mut self) {
        self.context_mut().paused = true;
    }

    fn unpause(&mut self) {
        self.context_mut().paused = false;
    }

    fn is_paused(&self) -> bool {
        self.context().paused
    }
}

/// The clock representing physics substep time. It is updated based on
/// [`Time<Physics>`](Physics) and the [`SubstepCount`] resource.
///
/// The clock is automatically set as the generic `Time` resource for
/// the [`SubstepSchedule`].
#[derive(Reflect, Clone, Copy, Debug, Default, PartialEq)]
pub struct Substeps;

pub(crate) trait TimePrecisionAdjusted {
    /// Returns how much time has advanced since the last update
    /// as [`Scalar`] seconds.
    fn delta_seconds_adjusted(&self) -> Scalar;
}

impl TimePrecisionAdjusted for Time {
    /// Returns how much time has advanced since the last [`update`](#method.update)
    /// as [`Scalar`] seconds.
    fn delta_seconds_adjusted(&self) -> Scalar {
        #[cfg(feature = "f32")]
        {
            self.delta_secs()
        }
        #[cfg(feature = "f64")]
        {
            self.delta_secs_f64()
        }
    }
}
