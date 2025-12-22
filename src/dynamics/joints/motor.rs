use crate::prelude::*;
use bevy::prelude::*;

/// The motor model determines how the motor force/torque is computed.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq, Hash)]
pub enum MotorModel {
    /// The motor force/torque is computed based on the acceleration required to reach the target.
    ///
    /// This model is more stable and easier to tune, but it ignores the mass of the bodies.
    #[default]
    AccelerationBased,
    /// The motor force/torque is computed directly from the stiffness and damping parameters.
    ///
    /// This model takes the mass of the bodies into account, resulting in more physically
    /// accurate behavior, but it may be harder to tune.
    ForceBased,
}

/// A motor for driving the angular motion of a [`RevoluteJoint`].
///
/// When attached to a revolute joint entity, this component applies torque
/// to drive the joint towards a target velocity and/or position.
///
/// For velocity control, set [`target_velocity`](Self::target_velocity) with zero [`stiffness`](Self::stiffness).
/// For position control, set [`target_position`](Self::target_position) with non-zero [`stiffness`](Self::stiffness).
///
/// # Timestep-Independent Spring-Damper
///
/// For position control that behaves consistently regardless of substep count, use
/// [`with_spring_parameters`](Self::with_spring_parameters) to set `frequency` and `damping_ratio`
/// instead of raw `stiffness` and `damping`. This uses an implicit Euler integration
/// that provides stable, predictable spring-damper behavior.
///
/// ```ignore
/// AngularJointMotor::new(0.0)
///     .with_spring_parameters(2.0, 0.7) // 2 Hz, critically damped
///     .with_target_position(target_angle)
/// ```
#[derive(Component, Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, PartialEq)]
pub struct AngularJointMotor {
    /// The target angular velocity (rad/s).
    pub target_velocity: Scalar,
    /// The target angle (rad) for position control. Only used when `stiffness` is non-zero.
    pub target_position: Scalar,
    /// The stiffness coefficient for position control (N·m/rad). Set to zero for pure velocity control.
    ///
    /// Note: If [`frequency`](Self::frequency) is set, it takes precedence over this value.
    pub stiffness: Scalar,
    /// The damping coefficient (N·m·s/rad).
    ///
    /// Note: If [`damping_ratio`](Self::damping_ratio) is set, it takes precedence over this value.
    pub damping: Scalar,
    /// The maximum torque the motor can apply (N·m).
    pub max_torque: Scalar,
    /// The motor model used for computing the motor torque.
    pub motor_model: MotorModel,
    /// The natural frequency of the spring-damper system in Hz.
    ///
    /// When set, this provides timestep-independent spring behavior using implicit Euler integration.
    /// Use [`with_spring_parameters`](Self::with_spring_parameters) to set this along with `damping_ratio`.
    pub frequency: Option<Scalar>,
    /// The damping ratio for the spring-damper system.
    ///
    /// - 0.0 = no damping (oscillates forever)
    /// - 1.0 = critically damped (fastest approach without overshoot)
    /// - > 1.0 = overdamped (slow approach without overshoot)
    /// - < 1.0 = underdamped (overshoots and oscillates)
    ///
    /// When set, this provides timestep-independent damping behavior using implicit Euler integration.
    /// Use [`with_spring_parameters`](Self::with_spring_parameters) to set this along with `frequency`.
    pub damping_ratio: Option<Scalar>,
}

impl Default for AngularJointMotor {
    fn default() -> Self {
        Self {
            target_velocity: 0.0,
            target_position: 0.0,
            stiffness: 0.0,
            damping: 0.0,
            max_torque: Scalar::MAX,
            motor_model: MotorModel::default(),
            frequency: None,
            damping_ratio: None,
        }
    }
}

impl AngularJointMotor {
    /// Creates a new angular motor with the given target velocity.
    ///
    /// This creates a velocity-mode motor with no position control.
    #[inline]
    pub const fn new(target_velocity: Scalar) -> Self {
        Self {
            target_velocity,
            target_position: 0.0,
            stiffness: 0.0,
            damping: 0.0,
            max_torque: Scalar::MAX,
            motor_model: MotorModel::AccelerationBased,
            frequency: None,
            damping_ratio: None,
        }
    }

    /// Creates a new angular motor targeting a specific position.
    ///
    /// This creates a position-mode motor that acts like a spring.
    #[inline]
    pub const fn with_target_position(target_position: Scalar, stiffness: Scalar) -> Self {
        Self {
            target_velocity: 0.0,
            target_position,
            stiffness,
            damping: 0.0,
            max_torque: Scalar::MAX,
            motor_model: MotorModel::AccelerationBased,
            frequency: None,
            damping_ratio: None,
        }
    }

    /// Sets the target angular velocity in radians per second.
    #[inline]
    pub const fn with_target_velocity(mut self, velocity: Scalar) -> Self {
        self.target_velocity = velocity;
        self
    }

    /// Sets the target position and stiffness for position control.
    #[inline]
    pub const fn with_position_target(
        mut self,
        target_position: Scalar,
        stiffness: Scalar,
    ) -> Self {
        self.target_position = target_position;
        self.stiffness = stiffness;
        self
    }

    /// Sets the damping coefficient.
    #[inline]
    pub const fn with_damping(mut self, damping: Scalar) -> Self {
        self.damping = damping;
        self
    }

    /// Sets the maximum torque the motor can apply.
    #[inline]
    pub const fn with_max_torque(mut self, max_torque: Scalar) -> Self {
        self.max_torque = max_torque;
        self
    }

    /// Sets the motor model.
    #[inline]
    pub const fn with_motor_model(mut self, model: MotorModel) -> Self {
        self.motor_model = model;
        self
    }

    /// Sets the spring-damper parameters for timestep-independent position control.
    ///
    /// This uses an implicit Euler integration formula that provides stable, predictable
    /// spring-damper behavior regardless of the physics substep count.
    ///
    /// # Parameters
    ///
    /// - `frequency`: The natural frequency of the spring in Hz. Higher values create stiffer springs.
    ///   A frequency of 1.0 Hz means the spring completes one oscillation per second (if underdamped).
    /// - `damping_ratio`: The damping ratio.
    ///   - 0.0 = no damping (oscillates forever)
    ///   - 1.0 = critically damped (fastest approach without overshoot)
    ///   - > 1.0 = overdamped (slower approach without overshoot)
    ///   - < 1.0 = underdamped (overshoots and oscillates)
    ///
    /// # Example
    ///
    /// ```ignore
    /// // Create a motor that acts like a critically damped spring at 2 Hz
    /// let motor = AngularJointMotor::new(0.0)
    ///     .with_spring_parameters(2.0, 1.0)
    ///     .with_target_position(target_angle);
    /// ```
    #[inline]
    pub const fn with_spring_parameters(
        mut self,
        frequency: Scalar,
        damping_ratio: Scalar,
    ) -> Self {
        self.frequency = Some(frequency);
        self.damping_ratio = Some(damping_ratio);
        self
    }

    /// Sets the target position for spring-damper control.
    ///
    /// This is similar to [`with_position_target`](Self::with_position_target) but doesn't
    /// require setting stiffness, which is useful when using spring parameters via
    /// [`with_spring_parameters`](Self::with_spring_parameters).
    #[inline]
    pub const fn with_target_position_value(mut self, target_position: Scalar) -> Self {
        self.target_position = target_position;
        self
    }

    /// Returns `true` if this motor uses spring-damper parameters (`frequency` and `damping_ratio`).
    #[inline]
    pub const fn uses_spring_parameters(&self) -> bool {
        self.frequency.is_some()
    }
}

/// A motor for driving the linear motion of a [`PrismaticJoint`].
///
/// When attached to a prismatic joint entity, this component applies force
/// to drive the joint towards a target velocity and/or position.
///
/// For velocity control, set [`target_velocity`](Self::target_velocity) with zero [`stiffness`](Self::stiffness).
/// For position control, set [`target_position`](Self::target_position) with non-zero [`stiffness`](Self::stiffness).
///
/// # Timestep-Independent Spring-Damper
///
/// For position control that behaves consistently regardless of substep count, use
/// [`with_spring_parameters`](Self::with_spring_parameters) to set `frequency` and `damping_ratio`
/// instead of raw `stiffness` and `damping`. This uses an implicit Euler integration
/// that provides stable, predictable spring-damper behavior.
///
/// ```ignore
/// LinearJointMotor::new(0.0)
///     .with_spring_parameters(2.0, 0.7) // 2 Hz, critically damped
///     .with_target_position(target_position)
/// ```
#[derive(Component, Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, PartialEq)]
pub struct LinearJointMotor {
    /// The target linear velocity (m/s).
    pub target_velocity: Scalar,
    /// The target position (m) for position control. Only used when `stiffness` is non-zero.
    pub target_position: Scalar,
    /// The stiffness coefficient for position control (N/m). Set to zero for pure velocity control.
    ///
    /// Note: If [`frequency`](Self::frequency) is set, it takes precedence over this value.
    pub stiffness: Scalar,
    /// The damping coefficient (N·s/m).
    ///
    /// Note: If [`damping_ratio`](Self::damping_ratio) is set, it takes precedence over this value.
    pub damping: Scalar,
    /// The maximum force the motor can apply (N).
    pub max_force: Scalar,
    /// The motor model used for computing the motor force.
    pub motor_model: MotorModel,
    /// The natural frequency of the spring-damper system in Hz.
    ///
    /// When set, this provides timestep-independent spring behavior using implicit Euler integration.
    /// Use [`with_spring_parameters`](Self::with_spring_parameters) to set this along with `damping_ratio`.
    pub frequency: Option<Scalar>,
    /// The damping ratio for the spring-damper system.
    ///
    /// - 0.0 = no damping (oscillates forever)
    /// - 1.0 = critically damped (fastest approach without overshoot)
    /// - > 1.0 = overdamped (slow approach without overshoot)
    /// - < 1.0 = underdamped (overshoots and oscillates)
    ///
    /// When set, this provides timestep-independent damping behavior using implicit Euler integration.
    /// Use [`with_spring_parameters`](Self::with_spring_parameters) to set this along with `frequency`.
    pub damping_ratio: Option<Scalar>,
}

impl Default for LinearJointMotor {
    fn default() -> Self {
        Self {
            target_velocity: 0.0,
            target_position: 0.0,
            stiffness: 0.0,
            damping: 0.0,
            max_force: Scalar::MAX,
            motor_model: MotorModel::default(),
            frequency: None,
            damping_ratio: None,
        }
    }
}

impl LinearJointMotor {
    /// Creates a new linear motor with the given target velocity.
    ///
    /// This creates a velocity-mode motor with no position control.
    #[inline]
    pub const fn new(target_velocity: Scalar) -> Self {
        Self {
            target_velocity,
            target_position: 0.0,
            stiffness: 0.0,
            damping: 0.0,
            max_force: Scalar::MAX,
            motor_model: MotorModel::AccelerationBased,
            frequency: None,
            damping_ratio: None,
        }
    }

    /// Creates a new linear motor targeting a specific position.
    ///
    /// This creates a position-mode motor that acts like a spring.
    #[inline]
    pub const fn with_target_position(target_position: Scalar, stiffness: Scalar) -> Self {
        Self {
            target_velocity: 0.0,
            target_position,
            stiffness,
            damping: 0.0,
            max_force: Scalar::MAX,
            motor_model: MotorModel::AccelerationBased,
            frequency: None,
            damping_ratio: None,
        }
    }

    /// Sets the target linear velocity in meters per second.
    #[inline]
    pub const fn with_target_velocity(mut self, velocity: Scalar) -> Self {
        self.target_velocity = velocity;
        self
    }

    /// Sets the target position and stiffness for position control.
    #[inline]
    pub const fn with_position_target(
        mut self,
        target_position: Scalar,
        stiffness: Scalar,
    ) -> Self {
        self.target_position = target_position;
        self.stiffness = stiffness;
        self
    }

    /// Sets the damping coefficient.
    #[inline]
    pub const fn with_damping(mut self, damping: Scalar) -> Self {
        self.damping = damping;
        self
    }

    /// Sets the maximum force the motor can apply.
    #[inline]
    pub const fn with_max_force(mut self, max_force: Scalar) -> Self {
        self.max_force = max_force;
        self
    }

    /// Sets the motor model.
    #[inline]
    pub const fn with_motor_model(mut self, model: MotorModel) -> Self {
        self.motor_model = model;
        self
    }

    /// Sets the spring-damper parameters for timestep-independent position control.
    ///
    /// This uses an implicit Euler integration formula that provides stable, predictable
    /// spring-damper behavior regardless of the physics substep count.
    ///
    /// # Parameters
    ///
    /// - `frequency`: The natural frequency of the spring in Hz. Higher values create stiffer springs.
    ///   A frequency of 1.0 Hz means the spring completes one oscillation per second (if underdamped).
    /// - `damping_ratio`: The damping ratio.
    ///   - 0.0 = no damping (oscillates forever)
    ///   - 1.0 = critically damped (fastest approach without overshoot)
    ///   - > 1.0 = overdamped (slower approach without overshoot)
    ///   - < 1.0 = underdamped (overshoots and oscillates)
    ///
    /// # Example
    ///
    /// ```ignore
    /// // Create a motor that acts like a critically damped spring at 2 Hz
    /// let motor = LinearJointMotor::new(0.0)
    ///     .with_spring_parameters(2.0, 1.0)
    ///     .with_target_position(target_position);
    /// ```
    #[inline]
    pub const fn with_spring_parameters(
        mut self,
        frequency: Scalar,
        damping_ratio: Scalar,
    ) -> Self {
        self.frequency = Some(frequency);
        self.damping_ratio = Some(damping_ratio);
        self
    }

    /// Sets the target position for spring-damper control.
    ///
    /// This is similar to [`with_position_target`](Self::with_position_target) but doesn't
    /// require setting stiffness, which is useful when using spring parameters via
    /// [`with_spring_parameters`](Self::with_spring_parameters).
    #[inline]
    pub const fn with_target_position_value(mut self, target_position: Scalar) -> Self {
        self.target_position = target_position;
        self
    }

    /// Returns `true` if this motor uses spring-damper parameters (`frequency` and `damping_ratio`).
    #[inline]
    pub const fn uses_spring_parameters(&self) -> bool {
        self.frequency.is_some()
    }
}
