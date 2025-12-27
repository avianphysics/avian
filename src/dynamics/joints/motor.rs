use crate::prelude::*;
use bevy::prelude::*;

/// The motor model determines how the motor force/torque is computed.
#[derive(Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub enum MotorModel {
    /// The motor force/torque is computed based on the acceleration required to reach the target.
    ///
    /// This model is more stable and easier to tune, but it ignores the mass of the bodies.
    ///
    /// - `stiffness`: The stiffness coefficient for position control. Set to zero for pure velocity control.
    /// - `damping`: The damping coefficient.
    AccelerationBased {
        /// The stiffness coefficient for position control.
        stiffness: Scalar,
        /// The damping coefficient.
        damping: Scalar,
    },
    /// The motor force/torque is computed directly from the stiffness and damping parameters.
    ///
    /// This model takes the mass of the bodies into account, resulting in more physically
    /// accurate behavior, but it may be harder to tune.
    ///
    /// - `stiffness`: The stiffness coefficient for position control. Set to zero for pure velocity control.
    /// - `damping`: The damping coefficient.
    ForceBased {
        /// The stiffness coefficient for position control.
        stiffness: Scalar,
        /// The damping coefficient.
        damping: Scalar,
    },
    /// A spring-damper model using implicit Euler integration for timestep-independent behavior.
    ///
    /// This model provides stable, predictable spring-damper dynamics regardless of the
    /// physics substep count.
    ///
    /// - `frequency`: The natural frequency of the spring in Hz. Higher values create stiffer springs.
    /// - `damping_ratio`: The damping ratio.
    ///   - 0.0 = no damping (oscillates forever)
    ///   - 1.0 = critically damped (fastest approach without overshoot)
    ///   - \> 1.0 = overdamped (slower approach without overshoot)
    ///   - < 1.0 = underdamped (overshoots and oscillates)
    SpringDamper {
        /// The natural frequency of the spring in Hz.
        frequency: Scalar,
        /// The damping ratio.
        damping_ratio: Scalar,
    },
}

impl Default for MotorModel {
    fn default() -> Self {
        Self::AccelerationBased {
            stiffness: 0.0,
            damping: 0.0,
        }
    }
}

/// A motor for driving the angular motion of a [`RevoluteJoint`].
///
/// Motors are configured as part of a joint, applying torque to drive
/// the joint towards a target velocity and/or position.
///
/// # Timestep-Independent Spring-Damper
///
/// For position control that behaves consistently regardless of substep count, use
/// [`MotorModel::SpringDamper`]. This uses an implicit Euler integration that provides
/// stable, predictable spring-damper behavior.
///
/// ```ignore
/// RevoluteJoint::new(entity1, entity2)
///     .with_motor(
///         AngularMotor::new(0.0)
///             .with_spring_damper(2.0, 1.0) // 2 Hz, critically damped
///             .with_target_position_value(target_angle)
///     )
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct AngularMotor {
    /// The target angular velocity (rad/s).
    pub target_velocity: Scalar,
    /// The target angle (rad) for position control.
    pub target_position: Scalar,
    /// The maximum torque the motor can apply (N·m).
    pub max_torque: Scalar,
    /// The motor model used for computing the motor torque.
    pub motor_model: MotorModel,
}

impl Default for AngularMotor {
    fn default() -> Self {
        Self {
            target_velocity: 0.0,
            target_position: 0.0,
            max_torque: Scalar::MAX,
            motor_model: MotorModel::default(),
        }
    }
}

impl AngularMotor {
    /// Creates a new angular motor with the given target velocity.
    ///
    /// This creates a velocity-mode motor with no position control.
    #[inline]
    pub const fn new(target_velocity: Scalar) -> Self {
        Self {
            target_velocity,
            target_position: 0.0,
            max_torque: Scalar::MAX,
            motor_model: MotorModel::AccelerationBased {
                stiffness: 0.0,
                damping: 0.0,
            },
        }
    }

    /// Sets the target angular velocity in radians per second.
    #[inline]
    pub const fn with_target_velocity(mut self, velocity: Scalar) -> Self {
        self.target_velocity = velocity;
        self
    }

    /// Sets the target position.
    #[inline]
    pub const fn with_target_position_value(mut self, target_position: Scalar) -> Self {
        self.target_position = target_position;
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

    /// Sets the motor to use a spring-damper model for timestep-independent position control.
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
    ///   - \> 1.0 = overdamped (slower approach without overshoot)
    ///   - < 1.0 = underdamped (overshoots and oscillates)
    ///
    /// # Example
    ///
    /// ```ignore
    /// // Create a motor that acts like a critically damped spring at 2 Hz
    /// let motor = AngularMotor::new(0.0)
    ///     .with_spring_damper(2.0, 1.0)
    ///     .with_target_position_value(target_angle);
    /// ```
    #[inline]
    pub const fn with_spring_damper(mut self, frequency: Scalar, damping_ratio: Scalar) -> Self {
        self.motor_model = MotorModel::SpringDamper {
            frequency,
            damping_ratio,
        };
        self
    }
}

/// A motor for driving the linear motion of a [`PrismaticJoint`].
///
/// Motors are configured as part of a joint, applying force to drive
/// the joint towards a target velocity and/or position.
///
/// # Timestep-Independent Spring-Damper
///
/// For position control that behaves consistently regardless of substep count, use
/// [`MotorModel::SpringDamper`]. This uses an implicit Euler integration that provides
/// stable, predictable spring-damper behavior.
///
/// ```ignore
/// PrismaticJoint::new(entity1, entity2)
///     .with_motor(
///         LinearMotor::new(0.0)
///             .with_spring_damper(2.0, 1.0) // 2 Hz, critically damped
///             .with_target_position_value(target_position)
///     )
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct LinearMotor {
    /// The target linear velocity (m/s).
    pub target_velocity: Scalar,
    /// The target position (m) for position control.
    pub target_position: Scalar,
    /// The maximum force the motor can apply (N).
    pub max_force: Scalar,
    /// The motor model used for computing the motor force.
    pub motor_model: MotorModel,
}

impl Default for LinearMotor {
    fn default() -> Self {
        Self {
            target_velocity: 0.0,
            target_position: 0.0,
            max_force: Scalar::MAX,
            motor_model: MotorModel::default(),
        }
    }
}

impl LinearMotor {
    /// Creates a new linear motor with the given target velocity.
    ///
    /// This creates a velocity-mode motor with no position control.
    #[inline]
    pub const fn new(target_velocity: Scalar) -> Self {
        Self {
            target_velocity,
            target_position: 0.0,
            max_force: Scalar::MAX,
            motor_model: MotorModel::AccelerationBased {
                stiffness: 0.0,
                damping: 0.0,
            },
        }
    }

    /// Sets the target linear velocity in meters per second.
    #[inline]
    pub const fn with_target_velocity(mut self, velocity: Scalar) -> Self {
        self.target_velocity = velocity;
        self
    }

    /// Sets the target position.
    #[inline]
    pub const fn with_target_position_value(mut self, target_position: Scalar) -> Self {
        self.target_position = target_position;
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

    /// Sets the motor to use a spring-damper model for timestep-independent position control.
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
    ///   - \> 1.0 = overdamped (slower approach without overshoot)
    ///   - < 1.0 = underdamped (overshoots and oscillates)
    ///
    /// # Example
    ///
    /// ```ignore
    /// // Create a motor that acts like a critically damped spring at 2 Hz
    /// let motor = LinearMotor::new(0.0)
    ///     .with_spring_damper(2.0, 1.0)
    ///     .with_target_position_value(target_position);
    /// ```
    #[inline]
    pub const fn with_spring_damper(mut self, frequency: Scalar, damping_ratio: Scalar) -> Self {
        self.motor_model = MotorModel::SpringDamper {
            frequency,
            damping_ratio,
        };
        self
    }
}
