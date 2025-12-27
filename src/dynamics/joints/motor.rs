use crate::prelude::*;
use bevy::prelude::*;

/// The motor model determines how the motor force/torque is computed.
#[derive(Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub enum MotorModel {
    /// A spring-damper model using implicit Euler integration for timestep-independent behavior.
    ///
    /// This model provides stable, predictable spring-damper dynamics regardless of the
    /// physics substep count. Ignores the mass of the bodies.
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
    /// The motor force/torque is computed based on the acceleration required to reach the target.
    ///
    /// This model is more stable than the `ForceBased` model and easier to tune, but it ignores the mass of the bodies.
    /// Prefer the `SpringDamper` model if it's an option.
    ///
    /// - `stiffness`: The stiffness coefficient for position control. Set to zero for pure velocity control.
    /// - `damping`: The damping coefficient.
    AccelerationBased {
        /// The stiffness coefficient for position control.
        stiffness: Scalar,
        /// The damping coefficient.
        damping: Scalar,
    },
}

impl Default for MotorModel {
    fn default() -> Self {
        Self::SpringDamper {
            frequency: 5.,
            damping_ratio: 1.,
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
///         AngularMotor::new(MotorModel::SpringDamper {
///             frequency: 2.0,
///             damping_ratio: 1.0,
///         })
///         .with_target_position(target_angle)
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
    /// Creates a new angular motor with the given motor model.
    #[inline]
    pub const fn new(motor_model: MotorModel) -> Self {
        Self {
            target_velocity: 0.0,
            target_position: 0.0,
            max_torque: Scalar::MAX,
            motor_model,
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
    pub const fn with_target_position(mut self, target_position: Scalar) -> Self {
        self.target_position = target_position;
        self
    }

    /// Sets the maximum torque the motor can apply.
    #[inline]
    pub const fn with_max_torque(mut self, max_torque: Scalar) -> Self {
        self.max_torque = max_torque;
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
///         LinearMotor::new(MotorModel::SpringDamper {
///             frequency: 2.0,
///             damping_ratio: 1.0,
///         })
///         .with_target_position(target_position)
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
    /// Creates a new linear motor with the given motor model.
    #[inline]
    pub const fn new(motor_model: MotorModel) -> Self {
        Self {
            target_velocity: 0.0,
            target_position: 0.0,
            max_force: Scalar::MAX,
            motor_model,
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
    pub const fn with_target_position(mut self, target_position: Scalar) -> Self {
        self.target_position = target_position;
        self
    }

    /// Sets the maximum force the motor can apply.
    #[inline]
    pub const fn with_max_force(mut self, max_force: Scalar) -> Self {
        self.max_force = max_force;
        self
    }
}
