use crate::prelude::*;
use bevy::prelude::*;

/// Determines how the joint motor force/torque is computed.
///
/// Different models offer trade-offs between ease of tuning and physical accuracy.
/// The default is a [`SpringDamper`](MotorModel::SpringDamper) model that provides
/// stable, predictable behavior across different configurations.
#[derive(Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub enum MotorModel {
    /// A spring-damper model using implicit Euler integration.
    ///
    /// Unlike the other models, this is unconditionally stable: the implicit formulation
    /// naturally limits the response as frequency increases, preventing overshoot and
    /// oscillation even with aggressive parameters. This makes it easier to tune than
    /// the other models, which can become unstable with high stiffness values.
    ///
    /// This is the recommended model for most use cases.
    ///
    /// # Parameters
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
    /// The model can be described by the following formula:
    ///
    /// ```text
    /// force = (stiffness * position_error) + (damping * velocity_error)
    /// ```
    ///
    /// This produces physically accurate forces/torques, but requires careful tuning of the
    /// stiffness and damping parameters based on the masses of the connected bodies.
    /// High stiffness values can cause instability (overshoot, oscillation, or divergence),
    /// so parameters must be chosen appropriately for your timestep and mass configuration.
    ///
    /// # Parameters
    ///
    /// - `stiffness`: The stiffness coefficient for position control. Set to zero for pure velocity control.
    /// - `damping`: The damping coefficient for velocity control.
    ForceBased {
        /// The stiffness coefficient for position control.
        stiffness: Scalar,
        /// The damping coefficient for velocity control.
        damping: Scalar,
    },

    /// The motor force/torque is computed based on the acceleration required to reach the target.
    ///
    /// The model can be described by the following formula:
    ///
    /// ```text
    /// acceleration = (stiffness * position_error) + (damping * velocity_error)
    /// ```
    ///
    /// This automatically scales the motor force/torque based on the masses of the bodies,
    /// resulting in consistent behavior across different mass configurations.
    /// It is therefore easier to tune compared to the [`ForceBased`](MotorModel::ForceBased) model,
    /// which requires manual adjustment of stiffness and damping based on mass.
    ///
    /// Note that high stiffness values can still cause instability. For unconditionally
    /// stable behavior, use the [`SpringDamper`](MotorModel::SpringDamper) model instead.
    ///
    /// # Parameters
    ///
    /// - `stiffness`: The stiffness coefficient for position control. Set to zero for pure velocity control.
    /// - `damping`: The damping coefficient for velocity control.
    AccelerationBased {
        /// The stiffness coefficient for position control.
        stiffness: Scalar,
        /// The damping coefficient for velocity control.
        damping: Scalar,
    },
}

impl Default for MotorModel {
    /// The default motor model: a critically damped spring-damper with 5 Hz frequency.
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl MotorModel {
    /// The default motor model: a critically damped spring-damper with 5 Hz frequency.
    pub const DEFAULT: Self = Self::SpringDamper {
        frequency: 5.0,
        damping_ratio: 1.0,
    };
}

/// A motor for driving the angular motion of a [`RevoluteJoint`].
///
/// Motors are configured as part of a joint, applying torque to drive
/// the joint towards a target velocity and/or position.
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
    /// Whether the motor is enabled.
    pub enabled: bool,
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
        Self::new(MotorModel::DEFAULT)
    }
}

impl AngularMotor {
    /// Creates a new angular motor with the given motor model.
    #[inline]
    pub const fn new(motor_model: MotorModel) -> Self {
        Self {
            enabled: true,
            target_velocity: 0.0,
            target_position: 0.0,
            max_torque: Scalar::MAX,
            motor_model,
        }
    }

    /// Creates a new disabled angular motor with the given motor model.
    ///
    /// To enable the motor later, use [`set_enabled`](Self::set_enabled).
    #[inline]
    pub const fn new_disabled(motor_model: MotorModel) -> Self {
        Self {
            enabled: false,
            ..Self::new(motor_model)
        }
    }

    /// Enables or disables the motor.
    #[inline]
    pub const fn set_enabled(&mut self, enabled: bool) -> &mut Self {
        self.enabled = enabled;
        self
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

    /// Sets the motor model used for computing the motor torque.
    #[inline]
    pub const fn with_motor_model(mut self, motor_model: MotorModel) -> Self {
        self.motor_model = motor_model;
        self
    }
}

/// A motor for driving the linear motion of a [`PrismaticJoint`].
///
/// Motors are configured as part of a joint, applying force to drive
/// the joint towards a target velocity and/or position.
///
/// # Spring-Damper Model
///
/// For stable position control that behaves consistently across different configurations,
/// use [`MotorModel::SpringDamper`]. This uses implicit Euler integration for
/// unconditional stability.
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
    /// Whether the motor is enabled.
    pub enabled: bool,
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
        Self::new(MotorModel::DEFAULT)
    }
}

impl LinearMotor {
    /// Creates a new linear motor with the given motor model.
    #[inline]
    pub const fn new(motor_model: MotorModel) -> Self {
        Self {
            enabled: true,
            target_velocity: 0.0,
            target_position: 0.0,
            max_force: Scalar::MAX,
            motor_model,
        }
    }

    /// Creates a new disabled linear motor with the given motor model.
    ///
    /// To enable the motor later, use [`set_enabled`](Self::set_enabled).
    #[inline]
    pub const fn new_disabled(motor_model: MotorModel) -> Self {
        Self {
            enabled: false,
            ..Self::new(motor_model)
        }
    }

    /// Enables or disables the motor.
    #[inline]
    pub const fn set_enabled(&mut self, enabled: bool) -> &mut Self {
        self.enabled = enabled;
        self
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

    /// Sets the motor model used for computing the motor force.
    #[inline]
    pub const fn with_motor_model(mut self, motor_model: MotorModel) -> Self {
        self.motor_model = motor_model;
        self
    }
}
