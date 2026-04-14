use crate::prelude::*;
use bevy::prelude::*;

/// The velocity of a [rigid body](RigidBody), representing the speed and direction
/// of movement in world space.
///
/// Velocity consists of:
///
/// - [`linear`](Velocity::linear) component representing the translational velocity
///   of the body, typically in meters per second.
/// - [`angular`](Velocity::angular) component representing the rotational velocity
///   of the body in radians per second.
///
/// # Construction
///
/// The initial velocity of a dynamic body can be set by adding the [`Velocity`] component:
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// # #[cfg(feature = "f32")]
/// fn setup(mut commands: Commands) {
///     // Spawn a dynamic body with an initial velocity of `10.0` units per second towards +X,
///     // and an angular velocity of `1.5` radians per second about the Z axis.
///     commands.spawn((
///         RigidBody::Dynamic,
///         Velocity {
#[cfg_attr(feature = "2d", doc = "            linear: Vec2::new(10.0, 0.0),")]
#[cfg_attr(feature = "3d", doc = "            linear: Vec3::new(10.0, 0.0, 0.0),")]
#[cfg_attr(feature = "2d", doc = "            angular: 1.5,")]
#[cfg_attr(feature = "3d", doc = "            angular: Vec3::new(0.0, 0.0, 1.5),")]
///         },
///     ));
/// }
/// #
/// # #[cfg(feature = "f64")]
/// # fn main() {}
/// ```
///
/// The [`Velocity`] component also provides constructor methods for convenience:
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// #
/// # #[cfg(feature = "f32")]
/// # const VELOCITIES: [Velocity; 4] = [
/// // Velocity with linear and angular components
/// Velocity::new(Vec3::new(10.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.5)),
///
/// // Velocity with only the linear component
/// Velocity::from_linear(Vec3::new(10.0, 0.0, 0.0)),
/// Velocity::from_linear_xyz(10.0, 0.0, 0.0),
///
/// // Velocity with only the angular component
/// Velocity::from_angular(Vec3::new(0.0, 0.0, 1.5)),
/// Velocity::from_angular_xyz(0.0, 0.0, 1.5),
/// # ];
/// #
/// # #[cfg(feature = "f64")]
/// # fn main() {}
/// ```
///
/// # Changing Velocity
///
/// Velocity can be modified at runtime to control the movement of bodies:
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// # #[cfg(feature = "f32")]
/// fn accelerate_linear(mut query: Query<&mut Velocity>, time: Res<Time>) {
///     let delta_secs = time.delta_secs();
///     for mut velocity in &mut query {
///         // Accelerate the entity towards +X at `2.0` units per second squared.
///         velocity.linear.x += 2.0 * delta_secs;
///     }
/// }
/// #
/// # #[cfg(feature = "f64")]
/// # fn main() {}
/// ```
///
/// For controlling velocity and movement using forces, impulses, and acceleration,
/// see the [`forces`] module.
///
/// # Point Velocity
///
/// The velocity at a specific point on the body can be computed using the [`Velocity::at_point`] method,
/// which takes into account both the linear and angular components of the velocity. This is useful
/// for determining the speed of a point on the body that is not at the center of mass.
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
////
/// # #[cfg(feature = "f32")]
/// fn point_velocity(query: Query<&Velocity>) {
///     for velocity in &query {
///         // Compute the velocity at a point 1 unit to the right of the center of mass.
#[cfg_attr(
    feature = "2d",
    doc = "         let point_velocity = velocity.at_point(Vec2::new(1.0, 0.0));"
)]
#[cfg_attr(
    feature = "3d",
    doc = "         let point_velocity = velocity.at_point(Vec3::new(1.0, 0.0, 0.0));"
)]
///         println!("Velocity at point: {:?}", point_velocity);
///     }
/// }
/// #
/// # #[cfg(feature = "f64")]
/// # fn main() {}
/// ```
///
/// # Related Components
///
/// - [`MaxSpeed`] for clamping the maximum linear and angular velocity of a body.
/// - [`Damping`] for gradually slowing down a body's velocity over time.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, PartialEq)]
pub struct Velocity {
    /// The linear velocity, typically in meters per second.
    ///
    /// This represents the translational velocity of the body in world space.
    pub linear: Vector,

    /// The angular velocity in radians per second.
    ///
    /// This represents the rotational velocity of the body in world space.
    pub angular: AngularVector,
}

impl Velocity {
    /// Zero velocity.
    pub const ZERO: Velocity = Velocity::new(Vector::ZERO, AngularVector::ZERO);

    /// Creates a new [`Velocity`] with the given linear and angular components.
    #[inline(always)]
    pub const fn new(linear: Vector, angular: AngularVector) -> Self {
        Self { linear, angular }
    }

    /// Creates a new [`Velocity`] with the given linear component and zero angular component.
    #[inline(always)]
    pub const fn from_linear(linear: Vector) -> Self {
        Self {
            linear,
            angular: AngularVector::ZERO,
        }
    }

    /// Creates a new [`Velocity`] with the given `x` and `y` values for the linear component
    /// and zero angular component.
    #[inline(always)]
    #[cfg(feature = "2d")]
    pub const fn from_linear_xy(x: Scalar, y: Scalar) -> Self {
        Self {
            linear: Vector::new(x, y),
            angular: AngularVector::ZERO,
        }
    }

    /// Creates a new [`Velocity`] with the given `x`, `y`, and `z` values for the linear component
    /// and zero angular component.
    #[inline(always)]
    #[cfg(feature = "3d")]
    pub const fn from_linear_xyz(x: Scalar, y: Scalar, z: Scalar) -> Self {
        Self {
            linear: Vector::new(x, y, z),
            angular: AngularVector::ZERO,
        }
    }

    /// Creates a new [`Velocity`] with the given angular component and zero linear component.
    #[inline(always)]
    pub const fn from_angular(angular: AngularVector) -> Self {
        Self {
            linear: Vector::ZERO,
            angular,
        }
    }

    /// Creates a new [`Velocity`] with the given `x`, `y`, and `z` values for the angular component
    /// and zero linear component.
    #[inline(always)]
    #[cfg(feature = "3d")]
    pub const fn from_angular_xyz(x: Scalar, y: Scalar, z: Scalar) -> Self {
        Self {
            linear: Vector::ZERO,
            angular: AngularVector::new(x, y, z),
        }
    }

    /// Computes the velocity at a given point relative to the center of mass.
    #[inline]
    pub fn at_point(&self, point: Vector) -> Vector {
        #[cfg(feature = "2d")]
        {
            self.linear + self.angular * point.perp()
        }
        #[cfg(feature = "3d")]
        {
            self.linear + self.angular.cross(point)
        }
    }

    /// Returns `true` if either the linear or angular component of the velocity is NaN.
    #[inline]
    pub fn is_nan(&self) -> bool {
        self.linear.is_nan() || self.angular.is_nan()
    }

    /// Returns `true` if both the linear and angular components of the velocity
    /// are neither infinite nor NaN.
    #[inline]
    pub fn is_finite(&self) -> bool {
        self.linear.is_finite() && self.angular.is_finite()
    }
}

/// The maximum movement speed of a [rigid body](RigidBody), clamping [`Velocity`],
///
/// This can be useful for limiting how fast bodies can move, and can help
/// control behavior and prevent instability.
///
/// The default maximum speed is infinite, meaning that velocity is not clamped by default.
///
/// # Example
///
/// The maximum speed of a body can be set by adding the [`MaxSpeed`] component:
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// fn setup(mut commands: Commands) {
///     // Spawn a dynamic body with linear velocity clamped to `100.0` units per second
///     // and angular velocity clamped to `5.0` radians per second.
///     commands.spawn((
///         RigidBody::Dynamic,
///         MaxSpeed {
///             linear: 100.0,
///             angular: 5.0,
///         },
///     ));
/// }
/// ```
///
/// The [`MaxSpeed`] component also provides constructor methods for convenience:
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # const MAX_SPEEDS: [MaxSpeed; 3] = [
/// // Max speed with linear and angular components
/// MaxSpeed::new(100.0, 5.0),
///
/// // Max speed with only the linear component
/// MaxSpeed::from_linear(100.0),
///
/// // Max speed with only the angular component
/// MaxSpeed::from_angular(5.0) ,
/// # ];
/// ```
///
/// # Related Components
///
/// - [`Velocity`] for the current speed and direction of movement of a body.
/// - [`Damping`] for gradually slowing down a body's velocity over time.
#[derive(Component, Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
#[doc(alias = "MaxVelocity")]
pub struct MaxSpeed {
    /// The maximum linear speed, clamping [`Velocity::linear`], typically in meters per second.
    ///
    /// Default: [`INFINITY`](Scalar::INFINITY)
    pub linear: Scalar,

    /// The maximum angular speed, clamping [`Velocity::angular`], in radians per second.
    ///
    /// Default: [`INFINITY`](Scalar::INFINITY)
    pub angular: Scalar,
}

impl Default for MaxSpeed {
    fn default() -> Self {
        Self::new(Scalar::INFINITY, Scalar::INFINITY)
    }
}

impl MaxSpeed {
    /// Creates a new [`MaxSpeed`] with the given linear and angular components.
    #[inline(always)]
    pub const fn new(linear: Scalar, angular: Scalar) -> Self {
        Self { linear, angular }
    }

    /// Creates a new [`MaxSpeed`] with the given linear component and infinite angular component.
    #[inline(always)]
    pub const fn from_linear(linear: Scalar) -> Self {
        Self {
            linear,
            angular: Scalar::INFINITY,
        }
    }

    /// Creates a new [`MaxSpeed`] with the given angular component and infinite linear component.
    #[inline(always)]
    pub const fn from_angular(angular: Scalar) -> Self {
        Self {
            linear: Scalar::INFINITY,
            angular,
        }
    }
}

/// Damping coefficients to gradually slow down a dynamic [rigid body](RigidBody), decreasing its
/// [`Velocity`] each time step.
///
/// This can be used for basic air resistance or other forms of drag, or to make bodies
/// come to rest more quickly after movement.
///
/// The damping is implemented as a [Padé approximation] of [exponential decay],
/// using the following formula:
///
/// ```text
/// v(t + Δt) = v(t) / (1 + damping * Δt)
/// ```
///
/// The default damping coefficients are `0.0`, which corresponds to no damping,
/// meaning that the body will not slow down over time unless acted upon by other forces.
///
/// [Padé approximation]: https://en.wikipedia.org/wiki/Pad%C3%A9_approximant
/// [exponential decay]: https://en.wikipedia.org/wiki/Exponential_decay
///
/// # Example
///
/// The damping coefficients of a body can be set by adding the [`Damping`] component:
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// fn setup(mut commands: Commands) {
///     // Spawn a dynamic body with linear damping of `0.8` and angular damping of `1.6`.
///     commands.spawn((
///         RigidBody::Dynamic,
///         Damping {
///             linear: 0.8,
///             angular: 1.6,
///         },
///     ));
/// }
/// ```
///
/// The [`Damping`] component also provides constructor methods for convenience:
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # const DAMPINGS: [Damping; 3] = [
/// // Damping with linear and angular components
/// Damping::new(0.8, 1.6),
///
/// // Damping with only the linear component
/// Damping::from_linear(0.8),
///
/// // Damping with only the angular component
/// Damping::from_angular(1.6),
/// # ];
/// ```
///
/// # Related Components
///
/// - [`Velocity`] for the current speed and direction of movement of a body.
/// - [`MaxSpeed`] for clamping the maximum linear and angular velocity of a body.
#[derive(Component, Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
pub struct Damping {
    /// The linear damping coefficient, which gradually slows down the linear velocity of the body.
    ///
    /// Default: `0.0` (no damping)
    pub linear: Scalar,

    /// The angular damping coefficient, which gradually slows down the angular velocity of the body.
    ///
    /// Default: `0.0` (no damping)
    pub angular: Scalar,
}

impl Default for Damping {
    fn default() -> Self {
        Self::new(0.0, 0.0)
    }
}

impl Damping {
    /// Creates a new [`Damping`] with the given linear and angular components.
    #[inline(always)]
    pub const fn new(linear: Scalar, angular: Scalar) -> Self {
        Self { linear, angular }
    }

    /// Creates a new [`Damping`] with the given linear component and zero angular component.
    #[inline(always)]
    pub const fn from_linear(linear: Scalar) -> Self {
        Self {
            linear,
            angular: 0.0,
        }
    }

    /// Creates a new [`Damping`] with the given angular component and zero linear component.
    #[inline(always)]
    pub const fn from_angular(angular: Scalar) -> Self {
        Self {
            linear: 0.0,
            angular,
        }
    }
}
