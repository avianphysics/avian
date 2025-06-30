//! **Joints** are a way to connect entities in a way that restricts their movement relative to each other.
//! They act as [constraints](dynamics::solver::xpbd#constraints) that restrict different *Degrees Of Freedom*
//! depending on the joint type.
//!
//! # Degrees of Freedom (DOF)
//!
//! In 3D, entities can normally translate and rotate along the `X`, `Y` and `Z` axes.
//! Therefore, they have 3 translational DOF and 3 rotational DOF, which is a total of 6 DOF.
//!
//! Joints reduce the number of DOF that entities have. For example, [revolute joints](RevoluteJoint)
//! only allow rotation around one axis.
//!
//! Below is a table containing the joints that are currently implemented.
//!
//! | Joint              | Allowed 2D DOF            | Allowed 3D DOF              |
//! | ------------------ | ------------------------- | --------------------------- |
//! | [`FixedJoint`]     | None                      | None                        |
//! | [`DistanceJoint`]  | 1 Translation, 1 Rotation | 2 Translations, 3 Rotations |
//! | [`PrismaticJoint`] | 1 Translation             | 1 Translation               |
//! | [`RevoluteJoint`]  | 1 Rotation                | 1 Rotation                  |
#![cfg_attr(
    feature = "3d",
    doc = "| [`SphericalJoint`] | 1 Rotation                | 3 Rotations                 |"
)]
//!
//! # Using Joints
//!
//! In Avian, joints are modeled as components. You can create a joint by simply spawning
//! an entity and adding the joint component you want, giving the connected entities as arguments
//! to the `new` method.
//!
//! ```
#![cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
//! use bevy::prelude::*;

//! fn setup(mut commands: Commands) {
//!     let entity1 = commands.spawn(RigidBody::Dynamic).id();
//!     let entity2 = commands.spawn(RigidBody::Dynamic).id();
//!     
//!     // Connect the bodies with a fixed joint
//!     commands.spawn(FixedJoint::new(entity1, entity2));
//! }
//! ```
//!
//! ## Stiffness
//!
//! You can control the stiffness of a joint with the `with_compliance` method.
//! *Compliance* refers to the inverse of stiffness, so using a compliance of 0 corresponds to
//! infinite stiffness.
//!
//! ## Attachment Positions
//!
//! By default, joints are connected to the centers of entities, but attachment positions can be used to change this.
//!
//! You can use `with_local_anchor_1` and `with_local_anchor_2` to set the attachment positions on the first
//! and second entity respectively.
//!
//! ## Damping
//!
//! You can configure the linear and angular damping caused by joints using the `with_linear_velocity_damping` and
//! `with_angular_velocity_damping` methods. Increasing the damping values will cause the velocities
//! of the connected entities to decrease faster.
//!
//! ## Other Configuration
//!
//! Different joints may have different configuration options. Many joints allow you to change the axis of allowed
//! translation or rotation, and they may have distance or angle limits along these axes.
//!
//! Take a look at the documentation and methods of each joint to see all of the configuration options.
//!
//! # Custom Joints
//!
//! Joints are [constraints](dynamics::solver::xpbd#constraints) that implement [`Joint`] and [`XpbdConstraint`].
//!
//! The process of creating a joint is essentially the same as [creating a constraint](dynamics::solver::xpbd#custom-constraints),
//! except you should also implement the [`Joint`] trait's methods. The trait has some useful helper methods
//! like `align_position` and `align_orientation` to reduce some common boilerplate.
//!
//! Many joints also have joint limits. You can use [`DistanceLimit`] and [`AngleLimit`] to help store these limits
//! and to compute the current distance from the specified limits.
//!
//! [See the code implementations](https://github.com/Jondolf/avian/tree/main/src/constraints/joints)
//! of the implemented joints to get a better idea of how to create joints.

mod fixed_angle_constraint;
mod point_constraint;

mod distance;
mod fixed;
mod prismatic;
mod revolute;
#[cfg(feature = "3d")]
mod spherical;

pub use fixed_angle_constraint::*;
pub use point_constraint::*;

pub use distance::*;
pub use fixed::*;
pub use prismatic::*;
pub use revolute::*;
#[cfg(feature = "3d")]
pub use spherical::*;

use crate::{dynamics::solver::xpbd::*, prelude::*};
use bevy::prelude::*;

/// A trait for [joints](self).
pub trait Joint: Component + PositionConstraint + AngularConstraint {
    /// Creates a new joint between two entities.
    fn new(entity1: Entity, entity2: Entity) -> Self;

    /// Returns the local attachment point on the first body.
    fn local_anchor_1(&self) -> Vector;

    /// Returns the local attachment point on the second body.
    fn local_anchor_2(&self) -> Vector;

    /// Returns the linear velocity damping applied by the joint.
    fn damping_linear(&self) -> Scalar;

    /// Returns the angular velocity damping applied by the joint.
    fn damping_angular(&self) -> Scalar;

    /// Returns the relative dominance of the bodies.
    ///
    /// If the relative dominance is positive, the first body is dominant
    /// and is considered to have infinite mass.
    fn relative_dominance(&self) -> i16;
}

/// A limit that indicates that the distance between two points should be between `min` and `max`.
#[derive(Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct DistanceLimit {
    /// The minimum distance between two points.
    pub min: Scalar,
    /// The maximum distance between two points.
    pub max: Scalar,
}

impl DistanceLimit {
    /// A `DistanceLimit` with `min` and `max` set to zero.
    pub const ZERO: Self = Self { min: 0.0, max: 0.0 };

    /// Creates a new `DistanceLimit`.
    pub const fn new(min: Scalar, max: Scalar) -> Self {
        Self { min, max }
    }

    /// Returns the direction and magnitude of the positional correction required
    /// to limit the given `separation` to be within the distance limit.
    pub fn compute_correction(&self, separation: Vector) -> (Vector, Scalar) {
        let distance_squared = separation.length_squared();

        if distance_squared <= Scalar::EPSILON {
            return (Vector::ZERO, 0.0);
        }

        let distance = distance_squared.sqrt();

        // Equation 25
        if distance < self.min {
            // Separation distance lower limit
            (separation / distance, (self.min - distance))
        } else if distance > self.max {
            // Separation distance upper limit
            (-separation / distance, (distance - self.max))
        } else {
            (Vector::ZERO, 0.0)
        }
    }

    /// Returns the positional correction required to limit the given `separation`
    /// to be within the distance limit along a given `axis`.
    pub fn compute_correction_along_axis(&self, separation: Vector, axis: Vector) -> Vector {
        let a = separation.dot(axis);

        // Equation 25
        if a < self.min {
            // Separation distance lower limit
            axis * (self.min - a)
        } else if a > self.max {
            // Separation distance upper limit
            -axis * (a - self.max)
        } else {
            Vector::ZERO
        }
    }
}

/// A limit that indicates that angles should be between `alpha` and `beta`.
#[derive(Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct AngleLimit {
    /// The minimum angle.
    pub min: Scalar,
    /// The maximum angle.
    pub max: Scalar,
}

impl AngleLimit {
    /// An `AngleLimit` with `alpha` and `beta` set to zero.
    pub const ZERO: Self = Self { min: 0.0, max: 0.0 };

    /// Creates a new `AngleLimit`.
    pub const fn new(min: Scalar, max: Scalar) -> Self {
        Self { min, max }
    }

    /// Returns the angular correction required to limit the `angle_difference`
    /// to be within the angle limits.
    #[cfg(feature = "2d")]
    pub fn compute_correction(
        &self,
        angle_difference: Scalar,
        max_correction: Scalar,
    ) -> Option<Scalar> {
        let correction = if angle_difference < self.min {
            angle_difference - self.min
        } else if angle_difference > self.max {
            angle_difference - self.max
        } else {
            return None;
        };

        Some(correction.min(max_correction))
    }

    /// Returns the angular correction required to limit the angle between `axis1` and `axis2`
    /// to be within the angle limits with respect to the `limit_axis`.
    #[cfg(feature = "3d")]
    pub fn compute_correction(
        &self,
        limit_axis: Vector,
        axis1: Vector,
        axis2: Vector,
        max_correction: Scalar,
    ) -> Option<Vector> {
        // [limit_axis, axis1, axis2] = [n, n1, n2] in XPBD rigid body paper.

        // Angle between axis1 and axis2 with respect to limit_axis.
        let mut phi = axis1.cross(axis2).dot(limit_axis).asin();

        // `asin` returns the angle in the [-pi/2, pi/2] range.
        // This is correct if the angle between n1 and n2 is acute,
        // but obtuse angles must be accounted for.
        if axis1.dot(axis2) < 0.0 {
            phi = PI - phi;
        }

        // Map the angle to the [-pi, pi] range.
        if phi > PI {
            phi -= TAU;
        }

        // The XPBD rigid body paper has this, but the angle
        // should already be in the correct range.
        //
        // if phi < -PI {
        //     phi += TAU;
        // }

        // Only apply a correction if the limit is violated.
        if phi < self.min || phi > self.max {
            // phi now represents the angle between axis1 and axis2.

            // Clamp phi to get the target angle.
            phi = phi.clamp(self.min, self.max);

            // Create a quaternion that represents the rotation.
            let rot = Quaternion::from_axis_angle(limit_axis, phi);

            // Rotate axis1 by the target angle and compute the correction.
            return Some((rot * axis1).cross(axis2).clamp_length_max(max_correction));
        }

        None
    }
}

/// A marker component that indicates that a [joint](self) is disabled
/// and should not constrain the bodies it is attached to.
/// Must be on the same entity as the joint.
///
/// This is useful for temporarily disabling a joint without removing it from the world.
/// To re-enable the joint, simply remove this component.
///
/// Note that when re-enabling the joint, the bodies may snap back violently
/// if they have moved significantly from the constrained positions while the joint was disabled.
///
/// # Related Components
///
/// - [`RigidBodyDisabled`]: Disables a rigid body.
/// - [`ColliderDisabled`]: Disables a collider.
#[derive(Reflect, Clone, Copy, Component, Debug, Default)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default)]
pub struct JointDisabled;
