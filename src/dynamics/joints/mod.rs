//! **Joints** are a way to connect rigid bodies in a way that restricts their movement relative to each other.
//! They act as constraints that restrict different *Degrees of Freedom* depending on the joint type.
//!
//! # Degrees of Freedom (DOF)
//!
#![cfg_attr(
    feature = "2d",
    doc = r#"In 2D, rigid bodies can normally translate along the x and y axes and rotate about the z axis.
Therefore, they have 2 translational DOF and 1 rotational DOF, a total of 3 DOF."#
)]
#![cfg_attr(
    feature = "3d",
    doc = r#"In 3D, rigid bodies can normally translate and rotate along the x, y, and z axes.
Therefore, they have 3 translational DOF and 3 rotational DOF, a total of 6 DOF."#
)]
//!
#![cfg_attr(feature="2d", doc = include_str!("./images/2d_dofs.svg"))]
#![cfg_attr(feature="3d", doc = include_str!("./images/3d_dofs.svg"))]
//!
//! Joints limit the degrees of freedom that bodies can have. For example, a [`RevoluteJoint`] or hinge
//! prevents any relative movement between two bodies, except for rotation about a single axis
//! at an anchor point.
//!
//! Below is a table containing all joints that are currently implemented.
//!
//! | Joint              | Allowed 2D DOF            | Allowed 3D DOF              |
//! | ------------------ | ------------------------- | --------------------------- |
//! | [`FixedJoint`]     | None                      | None                        |
//! | [`DistanceJoint`]  | 1 Translation, 1 Rotation | 2 Translations, 3 Rotations |
//! | [`PrismaticJoint`] | 1 Translation             | 1 Translation               |
//! | [`RevoluteJoint`]  | 1 Rotation                | 1 Rotation                  |
#![cfg_attr(
    feature = "3d",
    doc = "| [`SphericalJoint`] | -                         | 3 Rotations                 |"
)]
//!
//! # Using Joints
//!
//! In Avian, joints are modeled as components. Each joint is spawned as its own entity,
//! providing the [`Entity`] identifiers of the bodies it should constrain.
//!
//! ```
#![cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
//! use bevy::prelude::*;
//!
//! fn setup(mut commands: Commands) {
//!     let body1 = commands.spawn(RigidBody::Dynamic).id();
//!     let body2 = commands.spawn(RigidBody::Dynamic).id();
//!     
//!     // Connect the bodies with a fixed joint.
//!     commands.spawn(FixedJoint::new(body1, body2));
//! }
//! ```
//!
//! By default, the attached bodies can still collide with each other.
//! This behavior can be disabled with the [`JointCollisionDisabled`] component.
//!
//! ## Joint Frames
//!
//! By default, joints use body transforms as their reference for how to constrain the connected bodies.
//! For example, a [`RevoluteJoint`] aims to make the positions of the two bodies coincide in world space,
//! while allowing the bodies to rotate freely around a common axis.
//!
//! However, it can often be useful to define the attachment point or orientation separately from the body transform.
//! For example, you may want the [`RevoluteJoint`] to be attached to the corner of a body instead of its center,
//! and that the other body is rotated by 90 degrees relative to the first body.
//!
//! This can be done by configuring the [`JointFrame`] associated with each body. Each joint frame is expressed
//! by a local [`JointAnchor`] and [`JointBasis`] relative to the transforms of the bodies. The anchor determines
//! the attachment point, while the basis determines the orientation of the joint frame relative to the body transform.
//!
#![doc = include_str!("./images/joint_frame.svg")]
//!
//! Storing the frames in local space allows the initial configuration to be preserved even when the bodies are moved.
//! The frames can also be specified in global coordinates using [`JointFrame::global`], but they are automatically converted
//! to local frames during the next simulation step.
//!
//! Below is an example of configuring [`JointFrame`]s for a [`RevoluteJoint`].
//!
//! ```
#![cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
//! # use bevy::prelude::*;
//! # use core::f32::consts::PI;
//! #
//! # #[cfg(feature = "f32")]
//! # fn setup(mut commands: Commands) {
//! #     let body1 = commands.spawn(RigidBody::Dynamic).id();
//! #     let body2 = commands.spawn(RigidBody::Dynamic).id();
//! #
//! // Connect two bodies with a revolute joint.
//! // Set the global anchor point and rotate the first frame by 45 degrees about the local z axis.
//! commands.spawn((
//!     RevoluteJoint::new(body1, body2)
#![cfg_attr(feature = "2d", doc = "        .with_anchor(Vec2::new(5.0, 2.0))")]
#![cfg_attr(feature = "3d", doc = "        .with_anchor(Vec3::new(5.0, 2.0, 0.0))")]
#![cfg_attr(feature = "2d", doc = "        .with_local_basis1(PI / 4.0),")]
#![cfg_attr(
    feature = "3d",
    doc = "        .with_local_basis1(Quat::from_rotation_z(PI / 4.0)),"
)]
//! ));
//! # }
//! ```
//!
//! ## Damping
//!
//! By default, no work is done to dampen the movement of bodies connected by a joint.
//! A pendulum will swing indefinitely, unless explicitly stopped or it loses energy due to simulation inaccuracies.
//!
//! It can often be desirable to dampen the relative velocities of bodies connected by a joint to slow them down over time.
//! This can be done using the [`JointDamping`] component.
//!
//! ```
#![cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
//! # use bevy::prelude::*;
//! #
//! # fn setup(mut commands: Commands) {
//! #     let body1 = commands.spawn(RigidBody::Dynamic).id();
//! #     let body2 = commands.spawn(RigidBody::Dynamic).id();
//! #
//! // Connect two bodies with a distance joint.
//! // Apply linear and angular damping to the joint.
//! commands.spawn((
//!     DistanceJoint::new(body1, body2),
//!     JointDamping {
//!         linear: 0.1,  // Linear damping
//!         angular: 0.1, // Angular damping
//!     },
//! ));
//! # }
//! ```
//!
//! ## Reading Joint Forces
//!
//! Joints apply forces and torques to constrain the bodies they are attached to.
//! These forces can be read by adding the [`JointForces`] component to the joint entity:
//!
//! ```
#![cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
//! # use bevy::prelude::*;
//! #
//! # fn setup(mut commands: Commands) {
//! #     let body1 = commands.spawn(RigidBody::Dynamic).id();
//! #     let body2 = commands.spawn(RigidBody::Dynamic).id();
//! #
//! // Connect two bodies with a revolute joint.
//! // Read the forces applied by the joint.
//! commands.spawn((
//!     RevoluteJoint::new(body1, body2),
//!     JointForces::new(),
//! ));
//! # }
//! ```
//!
//! and querying for it in a system:
//!
//! ```
#![cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
//! # use bevy::prelude::*;
//! #
//! fn read_joint_forces(query: Query<&JointForces>) {
//!     for joint_forces in &query {
//!         println!("Joint force: {}", joint_forces.force());
//!     }
//! }
//! ```
//!
//! This can often be useful for determining when to "break" a joint with the [`JointDisabled`] component
//! when its forces exceed a certain threshold. An example of this can be found in the next section on disabling joints.
//!
//! ## Disabling Joints
//!
//! It can sometimes be useful to temporarily disable a joint without removing it from the world.
//! This can be done by adding the [`JointDisabled`] component to the joint entity.
//!
//! A common use case is to "break" a joint when its [`JointForces`] exceed a certain threshold.
//! This could be done with a system like the following:
//!
//! ```
#![cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
//! # use bevy::prelude::*;
//! #
//! const BREAK_THRESHOLD: f32 = 500.0; // Example threshold
//!
//! # #[cfg(feature = "f32")]
//! fn break_joints(
//!     mut commands: Commands,
//!     query: Query<(Entity, &JointForces), Without<JointDisabled>>,
//! ) {
//!     for (entity, joint_forces) in &query {
//!         if joint_forces.force().length() > BREAK_THRESHOLD {
//!             // Break the joint by adding the `JointDisabled` component.
//!             // Alternatively, you could simply remove the joint component or despawn the entity.
//!            commands.entity(entity).insert(JointDisabled);
//!         }
//!     }
//! }
//! ```
//!
//! Disabled joints can be re-enabled by removing the [`JointDisabled`] component.
//!
//! ## Other Configuration
//!
//! Different joints may have different configuration options. They may allow you to change the axis of allowed
//! translation or rotation, and apply limits or motors for those axes.
//!
//! Take a look at the documentation and methods of each joint to see all the different configuration options.

mod distance;
mod fixed;
mod motor;
mod prismatic;
mod revolute;
#[cfg(feature = "3d")]
mod spherical;
#[cfg(test)]
mod tests;

pub use distance::DistanceJoint;
pub use fixed::FixedJoint;
pub use motor::{AngularMotor, LinearMotor, MotorModel};
pub use prismatic::PrismaticJoint;
pub use revolute::RevoluteJoint;
#[cfg(feature = "3d")]
pub use spherical::SphericalJoint;

use crate::{dynamics::solver::joint_graph::JointGraph, prelude::*};
use bevy::{
    ecs::{entity::MapEntities, lifecycle::HookContext, world::DeferredWorld},
    prelude::*,
};

/// A plugin for managing and initializing [joints](self).
///
/// Note that this does *not* include the actual joint constraint solver.
/// For a built-in joint solver, enable the `xpbd_joints` feature, and use the [`XpbdSolverPlugin`].
pub struct JointPlugin;

impl Plugin for JointPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((
            fixed::plugin,
            distance::plugin,
            prismatic::plugin,
            revolute::plugin,
            #[cfg(feature = "3d")]
            spherical::plugin,
        ));

        app.configure_sets(
            PhysicsSchedule,
            JointSystems::PrepareLocalFrames
                .after(SolverSystems::PrepareSolverBodies)
                .before(SolverSystems::PrepareJoints),
        );
    }
}

/// System sets for [joints](dynamics::joints).
#[derive(SystemSet, Clone, Debug, Hash, PartialEq, Eq)]
pub enum JointSystems {
    /// A system set for preparing local [`JointFrame`]s.
    PrepareLocalFrames,
}

/// A trait for constraints between entities.
pub trait EntityConstraint<const ENTITY_COUNT: usize>: MapEntities {
    /// The entities participating in the constraint.
    fn entities(&self) -> [Entity; ENTITY_COUNT];
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

impl From<Scalar> for DistanceLimit {
    /// Converts the given `limit` into a [`DistanceLimit`] where `min == max`.
    fn from(limit: Scalar) -> DistanceLimit {
        DistanceLimit {
            min: limit,
            max: limit,
        }
    }
}

impl From<[Scalar; 2]> for DistanceLimit {
    /// Converts the given `[min, max]` array into a [`DistanceLimit`].
    fn from([min, max]: [Scalar; 2]) -> DistanceLimit {
        DistanceLimit { min, max }
    }
}

impl From<(Scalar, Scalar)> for DistanceLimit {
    /// Converts the given `(min, max)` pair into a [`DistanceLimit`].
    fn from((min, max): (Scalar, Scalar)) -> DistanceLimit {
        DistanceLimit { min, max }
    }
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

impl From<Scalar> for AngleLimit {
    /// Converts the given `limit` into a [`AngleLimit`] where `min == max`.
    fn from(limit: Scalar) -> AngleLimit {
        AngleLimit {
            min: limit,
            max: limit,
        }
    }
}

impl From<[Scalar; 2]> for AngleLimit {
    /// Converts the given `[min, max]` array into a [`AngleLimit`].
    fn from([min, max]: [Scalar; 2]) -> AngleLimit {
        AngleLimit { min, max }
    }
}

impl From<(Scalar, Scalar)> for AngleLimit {
    /// Converts the given `(min, max)` pair into a [`AngleLimit`].
    fn from((min, max): (Scalar, Scalar)) -> AngleLimit {
        AngleLimit { min, max }
    }
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

/// A marker component that indicates that a [joint](self) is disabled and should not constrain the bodies it is attached to.
/// Must be on the same entity as the joint.
///
/// This is useful for temporarily disabling a joint without removing it from the world.
/// To re-enable the joint, simply remove this component.
///
/// Note that when re-enabling the joint, the bodies may snap back violently
/// if they have moved significantly from the constrained positions while the joint was disabled.
///
/// # Example
///
/// A common use case is to "break" a joint when its [`JointForces`] exceed a certain threshold.
/// This could be done with a system like the following:
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// const BREAK_THRESHOLD: f32 = 500.0;
///
/// # #[cfg(feature = "f32")]
/// fn break_joints(
///     mut commands: Commands,
///     query: Query<(Entity, &JointForces), Without<JointDisabled>>,
/// ) {
///     for (entity, joint_forces) in &query {
///         if joint_forces.force().length() > BREAK_THRESHOLD {
///             // Break the joint by adding the `JointDisabled` component.
///             // Alternatively, you could simply remove the joint component or despawn the entity.
///            commands.entity(entity).insert(JointDisabled);
///         }
///     }
/// }
/// ```
///
/// Disabled joints can be re-enabled by removing the [`JointDisabled`] component.
///
/// # Related Components
///
/// - [`JointCollisionDisabled`]: Disables collision between bodies connected by a joint.
/// - [`RigidBodyDisabled`]: Disables a rigid body.
/// - [`ColliderDisabled`]: Disables a collider.
#[derive(Reflect, Clone, Copy, Component, Debug, Default)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default)]
pub struct JointDisabled;

/// A marker component that disables collision between [rigid bodies](RigidBody)
/// connected by a [joint](self). Must be on the same entity as the joint.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// # let entity1 = commands.spawn(RigidBody::Dynamic).id();
/// # let entity2 = commands.spawn(RigidBody::Dynamic).id();
/// #
/// // Connect the bodies with a fixed joint.
/// // Disable collision between the two bodies.
/// commands.spawn((
///     FixedJoint::new(entity1, entity2),
///     JointCollisionDisabled,
/// ));
/// # }
/// ```
#[derive(Component, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, PartialEq)]
#[component(on_add = JointCollisionDisabled::on_add, on_remove = JointCollisionDisabled::on_remove)]
pub struct JointCollisionDisabled;

// TODO: Turn these into observers.
impl JointCollisionDisabled {
    fn on_add(mut world: DeferredWorld, ctx: HookContext) {
        let entity = ctx.entity;

        // Update the `contacts_enabled` property of the joint edge.
        // Note: The `JointGraphPlugin` handles the removal of contacts between the bodies.
        let mut joint_graph = world.resource_mut::<JointGraph>();
        if let Some(joint_edge) = joint_graph.get_mut(entity) {
            joint_edge.collision_disabled = true;
        }
    }

    fn on_remove(mut world: DeferredWorld, ctx: HookContext) {
        let entity = ctx.entity;

        // Update the `contacts_enabled` property of the joint edge.
        let mut joint_graph = world.resource_mut::<JointGraph>();
        if let Some(joint_edge) = joint_graph.get_mut(entity) {
            joint_edge.collision_disabled = false;
        }
    }
}

/// A component for applying damping to the relative linear and angular velocities
/// of bodies connected by a [joint](self).
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// #     let body1 = commands.spawn(RigidBody::Dynamic).id();
/// #     let body2 = commands.spawn(RigidBody::Dynamic).id();
/// #
/// // Connect two bodies with a distance joint.
/// // Apply linear and angular damping to the joint.
/// commands.spawn((
///     DistanceJoint::new(body1, body2),
///     JointDamping {
///         linear: 0.1,  // Linear damping
///         angular: 0.1, // Angular damping
///     },
/// ));
/// # }
/// ```
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, PartialEq)]
pub struct JointDamping {
    /// Linear damping applied by the joint.
    pub linear: Scalar,
    /// Angular damping applied by the joint.
    pub angular: Scalar,
}

/// A component for reading the force and torque exerted by a [joint](self).
///
/// This is not inserted automatically for joints, and must be added manually.
///
/// # Example
///
/// The forces exerted by a joint can be read by adding the [`JointForces`] component to the joint entity:
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// #     let body1 = commands.spawn(RigidBody::Dynamic).id();
/// #     let body2 = commands.spawn(RigidBody::Dynamic).id();
/// #
/// // Connect two bodies with a revolute joint.
/// // Read the forces applied by the joint.
/// commands.spawn((
///     RevoluteJoint::new(body1, body2),
///     JointForces::new(),
/// ));
/// # }
/// ```
///
/// and querying for it in a system:
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// fn read_joint_forces(query: Query<&JointForces>) {
///     for joint_forces in &query {
///         println!("Joint force: {}", joint_forces.force());
///     }
/// }
/// ```
///
/// This can often be useful for determining when to "break" a joint with the [`JointDisabled`] component
/// when its forces exceed a certain threshold. An example of this can be found in the [`JointDisabled`] documentation.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, PartialEq)]
pub struct JointForces {
    force: Vector,
    torque: AngularVector,
    motor_force: Scalar,
}

impl JointForces {
    /// Creates a new [`JointForces`] for reading the forces applied by a joint.
    #[inline]
    pub const fn new() -> Self {
        Self {
            force: Vector::ZERO,
            torque: AngularVector::ZERO,
            motor_force: 0.0,
        }
    }

    /// Returns the force applied by the joint.
    #[inline]
    pub const fn force(&self) -> Vector {
        self.force
    }

    /// Returns the torque applied by the joint.
    #[inline]
    pub const fn torque(&self) -> AngularVector {
        self.torque
    }

    /// Returns the force or torque applied by the motor, if any.
    ///
    /// For angular motors ([`AngularMotor`]), this is the torque in N·m.
    /// For linear motors ([`LinearMotor`]), this is the force in N.
    #[inline]
    pub const fn motor_force(&self) -> Scalar {
        self.motor_force
    }

    /// Sets the force applied by the joint.
    ///
    /// This should be done automatically by the joint solver.
    #[inline]
    pub const fn set_force(&mut self, force: Vector) {
        self.force = force;
    }

    /// Sets the torque applied by the joint.
    ///
    /// This should be done automatically by the joint solver.
    #[inline]
    pub const fn set_torque(&mut self, torque: AngularVector) {
        self.torque = torque;
    }

    /// Sets the motor force or torque.
    ///
    /// This should be done automatically by the joint solver.
    #[inline]
    pub const fn set_motor_force(&mut self, motor_force: Scalar) {
        self.motor_force = motor_force;
    }
}

/// The [reference frame] of a body that is being constrained by a [joint](self).
///
/// Each joint defines a connection between the translation and rotation of two reference frames.
/// For example, a [`RevoluteJoint`] aims to make the positions of the two frames coincide in world space,
/// while allowing the frames to rotate freely around a common axis.
///
/// Reference frames for joints are expressed by a local [`JointAnchor`] and [`JointBasis`]
/// relative to the transforms of the bodies. The anchor determines the attachment point,
/// while the basis determines the orientation of the joint frame relative to the body transform.
/// Together, they form a local isometry that defines the joint frame.
///
#[doc = include_str!("./images/joint_frame.svg")]
///
/// Storing the frames in local space allows the initial configuration to be preserved even when the bodies are moved.
/// The frames can also be specified in global coordinates using [`JointFrame::global`], but they are automatically converted
/// to local frames during the next simulation step.
///
/// [reference frame]: https://en.wikipedia.org/wiki/Frame_of_reference
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// # use core::f32::consts::PI;
/// #
/// # #[cfg(feature = "f32")]
/// # fn setup(mut commands: Commands) {
/// #     let body1 = commands.spawn(RigidBody::Dynamic).id();
/// #     let body2 = commands.spawn(RigidBody::Dynamic).id();
/// #
/// // Connect two bodies with a revolute joint.
/// // Set the global anchor point and rotate the first frame by 45 degrees about the local z axis.
/// commands.spawn((
///     RevoluteJoint::new(body1, body2)
#[cfg_attr(feature = "2d", doc = "        .with_anchor(Vec2::new(5.0, 2.0))")]
#[cfg_attr(feature = "3d", doc = "        .with_anchor(Vec3::new(5.0, 2.0, 0.0))")]
#[cfg_attr(feature = "2d", doc = "        .with_local_basis1(PI / 4.0),")]
#[cfg_attr(
    feature = "3d",
    doc = "        .with_local_basis1(Quat::from_rotation_z(PI / 4.0)),"
)]
/// ));
/// # }
/// ```
#[derive(Clone, Copy, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct JointFrame {
    /// The translation of the joint frame relative to the body transform.
    ///
    /// This defines the anchor point where the bodies are attached to each other.
    pub anchor: JointAnchor,
    /// The rotation of the joint frame relative to the body transform.
    ///
    /// This defines the orientation of the basis relative to the body transform.
    pub basis: JointBasis,
}

impl JointFrame {
    /// The identity reference frame, with the local anchor and basis set to zero.
    ///
    /// This represents a reference frame that aligns with the body transform.
    pub const IDENTITY: Self = Self {
        anchor: JointAnchor::ZERO,
        basis: JointBasis::IDENTITY,
    };

    /// Creates a [`JointFrame`] with the given local isometry.
    #[inline]
    pub fn local(isometry: impl Into<Isometry>) -> Self {
        let isometry: Isometry = isometry.into();
        #[cfg(feature = "2d")]
        let anchor = isometry.translation.adjust_precision();
        #[cfg(feature = "3d")]
        let anchor = Vec3::from(isometry.translation).adjust_precision();
        Self {
            anchor: JointAnchor::Local(anchor),
            #[cfg(feature = "2d")]
            basis: JointBasis::Local(Rotation::from_sin_cos(
                isometry.rotation.sin as Scalar,
                isometry.rotation.cos as Scalar,
            )),
            #[cfg(feature = "3d")]
            basis: JointBasis::Local(isometry.rotation.adjust_precision()),
        }
    }

    /// Creates a [`JointFrame`] with the given global isometry.
    ///
    /// The global frame will be converted to a local frame relative to the body transform
    /// during the next simulation step.
    #[inline]
    pub fn global(isometry: impl Into<Isometry>) -> Self {
        let isometry: Isometry = isometry.into();
        #[cfg(feature = "2d")]
        let anchor = isometry.translation.adjust_precision();
        #[cfg(feature = "3d")]
        let anchor = Vec3::from(isometry.translation).adjust_precision();
        Self {
            anchor: JointAnchor::FromGlobal(anchor),
            #[cfg(feature = "2d")]
            basis: JointBasis::FromGlobal(Rotation::from_sin_cos(
                isometry.rotation.sin as Scalar,
                isometry.rotation.cos as Scalar,
            )),
            #[cfg(feature = "3d")]
            basis: JointBasis::FromGlobal(isometry.rotation.adjust_precision()),
        }
    }

    /// Returns the joint frame as a local isometry.
    ///
    /// If the frame is specified in global coordinates, this returns `None`.
    #[inline]
    #[allow(clippy::unnecessary_cast)]
    pub fn get_local_isometry(&self) -> Option<Isometry> {
        let translation = match self.anchor {
            JointAnchor::Local(anchor) => anchor.f32(),
            JointAnchor::FromGlobal(_) => return None,
        };
        let rotation = match self.basis {
            #[cfg(feature = "2d")]
            JointBasis::Local(basis) => Rot2::from_sin_cos(basis.sin as f32, basis.cos as f32),
            #[cfg(feature = "3d")]
            JointBasis::Local(basis) => basis.f32(),
            JointBasis::FromGlobal(_) => return None,
        };
        Some(Isometry::new(translation, rotation))
    }

    /// Returns the joint frame as a global isometry.
    ///
    /// If the frame is specified in local coordinates, this returns `None`.
    #[inline]
    #[allow(clippy::unnecessary_cast)]
    pub fn get_global_isometry(&self) -> Option<Isometry> {
        let translation = match self.anchor {
            JointAnchor::FromGlobal(anchor) => anchor.f32(),
            JointAnchor::Local(_) => return None,
        };
        let rotation = match self.basis {
            #[cfg(feature = "2d")]
            JointBasis::FromGlobal(basis) => Rot2::from_sin_cos(basis.sin as f32, basis.cos as f32),
            #[cfg(feature = "3d")]
            JointBasis::FromGlobal(basis) => basis.f32(),
            JointBasis::Local(_) => return None,
        };
        Some(Isometry::new(translation, rotation))
    }

    /// Computes a local frames for the given [`JointFrame`]s
    /// corresponding to the transforms of two bodies constrained by a joint.
    ///
    /// This is used to initialize local frames when a joint is inserted.
    #[inline]
    #[must_use]
    pub fn compute_local(
        frame1: Self,
        frame2: Self,
        pos1: Vector,
        pos2: Vector,
        rot1: &Rotation,
        rot2: &Rotation,
    ) -> [JointFrame; 2] {
        let [local_anchor1, local_anchor2] =
            JointAnchor::compute_local(frame1.anchor, frame2.anchor, pos1, pos2, rot1, rot2);
        let [local_basis1, local_basis2] =
            JointBasis::compute_local(frame1.basis, frame2.basis, rot1, rot2);

        [
            JointFrame {
                anchor: local_anchor1,
                basis: local_basis1,
            },
            JointFrame {
                anchor: local_anchor2,
                basis: local_basis2,
            },
        ]
    }
}

/// The translation of a [`JointFrame`], defining the anchor point where the bodies are attached to each other.
///
/// Each joint anchor is stored in local space relative to the transform of the body it is attached to.
/// This way, the initial configuration of the joint is preserved even when the body is moved.
///
/// The initial anchor can also be specified in world space using [`JointAnchor::FromGlobal`],
/// but it is automatically converted to [`JointAnchor::Local`] during the next simulation step.
///
/// By default, a local anchor of zero is used, and the anchor aligns with the body transform.
#[derive(Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub enum JointAnchor {
    /// The anchor point is specified in local coordinates relative to the body transform.
    Local(Vector),
    /// The anchor point is specified in global coordinates.
    FromGlobal(Vector),
}

impl Default for JointAnchor {
    fn default() -> Self {
        Self::ZERO
    }
}

impl JointAnchor {
    /// The anchor point at the local origin.
    ///
    /// This represents an anchor that aligns with the body transform.
    pub const ZERO: Self = Self::Local(Vector::ZERO);

    /// Computes [`JointAnchor::Local`]s for the given [`JointAnchor`]s
    /// corresponding to the transforms of two bodies constrained by a joint.
    pub fn compute_local(
        anchor1: Self,
        anchor2: Self,
        pos1: Vector,
        pos2: Vector,
        rot1: &Rotation,
        rot2: &Rotation,
    ) -> [Self; 2] {
        let [local_anchor1, local_anchor2] = match [anchor1, anchor2] {
            [JointAnchor::Local(anchor1), JointAnchor::Local(anchor2)] => [anchor1, anchor2],
            [
                JointAnchor::FromGlobal(anchor1),
                JointAnchor::FromGlobal(anchor2),
            ] => [
                rot1.inverse() * (anchor1 - pos1),
                rot2.inverse() * (anchor2 - pos2),
            ],
            [
                JointAnchor::Local(anchor1),
                JointAnchor::FromGlobal(anchor2),
            ] => [anchor1, rot2.inverse() * (anchor2 - pos2)],
            [
                JointAnchor::FromGlobal(anchor1),
                JointAnchor::Local(anchor2),
            ] => [rot1.inverse() * (anchor1 - pos1), anchor2],
        };

        [
            JointAnchor::Local(local_anchor1),
            JointAnchor::Local(local_anchor2),
        ]
    }
}

impl From<JointAnchor> for JointFrame {
    fn from(anchor: JointAnchor) -> Self {
        Self {
            anchor,
            basis: JointBasis::IDENTITY,
        }
    }
}

/// The rotation of a [`JointFrame`], defining the basis of the joint frame relative to the body transform.
///
/// Each joint basis is stored in local space relative to the transform of the body it is attached to.
/// This way, the initial configuration of the joint is preserved even when the body is moved.
///
/// The initial basis can also be specified in world space using [`JointBasis::FromGlobal`],
/// but it is automatically converted to [`JointBasis::Local`] during the next simulation step.
///
/// By default, a local identity basis is used, and the basis aligns with the body transform.
#[derive(Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub enum JointBasis {
    /// The basis is specified in local space relative to the body transform.
    Local(Rot),
    /// The basis is specified in world space.
    FromGlobal(Rot),
}

impl Default for JointBasis {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl JointBasis {
    /// The identity basis.
    ///
    /// This represents a basis that aligns with the body transform.
    pub const IDENTITY: Self = Self::Local(Rot::IDENTITY);

    /// Creates a [`JointBasis::Local`] from the given local `x_axis`.
    ///
    /// The y-axis is computed as the counterclockwise perpendicular axis to the x-axis.
    #[inline]
    #[cfg(feature = "2d")]
    pub fn from_local_x(x_axis: Vector) -> Self {
        Self::Local(orthonormal_basis([
            x_axis,
            Vector::new(-x_axis.y, x_axis.x),
        ]))
    }

    /// Creates a [`JointBasis::Local`] from the given local `y_axis`.
    ///
    /// The x-axis is computed as the clockwise perpendicular axis to the y-axis.
    #[inline]
    #[cfg(feature = "2d")]
    pub fn from_local_y(y_axis: Vector) -> Self {
        Self::Local(orthonormal_basis([
            Vector::new(y_axis.y, -y_axis.x),
            y_axis,
        ]))
    }

    /// Creates a [`JointBasis::Local`] from the given local `x_axis` and `y_axis`.
    ///
    /// The z-axis is computed as the cross product of the x and y axes.
    #[inline]
    #[cfg(feature = "3d")]
    pub fn from_local_xy(x_axis: Vector, y_axis: Vector) -> Self {
        Self::Local(orthonormal_basis([x_axis, y_axis, x_axis.cross(y_axis)]))
    }

    /// Creates a [`JointBasis::Local`] from the given local `x_axis` and `z_axis`.
    ///
    /// The y-axis is computed as the cross product of the z and x axes.
    #[inline]
    #[cfg(feature = "3d")]
    pub fn from_local_xz(x_axis: Vector, z_axis: Vector) -> Self {
        Self::Local(orthonormal_basis([x_axis, z_axis.cross(x_axis), z_axis]))
    }

    /// Creates a [`JointBasis::Local`] from the given local `y_axis` and `z_axis`.
    ///
    /// The x-axis is computed as the cross product of the y and z axes.
    #[inline]
    #[cfg(feature = "3d")]
    pub fn from_local_yz(y_axis: Vector, z_axis: Vector) -> Self {
        Self::Local(orthonormal_basis([y_axis.cross(z_axis), y_axis, z_axis]))
    }

    /// Creates a [`JointBasis::FromGlobal`] from the given global `x_axis`.
    ///
    /// The y-axis is computed as the counterclockwise perpendicular axis to the x-axis.
    #[inline]
    #[cfg(feature = "2d")]
    pub fn from_global_x(x_axis: Vector) -> Self {
        Self::FromGlobal(orthonormal_basis([
            x_axis,
            Vector::new(-x_axis.y, x_axis.x),
        ]))
    }

    /// Creates a [`JointBasis::FromGlobal`] from the given global `y_axis`.
    ///
    /// The x-axis is computed as the clockwise perpendicular axis to the y-axis.
    #[inline]
    #[cfg(feature = "2d")]
    pub fn from_global_y(y_axis: Vector) -> Self {
        Self::FromGlobal(orthonormal_basis([
            Vector::new(y_axis.y, -y_axis.x),
            y_axis,
        ]))
    }

    /// Creates a [`JointBasis::FromGlobal`] from the given global `x_axis` and `y_axis`.
    ///
    /// The z-axis is computed as the cross product of the x and y axes.
    #[inline]
    #[cfg(feature = "3d")]
    pub fn from_global_xy(x_axis: Vector, y_axis: Vector) -> Self {
        Self::FromGlobal(orthonormal_basis([x_axis, y_axis, x_axis.cross(y_axis)]))
    }

    /// Creates a [`JointBasis::FromGlobal`] from the given global `x_axis` and `z_axis`.
    ///
    /// The y-axis is computed as the cross product of the z and x axes.
    #[inline]
    #[cfg(feature = "3d")]
    pub fn from_global_xz(x_axis: Vector, z_axis: Vector) -> Self {
        Self::FromGlobal(orthonormal_basis([x_axis, z_axis.cross(x_axis), z_axis]))
    }

    /// Creates a [`JointBasis::FromGlobal`] from the given global `y_axis` and `z_axis`.
    ///
    /// The x-axis is computed as the cross product of the y and z axes.
    #[inline]
    #[cfg(feature = "3d")]
    pub fn from_global_yz(y_axis: Vector, z_axis: Vector) -> Self {
        Self::FromGlobal(orthonormal_basis([y_axis.cross(z_axis), y_axis, z_axis]))
    }

    /// Computes a [`JointBasis::Local`] for the given [`JointBasis`]s
    /// corresponding to the transforms of two bodies constrained by a joint.
    pub fn compute_local(rotation1: Self, rotation2: Self, rot1: &Rot, rot2: &Rot) -> [Self; 2] {
        let [local_basis1, local_basis2] = match [rotation1, rotation2] {
            [JointBasis::Local(basis1), JointBasis::Local(basis2)] => [basis1, basis2],
            [
                JointBasis::FromGlobal(basis1),
                JointBasis::FromGlobal(basis2),
            ] => [basis1 * rot1.inverse(), basis2 * rot2.inverse()],
            [JointBasis::Local(basis1), JointBasis::FromGlobal(basis2)] => {
                [basis1, basis2 * rot2.inverse()]
            }
            [JointBasis::FromGlobal(basis1), JointBasis::Local(basis2)] => {
                [basis1 * rot1.inverse(), basis2]
            }
        };

        [
            JointBasis::Local(local_basis1),
            JointBasis::Local(local_basis2),
        ]
    }
}

impl From<JointBasis> for JointFrame {
    fn from(basis: JointBasis) -> Self {
        Self {
            anchor: JointAnchor::ZERO,
            basis,
        }
    }
}
