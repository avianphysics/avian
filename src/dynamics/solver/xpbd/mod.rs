//! Extended Position-Based Dynamics (XPBD) constraint functionality.
//!
//! XPBD is a simulation method that solves constraints at the position-level.
//! Avian currently uses it for [joints](dynamics::solver::joints),
//! while contacts use an impulse-based approach.
//!
//! This module contains traits and systems for XPBD functionality.
//! The actual joint implementations are in [`dynamics::solver::joints`],
//! but it is also possible to create your own constraints.
//!
//! The following section has an overview of what exactly constraints are,
//! how they work with XPBD, and how you can define your own constraints.
//!
//! # Constraints
//!
//! **Constraints** are a way to model physical relationships between entities.
//! They are an integral part of physics simulation, and they can be used for things
//! like contact resolution, [joints](super::joints), soft bodies, and much more.
//!
//! At its core, a constraint is just a rule that is enforced by moving the participating entities in a way that satisfies that rule.
//! For example, a distance constraint is satisfied when the distance between two entities is equal to the desired distance.
//!
//! Most constraints in Avian are modeled as seperate entities with a component that implements [`XpbdConstraint`].
//! They contain a `solve` method that receives the states of the participating entities as parameters.
//! You can find more details on how to use each constraint by taking a look at their documentation.
//!
//! Below are the currently implemented XPBD-based constraints.
//!
//! - [Joints](super::joints)
//!     - [`FixedJoint`]
//!     - [`DistanceJoint`]
#![cfg_attr(feature = "3d", doc = "    - [`SphericalJoint`]")]
//!     - [`RevoluteJoint`]
//!     - [`PrismaticJoint`]
//!
//! Avian's [`ContactConstraint`](dynamics::solver::contact::ContactConstraint)
//! is impulse-based instead.
//!
//! ## Custom constraints
//!
//! In Avian, you can easily create your own XPBD constraints using the same APIs that the engine uses for its own constraints.
//!
//! First, create a struct and implement the [`EntityConstraint`] and [`XpbdConstraint`] traits,
//! giving the number of participating entities using generics. It should look similar to this:
//!
//! ```
#![cfg_attr(
    feature = "2d",
    doc = "use avian2d::{prelude::*, dynamics::solver::xpbd::XpbdConstraint};"
)]
#![cfg_attr(
    feature = "3d",
    doc = "use avian3d::{prelude::*, dynamics::solver::xpbd::XpbdConstraint};"
)]
//! use bevy::{ecs::entity::{EntityMapper, MapEntities}, prelude::*};
//!
//! struct CustomConstraint {
//!     entity1: Entity,
//!     entity2: Entity,
//!     lagrange: f32,
//! }
//!
//! // This tells the solver how to get the entities from the constraint.
//! # #[cfg(feature = "f32")]
//! impl EntityConstraint<2> for CustomConstraint {
//!     fn entities(&self) -> [Entity; 2] {
//!         [self.entity1, self.entity2]
//!     }
//! }
//!
//! # #[cfg(feature = "f32")]
//! impl XpbdConstraint<2> for CustomConstraint {
//!     fn clear_lagrange_multipliers(&mut self) {
//!         self.lagrange = 0.0;
//!     }
//!
//!     fn prepare(&mut self, bodies: [&RigidBodyQueryReadOnlyItem; 2], dt: f32) {
//!          // Prepare the constraint for `solve` (compute current anchor positions, offsets, and so on).
//!          // This runs before the substepping loop.
//!     }
//!
//!     fn solve(
//!         &mut self,
//!         bodies: [&mut SolverBody; 2],
//!         inertias: [&SolverBodyInertia; 2],
//!         dt: f32,
//!     ) {
//!         // Solve the constraint by applying corrections to the `delta_position`
//!         // and `delta_rotation` of the participating bodies.
//!         // This runs at each substep.
//!     }
//! }
//!
//! impl MapEntities for CustomConstraint {
//!     fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
//!        self.entity1 = entity_mapper.get_mapped(self.entity1);
//!        self.entity2 = entity_mapper.get_mapped(self.entity2);
//!     }
//! }
//! ```
//!
//! Take a look at [`XpbdConstraint::solve`] and the constraint [theory](#theory) to learn more about what to put in `solve`.
//!
//! Next, we need to add a system that solves the constraint during each run of the [solver](dynamics::solver).
//! If your constraint is a component like Avian's joints, you can use the generic [`solve_xpbd_joint`]
//! system that handles some of the background work for you.
//!
//! Add the `solve_xpbd_joint::<YourConstraint>` system to the [substepping schedule's](SubstepSchedule)
//! [`SubstepSolverSet::SolveUserConstraints`](dynamics::solver::SubstepSolverSet::SolveUserConstraints) system set.
//! It should look like this:
//!
//! ```ignore
//! // Prepare custom constraint
//! app.add_systems(
//!     PhysicsSchedule,
//!     prepare_xpbd_joint::<CustomConstraint>
//!         .in_set(SolverSet::PrepareJoints),
//! );
//!
//! // Solve custom constraint
//! app.add_systems(
//!     SubstepSchedule,
//!     solve_xpbd_joint::<CustomConstraint>
//!         .in_set(SubstepSolverSet::SolveUserConstraints),
//! );
//! ```
//!
//! Now, just spawn an instance of the constraint, give it the participating entities, and the constraint should be getting
//! solved automatically according to the `solve` method!
//!
//! You can find a working example of a custom constraint
//! [here](https://github.com/Jondolf/avian/blob/main/crates/avian3d/examples/custom_constraint.rs).
//!
//! ## Theory
//!
//! In this section, you can learn some of the theory behind how position-based constraints work. Understanding the theory and maths isn't
//! important for using constraints, but it can be useful if you want to [create your own constraints](#custom-constraints).
//!
//! **Note**: In the following theory, primarily the word "particle" is used, but the same logic applies to normal
//! [rigid bodies](RigidBody) as well. However, unlike particles, rigid bodies can also have angular quantities such as
//! [rotation](Rotation) and [angular inertia](AngularInertia), so constraints can also affect their orientation. This is explained
//! in more detail [at the end](#rigid-body-constraints).
//!
//! ### Constraint functions
//!
//! At the mathematical level, each constraint has a *constraint function* `C(x)` that takes the state
//! of the particles as parameters and outputs a scalar value. The goal of the constraint is to move the particles
//! in a way that the output *satisfies* a constraint equation.
//!
//! For *equality constraints* the equation takes the form `C(x) = 0`. In other words, the constraint tries to
//! *minimize* the value of `C(x)` to be as close to zero as possible. When the equation is true, the constraint is *satisfied*.
//!
//! For a distance constraint, the constraint function would be `C(x) = distance - rest_distance`,
//! because this would be zero when the distance is equal to the desired rest distance.
//!
//! For *inequality constraints* the equation instead takes the form `C(x) >= 0`. These constraints are only applied
//! when `C(x) < 0`, which is useful for things like static friction and [joint limits](super::joints#joint-limits).
//!
//! ### Constraint gradients
//!
//! To know what directions the particles should be moved towards, constraints compute a *constraint gradient* `▽C(x)`
//! for each particle. It is a vector that points in the direction in which the constraint function value `C` increases the most.
//! The length of the gradient indicates how much `C` changes when moving the particle by one unit. This is often equal to one.
//!
//! In a case where two particles are being constrained by a distance constraint, and the particles are outside of the
//! rest distance, the gradient vector would point away from the other particle, because it would increase the distance
//! even further.
//!
//! ### Lagrange multipliers
//!
//! In the context of constraints, a Lagrange multiplier `λ` corresponds to the signed magnitude of the constraint force.
//! It is a scalar value that is the same for all of the constraint's participating particles, and it is used for computing
//! the correction that the constraint should apply to the particles along the gradients.
//!
//! In XPBD, the Lagrange multiplier update `Δλ` during a substep is computed by dividing the opposite of `C`
//! by the sum of the products of the inverse masses and squared gradient lengths plus an additional compliance term:
//!
//! ```text
//! Δλ = -C / (sum(w_i * |▽C_i|^2) + α / h^2)
//! ```
//!
//! where `w_i` is the inverse mass of particle `i`, `|▽C_i|` is the length of the gradient vector for particle `i`,
//! `α` is the constraint's compliance (inverse of stiffness) and `h` is the substep size. Using `α = 0`
//! corresponds to infinite stiffness.
//!
//! The minus sign is there because the gradients point in the direction in which `C` increases the most,
//! and we instead want to minimize `C`.
//!
//! Note that if the gradients are normalized, as is often the case, the squared gradient lengths can be omitted from the
//! calculation.
//!
//! ### Solving constraints
//!
//! Once we have computed the Lagrange multiplier `λ`, we can compute the positional correction for a given particle
//! as the product of the Lagrange multiplier and the particle's inverse mass and gradient vector:
//!
//! ```text
//! Δx_i = Δλ * w_i * ▽C_i
//! ```
//!
//! In other words, we typically move the particle along the gradient by `Δλ` proportional to the particle's inverse mass.
//!
//! ### Rigid body constraints
//!
//! Unlike particles, [rigid bodies](RigidBody) also have angular quantities like [rotation](Rotation),
//! [angular velocity](AngularVelocity) and [angular inertia](AngularInertia). In addition, constraints can be applied at specific
//! points in the body, like contact positions or joint attachment positions, which also affects the orientation.
//!
//! When the constraint is not applied at the center of mass, the inverse mass in the computation of `Δλ` must
//! be replaced with a *generalized inverse mass* that is essentially the effective mass when applying the constraint
//! at some specified position.
//!
//! For a positional constraint applied at position `r_i`, the generalized inverse mass computation for body `i` looks like this:
//!
//! ```text
//! w_i = 1 / m_i + (r_i x ▽C_i)^T * I_i^-1 * (r_i x ▽C_i)
//! ```
//!
//! where `m_i` is the [mass](Mass) of body `i`, `I_i^-1` is the inverse [angular inertia tensor](AngularInertia), and `^T` refers to the
//! transpose of a vector. Note that the value of the inertia tensor depends on the orientation of the body, so it should be
//! recomputed each time the constraint is solved.
//!
//! For an angular constraint where the gradient vector is the rotation axis, the generalized inverse mass computation instead
//! looks like this:
//!
//! ```text
//! w_i = ▽C_i^T * I_i^-1 * ▽C_i
//! ```
//!
//! Once we have computed the Lagrange multiplier update, we can apply the positional correction as shown in the
//! [previous section](#solving-constraints).
//!
//! However, angular constraints are handled differently. If the constraint function's value is the rotation angle and
//! the gradient vector is the rotation axis, we can compute the angular correction for a given body like this:
//!
//! ```text
//! Δq_i = 0.5 * [I_i^-1 * (r_i x (Δλ * ▽C_i)), 0] * q_i
//! ```
//!
//! where `q_i` is the [rotation](Rotation) of body `i` and `r_i` is a vector pointing from the body's center of mass to some
//! attachment position.

mod angular_constraint;
mod positional_constraint;

pub use angular_constraint::AngularConstraint;
pub use positional_constraint::PositionConstraint;

use crate::prelude::*;
use bevy::{
    ecs::{component::Mutable, entity::MapEntities},
    prelude::*,
};
use core::cmp::Ordering;

use super::solver_body::{SolverBody, SolverBodyInertia};

/// A trait for constraints between entities.
pub trait EntityConstraint<const ENTITY_COUNT: usize>: MapEntities {
    /// The entities participating in the constraint.
    fn entities(&self) -> [Entity; ENTITY_COUNT];
}

/// A trait for all XPBD [constraints](self#constraints).
pub trait XpbdConstraint<const ENTITY_COUNT: usize> {
    /// Prepares the constraint for solving.
    fn prepare(&mut self, bodies: [&RigidBodyQueryReadOnlyItem; ENTITY_COUNT], dt: Scalar);

    /// Solves the constraint.
    ///
    /// There are two main steps to solving a constraint:
    ///
    /// 1. Compute the generalized inverse masses, [gradients](self#constraint-gradients)
    ///    and the [Lagrange multiplier](self#lagrange-multipliers) update.
    /// 2. Apply corrections along the gradients using the Lagrange multiplier update.
    ///
    /// [`XpbdConstraint`] provides the [`compute_lagrange_update`](XpbdConstraint::compute_lagrange_update)
    /// method for all constraints. It requires the gradients and inverse masses of the participating entities.
    ///
    /// For constraints between two bodies, you can implement [`PositionConstraint`]. and [`AngularConstraint`]
    /// to get the associated `compute_generalized_inverse_mass`, `apply_positional_correction` and
    /// `apply_angular_correction` methods. Otherwise you must implement the generalized inverse mass
    /// computations and correction applying logic yourself.
    ///
    /// You can find a working example of a custom constraint
    /// [here](https://github.com/Jondolf/avian/blob/main/crates/avian3d/examples/custom_constraint.rs).
    fn solve(
        &mut self,
        bodies: [&mut SolverBody; ENTITY_COUNT],
        inertias: [&SolverBodyInertia; ENTITY_COUNT],
        dt: Scalar,
    );

    /// Computes how much a constraint's [Lagrange multiplier](self#lagrange-multipliers) changes when projecting
    /// the constraint for all participating particles.
    ///
    /// `c` is a scalar value returned by the [constraint function](self#constraint-functions).
    /// When it is zero, the constraint is satisfied.
    ///
    /// Each particle should have a corresponding [gradient](self#constraint-gradients) in `gradients`.
    /// A gradient is a vector that refers to the direction in which `c` increases the most.
    ///
    /// See the [constraint theory](#theory) for more information.
    fn compute_lagrange_update_with_gradients(
        &self,
        lagrange: Scalar,
        c: Scalar,
        gradients: &[Vector],
        inverse_masses: &[Scalar],
        compliance: Scalar,
        dt: Scalar,
    ) -> Scalar {
        // Compute the sum of all inverse masses multiplied by the squared lengths of the corresponding gradients.
        let w_sum = inverse_masses
            .iter()
            .enumerate()
            .fold(0.0, |acc, (i, w)| acc + *w * gradients[i].length_squared());

        // Avoid division by zero
        if w_sum <= Scalar::EPSILON {
            return 0.0;
        }

        // tilde_a = a/h^2
        let tilde_compliance = compliance / dt.powi(2);

        (-c - tilde_compliance * lagrange) / (w_sum + tilde_compliance)
    }

    /// Computes how much a constraint's [Lagrange multiplier](self#lagrange-multipliers) changes when projecting
    /// the constraint for all participating particles. The constraint gradients are assumed to be unit-length.
    ///
    /// `c` is a scalar value returned by the [constraint function](self#constraint-functions).
    /// When it is zero, the constraint is satisfied.
    ///
    /// See the [constraint theory](#theory) for more information.
    fn compute_lagrange_update(
        &self,
        lagrange: Scalar,
        c: Scalar,
        inverse_masses: &[Scalar],
        compliance: Scalar,
        dt: Scalar,
    ) -> Scalar {
        // Compute the sum of all inverse masses.
        // The gradients are unit length, so they don't need to be considered.
        let w_sum: Scalar = inverse_masses.iter().copied().sum();

        // Avoid division by zero
        if w_sum <= Scalar::EPSILON {
            return 0.0;
        }

        // tilde_a = a/h^2
        let tilde_compliance = compliance / dt.powi(2);

        (-c - tilde_compliance * lagrange) / (w_sum + tilde_compliance)
    }

    /// Sets the constraint's [Lagrange multipliers](self#lagrange-multipliers) to 0.
    fn clear_lagrange_multipliers(&mut self) {}
}

/// Iterates through the XPBD joints of a given type and solves them.
pub fn prepare_xpbd_joint<
    C: Joint + EntityConstraint<2> + XpbdConstraint<2> + Component<Mutability = Mutable>,
>(
    bodies: Query<RigidBodyQueryReadOnly, Without<RigidBodyDisabled>>,
    mut joints: Query<&mut C, (Without<RigidBody>, Without<JointDisabled>)>,
    time: Res<Time>,
    mut commands: Commands,
) {
    let delta_secs = time.delta_seconds_adjusted();

    for mut joint in &mut joints {
        // Get components for entities
        if let Ok([body1, body2]) = bodies.get_many(joint.entities()) {
            joint.prepare([&body1, &body2], delta_secs);

            // Wake up the bodies if they are sleeping.
            // TODO: Simulation islands will let us handle this better.
            if body1.is_sleeping {
                commands.queue(WakeUpBody(body1.entity));
            }
            if body2.is_sleeping {
                commands.queue(WakeUpBody(body2.entity));
            }
        }
    }
}

/// Iterates through the XPBD joints of a given type and solves them.
pub fn solve_xpbd_joint<
    C: Joint + EntityConstraint<2> + XpbdConstraint<2> + Component<Mutability = Mutable>,
>(
    bodies: Query<(&mut SolverBody, &SolverBodyInertia), Without<RigidBodyDisabled>>,
    mut joints: Query<&mut C, (Without<RigidBody>, Without<JointDisabled>)>,
    time: Res<Time>,
) {
    let delta_secs = time.delta_seconds_adjusted();

    // Clear Lagrange multipliers
    joints
        .iter_mut()
        .for_each(|mut c| c.clear_lagrange_multipliers());

    let mut dummy_body1 = SolverBody::default();
    let mut dummy_body2 = SolverBody::default();
    let dummy_inertia = SolverBodyInertia::default();

    for mut joint in &mut joints {
        let [entity1, entity2] = joint.entities();

        let (mut body1, mut inertia1) = (&mut dummy_body1, &dummy_inertia);
        let (mut body2, mut inertia2) = (&mut dummy_body2, &dummy_inertia);

        // Get the solver bodies for the two colliding entities.
        if let Ok((body, inertia)) = unsafe { bodies.get_unchecked(entity1) } {
            body1 = body.into_inner();
            inertia1 = inertia;
        }
        if let Ok((body, inertia)) = unsafe { bodies.get_unchecked(entity2) } {
            body2 = body.into_inner();
            inertia2 = inertia;
        }

        // If a body has a higher dominance, it is treated as a static or kinematic body.
        match joint.relative_dominance().cmp(&0) {
            Ordering::Greater => inertia1 = &dummy_inertia,
            Ordering::Less => inertia2 = &dummy_inertia,
            _ => {}
        }

        joint.solve([body1, body2], [inertia1, inertia2], delta_secs);
    }
}

/// Updates the linear velocity of all dynamic bodies based on the change in position from the XPBD solver.
pub(super) fn project_linear_velocity(
    mut bodies: Query<(&mut SolverBody, &PreSolveDeltaPosition), RigidBodyActiveFilter>,
    time: Res<Time>,
) {
    let delta_secs = time.delta_seconds_adjusted();

    for (mut body, pre_solve_delta_pos) in &mut bodies {
        // v = (x - x_prev) / h
        let new_lin_vel = (body.delta_position - pre_solve_delta_pos.0) / delta_secs;
        body.linear_velocity += new_lin_vel;
    }
}

/// Updates the angular velocity of all dynamic bodies based on the change in rotation from the XPBD solver.
#[cfg(feature = "2d")]
pub(super) fn project_angular_velocity(
    mut bodies: Query<(&mut SolverBody, &PreSolveDeltaRotation), RigidBodyActiveFilter>,
    time: Res<Time>,
) {
    let delta_secs = time.delta_seconds_adjusted();

    for (mut body, pre_solve_delta_rot) in &mut bodies {
        let new_ang_vel = pre_solve_delta_rot.angle_between(body.delta_rotation) / delta_secs;
        body.angular_velocity += new_ang_vel;
    }
}

/// Updates the angular velocity of all dynamic bodies based on the change in rotation from the XPBD solver.
#[cfg(feature = "3d")]
pub(super) fn project_angular_velocity(
    mut bodies: Query<(&mut SolverBody, &PreSolveDeltaRotation), RigidBodyActiveFilter>,
    time: Res<Time>,
) {
    let delta_secs = time.delta_seconds_adjusted();

    for (mut body, pre_solve_delta_rot) in &mut bodies {
        let delta_rot = body
            .delta_rotation
            .mul_quat(pre_solve_delta_rot.inverse().0);

        let mut new_ang_vel = 2.0 * delta_rot.xyz() / delta_secs;

        if delta_rot.w < 0.0 {
            new_ang_vel = -new_ang_vel;
        }

        body.angular_velocity += new_ang_vel;
    }
}
