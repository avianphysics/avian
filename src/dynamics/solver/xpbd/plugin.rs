use core::cmp::Ordering;

use super::joints::*;
use crate::{
    dynamics::{
        joints::EntityConstraint,
        solver::{
            SolverConfig,
            schedule::SubstepSolverSystems,
            solver_body::{SolverBody, SolverBodyInertia},
            xpbd::{XpbdConstraint, XpbdConstraintSolverData},
        },
    },
    prelude::*,
};
use bevy::{ecs::component::Mutable, prelude::*};

/// A plugin for a joint solver using Extended Position-Based Dynamics (XPBD).
pub struct XpbdSolverPlugin;

impl Plugin for XpbdSolverPlugin {
    fn build(&self, app: &mut App) {
        app.register_required_components::<FixedJoint, FixedJointSolverData>();
        app.register_required_components::<RevoluteJoint, RevoluteJointSolverData>();
        #[cfg(feature = "3d")]
        app.register_required_components::<SphericalJoint, SphericalJointSolverData>();
        app.register_required_components::<PrismaticJoint, PrismaticJointSolverData>();
        app.register_required_components::<DistanceJoint, DistanceJointSolverData>();

        // Configure scheduling.
        app.configure_sets(
            SubstepSchedule,
            (
                XpbdSolverSystems::SolveConstraints,
                XpbdSolverSystems::SolveUserConstraints,
                XpbdSolverSystems::VelocityProjection,
            )
                .chain()
                .after(SubstepSolverSystems::Relax)
                .before(SubstepSolverSystems::Damping),
        );

        // Prepare joints before the substepping loop.
        app.add_systems(
            PhysicsSchedule,
            (
                prepare_xpbd_joint::<FixedJoint>,
                prepare_xpbd_joint::<RevoluteJoint>,
                #[cfg(feature = "3d")]
                prepare_xpbd_joint::<SphericalJoint>,
                prepare_xpbd_joint::<PrismaticJoint>,
                prepare_xpbd_joint::<DistanceJoint>,
            )
                .chain()
                .in_set(SolverSystems::PrepareJoints),
        );

        // Warm start motor constraints.
        // These are chained to avoid ambiguity, and marked ambiguous_with the contact
        // warm start since motor and contact warm starting are independent operations
        // that both add to body velocities.
        app.add_systems(
            SubstepSchedule,
            (
                warm_start_xpbd_motors::<RevoluteJoint>,
                warm_start_xpbd_motors::<PrismaticJoint>,
            )
                .chain()
                .ambiguous_with_all()
                .in_set(SubstepSolverSystems::WarmStart),
        );

        // Solve joints with XPBD.
        app.add_systems(
            SubstepSchedule,
            (
                |mut query: Query<
                    (
                        &SolverBody,
                        &mut PreSolveDeltaPosition,
                        &mut PreSolveDeltaRotation,
                    ),
                    Without<RigidBodyDisabled>,
                >| {
                    for (body, mut pre_solve_delta_position, mut pre_solve_delta_rotation) in
                        &mut query
                    {
                        // Store the previous delta translation and rotation for XPBD velocity updates.
                        pre_solve_delta_position.0 = body.delta_position;
                        pre_solve_delta_rotation.0 = body.delta_rotation;
                    }
                },
                solve_xpbd_joint::<FixedJoint>,
                solve_xpbd_joint::<RevoluteJoint>,
                #[cfg(feature = "3d")]
                solve_xpbd_joint::<SphericalJoint>,
                solve_xpbd_joint::<PrismaticJoint>,
                solve_xpbd_joint::<DistanceJoint>,
            )
                .chain()
                .in_set(XpbdSolverSystems::SolveConstraints),
        );

        // Perform XPBD velocity updates after constraint solving.
        app.add_systems(
            SubstepSchedule,
            (project_linear_velocity, project_angular_velocity)
                .chain()
                .in_set(XpbdSolverSystems::VelocityProjection),
        );

        // Write back the forces applied by the XPBD joints.
        app.add_systems(
            PhysicsSchedule,
            (
                writeback_joint_forces::<FixedJoint>,
                writeback_joint_forces::<RevoluteJoint>,
                #[cfg(feature = "3d")]
                writeback_joint_forces::<SphericalJoint>,
                writeback_joint_forces::<PrismaticJoint>,
                writeback_joint_forces::<DistanceJoint>,
            )
                .chain()
                .in_set(SolverSystems::Finalize),
        );
    }
}

/// System sets for the XPBD constraint solver in the [`SubstepSchedule`].
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum XpbdSolverSystems {
    /// Solves constraints using Extended Position-Based Dynamics (XPBD).
    SolveConstraints,
    /// A system set for user constraints.
    SolveUserConstraints,
    /// Performs velocity updates after XPBD constraint solving.
    VelocityProjection,
}

/// Iterates through the XPBD joints of a given type and solves them.
pub fn prepare_xpbd_joint<
    C: Component<Mutability = Mutable> + EntityConstraint<2> + XpbdConstraint<2>,
>(
    bodies: Query<RigidBodyQueryReadOnly, Without<RigidBodyDisabled>>,
    mut joints: Query<(&mut C, &mut C::SolverData), (Without<RigidBody>, Without<JointDisabled>)>,
) where
    C::SolverData: Component<Mutability = Mutable>,
{
    for (mut joint, mut solver_data) in &mut joints {
        // Clear the Lagrange multipliers.
        solver_data.clear_lagrange_multipliers();

        // Get components for entities
        if let Ok([body1, body2]) = bodies.get_many(joint.entities()) {
            joint.prepare([&body1, &body2], &mut solver_data);
        }
    }
}

/// Iterates through the XPBD joints of a given type and solves them.
pub fn solve_xpbd_joint<
    C: Component<Mutability = Mutable> + EntityConstraint<2> + XpbdConstraint<2>,
>(
    bodies: Query<(&mut SolverBody, &SolverBodyInertia), Without<RigidBodyDisabled>>,
    mut joints: Query<(&mut C, &mut C::SolverData), (Without<RigidBody>, Without<JointDisabled>)>,
    time: Res<Time>,
) where
    C::SolverData: Component<Mutability = Mutable>,
{
    let delta_secs = time.delta_seconds_adjusted();

    let mut dummy_body1 = SolverBody::default();
    let mut dummy_body2 = SolverBody::default();

    for (mut joint, mut solver_data) in &mut joints {
        let [entity1, entity2] = joint.entities();

        let (mut body1, mut inertia1) = (&mut dummy_body1, &SolverBodyInertia::DUMMY);
        let (mut body2, mut inertia2) = (&mut dummy_body2, &SolverBodyInertia::DUMMY);

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
        match (inertia1.dominance() - inertia2.dominance()).cmp(&0) {
            Ordering::Greater => inertia1 = &SolverBodyInertia::DUMMY,
            Ordering::Less => inertia2 = &SolverBodyInertia::DUMMY,
            _ => {}
        }

        joint.solve(
            [body1, body2],
            [inertia1, inertia2],
            &mut solver_data,
            delta_secs,
        );
    }
}

/// Warm starts the motor constraints for joints of a given type.
///
/// This applies the motor impulses from the previous frame as velocity changes,
/// improving convergence for motors that need to maintain steady forces.
pub fn warm_start_xpbd_motors<
    C: Component<Mutability = Mutable> + EntityConstraint<2> + XpbdConstraint<2>,
>(
    bodies: Query<(&mut SolverBody, &SolverBodyInertia), Without<RigidBodyDisabled>>,
    mut joints: Query<(&C, &mut C::SolverData), (Without<RigidBody>, Without<JointDisabled>)>,
    time: Res<Time>,
    solver_config: Res<SolverConfig>,
) where
    C::SolverData: Component<Mutability = Mutable>,
{
    let delta_secs = time.delta_seconds_adjusted();

    let mut dummy_body1 = SolverBody::default();
    let mut dummy_body2 = SolverBody::default();

    for (joint, mut solver_data) in &mut joints {
        let [entity1, entity2] = joint.entities();

        let (mut body1, mut inertia1) = (&mut dummy_body1, &SolverBodyInertia::DUMMY);
        let (mut body2, mut inertia2) = (&mut dummy_body2, &SolverBodyInertia::DUMMY);

        if let Ok((body, inertia)) = unsafe { bodies.get_unchecked(entity1) } {
            body1 = body.into_inner();
            inertia1 = inertia;
        }
        if let Ok((body, inertia)) = unsafe { bodies.get_unchecked(entity2) } {
            body2 = body.into_inner();
            inertia2 = inertia;
        }

        // If a body has a higher dominance, it is treated as a static or kinematic body.
        match (inertia1.dominance() - inertia2.dominance()).cmp(&0) {
            Ordering::Greater => inertia1 = &SolverBodyInertia::DUMMY,
            Ordering::Less => inertia2 = &SolverBodyInertia::DUMMY,
            _ => {}
        }

        joint.warm_start_motors(
            [body1, body2],
            [inertia1, inertia2],
            &mut solver_data,
            delta_secs,
            solver_config.warm_start_coefficient,
        );
    }
}

/// Updates the linear velocity of all dynamic bodies based on the change in position from the XPBD solver.
fn project_linear_velocity(
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
fn project_angular_velocity(
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
fn project_angular_velocity(
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

fn writeback_joint_forces<C: Component + EntityConstraint<2> + XpbdConstraint<2>>(
    mut joints: Query<(&C::SolverData, &mut JointForces)>,
    time: Res<Time>,
    substep_count: Res<SubstepCount>,
) where
    C::SolverData: Component<Mutability = Mutable>,
{
    let delta_secs = time.delta_seconds_adjusted();

    // Detailed Rigid Body Simulation with Extended Position Based Dynamics by Müller et al.
    // states that  `f = λ * n / h²`. However, with substepping, it seems that we need to accumulate
    // Lagrange multipliers across substeps, and use the formula `f = λ * n / dt^2 * substep_count`.
    let rhs = (delta_secs * delta_secs).recip_or_zero() * substep_count.0 as Scalar;

    for (solver_data, mut forces) in &mut joints {
        forces.set_force(solver_data.total_position_lagrange() * rhs);
        forces.set_torque(solver_data.total_rotation_lagrange() * rhs);
        forces.set_motor_force(solver_data.total_motor_lagrange() * rhs);
    }
}
