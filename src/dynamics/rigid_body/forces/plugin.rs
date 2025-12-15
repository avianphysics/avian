use crate::{
    dynamics::{
        integrator::{
            self, CustomVelocityIntegration, IntegrationSystems, VelocityIntegrationData,
        },
        solver::{
            SolverDiagnostics,
            solver_body::{SolverBody, SolverBodyInertia},
        },
    },
    prelude::*,
};
use bevy::prelude::*;

use super::AccumulatedLocalAcceleration;

/// A plugin for managing and applying external forces, torques, and accelerations for [rigid bodies](RigidBody).
///
/// See the [module-level documentation](crate::dynamics::rigid_body::forces) for more general information about forces in Avian.
pub struct ForcePlugin;

impl Plugin for ForcePlugin {
    fn build(&self, app: &mut App) {
        // Set up system sets.
        app.configure_sets(
            PhysicsSchedule,
            (
                ForceSystems::ApplyConstantForces
                    .in_set(IntegrationSystems::UpdateVelocityIncrements)
                    .before(integrator::pre_process_velocity_increments),
                ForceSystems::Clear.in_set(SolverSystems::PostSubstep),
            ),
        );
        app.configure_sets(
            SubstepSchedule,
            ForceSystems::ApplyLocalAcceleration
                .in_set(IntegrationSystems::Velocity)
                .before(integrator::integrate_velocities),
        );

        // Accumulate constant forces, torques, and accelerations.
        app.add_systems(
            PhysicsSchedule,
            (
                apply_constant_forces,
                apply_constant_torques,
                apply_constant_linear_acceleration,
                apply_constant_angular_acceleration,
                apply_constant_local_forces,
                #[cfg(feature = "3d")]
                apply_constant_local_torques,
                apply_constant_local_linear_acceleration,
                #[cfg(feature = "3d")]
                apply_constant_local_angular_acceleration,
            )
                .chain()
                .in_set(ForceSystems::ApplyConstantForces),
        );

        // Apply local forces and accelerations.
        // This is done in the substepping loop, because the orientations of bodies can change between substeps.
        app.add_systems(
            SubstepSchedule,
            apply_local_acceleration.in_set(ForceSystems::ApplyLocalAcceleration),
        );

        // Clear accumulated forces and accelerations.
        app.add_systems(
            PhysicsSchedule,
            clear_accumulated_local_acceleration.in_set(ForceSystems::Clear),
        );
    }
}

/// System sets for managing and applying forces, torques, and accelerations for [rigid bodies](RigidBody).
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ForceSystems {
    /// Adds [`ConstantForce`], [`ConstantTorque`], [`ConstantLinearAcceleration`], and [`ConstantAngularAcceleration`]
    #[cfg_attr(
        feature = "2d",
        doc = "to [`VelocityIntegrationData`], and [`ConstantLocalForce`] and [`ConstantLocalLinearAcceleration`]"
    )]
    #[cfg_attr(
        feature = "3d",
        doc = "to [`VelocityIntegrationData`], and [`ConstantLocalForce`], [`ConstantLocalTorque`], [`ConstantLocalLinearAcceleration`], and [`ConstantLocalAngularAcceleration`]"
    )]
    /// to [`AccumulatedLocalAcceleration`].
    ApplyConstantForces,
    /// Applies [`AccumulatedLocalAcceleration`] to the linear and angular velocities of bodies.
    ApplyLocalAcceleration,
    /// Clears [`AccumulatedLocalAcceleration`] for all bodies.
    Clear,
}

/// Applies [`ConstantForce`] to the accumulated forces.
fn apply_constant_forces(
    mut bodies: Query<(&mut VelocityIntegrationData, &ComputedMass, &ConstantForce)>,
) {
    bodies
        .iter_mut()
        .for_each(|(mut integration, mass, constant_force)| {
            integration.linear_increment += mass.inverse() * constant_force.0;
        })
}

/// Applies [`ConstantTorque`] to the accumulated torques.
fn apply_constant_torques(
    mut bodies: Query<(
        &mut VelocityIntegrationData,
        &SolverBodyInertia,
        &ConstantTorque,
    )>,
) {
    bodies
        .iter_mut()
        .for_each(|(mut integration, inertia, constant_torque)| {
            integration.angular_increment +=
                inertia.effective_inv_angular_inertia() * constant_torque.0;
        })
}

/// Applies [`ConstantLinearAcceleration`] to the linear velocity increments.
fn apply_constant_linear_acceleration(
    mut bodies: Query<(&mut VelocityIntegrationData, &ConstantLinearAcceleration)>,
) {
    bodies
        .iter_mut()
        .for_each(|(mut integration, constant_acceleration)| {
            integration.linear_increment += constant_acceleration.0;
        })
}

/// Applies [`ConstantAngularAcceleration`] to the angular velocity increments.
fn apply_constant_angular_acceleration(
    mut bodies: Query<(&mut VelocityIntegrationData, &ConstantAngularAcceleration)>,
) {
    bodies
        .iter_mut()
        .for_each(|(mut integration, constant_acceleration)| {
            integration.angular_increment += constant_acceleration.0;
        })
}

/// Applies [`ConstantLocalForce`] to the accumulated forces.
fn apply_constant_local_forces(
    mut bodies: Query<(
        &mut AccumulatedLocalAcceleration,
        &ComputedMass,
        &ConstantLocalForce,
    )>,
) {
    bodies
        .iter_mut()
        .for_each(|(mut acceleration, mass, constant_force)| {
            acceleration.linear += mass.inverse() * constant_force.0;
        })
}

/// Applies [`ConstantLocalTorque`] to the accumulated torques.
#[cfg(feature = "3d")]
fn apply_constant_local_torques(
    mut bodies: Query<(
        &mut AccumulatedLocalAcceleration,
        &ComputedAngularInertia,
        &ConstantLocalTorque,
    )>,
) {
    bodies
        .iter_mut()
        .for_each(|(mut acceleration, angular_inertia, constant_torque)| {
            acceleration.angular += angular_inertia.inverse() * constant_torque.0;
        })
}

/// Applies [`ConstantLocalLinearAcceleration`] to the accumulated local acceleration.
fn apply_constant_local_linear_acceleration(
    mut bodies: Query<(
        &mut AccumulatedLocalAcceleration,
        &ConstantLocalLinearAcceleration,
    )>,
) {
    bodies
        .iter_mut()
        .for_each(|(mut acceleration, constant_acceleration)| {
            acceleration.linear += constant_acceleration.0;
        })
}

/// Applies [`ConstantLocalAngularAcceleration`] to the accumulated local acceleration.
#[cfg(feature = "3d")]
fn apply_constant_local_angular_acceleration(
    mut bodies: Query<(
        &mut AccumulatedLocalAcceleration,
        &ConstantLocalAngularAcceleration,
    )>,
) {
    bodies
        .iter_mut()
        .for_each(|(mut acceleration, constant_acceleration)| {
            acceleration.angular += constant_acceleration.0;
        })
}

/// Applies [`AccumulatedLocalAcceleration`] to the linear and angular velocity of bodies.
///
/// This should run in the substepping loop, just before [`IntegrationSystems::Velocity`].
fn apply_local_acceleration(
    mut bodies: Query<
        (&mut SolverBody, &AccumulatedLocalAcceleration, &Rotation),
        Without<CustomVelocityIntegration>,
    >,
    mut diagnostics: ResMut<SolverDiagnostics>,
    time: Res<Time<Substeps>>,
) {
    let start = crate::utils::Instant::now();

    let delta_secs = time.delta_secs_f64() as Scalar;

    bodies
        .iter_mut()
        .for_each(|(mut body, acceleration, rotation)| {
            let rotation = body.delta_rotation * *rotation;
            let locked_axes = body.flags.locked_axes();

            // Compute the world space velocity increments with locked axes applied.
            let world_linear_acceleration =
                locked_axes.apply_to_vec(rotation * acceleration.linear);
            #[cfg(feature = "3d")]
            let world_angular_acceleration =
                locked_axes.apply_to_vec(rotation * acceleration.angular);

            // Apply acceleration.
            body.linear_velocity += world_linear_acceleration * delta_secs;
            #[cfg(feature = "3d")]
            {
                body.angular_velocity += world_angular_acceleration * delta_secs;
            }
        });

    diagnostics.integrate_velocities += start.elapsed();
}

fn clear_accumulated_local_acceleration(mut query: Query<&mut AccumulatedLocalAcceleration>) {
    query.iter_mut().for_each(|mut acceleration| {
        acceleration.linear = Vector::ZERO;
        #[cfg(feature = "3d")]
        {
            acceleration.angular = Vector::ZERO;
        }
    });
}
