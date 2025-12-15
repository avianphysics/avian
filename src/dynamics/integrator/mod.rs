//! Applies forces and velocities to bodies in order to move them according to the equations of motion
//! using numerical integration.
//!
//! See [`IntegratorPlugin`].

use crate::prelude::*;
use bevy::{
    ecs::{intern::Interned, query::QueryData, schedule::ScheduleLabel},
    prelude::*,
};
use dynamics::solver::SolverDiagnostics;

use super::solver::solver_body::SolverBody;

/// Integrates Newton's 2nd law of motion, applying forces and moving entities according to their velocities.
///
/// This acts as a prediction for the next positions and orientations of the bodies. The [solver](dynamics::solver)
/// corrects these predicted positions to take constraints like contacts and joints into account.
///
/// Currently, only the [semi-implicit (symplectic) Euler](https://en.wikipedia.org/wiki/Semi-implicit_Euler_method)
/// integration scheme is supported. It is the standard for game physics, being stable, efficient, and sufficiently accurate.
///
/// See [`IntegrationSystems`] for the system sets used by this plugin.
pub struct IntegratorPlugin {
    schedule: Interned<dyn ScheduleLabel>,
}

impl IntegratorPlugin {
    /// Creates an [`IntegratorPlugin`] with the schedule that the integration systems should run in.
    ///
    /// The default schedule is [`SubstepSchedule`].
    pub fn new(schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: schedule.intern(),
        }
    }
}

impl Default for IntegratorPlugin {
    fn default() -> Self {
        Self::new(SubstepSchedule)
    }
}

impl Plugin for IntegratorPlugin {
    fn build(&self, app: &mut App) {
        // Add `VelocityIntegrationData` to all `SolverBody`s.
        app.register_required_components::<SolverBody, VelocityIntegrationData>();

        app.init_resource::<Gravity>();

        app.configure_sets(
            PhysicsSchedule,
            (
                IntegrationSystems::UpdateVelocityIncrements
                    .in_set(SolverSystems::PreSubstep)
                    .before(IntegrationSystems::Velocity),
                IntegrationSystems::ClearVelocityIncrements
                    .in_set(SolverSystems::PostSubstep)
                    .after(IntegrationSystems::Velocity),
            ),
        );

        app.add_systems(
            PhysicsSchedule,
            (
                pre_process_velocity_increments
                    .in_set(IntegrationSystems::UpdateVelocityIncrements),
                clear_velocity_increments.in_set(IntegrationSystems::ClearVelocityIncrements),
            ),
        );

        app.configure_sets(
            self.schedule.intern(),
            (IntegrationSystems::Velocity, IntegrationSystems::Position).chain(),
        );

        app.add_systems(
            self.schedule.intern(),
            (
                (integrate_velocities, clamp_velocities)
                    .chain()
                    .in_set(IntegrationSystems::Velocity),
                integrate_positions.in_set(IntegrationSystems::Position),
            ),
        );
    }
}

/// System sets for position and velocity integration,
/// applying forces and moving bodies based on velocity.
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum IntegrationSystems {
    /// Applies gravity and locked axes to the linear and angular velocity increments of bodies,
    /// and multiplies them by the substep delta time to get the final per-substep increments.
    ///
    /// Runs in the [`PhysicsSchedule`], in [`SolverSystems::PreSubstep`].
    UpdateVelocityIncrements,
    /// Applies velocity increments to the linear and angular velocities of bodies.
    ///
    /// Typically runs in the [`SubstepSchedule`], in [`IntegrationSystems::Velocity`].
    Velocity,
    /// Moves bodies based on their current velocities and the physics time step.
    ///
    /// Typically runs in the [`SubstepSchedule`], in [`IntegrationSystems::Position`].
    Position,
    /// Clears the velocity increments of bodies after the substepping loop.
    ///
    /// Runs in the [`PhysicsSchedule`], in [`SolverSystems::PostSubstep`].
    ClearVelocityIncrements,
}

/// A deprecated alias for [`IntegrationSystems`].
#[deprecated(since = "0.4.0", note = "Renamed to `IntegrationSystems`")]
pub type IntegrationSet = IntegrationSystems;

/// A resource for the global gravitational acceleration.
///
/// The default is an acceleration of 9.81 m/s^2 pointing down, which is approximate to the gravitational
/// acceleration near Earth's surface. Note that if you are using pixels as length units in 2D,
/// this gravity will be tiny. You should modify the gravity to fit your application.
///
/// You can also control how gravity affects a specific [rigid body](RigidBody) using the [`GravityScale`]
/// component. The magnitude of the gravity will be multiplied by this scaling factor.
///
/// # Example
///
/// ```no_run
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// # #[cfg(feature = "f32")]
/// fn main() {
///     App::new()
///         .add_plugins((DefaultPlugins, PhysicsPlugins::default()))
#[cfg_attr(
    feature = "2d",
    doc = "         .insert_resource(Gravity(Vec2::NEG_Y * 100.0))"
)]
#[cfg_attr(
    feature = "3d",
    doc = "         .insert_resource(Gravity(Vec3::NEG_Y * 19.6))"
)]
///         .run();
/// }
/// # #[cfg(not(feature = "f32"))]
/// # fn main() {} // Doc test needs main
/// ```
///
/// You can also modify gravity while the app is running.
#[derive(Reflect, Resource, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Resource)]
pub struct Gravity(pub Vector);

impl Default for Gravity {
    fn default() -> Self {
        Self(Vector::Y * -9.81)
    }
}

impl Gravity {
    /// Zero gravity.
    pub const ZERO: Gravity = Gravity(Vector::ZERO);
}

/// A marker component for bodies that use custom velocity integration.
///
/// This means that gravity, damping, and external forces will not be applied automatically.
/// You are responsible for applying all forces and updating velocities manually.
///
/// Exceptions include:
///
/// - Contact impulses and joint impulses for dynamic bodies
/// - Impulses applied via [`Forces`]
#[derive(Component, Debug, Default, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default)]
pub struct CustomVelocityIntegration;

/// A marker component for bodies that use custom position integration.
///
/// This means that the body's position and rotation will not be updated automatically
/// based on velocity. You are responsible for updating the position and rotation manually.
///
/// This can be useful for implementing kinematic bodies that are moved according to custom logic,
/// such as with [`MoveAndSlide`].
#[derive(Component, Debug, Default, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default)]
pub struct CustomPositionIntegration;

/// Pre-computed data for speeding up velocity integration.
///
/// This includes:
///
/// - Velocity increments for [`Gravity`].
/// - Velocity increments for [`ConstantForce`], [`ConstantTorque`], [`ConstantLinearAcceleration`], and [`ConstantAngularAcceleration`].
/// - Velocity increments for forces, torques, and accelerations applied using [`Forces`].
/// - Cached operands for applying linear and angular velocity damping.
///
/// The values are computed once per time step, and applied to the body at each substep
/// with basic addition and multiplication. This moves the more expensive operations
/// and branching out of the substepping loop.
// -----------------------
// 20 bytes in 2D with f32
// 32 bytes in 3D with f32
#[derive(Component, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
pub struct VelocityIntegrationData {
    /// The linear velocity increment to be applied to the body at each substep.
    ///
    /// **Note:** This is treated as linear acceleration until [`IntegrationSystems::UpdateVelocityIncrements`].
    /// where it is multiplied by the time step to get the corresponding velocity increment.
    pub linear_increment: Vector,
    /// The angular velocity increment to be applied to the body at each substep.
    ///
    /// **Note:** This is treated as angular acceleration until [`IntegrationSystems::UpdateVelocityIncrements`].
    /// where it is multiplied by the time step to get the corresponding velocity increment.
    pub angular_increment: AngularVector,
    /// The right-hand side of the linear damping equation,
    /// `1 / (1 + dt * c)`, where `c` is the damping coefficient.
    pub linear_damping_rhs: Scalar,
    /// The right-hand side of the angular damping equation,
    /// `1 / (1 + dt * c)`, where `c` is the damping coefficient.
    pub angular_damping_rhs: Scalar,
}

impl VelocityIntegrationData {
    /// Applies a given linear acceleration to the body.
    pub fn apply_linear_acceleration(&mut self, acceleration: Vector) {
        self.linear_increment += acceleration;
    }

    /// Applies a given angular acceleration to the body.
    pub fn apply_angular_acceleration(&mut self, acceleration: AngularVector) {
        self.angular_increment += acceleration;
    }

    /// Updates the cached right-hand side of the linear damping equation,
    /// `1 / (1 + dt * c)`, where `c` is the damping coefficient.
    pub fn update_linear_damping_rhs(&mut self, damping_coefficient: Scalar, delta_secs: Scalar) {
        self.linear_damping_rhs = 1.0 / (1.0 + delta_secs * damping_coefficient);
    }

    /// Updates the cached right-hand side of the angular damping equation,
    /// `1 / (1 + dt * c)`, where `c` is the damping coefficient.
    pub fn update_angular_damping_rhs(&mut self, damping_coefficient: Scalar, delta_secs: Scalar) {
        self.angular_damping_rhs = 1.0 / (1.0 + delta_secs * damping_coefficient);
    }
}

/// Applies gravity and locked axes to the linear and angular velocity increments of bodies.
pub fn pre_process_velocity_increments(
    mut bodies: Query<(
        &RigidBody,
        &mut VelocityIntegrationData,
        Option<&LinearDamping>,
        Option<&AngularDamping>,
        Option<&GravityScale>,
        Option<&LockedAxes>,
    )>,
    gravity: Res<Gravity>,
    time: Res<Time<Substeps>>,
    mut diagnostics: ResMut<SolverDiagnostics>,
) {
    let start = crate::utils::Instant::now();

    let delta_secs = time.delta_secs_f64() as Scalar;

    // TODO: Do we want to skip kinematic bodies here?
    bodies.par_iter_mut().for_each(
        |(rb, mut integration, lin_damping, ang_damping, gravity_scale, locked_axes)| {
            if !rb.is_dynamic() {
                // Skip non-dynamic bodies.
                return;
            }

            let locked_axes = locked_axes.map_or(LockedAxes::default(), |locked_axes| *locked_axes);

            // Update the cached right-hand side of the velocity damping equation,
            // `1 / (1 + dt * c)`, where `c` is the damping coefficient.
            let lin_damping = lin_damping.map_or(0.0, |damping| damping.0);
            let ang_damping = ang_damping.map_or(0.0, |damping| damping.0);
            integration.update_linear_damping_rhs(lin_damping, delta_secs);
            integration.update_angular_damping_rhs(ang_damping, delta_secs);

            // NOTE: The `ForcePlugin` handles the application of external forces and torques.
            // NOTE: The velocity increments are treated as accelerations at this point.

            // Apply gravity.
            integration.linear_increment += gravity.0 * gravity_scale.map_or(1.0, |scale| scale.0);

            // Apply locked axes.
            integration.linear_increment = locked_axes.apply_to_vec(integration.linear_increment);
            integration.angular_increment =
                locked_axes.apply_to_angular_velocity(integration.angular_increment);

            // The velocity increments are treated as accelerations until this point.
            // Multiply by the time step to get the final velocity increments.
            integration.linear_increment *= delta_secs;
            integration.angular_increment *= delta_secs;
        },
    );

    diagnostics.update_velocity_increments += start.elapsed();
}

/// Clears the velocity increments of bodies after the substepping loop.
fn clear_velocity_increments(
    mut bodies: Query<&mut VelocityIntegrationData, With<SolverBody>>,
    mut diagnostics: ResMut<SolverDiagnostics>,
) {
    let start = crate::utils::Instant::now();

    bodies.par_iter_mut().for_each(|mut integration| {
        integration.linear_increment = Vector::ZERO;
        integration.angular_increment = AngularVector::ZERO;
    });

    diagnostics.update_velocity_increments += start.elapsed();
}

#[derive(QueryData)]
#[query_data(mutable)]
#[doc(hidden)]
pub struct VelocityIntegrationQuery {
    solver_body: &'static mut SolverBody,
    integration: &'static mut VelocityIntegrationData,
    #[cfg(feature = "3d")]
    angular_inertia: &'static ComputedAngularInertia,
    #[cfg(feature = "3d")]
    rotation: &'static Rotation,
}

/// Integrates the velocities of bodies by applying velocity increments and damping.
pub fn integrate_velocities(
    mut bodies: Query<
        VelocityIntegrationQuery,
        (RigidBodyActiveFilter, Without<CustomVelocityIntegration>),
    >,
    mut diagnostics: ResMut<SolverDiagnostics>,
    #[cfg(feature = "3d")] time: Res<Time>,
) {
    let start = crate::utils::Instant::now();

    #[cfg(feature = "3d")]
    let delta_secs = time.delta_secs_f64() as Scalar;

    bodies.par_iter_mut().for_each(|mut body| {
        if body.solver_body.flags.is_kinematic() {
            // Skip kinematic bodies.
            return;
        }

        // Apply velocity damping.
        body.solver_body.linear_velocity *= body.integration.linear_damping_rhs;
        body.solver_body.angular_velocity *= body.integration.angular_damping_rhs;

        // Apply velocity increments.
        body.solver_body.linear_velocity += body.integration.linear_increment;
        body.solver_body.angular_velocity += body.integration.angular_increment;

        #[cfg(feature = "3d")]
        {
            if body.solver_body.is_gyroscopic() {
                // TODO: Should this be opt-in with a `GyroscopicMotion` component?
                // TODO: It's a bit unfortunate that this has to run in the substepping loop
                //       rather than pre-computing the velocity increments once per time step.
                //       This needs to be done because the gyroscopic torque relies on up-to-date rotations
                //       and world-space angular inertia tensors. Omitting the change in orientation would
                //       lead to worse accuracy and angular momentum not being conserved.
                let rotation = body.solver_body.delta_rotation.0 * body.rotation.0;
                solve_gyroscopic_torque(
                    &mut body.solver_body.angular_velocity,
                    rotation,
                    body.angular_inertia,
                    delta_secs,
                );
            }
        }
    });

    diagnostics.integrate_velocities += start.elapsed();
}

/// Applies the effects of gyroscopic motion to the given angular velocity.
///
/// Gyroscopic motion is the tendency of a rotating object to maintain its axis of rotation
/// unless acted upon by an external torque. It manifests as objects with non-uniform angular
/// inertia tensors seemingly wobbling as they spin in the air or on the ground.
///
/// Gyroscopic motion is important for realistic spinning behavior, and for simulating
/// gyroscopic phenomena such as the Dzhanibekov effect.
#[cfg(feature = "3d")]
#[inline]
pub fn solve_gyroscopic_torque(
    ang_vel: &mut Vector,
    rotation: Quaternion,
    local_inertia: &ComputedAngularInertia,
    delta_secs: Scalar,
) {
    // References:
    // - The "Gyroscopic Motion" section of Erin Catto's GDC 2015 slides on Numerical Methods.
    //   https://box2d.org/files/ErinCatto_NumericalMethods_GDC2015.pdf
    // - Jolt Physics - MotionProperties::ApplyGyroscopicForceInternal
    //   https://github.com/jrouwe/JoltPhysics/blob/d497df2b9b0fa9aaf41295e1406079c23148232d/Jolt/Physics/Body/MotionProperties.inl#L102
    //
    // Erin Catto's GDC presentation suggests using implicit Euler for gyroscopic torque,
    // as semi-implicit Euler can easily blow up with larger time steps due to extrapolating velocity.
    // The extrapolation diverges quickly because gyroscopic torque is quadratic in the angular velocity.
    //
    // However, implicit Euler is measurably more expensive than semi-implicit Euler.
    // We instead take inspiration from Jolt, and use semi-implicit Euler integration,
    // clamping the magnitude of the angular momentum to remain the same.
    // This is efficient, prevents energy from being introduced into the system,
    // and produces reasonably accurate results for game purposes.

    // Convert angular velocity to body space so that we can use the local angular inertia.
    let local_ang_vel = rotation.inverse() * *ang_vel;

    // Compute body-space angular momentum.
    let local_momentum = local_inertia.tensor() * local_ang_vel;

    // The gyroscopic torque is given by:
    //
    // T = -ω x I ω = -ω x L
    //
    // where ω is the angular velocity, I is the angular inertia tensor,
    // and L is the angular momentum.
    //
    // The change in angular momentum is given by:
    //
    // ΔL = T Δt = -ω x L Δt
    //
    // Thus, we can compute the new angular momentum as:
    //
    // L' = L + ΔL = L - Δt (ω x L)
    let mut new_local_momentum = local_momentum - delta_secs * local_ang_vel.cross(local_momentum);

    // Make sure the magnitude of the angular momentum remains the same to avoid introducing
    // energy into the system due to the extrapolation done by semi-implicit Euler integration.
    let new_local_momentum_length_squared = new_local_momentum.length_squared();
    if new_local_momentum_length_squared == 0.0 {
        *ang_vel = Vector::ZERO;
        return;
    }
    new_local_momentum *=
        (local_momentum.length_squared() / new_local_momentum_length_squared).sqrt();

    // Convert back to world-space angular velocity.
    let local_inverse_inertia = local_inertia.inverse();
    *ang_vel = rotation * (local_inverse_inertia * new_local_momentum);
}

// NOTE: If the majority of bodies have clamped velocities, it would be more efficient
//       to do this in `integrate_velocities` rather than in a separate system.
//       By doing this in a separate system, we're optimizing for the assumption
//       that only some bodies have clamped velocities.
/// Clamps the velocities of bodies to [`MaxLinearSpeed`] and [`MaxAngularSpeed`].
fn clamp_velocities(
    mut bodies: ParamSet<(
        Query<(&mut SolverBody, &MaxLinearSpeed)>,
        Query<(&mut SolverBody, &MaxAngularSpeed)>,
    )>,
    mut diagnostics: ResMut<SolverDiagnostics>,
) {
    let start = crate::utils::Instant::now();

    // Clamp linear velocity.
    bodies.p0().iter_mut().for_each(|(mut body, max_speed)| {
        let linear_speed_squared = body.linear_velocity.length_squared();
        if linear_speed_squared > max_speed.0 * max_speed.0 {
            body.linear_velocity *= max_speed.0 / linear_speed_squared.sqrt();
        }
    });

    // Clamp angular velocity.
    bodies.p1().iter_mut().for_each(|(mut body, max_speed)| {
        #[cfg(feature = "2d")]
        if body.angular_velocity.abs() > max_speed.0 {
            body.angular_velocity = max_speed.copysign(body.angular_velocity);
        }
        #[cfg(feature = "3d")]
        {
            let angular_speed_squared = body.angular_velocity.length_squared();
            if angular_speed_squared > max_speed.0 * max_speed.0 {
                body.angular_velocity *= max_speed.0 / angular_speed_squared.sqrt();
            }
        }
    });

    diagnostics.integrate_velocities += start.elapsed();
}

/// Integrates the positions of bodies based on their velocities and the time step.
pub fn integrate_positions(
    mut solver_bodies: Query<&mut SolverBody, Without<CustomPositionIntegration>>,
    time: Res<Time>,
    mut diagnostics: ResMut<SolverDiagnostics>,
) {
    let start = crate::utils::Instant::now();

    let delta_secs = time.delta_seconds_adjusted();

    solver_bodies.par_iter_mut().for_each(|body| {
        let SolverBody {
            linear_velocity,
            angular_velocity,
            delta_position,
            delta_rotation,
            ..
        } = body.into_inner();

        *delta_position += *linear_velocity * delta_secs;
        #[cfg(feature = "2d")]
        {
            // Note: We should probably use `add_angle_fast` here
            *delta_rotation = Rotation::radians(*angular_velocity * delta_secs) * *delta_rotation;
        }
        #[cfg(feature = "3d")]
        {
            delta_rotation.0 =
                Quaternion::from_scaled_axis(*angular_velocity * delta_secs) * delta_rotation.0;
        }
    });

    diagnostics.integrate_positions += start.elapsed();
}

#[cfg(test)]
mod tests {
    use core::time::Duration;

    use approx::assert_relative_eq;

    use crate::prelude::*;
    use bevy::{mesh::MeshPlugin, prelude::*, time::TimeUpdateStrategy};

    fn create_app() -> App {
        let mut app = App::new();
        app.add_plugins((
            MinimalPlugins,
            PhysicsPlugins::default(),
            TransformPlugin,
            #[cfg(feature = "bevy_scene")]
            AssetPlugin::default(),
            #[cfg(feature = "bevy_scene")]
            bevy::scene::ScenePlugin,
            MeshPlugin,
        ));
        app
    }

    #[test]
    fn semi_implicit_euler() {
        let mut app = create_app();
        app.insert_resource(SubstepCount(1));
        app.finish();

        let body_entity = app
            .world_mut()
            .spawn((
                RigidBody::Dynamic,
                #[cfg(feature = "2d")]
                {
                    (
                        MassPropertiesBundle::from_shape(&Rectangle::from_length(1.0), 1.0),
                        AngularVelocity(2.0),
                    )
                },
                #[cfg(feature = "3d")]
                {
                    (
                        MassPropertiesBundle::from_shape(&Cuboid::from_length(1.0), 1.0),
                        AngularVelocity(Vector::Z * 2.0),
                    )
                },
            ))
            .id();

        // Step by 100 steps of 0.1 seconds.
        app.insert_resource(Time::from_hz(10.0));
        app.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f64(
            1.0 / 10.0,
        )));

        // Initialize the app.
        app.update();

        for _ in 0..100 {
            app.update();
        }

        // Get the body after the simulation.
        let entity_ref = app.world_mut().entity(body_entity);
        let position = entity_ref.get::<Position>().unwrap().0;
        let rotation = *entity_ref.get::<Rotation>().unwrap();
        let linear_velocity = entity_ref.get::<LinearVelocity>().unwrap().0;
        let angular_velocity = entity_ref.get::<AngularVelocity>().unwrap().0;

        // Euler methods have some precision issues, but this seems weirdly inaccurate.
        assert_relative_eq!(position, Vector::NEG_Y * 490.5, epsilon = 10.0);

        #[cfg(feature = "2d")]
        assert_relative_eq!(
            rotation.as_radians(),
            Rotation::radians(20.0).as_radians(),
            epsilon = 0.00001
        );
        #[cfg(feature = "3d")]
        assert_relative_eq!(
            rotation.0,
            Quaternion::from_rotation_z(20.0),
            epsilon = 0.01
        );

        assert_relative_eq!(linear_velocity, Vector::NEG_Y * 98.1, epsilon = 0.0001);
        #[cfg(feature = "2d")]
        assert_relative_eq!(angular_velocity, 2.0, epsilon = 0.00001);
        #[cfg(feature = "3d")]
        assert_relative_eq!(angular_velocity, Vector::Z * 2.0, epsilon = 0.00001);
    }
}
