//! The *semi-implicit* or *symplectic* Euler [integration](super) scheme.
//!
//! [Semi-implicit Euler](https://en.wikipedia.org/wiki/Semi-implicit_Euler_method)
//! integration is the most common integration scheme because it is simpler and more
//! efficient than implicit Euler integration, has great energy conservation,
//! and provides much better accuracy than explicit Euler integration.
//!
//! Semi-implicit Euler integration evalutes the acceleration at
//! the current timestep and the velocity at the next timestep:
//!
//! ```text
//! v = v_0 + a * Δt (linear velocity)
//! ω = ω_0 + α * Δt (angular velocity)
//! ```
//!
//! and computes the new position:
//!
//! ```text
//! x = x_0 + v * Δt (position)
//! θ = θ_0 + ω * Δt (rotation)
//! ```
//!
//! This order is opposite to explicit Euler integration, which uses the velocity
//! at the current timestep instead of the next timestep. The explicit approach
//! can lead to bodies gaining energy over time, which is why the semi-implicit
//! approach is typically preferred.

use super::*;

/// Integrates velocity based on the given forces in order to find
/// the linear and angular velocity after `delta_seconds` have passed.
///
/// This uses [semi-implicit (symplectic) Euler integration](self).
#[allow(clippy::too_many_arguments)]
pub fn integrate_velocity(
    lin_vel: &mut Vector,
    ang_vel: &mut AngularValue,
    force: Vector,
    torque: TorqueValue,
    mass: ComputedMass,
    angular_inertia: &ComputedAngularInertia,
    #[cfg(feature = "3d")] global_angular_inertia: &GlobalAngularInertia,
    #[cfg(feature = "3d")] rotation: Rotation,
    locked_axes: LockedAxes,
    gravity: Vector,
    delta_seconds: Scalar,
    #[cfg(feature = "3d")] is_gyroscopic: bool,
) {
    // Compute linear acceleration.
    let lin_acc = linear_acceleration(force, mass, locked_axes, gravity);

    // Compute next linear velocity.
    // v = v_0 + a * Δt
    *lin_vel += lin_acc * delta_seconds;

    // Compute angular acceleration.
    #[cfg(feature = "2d")]
    let ang_acc = angular_acceleration(torque, angular_inertia, locked_axes);
    #[cfg(feature = "3d")]
    let ang_acc = angular_acceleration(torque, global_angular_inertia, locked_axes);

    // Compute angular velocity delta.
    // Δω = α * Δt
    *ang_vel += ang_acc * delta_seconds;

    #[cfg(feature = "3d")]
    {
        if is_gyroscopic {
            // Handle gyroscopic motion, which accounts for non-spherical shapes
            // that may wobble as they spin in the air.
            solve_gyroscopic_torque(ang_vel, rotation.0, angular_inertia, delta_seconds);
            *ang_vel = locked_axes.apply_to_angular_velocity(*ang_vel);
        }
    }
}

/// Integrates position and rotation based on the given velocities in order to
/// find the position and rotation after `delta_seconds` have passed.
///
/// This uses [semi-implicit (symplectic) Euler integration](self).
pub fn integrate_position(
    pos: &mut Vector,
    rot: &mut Rotation,
    lin_vel: Vector,
    ang_vel: AngularValue,
    locked_axes: LockedAxes,
    delta_seconds: Scalar,
) {
    let lin_vel = locked_axes.apply_to_vec(lin_vel);

    // x = x_0 + v * Δt
    *pos += lin_vel * delta_seconds;

    // Effective inverse inertia along each rotational axis
    let ang_vel = locked_axes.apply_to_angular_velocity(ang_vel);

    // θ = θ_0 + ω * Δt
    #[cfg(feature = "2d")]
    {
        let delta_rot = Rotation::radians(ang_vel * delta_seconds);
        *rot = delta_rot * *rot;
        *rot = rot.fast_renormalize();
    }
    #[cfg(feature = "3d")]
    {
        let delta_rot = Quaternion::from_scaled_axis(ang_vel * delta_seconds);
        rot.0 = delta_rot * rot.0;
        *rot = rot.fast_renormalize();
    }
}

/// Computes linear acceleration based on the given forces and mass.
#[inline]
pub fn linear_acceleration(
    force: Vector,
    mass: ComputedMass,
    locked_axes: LockedAxes,
    gravity: Vector,
) -> Vector {
    // Effective inverse mass along each axis
    let effective_inverse_mass = locked_axes.apply_to_vec(Vector::splat(mass.inverse()));

    // Newton's 2nd law for translational movement:
    //
    // F = m * a
    // a = F / m
    //
    // where a is the acceleration, F is the force, and m is the mass.
    //
    // `gravity` below is the gravitational acceleration,
    // so it doesn't need to be divided by mass.
    force * effective_inverse_mass + locked_axes.apply_to_vec(gravity)
}

/// Computes angular acceleration based on the current angular velocity, torque, and inertia.
#[cfg_attr(
    feature = "3d",
    doc = "
Note that this does not account for gyroscopic motion. To compute the gyroscopic angular velocity
correction, use `solve_gyroscopic_torque`."
)]
#[inline]
pub fn angular_acceleration(
    torque: TorqueValue,
    global_angular_inertia: &ComputedAngularInertia,
    locked_axes: LockedAxes,
) -> AngularValue {
    // Effective inverse inertia along each axis
    let effective_angular_inertia = locked_axes.apply_to_angular_inertia(*global_angular_inertia);

    // Newton's 2nd law for rotational movement:
    //
    // τ = I * α
    // α = τ / I
    //
    // where α (alpha) is the angular acceleration,
    // τ (tau) is the torque, and I is the moment of inertia.
    effective_angular_inertia.inverse() * torque
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

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;

    #[test]
    fn semi_implicit_euler() {
        let mut position = Vector::ZERO;
        let mut rotation = Rotation::default();

        let mut linear_velocity = Vector::ZERO;
        #[cfg(feature = "2d")]
        let mut angular_velocity = 2.0;
        #[cfg(feature = "3d")]
        let mut angular_velocity = Vector::Z * 2.0;

        let mass = ComputedMass::new(1.0);
        #[cfg(feature = "2d")]
        let angular_inertia = ComputedAngularInertia::new(1.0);
        #[cfg(feature = "3d")]
        let angular_inertia = ComputedAngularInertia::new(Vector::ONE);

        let gravity = Vector::NEG_Y * 9.81;

        // Step by 100 steps of 0.1 seconds
        for _ in 0..100 {
            integrate_velocity(
                &mut linear_velocity,
                &mut angular_velocity,
                default(),
                default(),
                mass,
                &angular_inertia,
                #[cfg(feature = "3d")]
                &GlobalAngularInertia::new(angular_inertia, rotation),
                #[cfg(feature = "3d")]
                rotation,
                default(),
                gravity,
                1.0 / 10.0,
                #[cfg(feature = "3d")]
                true,
            );
            integrate_position(
                &mut position,
                &mut rotation,
                linear_velocity,
                angular_velocity,
                default(),
                1.0 / 10.0,
            );
        }

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
