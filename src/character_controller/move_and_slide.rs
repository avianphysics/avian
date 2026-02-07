//! Contains the *move and slide* algorithm and utilities for kinematic character controllers.
//!
//! See the documentation of [`MoveAndSlide`] for more information.

pub use super::velocity_project::*;

use crate::{collision::collider::contact_query::contact_manifolds, prelude::*};
use bevy::{ecs::system::SystemParam, prelude::*};
use core::time::Duration;

/// Needed to improve stability when `n.dot(dir)` happens to be very close to zero.
const DOT_EPSILON: Scalar = 0.005;

/// Cosine of 5 degrees.
#[allow(clippy::excessive_precision)]
pub const COS_5_DEGREES: Scalar = 0.99619469809;

/// A [`SystemParam`] for the *move and slide* algorithm, also known as *collide and slide* or *step slide*.
///
/// Move and slide is the core movement and collision algorithm used by most kinematic character controllers.
/// It attempts to move a shape along a desired velocity vector, sliding along any colliders that are hit on the way.
///
/// # Algorithm
///
/// At a high level, the algorithm works as follows:
///
/// 1. Sweep the shape along the desired velocity vector.
/// 2. If no collision is detected, move the full distance.
/// 3. If a collision is detected:
///    - Move up to the point of collision.
///    - Project the remaining velocity onto the contact surfaces to obtain a new sliding velocity.
/// 4. Repeat with the new sliding velocity until movement is complete.
///
/// The algorithm also includes depenetration passes before and after movement to improve stability
/// and ensure that the shape does not intersect any colliders.
///
/// # Configuration
///
/// [`MoveAndSlideConfig`] allows configuring various aspects of the algorithm.
/// See its documentation for more information.
///
/// Additionally, [`move_and_slide`](MoveAndSlide::move_and_slide) can be given a callback that is called
/// for each contact surface that is detected during movement. This allows for custom handling of collisions,
/// such as triggering events, or modifying movement based on specific colliders.
///
/// # Other Utilities
///
/// In addition to the main `move_and_slide` method, this system parameter also provides utilities for:
///
/// - Performing shape casts optimized for movement via [`cast_move`](MoveAndSlide::cast_move).
/// - Depenetrating shapes that are intersecting colliders via [`depenetrate`](MoveAndSlide::depenetrate).
/// - Performing intersection tests via [`intersections`](MoveAndSlide::intersections).
/// - Projecting velocities to slide along contact planes via [`project_velocity`](MoveAndSlide::project_velocity).
///
/// These methods are used internally by the move and slide algorithm, but can also be used independently
/// for custom movement and collision handling.
///
/// # Resources
///
/// Some useful resources for learning more about the move and slide algorithm include:
///
/// - [*Collide And Slide - \*Actually Decent\* Character Collision From Scratch*](https://youtu.be/YR6Q7dUz2uk) by [Poke Dev](https://www.youtube.com/@poke_gamedev) (video)
/// - [`PM_SlideMove`](https://github.com/id-Software/Quake-III-Arena/blob/dbe4ddb10315479fc00086f08e25d968b4b43c49/code/game/bg_slidemove.c#L45) in Quake III Arena (source code)
///
/// Note that while the high-level concepts are similar across different implementations, details may vary.
#[derive(SystemParam)]
#[doc(alias = "CollideAndSlide")]
#[doc(alias = "StepSlide")]
pub struct MoveAndSlide<'w, 's> {
    /// The [`SpatialQuery`] system parameter used to perform shape casts and other geometric queries.
    pub spatial_query: SpatialQuery<'w, 's>,
    /// The [`Query`] used to query for colliders.
    pub colliders: Query<
        'w,
        's,
        (
            &'static Collider,
            &'static Position,
            &'static Rotation,
            Option<&'static CollisionLayers>,
        ),
        (With<ColliderOf>, Without<Sensor>),
    >,
    /// A units-per-meter scaling factor that adjusts some thresholds and tolerances
    /// to the scale of the world for better behavior.
    pub length_unit: Res<'w, PhysicsLengthUnit>,
}

/// Configuration for [`MoveAndSlide::move_and_slide`].
#[derive(Clone, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct MoveAndSlideConfig {
    /// How many iterations to use when moving the character.
    ///
    /// A single iteration consists of:
    ///
    /// - Moving the character as far as possible in the desired velocity direction.
    /// - Modifying the velocity to slide along any contact surfaces.
    ///
    /// Increasing this allows the character to slide along more surfaces in a single frame,
    /// which can help with complex geometry and high speeds, but increases computation time.
    pub move_and_slide_iterations: usize,

    /// How many iterations to use when performing depenetration.
    ///
    /// Depenetration is an iterative process that solves penetrations for all planes,
    /// until we either reached [`MoveAndSlideConfig::move_and_slide_iterations`]
    /// or the accumulated error is less than [`MoveAndSlideConfig::max_depenetration_error`].
    ///
    /// To disable depenetration, set this to `0`.
    pub depenetration_iterations: usize,

    /// The target error to achieve when performing depenetration.
    ///
    /// Depenetration is an iterative process that solves penetrations for all planes,
    /// until we either reached [`MoveAndSlideConfig::move_and_slide_iterations`]
    /// or the accumulated error is less than [`MoveAndSlideConfig::max_depenetration_error`].
    ///
    /// This is implicitly scaled by the [`PhysicsLengthUnit`].
    pub max_depenetration_error: Scalar,

    /// The maximum penetration depth that is allowed for a contact to be resolved during depenetration.
    ///
    /// This is used to reject invalid contacts that have an excessively high penetration depth,
    /// which can lead to clipping through geometry. This may be removed in the future once the
    /// collision errors in the underlying collision detection system are fixed.
    ///
    /// This is implicitly scaled by the [`PhysicsLengthUnit`].
    pub penetration_rejection_threshold: Scalar,

    /// A minimal distance to always keep between the collider and any other colliders.
    ///
    /// This exists to improve numerical stability and ensure that the collider never intersects anything.
    /// Set this to a small enough value that you don't see visual artifacts but have good stability.
    ///
    /// Increase the value if you notice your character getting stuck in geometry.
    /// Decrease it when you notice jittering, especially around V-shaped walls.
    ///
    /// This is implicitly scaled by the [`PhysicsLengthUnit`].
    pub skin_width: Scalar,

    /// The initial planes to consider for the move and slide algorithm.
    ///
    /// This will be expanded during the algorithm with contact planes, but you can also initialize it
    /// with some predefined planes that the algorithm should never move against.
    ///
    /// A common use case is adding the ground plane when a character controller is standing or walking on the ground.
    pub planes: Vec<Dir>,

    /// The dot product threshold to consider two planes as similar when pruning nearly parallel planes.
    /// The comparison used is `n1.dot(n2) >= plane_similarity_dot_threshold`.
    ///
    /// This is used to reduce the number of planes considered during move and slide,
    /// which can improve performance for dense geomtry. However, setting this value too high
    /// can lead to unwanted behavior, as it may discard important planes.
    ///
    /// The default value of [`COS_5_DEGREES`] (≈0.996) corresponds to a 5 degree angle between the planes.
    pub plane_similarity_dot_threshold: Scalar,

    /// The maximum number of planes to solve while performing move and slide.
    ///
    /// If the number of planes exceeds this value, the algorithm will stop collecting new planes.
    /// This is a safety measure to prevent excessive computation time for dense geometry.
    pub max_planes: usize,
}

impl Default for MoveAndSlideConfig {
    fn default() -> Self {
        let default_depen_cfg = DepenetrationConfig::default();
        Self {
            move_and_slide_iterations: 4,
            depenetration_iterations: default_depen_cfg.depenetration_iterations,
            max_depenetration_error: default_depen_cfg.max_depenetration_error,
            penetration_rejection_threshold: default_depen_cfg.penetration_rejection_threshold,
            skin_width: default_depen_cfg.skin_width,
            planes: Vec::new(),
            plane_similarity_dot_threshold: COS_5_DEGREES,
            max_planes: 20,
        }
    }
}

/// Configuration for [`MoveAndSlide::depenetrate`].
#[derive(Clone, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct DepenetrationConfig {
    /// How many iterations to use when performing depenetration.
    ///
    /// Depenetration is an iterative process that solves penetrations for all planes,
    /// until we either reached [`MoveAndSlideConfig::move_and_slide_iterations`]
    /// or the accumulated error is less than [`MoveAndSlideConfig::max_depenetration_error`].
    ///
    /// To disable depenetration, set this to `0`.
    pub depenetration_iterations: usize,

    /// The target error to achieve when performing depenetration.
    ///
    /// Depenetration is an iterative process that solves penetrations for all planes,
    /// until we either reached [`MoveAndSlideConfig::move_and_slide_iterations`]
    /// or the accumulated error is less than [`MoveAndSlideConfig::max_depenetration_error`].
    ///
    /// This is implicitly scaled by the [`PhysicsLengthUnit`].
    pub max_depenetration_error: Scalar,

    /// The maximum penetration depth that is allowed for a contact to be resolved during depenetration.
    ///
    /// This is used to reject invalid contacts that have an excessively high penetration depth,
    /// which can lead to clipping through geometry. This may be removed in the future once the
    /// collision errors in the underlying collision detection system are fixed.
    ///
    /// This is implicitly scaled by the [`PhysicsLengthUnit`].
    pub penetration_rejection_threshold: Scalar,

    /// A minimal distance to always keep between the collider and any other colliders.
    ///
    /// This exists to improve numerical stability and ensure that the collider never intersects anything.
    /// Set this to a small enough value that you don't see visual artifacts but have good stability.
    ///
    /// Increase the value if you notice your character getting stuck in geometry.
    /// Decrease it when you notice jittering, especially around V-shaped walls.
    ///
    /// This is implicitly scaled by the [`PhysicsLengthUnit`].
    pub skin_width: Scalar,
}

impl Default for DepenetrationConfig {
    fn default() -> Self {
        Self {
            depenetration_iterations: 16,
            max_depenetration_error: 0.0001,
            penetration_rejection_threshold: 0.5,
            skin_width: 0.01,
        }
    }
}

impl From<&MoveAndSlideConfig> for DepenetrationConfig {
    fn from(config: &MoveAndSlideConfig) -> Self {
        Self {
            depenetration_iterations: config.depenetration_iterations,
            max_depenetration_error: config.max_depenetration_error,
            penetration_rejection_threshold: config.penetration_rejection_threshold,
            skin_width: config.skin_width,
        }
    }
}

/// Output from [`MoveAndSlide::move_and_slide`].
#[derive(Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct MoveAndSlideOutput {
    /// The final position of the character after move and slide.
    ///
    /// Set your [`Transform::translation`] to this value.
    pub position: Vector,

    /// The final velocity of the character after move and slide.
    ///
    /// This corresponds to the remaining velocity after the algorithm has slid along all contact surfaces.
    /// For example, if the character is trying to move to the right, but there is a ramp in its path,
    /// the projected velocity will point up the ramp, with reduced magnitude.
    ///
    /// It is useful to store this value or apply it to [`LinearVelocity`] and use it as the input velocity
    /// for the next frame's call to [`MoveAndSlide::move_and_slide`].
    ///
    /// Note that if you apply this to [`LinearVelocity`], it is recommended to use [`CustomPositionIntegration`].
    /// This ways, the character's position is only updated via the move and slide algorithm,
    /// and not also by the physics integrator.
    pub projected_velocity: Vector,
}

/// Data related to a hit during [`MoveAndSlide::move_and_slide`].
#[derive(Debug, PartialEq)]
pub struct MoveAndSlideHitData<'a> {
    /// The entity of the collider that was hit by the shape.
    pub entity: Entity,

    /// The maximum distance that is safe to move in the given direction so that the collider
    /// still keeps a distance of `skin_width` to the other colliders.
    ///
    /// This is `0.0` when any of the following is true:
    ///
    /// - The collider started off intersecting another collider.
    /// - The collider is moving toward another collider that is already closer than `skin_width`.
    ///
    /// If you want to know the real distance to the next collision, use [`Self::collision_distance`].
    pub distance: Scalar,

    /// The hit point on the shape that was hit, expressed in world space.
    pub point: Vector,

    /// The outward surface normal on the hit shape at `point`, expressed in world space.
    pub normal: &'a mut Dir,

    /// The position of the collider at the time of the move and slide iteration.
    pub position: &'a mut Vector,

    /// The velocity of the collider at the time of the move and slide iteration.
    pub velocity: &'a mut Vector,

    /// The raw distance to the next collision, not respecting skin width.
    /// To move the shape, use [`Self::distance`] instead.
    #[doc(alias = "time_of_impact")]
    pub collision_distance: Scalar,
}

impl<'a> MoveAndSlideHitData<'a> {
    /// Whether the collider started off already intersecting another collider when it was cast.
    ///
    /// Note that this will be `false` if the collider was closer than `skin_width`, but not physically intersecting.
    pub fn intersects(&self) -> bool {
        self.collision_distance == 0.0
    }
}

/// Indicates how to handle a hit detected during [`MoveAndSlide::move_and_slide`].
///
/// This is returned by the `on_hit` callback provided to [`MoveAndSlide::move_and_slide`].
#[derive(Debug, PartialEq)]
pub enum MoveAndSlideHitResponse {
    /// Accept the hit and continue the move and slide algorithm.
    Accept,

    /// Ignore the hit and continue the move and slide algorithm.
    ///
    /// Note that the shape will still be moved up to the point of collision,
    /// but the velocity will not be modified to slide along the surface.
    Ignore,

    /// Ignore the hit and abort the move and slide algorithm.
    ///
    /// Note that the shape will still be moved up to the point of collision,
    /// but no further movement or velocity modification will be performed.
    Abort,
}

/// Data related to a hit during [`MoveAndSlide::cast_move`].
#[derive(Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct MoveHitData {
    /// The entity of the collider that was hit by the shape.
    pub entity: Entity,

    /// The maximum distance that is safe to move in the given direction so that the collider
    /// still keeps a distance of `skin_width` to the other colliders.
    ///
    /// This is `0.0` when any of the following is true:
    ///
    /// - The collider started off intersecting another collider.
    /// - The collider is moving toward another collider that is already closer than `skin_width`.
    ///
    /// If you want to know the real distance to the next collision, use [`Self::collision_distance`].
    #[doc(alias = "time_of_impact")]
    pub distance: Scalar,

    /// The closest point on the shape that was hit, expressed in world space.
    ///
    /// If the shapes are penetrating or the target distance is greater than zero,
    /// this will be different from `point2`.
    pub point1: Vector,

    /// The closest point on the shape that was cast, expressed in world space.
    ///
    /// If the shapes are penetrating or the target distance is greater than zero,
    /// this will be different from `point1`.
    pub point2: Vector,

    /// The outward surface normal on the hit shape at `point1`, expressed in world space.
    pub normal1: Vector,

    /// The outward surface normal on the cast shape at `point2`, expressed in world space.
    pub normal2: Vector,

    /// The raw distance to the next collision, not respecting skin width.
    /// To move the shape, use [`Self::distance`] instead.
    #[doc(alias = "time_of_impact")]
    pub collision_distance: Scalar,
}

impl MoveHitData {
    /// Whether the collider started off already intersecting another collider when it was cast.
    ///
    /// Note that this will be `false` if the collider was closer than `skin_width`, but not physically intersecting.
    pub fn intersects(self) -> bool {
        self.collision_distance == 0.0
    }
}

impl<'w, 's> MoveAndSlide<'w, 's> {
    /// Moves a shape along a given velocity vector, sliding along any colliders that are hit on the way.
    ///
    /// See [`MoveAndSlide`] for an overview of the algorithm.
    ///
    /// # Arguments
    ///
    /// - `shape`: The shape being cast represented as a [`Collider`].
    /// - `shape_position`: Where the shape is cast from.
    /// - `shape_rotation`: The rotation of the shape being cast.
    /// - `velocity`: The initial velocity vector along which to move the shape. This will be modified to reflect sliding along surfaces.
    /// - `delta_time`: The duration over which to move the shape. `velocity * delta_time` gives the total desired movement vector.
    /// - `config`: A [`MoveAndSlideConfig`] that determines the behavior of the move and slide. [`MoveAndSlideConfig::default()`] should be a good start for most cases.
    /// - `filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query. It is highly recommended to exclude the entity holding the collider itself,
    ///   otherwise the character will collide with itself.
    /// - `on_hit`: A callback that is called when a collider is hit as part of the move and slide iterations. The returned [`MoveAndSlideHitResponse`] determines how to handle the hit.
    ///   If you don't have any special handling per collision, you can pass `|_| MoveAndSlideHitResponse::Accept`.
    ///
    /// # Example
    ///
    /// ```
    /// use bevy::prelude::*;
    /// use std::collections::HashSet;
    #[cfg_attr(
        feature = "2d",
        doc = "use avian2d::{prelude::*, math::{Vector, AdjustPrecision as _, AsF32 as _}};"
    )]
    #[cfg_attr(
        feature = "3d",
        doc = "use avian3d::{prelude::*, math::{Vector, AdjustPrecision as _, AsF32 as _}};"
    )]
    ///
    /// #[derive(Component)]
    /// struct CharacterController {
    ///     velocity: Vector,
    /// }
    ///
    /// fn perform_move_and_slide(
    ///     player: Single<(Entity, &Collider, &mut CharacterController, &mut Transform)>,
    ///     move_and_slide: MoveAndSlide,
    ///     time: Res<Time>
    /// ) {
    ///     let (entity, collider, mut controller, mut transform) = player.into_inner();
    ///     let velocity = controller.velocity + Vector::X * 10.0;
    ///     let filter = SpatialQueryFilter::from_excluded_entities([entity]);
    ///     let mut collisions = HashSet::new();
    ///     let out = move_and_slide.move_and_slide(
    ///         collider,
    #[cfg_attr(
        feature = "2d",
        doc = "         transform.translation.xy().adjust_precision(),"
    )]
    #[cfg_attr(
        feature = "3d",
        doc = "         transform.translation.adjust_precision(),"
    )]
    #[cfg_attr(
        feature = "2d",
        doc = "         transform.rotation.to_euler(EulerRot::XYZ).2.adjust_precision(),"
    )]
    #[cfg_attr(
        feature = "3d",
        doc = "         transform.rotation.adjust_precision(),"
    )]
    ///         velocity,
    ///         time.delta(),
    ///         &MoveAndSlideConfig::default(),
    ///         &filter,
    ///         |hit| {
    ///             collisions.insert(hit.entity);
    ///             MoveAndSlideHitResponse::Accept
    ///         },
    ///     );
    #[cfg_attr(
        feature = "2d",
        doc = "     transform.translation = out.position.f32().extend(0.0);"
    )]
    #[cfg_attr(
        feature = "3d",
        doc = "     transform.translation = out.position.f32();"
    )]
    ///     controller.velocity = out.projected_velocity;
    ///     info!("Colliding with entities: {:?}", collisions);
    /// }
    /// ```
    #[must_use]
    #[doc(alias = "collide_and_slide")]
    #[doc(alias = "step_slide")]
    pub fn move_and_slide(
        &self,
        shape: &Collider,
        shape_position: Vector,
        shape_rotation: RotationValue,
        mut velocity: Vector,
        delta_time: Duration,
        config: &MoveAndSlideConfig,
        filter: &SpatialQueryFilter,
        mut on_hit: impl FnMut(MoveAndSlideHitData) -> MoveAndSlideHitResponse,
    ) -> MoveAndSlideOutput {
        let mut position = shape_position;
        let mut time_left = {
            #[cfg(feature = "f32")]
            {
                delta_time.as_secs_f32()
            }
            #[cfg(feature = "f64")]
            {
                delta_time.as_secs_f64()
            }
        };
        let skin_width = self.length_unit.0 * config.skin_width;

        // Initial depenetration pass
        let depenetration_offset =
            self.depenetrate(shape, position, shape_rotation, &config.into(), filter);
        position += depenetration_offset;

        // Main move and slide loop:
        // 1. Sweep the shape along the velocity vector
        // 2. If we hit something, move up to the hit point
        // 3. Collect contact planes
        // 4. Project velocity to slide along contact planes
        // 5. Repeat until we run out of iterations or time
        for _ in 0..config.move_and_slide_iterations {
            let sweep = time_left * velocity;
            let Some((vel_dir, distance)) = Dir::new_and_length(sweep.f32()).ok() else {
                // No movement left
                break;
            };
            let distance = distance.adjust_precision();

            const MIN_DISTANCE: Scalar = 1e-4;
            if distance < MIN_DISTANCE {
                break;
            }

            // Sweep the shape along the velocity vector.
            let Some(sweep_hit) =
                self.cast_move(shape, position, shape_rotation, sweep, skin_width, filter)
            else {
                // No collision, move the full distance.
                position += sweep;
                break;
            };
            let point = sweep_hit.point2;

            // Move up to the hit point.
            time_left -= time_left * (sweep_hit.distance / distance);
            position += vel_dir.adjust_precision() * sweep_hit.distance;

            // Initialize velocity clipping planes with the user-defined planes.
            // This often includes a ground plane.
            let mut planes: Vec<Dir> = config.planes.clone();

            // We need to add the sweep hit's plane explicitly, as `contact_manifolds` sometimes returns nothing
            // due to a Parry bug. Otherwise, `contact_manifolds` would pick up this normal anyways.
            // TODO: Remove this once the collision bug is fixed.
            let hit_response = on_hit(MoveAndSlideHitData {
                entity: sweep_hit.entity,
                point,
                normal: &mut Dir::new_unchecked(sweep_hit.normal1.f32()),
                collision_distance: sweep_hit.collision_distance,
                distance: sweep_hit.distance,
                position: &mut position,
                velocity: &mut velocity,
            });

            if hit_response == MoveAndSlideHitResponse::Accept {
                planes.push(Dir::new_unchecked(sweep_hit.normal1.f32()));
            } else if hit_response == MoveAndSlideHitResponse::Abort {
                break;
            }

            // Collect contact planes.
            let mut aborted = false;
            self.intersections(
                shape,
                position,
                shape_rotation,
                // Use a slightly larger skin width to ensure we catch all contacts for velocity clipping.
                // Depenetration still uses just the normal skin width.
                skin_width * 2.0,
                filter,
                |contact_point, mut normal| {
                    // Check if this plane is nearly parallel to an existing one.
                    // This can help prune redundant planes for velocity clipping.
                    for existing_normal in planes.iter_mut() {
                        if normal.dot(**existing_normal) as Scalar
                            >= config.plane_similarity_dot_threshold
                        {
                            // Keep the most blocking version of the plane.
                            let n_dot_v = normal.adjust_precision().dot(velocity);
                            let existing_n_dot_v = existing_normal.adjust_precision().dot(velocity);
                            if n_dot_v < existing_n_dot_v {
                                *existing_normal = normal;
                            }
                            return true;
                        }
                    }

                    if planes.len() >= config.max_planes {
                        return false;
                    }

                    // Call the user-defined hit callback.
                    let hit_response = on_hit(MoveAndSlideHitData {
                        entity: sweep_hit.entity,
                        point: contact_point.point,
                        normal: &mut normal,
                        collision_distance: sweep_hit.collision_distance,
                        distance: sweep_hit.distance,
                        position: &mut position,
                        velocity: &mut velocity,
                    });

                    match hit_response {
                        MoveAndSlideHitResponse::Accept => {
                            // Add the contact plane for velocity clipping.
                            planes.push(normal);
                            true
                        }
                        MoveAndSlideHitResponse::Ignore => true,
                        MoveAndSlideHitResponse::Abort => {
                            aborted = true;
                            false
                        }
                    }
                },
            );

            // Project velocity to slide along contact planes.
            velocity = Self::project_velocity(velocity, &planes);

            if aborted {
                break;
            }
        }

        // Final depenetration pass
        // TODO: We could get the intersections from the last iteration and avoid re-querying them here.
        let depenetration_offset =
            self.depenetrate(shape, position, shape_rotation, &config.into(), filter);
        position += depenetration_offset;

        MoveAndSlideOutput {
            position,
            projected_velocity: velocity,
        }
    }

    /// A [shape cast](spatial_query#shapecasting) optimized for movement. Use this if you want to move a collider
    /// with a given velocity and stop so that it keeps a distance of `skin_width` from the first collider on its path.
    ///
    /// This operation is most useful when you ensure that the character is not intersecting any colliders before moving.
    /// To do this, call [`MoveAndSlide::depenetrate`] and add the resulting offset vector to the character's position
    /// before calling this method. See the example below.
    ///
    /// It is often useful to clip the velocity afterwards so that it no longer points into the contact plane using [`Self::project_velocity`].
    ///
    /// # Arguments
    ///
    /// - `shape`: The shape being cast represented as a [`Collider`].
    /// - `shape_position`: Where the shape is cast from.
    /// - `shape_rotation`: The rotation of the shape being cast.
    /// - `movement`: The direction and magnitude of the movement. If this is [`Vector::ZERO`], this method can still return `Some(MoveHitData)` if the shape started off intersecting a collider.
    /// - `skin_width`: A [`ShapeCastConfig`] that determines the behavior of the cast.
    /// - `filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query. It is highly recommended to exclude the entity holding the collider itself,
    ///   otherwise the character will collide with itself.
    ///
    /// # Returns
    ///
    /// - `Some(MoveHitData)` if the shape hit a collider on the way, or started off intersecting a collider.
    /// - `None` if the shape is able to move the full distance without hitting a collider.
    ///
    /// # Example
    ///
    /// ```
    /// use bevy::prelude::*;
    #[cfg_attr(
        feature = "2d",
        doc = "use avian2d::{prelude::*, math::{Vector, Dir, AdjustPrecision as _, AsF32 as _}};"
    )]
    #[cfg_attr(
        feature = "3d",
        doc = "use avian3d::{prelude::*, math::{Vector, Dir, AdjustPrecision as _, AsF32 as _}};"
    )]
    ///
    /// #[derive(Component)]
    /// struct CharacterController {
    ///     velocity: Vector,
    /// }
    ///
    /// fn perform_cast_move(
    ///     player: Single<(Entity, &Collider, &mut CharacterController, &mut Transform)>,
    ///     move_and_slide: MoveAndSlide,
    ///     time: Res<Time>
    /// ) {
    ///     let (entity, collider, mut controller, mut transform) = player.into_inner();
    ///     let filter = SpatialQueryFilter::from_excluded_entities([entity]);
    ///     let config = MoveAndSlideConfig::default();
    ///
    ///     // Ensure that the character is not intersecting with any colliders.
    ///     let offset = move_and_slide.depenetrate(
    ///         collider,
    #[cfg_attr(
        feature = "2d",
        doc = "         transform.translation.xy().adjust_precision(),"
    )]
    #[cfg_attr(
        feature = "3d",
        doc = "         transform.translation.adjust_precision(),"
    )]
    #[cfg_attr(
        feature = "2d",
        doc = "         transform.rotation.to_euler(EulerRot::XYZ).2.adjust_precision(),"
    )]
    #[cfg_attr(
        feature = "3d",
        doc = "         transform.rotation.adjust_precision(),"
    )]
    ///         &((&config).into()),
    ///         &filter,
    ///     );
    #[cfg_attr(
        feature = "2d",
        doc = "     transform.translation += offset.f32().extend(0.0);"
    )]
    #[cfg_attr(feature = "3d", doc = "     transform.translation += offset.f32();")]
    ///     let velocity = controller.velocity;
    ///
    ///     let hit = move_and_slide.cast_move(
    ///         collider,
    #[cfg_attr(
        feature = "2d",
        doc = "         transform.translation.xy().adjust_precision(),"
    )]
    #[cfg_attr(
        feature = "3d",
        doc = "         transform.translation.adjust_precision(),"
    )]
    #[cfg_attr(
        feature = "2d",
        doc = "         transform.rotation.to_euler(EulerRot::XYZ).2.adjust_precision(),"
    )]
    #[cfg_attr(
        feature = "3d",
        doc = "         transform.rotation.adjust_precision(),"
    )]
    ///         velocity * time.delta_secs().adjust_precision(),
    ///         config.skin_width,
    ///         &filter,
    ///     );
    ///     if let Some(hit) = hit {
    ///         // We collided with something on the way. Advance as much as possible.
    #[cfg_attr(
        feature = "2d",
        doc = "         transform.translation += (velocity.normalize_or_zero() * hit.distance).extend(0.0).f32();"
    )]
    #[cfg_attr(
        feature = "3d",
        doc = "         transform.translation += (velocity.normalize_or_zero() * hit.distance).f32();"
    )]
    ///         // Then project the velocity to make sure it no longer points towards the contact plane.
    ///         controller.velocity =
    ///             MoveAndSlide::project_velocity(velocity, &[Dir::new_unchecked(hit.normal1.f32())])
    ///     } else {
    ///         // We traveled the full distance without colliding.
    #[cfg_attr(
        feature = "2d",
        doc = "         transform.translation += velocity.extend(0.0).f32();"
    )]
    #[cfg_attr(
        feature = "3d",
        doc = "         transform.translation += velocity.f32();"
    )]
    ///     }
    /// }
    /// ```
    ///
    /// # Related methods
    ///
    /// - [`SpatialQuery::cast_shape`]
    #[must_use]
    #[doc(alias = "sweep")]
    pub fn cast_move(
        &self,
        shape: &Collider,
        shape_position: Vector,
        shape_rotation: RotationValue,
        movement: Vector,
        skin_width: Scalar,
        filter: &SpatialQueryFilter,
    ) -> Option<MoveHitData> {
        let (direction, distance) = Dir::new_and_length(movement.f32()).unwrap_or((Dir::X, 0.0));
        let distance = distance.adjust_precision();
        let shape_hit = self.spatial_query.cast_shape_predicate(
            shape,
            shape_position,
            shape_rotation,
            direction,
            &ShapeCastConfig {
                ignore_origin_penetration: true,
                ..ShapeCastConfig::from_max_distance(distance)
            },
            filter,
            // Make sure we don't hit sensors.
            // TODO: Replace this when spatial queries support excluding sensors directly.
            &|entity| self.colliders.contains(entity),
        )?;
        let safe_distance = if distance == 0.0 {
            0.0
        } else {
            Self::pull_back(shape_hit, direction, skin_width)
        };
        Some(MoveHitData {
            distance: safe_distance,
            collision_distance: distance,
            entity: shape_hit.entity,
            point1: shape_hit.point1,
            point2: shape_hit.point2,
            normal1: shape_hit.normal1,
            normal2: shape_hit.normal2,
        })
    }

    /// Returns a [`ShapeHitData::distance`] that is reduced such that the hit distance is at least `skin_width`.
    /// The result will never be negative, so if the hit is already closer than `skin_width`, the returned distance will be zero.
    #[must_use]
    fn pull_back(hit: ShapeHitData, dir: Dir, skin_width: Scalar) -> Scalar {
        let dot = dir.adjust_precision().dot(-hit.normal1).max(DOT_EPSILON);
        let skin_distance = skin_width / dot;
        (hit.distance - skin_distance).max(0.0)
    }

    /// Moves a collider so that it no longer intersects any other collider and keeps a minimum distance
    /// of [`DepenetrationConfig::skin_width`] scaled by the [`PhysicsLengthUnit`].
    ///
    /// Depenetration is an iterative process that solves penetrations for all planes, until we either reached
    /// [`MoveAndSlideConfig::move_and_slide_iterations`] or the accumulated error is less than [`MoveAndSlideConfig::max_depenetration_error`].
    /// If the maximum number of iterations was reached before the error is below the threshold, the current best attempt is returned,
    /// in which case the collider may still be intersecting with other colliders.
    ///
    /// This method is equivalent to calling [`Self::depenetrate_intersections`] with the results of [`Self::intersections`].
    ///
    /// # Arguments
    ///
    /// - `shape`: The shape that intersections are tested against represented as a [`Collider`].
    /// - `shape_position`: The position of the shape.
    /// - `shape_rotation`: The rotation of the shape.
    /// - `config`: A [`DepenetrationConfig`] that determines the behavior of the depenetration. [`DepenetrationConfig::default()`] should be a good start for most cases.
    /// - `filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query.
    ///
    /// # Example
    ///
    /// ```
    /// use bevy::prelude::*;
    #[cfg_attr(
        feature = "2d",
        doc = "use avian2d::{prelude::*, character_controller::move_and_slide::DepenetrationConfig, math::{AdjustPrecision as _, AsF32 as _}};"
    )]
    #[cfg_attr(
        feature = "3d",
        doc = "use avian3d::{prelude::*, character_controller::move_and_slide::DepenetrationConfig, math::{AdjustPrecision as _, AsF32 as _}};"
    )]
    /// fn depenetrate_player(
    ///     player: Single<(Entity, &Collider, &mut Transform)>,
    ///     move_and_slide: MoveAndSlide,
    ///     time: Res<Time>
    /// ) {
    ///     let (entity, collider, mut transform) = player.into_inner();
    ///     let filter = SpatialQueryFilter::from_excluded_entities([entity]);
    ///
    ///     let offset = move_and_slide.depenetrate(
    ///         collider,
    #[cfg_attr(
        feature = "2d",
        doc = "         transform.translation.xy().adjust_precision(),"
    )]
    #[cfg_attr(
        feature = "3d",
        doc = "         transform.translation.adjust_precision(),"
    )]
    #[cfg_attr(
        feature = "2d",
        doc = "         transform.rotation.to_euler(EulerRot::XYZ).2.adjust_precision(),"
    )]
    #[cfg_attr(
        feature = "3d",
        doc = "         transform.rotation.adjust_precision(),"
    )]
    ///         &DepenetrationConfig::default(),
    ///         &filter,
    ///     );
    #[cfg_attr(
        feature = "2d",
        doc = "     transform.translation += offset.f32().extend(0.0);"
    )]
    #[cfg_attr(feature = "3d", doc = "     transform.translation += offset.f32();")]
    /// }
    /// ```
    ///
    /// See also [`MoveAndSlide::cast_move`] for a typical usage scenario.
    ///
    /// # Related methods
    ///
    /// - [`MoveAndSlide::intersections`]
    /// - [`MoveAndSlide::depenetrate_intersections`]
    pub fn depenetrate(
        &self,
        shape: &Collider,
        shape_position: Vector,
        shape_rotation: RotationValue,
        config: &DepenetrationConfig,
        filter: &SpatialQueryFilter,
    ) -> Vector {
        if config.depenetration_iterations == 0 {
            // Depenetration disabled
            return Vector::ZERO;
        }

        let mut intersections = Vec::new();
        self.intersections(
            shape,
            shape_position,
            shape_rotation,
            self.length_unit.0 * config.skin_width,
            filter,
            |contact_point, normal| {
                intersections.push((
                    normal,
                    contact_point.penetration + self.length_unit.0 * config.skin_width,
                ));
                true
            },
        );
        self.depenetrate_intersections(config, &intersections)
    }

    /// Manual version of [`MoveAndSlide::depenetrate`].
    ///
    /// Moves a collider so that it no longer intersects any other collider and keeps a minimum distance
    /// of [`DepenetrationConfig::skin_width`] scaled by the [`PhysicsLengthUnit`]. The intersections
    /// should be provided as a list of contact plane normals and penetration distances, which can be obtained
    /// via [`MoveAndSlide::intersections`].
    ///
    /// Depenetration is an iterative process that solves penetrations for all planes, until we either reached
    /// [`MoveAndSlideConfig::move_and_slide_iterations`] or the accumulated error is less than [`MoveAndSlideConfig::max_depenetration_error`].
    /// If the maximum number of iterations was reached before the error is below the threshold, the current best attempt is returned,
    /// in which case the collider may still be intersecting with other colliders.
    ///
    /// # Arguments
    ///
    /// - `config`: A [`DepenetrationConfig`] that determines the behavior of the depenetration. [`DepenetrationConfig::default()`] should be a good start for most cases.
    /// - `intersections`: A list of contact plane normals and penetration distances representing the intersections to resolve.
    ///
    /// # Returns
    ///
    /// A displacement vector that can be added to the `shape_position` to resolve the intersections,
    /// or the best attempt if the max iterations were reached before a solution was found.
    ///
    /// # Example
    ///
    /// ```
    /// use bevy::prelude::*;
    #[cfg_attr(
        feature = "2d",
        doc = "use avian2d::{prelude::*, character_controller::move_and_slide::DepenetrationConfig, math::{AdjustPrecision as _, AsF32 as _}};"
    )]
    #[cfg_attr(
        feature = "3d",
        doc = "use avian3d::{prelude::*, character_controller::move_and_slide::DepenetrationConfig, math::{AdjustPrecision as _, AsF32 as _}};"
    )]
    /// fn depenetrate_player_manually(
    ///     player: Single<(Entity, &Collider, &mut Transform)>,
    ///     move_and_slide: MoveAndSlide,
    ///     time: Res<Time>
    /// ) {
    ///     let (entity, collider, mut transform) = player.into_inner();
    ///     let filter = SpatialQueryFilter::from_excluded_entities([entity]);
    ///     let config = DepenetrationConfig::default();
    ///
    ///     let mut intersections = Vec::new();
    ///     move_and_slide.intersections(
    ///         collider,
    #[cfg_attr(
        feature = "2d",
        doc = "         transform.translation.xy().adjust_precision(),"
    )]
    #[cfg_attr(
        feature = "3d",
        doc = "         transform.translation.adjust_precision(),"
    )]
    #[cfg_attr(
        feature = "2d",
        doc = "         transform.rotation.to_euler(EulerRot::XYZ).2.adjust_precision(),"
    )]
    #[cfg_attr(
        feature = "3d",
        doc = "         transform.rotation.adjust_precision(),"
    )]
    ///         config.skin_width,
    ///         &filter,
    ///         |contact_point, normal| {
    ///             intersections.push((normal, contact_point.penetration + config.skin_width));
    ///             true
    ///         },
    ///     );
    ///     let offset = move_and_slide.depenetrate_intersections(&config, &intersections);
    #[cfg_attr(
        feature = "2d",
        doc = "     transform.translation += offset.f32().extend(0.0);"
    )]
    #[cfg_attr(feature = "3d", doc = "     transform.translation += offset.f32();")]
    /// }
    /// ```
    ///
    /// # Related methods
    ///
    /// - [`MoveAndSlide::intersections`]
    /// - [`MoveAndSlide::depenetrate`]
    #[must_use]
    pub fn depenetrate_intersections(
        &self,
        config: &DepenetrationConfig,
        intersections: &[(Dir, Scalar)],
    ) -> Vector {
        let mut fixup = Vector::ZERO;

        // Gauss-Seidel style iterative depenetration
        for _ in 0..config.depenetration_iterations {
            let mut total_error = 0.0;

            for (normal, dist) in intersections {
                if *dist > self.length_unit.0 * config.penetration_rejection_threshold {
                    continue;
                }
                let normal = normal.adjust_precision();
                let error = (dist - fixup.dot(normal)).max(0.0);
                total_error += error;
                fixup += error * normal;
            }

            if total_error < self.length_unit.0 * config.max_depenetration_error {
                break;
            }
        }

        fixup
    }

    /// An [intersection test](spatial_query#intersection-tests) that calls a callback for each [`Collider`] found
    /// that is closer to the given `shape` with a given position and rotation than `prediction_distance`.
    ///
    /// # Arguments
    ///
    /// - `shape`: The shape that intersections are tested against represented as a [`Collider`].
    /// - `shape_position`: The position of the shape.
    /// - `shape_rotation`: The rotation of the shape.
    /// - `filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query.
    /// - `prediction_distance`: An extra margin applied to the [`Collider`].
    /// - `callback`: A callback that is called for each intersection found. The callback receives the deepest contact point and the contact normal.
    ///   Returning `false` will stop further processing of intersections.
    ///
    /// # Example
    ///
    /// See [`MoveAndSlide::depenetrate_intersections`] for a typical usage scenario.
    ///
    /// # Related methods
    ///
    /// - [`MoveAndSlide::depenetrate_intersections`]
    /// - [`MoveAndSlide::depenetrate`]
    pub fn intersections(
        &self,
        shape: &Collider,
        shape_position: Vector,
        shape_rotation: RotationValue,
        prediction_distance: Scalar,
        filter: &SpatialQueryFilter,
        mut callback: impl FnMut(&ContactPoint, Dir) -> bool,
    ) {
        let expanded_aabb = shape
            .aabb(shape_position, shape_rotation)
            .grow(Vector::splat(prediction_distance));
        let aabb_intersections = self
            .spatial_query
            .aabb_intersections_with_aabb(expanded_aabb);

        'outer: for intersection_entity in aabb_intersections {
            let Ok((intersection_collider, intersection_pos, intersection_rot, layers)) =
                self.colliders.get(intersection_entity)
            else {
                continue;
            };
            let layers = layers.copied().unwrap_or_default();
            if !filter.test(intersection_entity, layers) {
                continue;
            }
            let mut manifolds = Vec::new();
            contact_manifolds(
                shape,
                shape_position,
                shape_rotation,
                intersection_collider,
                *intersection_pos,
                *intersection_rot,
                prediction_distance,
                &mut manifolds,
            );
            for manifold in manifolds {
                let Some(deepest) = manifold.find_deepest_contact() else {
                    continue;
                };

                let normal = Dir::new_unchecked(-manifold.normal.f32());

                if !callback(deepest, normal) {
                    // Abort further processing.
                    break 'outer;
                }
            }
        }
    }

    /// Projects input velocity `v` onto the planes defined by the given `normals`.
    /// This ensures that `velocity` does not point into any of the planes, but along them.
    ///
    /// This is often used after [`MoveAndSlide::cast_move`] to ensure a character moved that way
    /// does not try to continue moving into colliding geometry.
    #[must_use]
    pub fn project_velocity(v: Vector, normals: &[Dir]) -> Vector {
        project_velocity(v, normals)
    }
}
