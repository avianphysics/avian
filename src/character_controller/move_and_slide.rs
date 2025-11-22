//! Contains the *move and slide* algorithm and utilities for kinematic character controllers.
//!
//! See the documentation of [`MoveAndSlide`] for more information.

use crate::{collision::collider::contact_query::contact_manifolds, prelude::*};
use bevy::{ecs::system::SystemParam, prelude::*};

/// A [`SystemParam`] for the *move and slide* algorithm, also known as *collide and slide* or *step slide*.
///
/// Move and slide is the core movement and collision algorithm used by most kinematic character controllers.
/// It attempts to move a shape along a desired velocity vector, while sliding along any colliders it hits on the way.
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
/// 4. Repeat with the new sliding velocity.
///
/// The algorithm also includes depenetration passes before and after movement to ensure the shape is not intersecting any colliders.
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
    /// The [`SpatialQueryPipeline`] used to perform spatial queries.
    pub query_pipeline: Res<'w, SpatialQueryPipeline>,
    /// The [`Query`] used to query colliders.
    pub colliders: Query<
        'w,
        's,
        (
            &'static Collider,
            &'static Position,
            &'static Rotation,
            Option<&'static CollisionLayers>,
        ),
    >,
    /// The [`Time`] resource, used primarily to calculate the delta-time.
    pub time: Res<'w, Time>,
}

impl<'w, 's> MoveAndSlide<'w, 's> {
    /// Moves a shape along a given velocity vector while sliding along any colliders it hits on the way.
    ///
    /// See [`MoveAndSlide`] for an overview of the algorithm.
    ///
    /// # Arguments
    ///
    /// - `shape`: The shape being cast represented as a [`Collider`].
    /// - `shape_position`: Where the shape is cast from.
    /// - `shape_rotation`: The rotation of the shape being cast.
    /// - `velocity`: The direction and magnitude of the movement. If this is [`Vector::ZERO`], no movement is performed, but the collider is still depenetrated.
    /// - `config`: A [`MoveAndSlideConfig`] that determines the behavior of the move and slide. [`MoveAndSlideConfig::default()`] should be a good start for most cases.
    /// - `filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query. It is highly recommended to exclude the entity holding the collider itself,
    ///   otherwise the character will collide with itself.
    /// - `on_hit`: A callback that is called when a collider is hit as part of the move and slide iterations. Returning `false` will abort the move and slide operation.
    ///   Starting intersections, i.e. those that happen when the collider is already stuck in another collider, will be reported to this callback, but aborted even if the callback returns `true`.
    ///   If you don't have any special handling per collision, you can pass `|_| true`.
    ///
    /// # Example
    ///
    /// ```rust
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
    #[cfg_attr(feature = "3d", doc = "         transform.rotation,")]
    ///         velocity,
    ///         &MoveAndSlideConfig::default(),
    ///         &filter,
    ///         |hit| {
    ///             collisions.insert(hit.entity);
    ///             true
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
    ///     controller.velocity = out.clipped_velocity;
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
        config: &MoveAndSlideConfig,
        filter: &SpatialQueryFilter,
        mut on_hit: impl FnMut(MoveAndSlideHitData) -> bool,
    ) -> MoveAndSlideOutput {
        // High level overview:
        // 1. Initial Gauss-Seidel depenetration pass
        // 2. For each iteration, until movement is done or max iterations reached:
        //    - Sweep the shape along the velocity vector
        //    - If we hit something, move up to the hit point
        //    - Collect contact planes
        //    - Depenetrate based on intersections
        //    - Project velocity to be parallel to all contact planes
        let mut position = shape_position;
        let original_velocity = velocity;
        let mut time_left = {
            #[cfg(feature = "f32")]
            {
                self.time.delta_secs()
            }
            #[cfg(feature = "f64")]
            {
                self.time.delta_secs_f64()
            }
        };

        // Initial depenetration pass
        let mut intersections = Vec::new();
        self.intersections(
            shape,
            position,
            shape_rotation,
            config.skin_width,
            filter,
            |contact_point, normal| {
                // TODO: Should we call on_hit here?
                intersections.push((normal, contact_point.penetration + config.skin_width));
                true
            },
        );
        let depenetration_offset = Self::depenetrate(&config.into(), &intersections);
        position += depenetration_offset;

        // Main move and slide loop:
        // 1. Sweep the shape along the velocity vector
        // 2. If we hit something, move up to the hit point
        // 3. Collect contact planes
        // 4. Depenetrate based on intersections
        // 5. Project velocity to be parallel to all contact planes
        // 6. Repeat until we run out of iterations or time
        'outer: for _ in 0..config.move_and_slide_iterations {
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
            let Some(sweep_hit) = self.cast_move(
                shape,
                position,
                shape_rotation,
                sweep,
                config.skin_width,
                filter,
            ) else {
                // No collision, move the full distance.
                position += sweep;
                break;
            };

            if sweep_hit.intersects() {
                // The entity is completely trapped in another solid.
                velocity = Vector::ZERO;
                break 'outer;
            }

            // Move up to the hit point.
            time_left -= time_left * (sweep_hit.distance / distance);
            position += vel_dir.adjust_precision() * sweep_hit.distance;

            // Initialize velocity clipping planes with the user-defined planes.
            // This often includes a ground plane.
            let mut planes = config.planes.clone();

            // Store penetrating contacts for depenetration.
            let mut intersections = Vec::new();

            // Collect contact planes.
            self.intersections(
                shape,
                position,
                shape_rotation,
                // Use a slightly larger skin width to ensure we catch all contacts for velocity clipping.
                // Depenetration still uses just the normal skin width.
                config.skin_width * 2.0,
                filter,
                |contact_point, normal| {
                    if planes.len() >= config.max_planes {
                        return false;
                    }

                    if !on_hit(MoveAndSlideHitData {
                        intersects: false,
                        entity: sweep_hit.entity,
                        point: contact_point.point,
                        normal,
                        collision_distance: sweep_hit.collision_distance,
                        distance: sweep_hit.distance,
                        position,
                        velocity,
                    }) {
                        return false;
                    }

                    // Add the contact plane for velocity clipping.
                    planes.push(normal);

                    // Store penetrating contacts for depenetration.
                    let total_penetration = contact_point.penetration + config.skin_width;
                    if total_penetration > 0.0 {
                        intersections.push((normal, total_penetration));
                    }

                    true
                },
            );

            // Depenetrate based on intersections.
            let depenetration_offset = Self::depenetrate(&config.into(), &intersections);
            position += depenetration_offset;

            // Project velocity to be parallel to all contact planes.
            velocity = Self::project_velocity(velocity, &planes);

            // If the original velocity is against the original velocity, stop dead
            // to avoid tiny occilations in sloping corners.
            if velocity.dot(original_velocity) <= -DOT_EPSILON {
                velocity = Vector::ZERO;
                break 'outer;
            }
        }

        MoveAndSlideOutput {
            position,
            clipped_velocity: velocity,
        }
    }

    /// A [shape cast](spatial_query#shapecasting) optimized for movement. Use this if you want to move a collider with a given velocity and stop so that
    /// it keeps a distance of `skin_width` from the first collider on its path.
    ///
    /// This operation is most useful when you ensure that the character is not intersecting any colliders before moving. You can do so by calling [`MoveAndSlide::depenetrate`]
    /// and adding the resulting offset vector to the character's position before calling this method. See the example below.
    ///
    /// You will often find it useful to afterwards clip the velocity so that it no longer points into the collision plane by using [`Self::clip_velocity`].
    ///
    /// # Arguments
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
    /// ```rust
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
    ///     let velocity = controller.velocity;
    ///     let filter = SpatialQueryFilter::from_excluded_entities([entity]);
    ///
    ///     // Note: you probably want to call `MoveAndSlide::depenetrate()` here before calling `cast_move()`
    ///     // to ensure that the character is not intersecting with any colliders. See that method's documentation for more details.
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
    #[cfg_attr(feature = "3d", doc = "         transform.rotation,")]
    ///         velocity * time.delta_secs().adjust_precision(),
    ///         MoveAndSlideConfig::default().skin_width,
    ///         &filter,
    ///     );
    ///     if let Some(hit) = hit {
    ///         // We collided with something on the way. Advance as much as possible
    #[cfg_attr(
        feature = "2d",
        doc = "         transform.translation += (velocity.normalize_or_zero() * hit.distance).extend(0.0).f32();"
    )]
    #[cfg_attr(
        feature = "3d",
        doc = "         transform.translation += (velocity.normalize_or_zero() * hit.distance).f32();"
    )]
    ///         // Then project the velocity to make sure it no longer points towards the collision plane
    ///         controller.velocity =
    ///             MoveAndSlide::project_velocity(velocity, &[Dir::new_unchecked(hit.normal1.f32())])
    ///     } else {
    ///         // We traveled the full distance without colliding
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
        let shape_hit = self.query_pipeline.cast_shape(
            shape,
            shape_position,
            shape_rotation,
            direction,
            &ShapeCastConfig::from_max_distance(distance),
            filter,
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
    ///
    /// # Example
    ///
    /// TODO: check if the character is allowed to stand up
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
            .query_pipeline
            .aabb_intersections_with_aabb(expanded_aabb);
        for intersection_entity in aabb_intersections {
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
                callback(deepest, normal);
            }
        }
    }

    /// Moves a collider so that it no longer intersects any other collider and keeps a minimum distance of [`DepenetrationConfig::skin_width`] to all.
    /// The intersections should be provided as a list of contact plane normals and penetration distances, which can be obtained via [`MoveAndSlide::intersections_callback`].
    ///
    /// Depenetration is an iterative process that solves penetrations for all planes bit-by-bit, until we either reached [`MoveAndSlideConfig::move_and_slide_iterations`]
    /// or the accumulated error is less than [`MoveAndSlideConfig::max_depenetration_error`]. If the max iterations were reached before the error was below the threshold,
    /// the current best attempt is returned, in which case the collider may still be intersecting with other colliders.
    ///
    /// # Arguments
    ///
    /// - `config`: A [`DepenetrationConfig`] that determines the behavior of the depenetration. [`DepenetrationConfig::default()`] should be a good start for most cases.
    ///
    /// # Returns
    ///
    /// A displacement vector that can be added to the `shape_position` to resolve the intersections,
    /// or the best attempt if the max iterations were reached before a solution was found.
    ///
    /// # Example
    ///
    /// TODO: use
    ///
    /// See also [`MoveAndSlide::cast_move`] for a typical usage scenario.
    #[must_use]
    pub fn depenetrate(config: &DepenetrationConfig, intersections: &[(Dir, Scalar)]) -> Vector {
        if intersections.is_empty() {
            return Vector::ZERO;
        }

        let mut fixup = Vector::ZERO;
        for _ in 0..config.depenetration_iterations {
            let mut total_error = 0.0;
            for (normal, dist) in intersections {
                let normal = normal.adjust_precision();
                let error = (dist - fixup.dot(normal)).max(0.0);
                total_error += error;
                fixup += error * normal;
            }
            if total_error < config.max_depenetration_error {
                break;
            }
        }
        fixup
    }

    /// Projects input velocity `v` onto the convex cone defined by the provided contact `normals`.
    /// This ensures that `velocity` does not point into any of the given `planes`, but along them.
    ///
    /// Returns the projected velocity. If there are no planes, the velocity is returned unchanged.
    ///
    /// Often used after [`MoveAndSlide::cast_move`] to ensure a character moved that way does not try to continue moving into colliding geometry.
    /// See that method for example usage.
    pub fn project_velocity(v: Vector, normals: &[Dir]) -> Vector {
        // Case 1: Check if v is inside the cone
        if normals
            .iter()
            .all(|n| v.dot(n.adjust_precision()) >= -DOT_EPSILON)
        {
            return v;
        }

        // Best candidate so far
        let mut best_projection = Vector::ZERO;
        let mut best_distance_sq = Scalar::INFINITY;

        // Helper to test halfspace validity
        let is_valid = |projection: Vector| {
            normals
                .iter()
                .all(|n| projection.dot(n.adjust_precision()) >= -DOT_EPSILON)
        };

        // Case 2a: Face projections (single-plane active set)
        for n in normals {
            let n = n.adjust_precision();
            let v_dot_n = v.dot(n);
            if v_dot_n < 0.0 {
                // Project v onto the plane defined by n:
                // projection = v - (v·n) n
                let projection = v - v_dot_n * n;

                // Check if better than previous best and valid
                let distance_sq = v.distance_squared(projection);
                if distance_sq < best_distance_sq && is_valid(projection) {
                    best_distance_sq = distance_sq;
                    best_projection = projection;
                }
            }
        }

        // Case 2b: Edge projections (two-plane active set)
        // TODO: Can we optimize this from O(n^3) to O(n^2)?
        #[cfg(feature = "3d")]
        {
            let n = normals.len();
            for i in 0..n {
                let ni = normals[i].adjust_precision();
                for nj in normals
                    .iter()
                    .take(n)
                    .skip(i + 1)
                    .map(|n| n.adjust_precision())
                {
                    // Compute edge direction e = ni x nj
                    let e = ni.cross(nj);
                    let e_length_sq = e.length_squared();
                    if e_length_sq < DOT_EPSILON {
                        // Nearly parallel edge
                        continue;
                    }

                    // Project v onto the line spanned by e:
                    // projection = ((v·e) / |e|²) e
                    let projection = e * (v.dot(e) / e_length_sq);

                    // Check if better than previous best and valid
                    let distance_sq = v.distance_squared(projection);
                    if distance_sq < best_distance_sq && is_valid(projection) {
                        best_distance_sq = distance_sq;
                        best_projection = projection;
                    }
                }
            }
        }

        // Case 3: If no candidate is found, the projection is at the apex (the origin)
        if best_distance_sq.is_infinite() {
            Vector::ZERO
        } else {
            best_projection
        }
    }
}

/// Needed to not accidentally explode when `n.dot(dir)` happens to be very close to zero.
const DOT_EPSILON: Scalar = 0.005;

/// Data related to a hit during a [`MoveAndSlide::move_and_slide`].
#[derive(Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct MoveAndSlideHitData {
    /// The entity of the collider that was hit by the shape.
    pub entity: Entity,

    /// The maximum distance that is safe to move in the given direction so that the collider still keeps a distance of `skin_width` to the other colliders.
    /// Is 0.0 when
    /// - The collider started off intersecting another collider.
    /// - The collider is moving toward another collider that is already closer than `skin_width`.
    ///
    /// If you want to know the real distance to the next collision, use [`Self::collision_distance`].
    pub distance: Scalar,

    /// The hit point point on the shape that was hit, expressed in world space.
    pub point: Vector,

    /// The outward surface normal on the hit shape at `point`, expressed in world space.
    pub normal: Dir,

    /// Whether the current move and slide iteration started off with the collider intersecting another collider.
    pub intersects: bool,

    /// The position of the collider at the point of the move and slide iteration.
    pub position: Vector,

    /// The velocity of the collider at the point of the move and slide iteration.
    pub velocity: Vector,

    /// The raw distance to the next collision, not respecting skin width.
    /// To move the shape, use [`Self::distance`] instead.
    #[doc(alias = "time_of_impact")]
    pub collision_distance: Scalar,
}

impl MoveAndSlideHitData {
    /// Whether the collider started off already intersecting another collider when it was cast.
    /// Note that this will be `false` if the collider was closer than `skin_width`, but not physically intersecting.
    pub fn intersects(&self) -> bool {
        self.collision_distance == 0.0
    }
}

/// Data related to a hit during a [`MoveAndSlide::cast_move`].
#[derive(Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct MoveHitData {
    /// The entity of the collider that was hit by the shape.
    pub entity: Entity,

    /// The maximum distance that is safe to move in the given direction so that the collider still keeps a distance of `skin_width` to the other colliders.
    /// Is 0.0 when
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
    /// Note that this will be `false` if the collider was closer than `skin_width`, but not physically intersecting.
    pub fn intersects(self) -> bool {
        self.collision_distance == 0.0
    }
}

/// Configuration for a [`MoveAndSlide::move_and_slide`].
#[derive(Clone, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct MoveAndSlideConfig {
    /// How many iterations to use when moving the character. A single iteration consists of
    /// - Performing depenetration
    /// - Moving the character as far as possible in the desired velocity
    /// - Modifying the velocity to slide along any colliding planes
    pub move_and_slide_iterations: usize,

    /// How many iterations to use when performing depenetration.
    /// Depenetration is an iterative process that solves penetrations for all planes bit-by-bit,
    /// until we either reached [`MoveAndSlideConfig::move_and_slide_iterations`] or the accumulated error is less than [`MoveAndSlideConfig::max_depenetration_error`].
    pub depenetration_iterations: usize,

    /// The target error to achieve when performing depenetration.
    /// Depenetration is an iterative process that solves penetrations for all planes bit-by-bit,
    /// until we either reached [`MoveAndSlideConfig::move_and_slide_iterations`] or the accumulated error is less than [`MoveAndSlideConfig::max_depenetration_error`].
    pub max_depenetration_error: Scalar,

    /// A minimal distance to always keep between the collider and any other colliders.
    /// This is here to ensure that the collider never intersects anything, even when numeric errors accumulate.
    /// Set this to a very small value.
    ///
    /// Increase the value if you notice your character getting stuck in geometry.
    /// Decrease it when you notice jittering, especially around V-shaped walls.
    pub skin_width: Scalar,

    /// The initial planes to consider for a move-and-slide operation. This will be expanded during the algorithm with
    /// the colliding planes, but you can also initialize it with some planes you want to make sure the algorithm will never move against.
    ///
    /// A good use-case for this is adding the ground plane when a character controller is standing or walking on the ground.
    pub planes: Vec<Dir>,

    /// The maximum number of planes to solve while performing move-and-slide. If the collided planes exceed this number, the move is aborted and the velocity is set to zero.
    /// Realistically, this will probably never be reached, unless you have very exotic geometry and very high velocity.
    pub max_planes: usize,
}

/// Configuration for a [`MoveAndSlide::depenetrate`].
#[derive(Clone, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct DepenetrationConfig {
    /// How many iterations to use when performing depenetration.
    /// Depenetration is an iterative process that solves penetrations for all planes bit-by-bit,
    /// until we either reached [`MoveAndSlideConfig::move_and_slide_iterations`] or the accumulated error is less than [`MoveAndSlideConfig::max_depenetration_error`].
    pub depenetration_iterations: usize,

    /// The target error to achieve when performing depenetration.
    /// Depenetration is an iterative process that solves penetrations for all planes bit-by-bit,
    /// until we either reached [`MoveAndSlideConfig::move_and_slide_iterations`] or the accumulated error is less than [`MoveAndSlideConfig::max_depenetration_error`].
    pub max_depenetration_error: Scalar,

    /// A minimal distance to always keep between the collider and any other colliders.
    /// This is here to ensure that the collider never intersects anything, even when numeric errors accumulate.
    /// Set this to a very small value.
    ///
    /// Increase the value if you notice your character getting stuck in geometry.
    /// Decrease it when you notice jittering, especially around V-shaped walls.
    pub skin_width: Scalar,
}

impl Default for DepenetrationConfig {
    fn default() -> Self {
        Self {
            depenetration_iterations: 16,
            max_depenetration_error: {
                #[cfg(feature = "3d")]
                {
                    0.0001
                }
                #[cfg(feature = "2d")]
                {
                    0.01
                }
            },
            skin_width: {
                #[cfg(feature = "3d")]
                {
                    0.002
                }
                #[cfg(feature = "2d")]
                {
                    0.2
                }
            },
        }
    }
}

impl From<&MoveAndSlideConfig> for DepenetrationConfig {
    fn from(config: &MoveAndSlideConfig) -> Self {
        Self {
            depenetration_iterations: config.depenetration_iterations,
            max_depenetration_error: config.max_depenetration_error,
            skin_width: config.skin_width,
        }
    }
}

/// Output from a [`MoveAndSlide::move_and_slide`].
#[derive(Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct MoveAndSlideOutput {
    /// The final position of the character after move and slide. Set your [`Transform::translation`] to this value.
    pub position: Vector,

    /// The final velocity of the character after move and slide. This corresponds to the actual velocity, not the wished velocity.
    /// For example, if the character is trying to move to the right and there's a ramp on its path, this vector will point up the ramp.
    /// It is useful to store this value and apply your wish movement vectors, friction, gravity, etc. on it before handing it to [`MoveAndSlide::move_and_slide`] as the input `velocity`.
    /// You can also ignore this value if you don't wish to preserve momentum.
    ///
    /// Do *not* set [`LinearVelocity`] to this value, as that would apply the movement twice and cause intersections. Instead, set [`Transform::translation`] to [`MoveAndSlideOutput::position`].
    pub clipped_velocity: Vector,
}

impl Default for MoveAndSlideConfig {
    fn default() -> Self {
        let default_depen_cfg = DepenetrationConfig::default();
        Self {
            move_and_slide_iterations: 4,
            depenetration_iterations: default_depen_cfg.depenetration_iterations,
            max_depenetration_error: default_depen_cfg.max_depenetration_error,
            planes: Vec::new(),
            max_planes: 20,
            skin_width: default_depen_cfg.skin_width,
        }
    }
}
