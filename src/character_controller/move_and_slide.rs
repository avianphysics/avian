//! Contains [`MoveAndSlide`] and related types. See that struct for more information.

use crate::{collision::collider::contact_query::contact_manifolds, prelude::*};
use bevy::{ecs::system::SystemParam, prelude::*};

/// A [`SystemParam`] for performing move and slide operations via the [`MoveAndSlide::move_and_slide`] method.
/// "Move and slide", a.k.a. "collide and slide" or "step slide", is the algorithm at the heart of kinematic character controllers.
/// This algorithm basically says
///- Please move in this direction
///- if you collide with anything, slide along it
///- make sure you're not intersecting with anything, and report everything you collide with
///
/// See the video [Collide and slide - Collision detection algorithm](https://www.youtube.com/watch?v=YR6Q7dUz2uk) for an in-depth explanation.
///
/// Also contains various helper methods that are useful for building kinematic character controllers.
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
    /// Performs the move and slide operation.
    /// This algorithm basically says
    ///- Please move in this direction
    ///- if you collide with anything, slide along it
    ///- make sure you're not intersecting with anything, and report everything you collide with
    ///
    /// See the video [Collide and slide - Collision detection algorithm](https://www.youtube.com/watch?v=YR6Q7dUz2uk) for an in-depth explanation.
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
    /// - `on_hit`: A callback that is called when a collider is hit as part of the move and slide iterations. Returning `false` will abort the move and slide operation.\
    ///   Starting intersections, i.e. those that happen when the collider is already stuck in another collider, will be reported to this callback, but aborted even if the callback returns `true`.
    ///   If you don't have any special handling per collision, you can pass `|_| true`.
    ///
    /// # Example
    ///
    /// ```rust
    /// use bevy::prelude::*;
    #[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
    #[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
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
    ///     let mut collisions = EntityHashSet::new();
    ///     let out = move_and_slide.move_and_slide(
    ///         collider,
    ///         transform.translation,
    ///         transform.rotation,
    ///         velocity,
    ///         &MoveAndSlideConfig::default(),
    ///         &filter,
    ///         |hit| {
    ///             collisions.insert(hit.entity);
    ///             true
    ///         },
    ///     );
    ///     transform.translation = out.position;
    ///     controller.velocity = out.velocity;
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
        // - for each iteration:
        //   - Gauss-Seidel out of any penetration
        //   - Shape cast to first hit
        //   - Pull back the distance so were are `skin_width` away from the plane
        //   - push collided plane
        //   - change velocity according to colliding planes
        //     - 2 planes: slide along in 3d, abort in 2d
        //     - 3 planes: abort
        // - perform final depenetration
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
        let mut planes = config.planes.clone();

        'outer: for _ in 0..config.move_and_slide_iterations {
            let sweep = time_left * velocity;
            let Some((vel_dir, distance)) = Dir::new_and_length(sweep.f32()).ok() else {
                // no more movement to go
                break;
            };
            let distance = distance.adjust_precision();
            const MIN_DISTANCE: Scalar = 0.0001;
            if distance < MIN_DISTANCE {
                break;
            }

            let depenetration_offset =
                self.depenetrate(shape, position, shape_rotation, &config.into(), filter);
            position += depenetration_offset;

            let hit = self.cast_move(
                shape,
                position,
                shape_rotation,
                sweep,
                config.skin_width,
                filter,
            );
            let Some(sweep_hit) = hit else {
                // moved the entire distance
                position += sweep;
                break;
            };
            if !on_hit(MoveAndSlideHitData {
                entity: sweep_hit.entity,
                point1: sweep_hit.point1,
                point2: sweep_hit.point2,
                normal1: sweep_hit.normal1,
                normal2: sweep_hit.normal2,
                collision_distance: sweep_hit.collision_distance,
                distance: sweep_hit.distance,
                position,
                velocity,
            }) {
                velocity = Vector::ZERO;
                break 'outer;
            }
            if sweep_hit.intersects() {
                // entity is completely trapped in another solid
                velocity = Vector::ZERO;
                break 'outer;
            }
            time_left -= time_left * (sweep_hit.distance / distance);

            position += vel_dir.adjust_precision() * sweep_hit.distance;

            // if this is the same plane we hit before, nudge velocity
            // out along it, which fixes some epsilon issues with
            // non-axial planes
            for plane in planes.iter() {
                if sweep_hit.normal1.dot(plane.adjust_precision()) > (1.0 - DOT_EPSILON) {
                    velocity += sweep_hit.normal1 * config.duplicate_plane_nudge;
                    continue 'outer;
                }
            }
            if planes.len() >= config.max_planes {
                velocity = Vector::ZERO;
                break 'outer;
            }
            planes.push(Dir::new_unchecked(sweep_hit.normal1.f32()));

            // modify velocity so it parallels all of the clip planes

            // find a plane that it enters
            for i in 0..planes.len() {
                let into = velocity.dot(planes[i].adjust_precision());
                if into >= 0.0 {
                    // move doesn't interact with the plane
                    continue;
                }

                // slide along the plane
                #[cfg_attr(feature = "2d", expect(unused_mut, reason = "only used in 3D branch"))]
                let mut current_clip_velocity = Self::clip_velocity(velocity, &[planes[i]]);

                // see if there is a second plane that the new move enters
                #[cfg_attr(
                    feature = "2d",
                    expect(
                        clippy::needless_range_loop,
                        reason = "This variant is cleaner in this case"
                    )
                )]
                for j in 0..planes.len() {
                    if j == i {
                        continue;
                    }
                    if current_clip_velocity.dot(planes[j].adjust_precision()) >= DOT_EPSILON {
                        // move doesn't interact with the plane
                        continue;
                    }
                    #[cfg(feature = "2d")]
                    {
                        // stop dead at a double plane interaction
                        velocity = Vector::ZERO;
                        break 'outer;
                    }
                    #[cfg(feature = "3d")]
                    {
                        // try clipping the move to the plane
                        current_clip_velocity =
                            Self::clip_velocity(current_clip_velocity, &[planes[j]]);

                        // see if it goes back into the first clip plane
                        if current_clip_velocity.dot(planes[i].adjust_precision()) >= DOT_EPSILON {
                            continue;
                        }

                        // slide the original velocity along the crease
                        let dir = planes[i]
                            .adjust_precision()
                            .cross(planes[j].adjust_precision())
                            // no need to check for zero length:
                            // we would have early-returned if planes[i] and planes[j] were equivalent,
                            // and we expect them both to be non-zero
                            .normalize();
                        let d = dir.dot(velocity);
                        current_clip_velocity = dir * d;

                        // see if there is a third plane the the new move enters
                        #[expect(
                            clippy::needless_range_loop,
                            reason = "This variant is cleaner in this case"
                        )]
                        for k in 0..planes.len() {
                            if k == i || k == j {
                                continue;
                            }

                            if current_clip_velocity.dot(planes[k].adjust_precision())
                                >= DOT_EPSILON
                            {
                                // move doesn't interact with the plane
                                continue;
                            }

                            // stop dead at a triple plane interaction
                            velocity = Vector::ZERO;
                            break 'outer;
                        }
                    }
                }
                // if we have fixed all interactions, try another move
                velocity = current_clip_velocity;
                break;
            }

            // if original velocity is against the original velocity, stop dead
            // to avoid tiny occilations in sloping corners
            if velocity.dot(original_velocity) <= -DOT_EPSILON {
                velocity = Vector::ZERO;
                break 'outer;
            }
        }

        let depenetration_offset =
            self.depenetrate(shape, position, shape_rotation, &config.into(), filter);
        position += depenetration_offset;

        MoveAndSlideOutput {
            position,
            clipped_velocity: velocity,
        }
    }

    /// Clips `velocity` so that it does not point into any of the given `planes` as expressed by their normals.
    /// If there are no planes, the velocity is returned unchanged.
    ///
    /// Often used after [`MoveAndSlide::cast_move`] to ensure a character moved that way does not try to continue moving into colliding geometry.
    /// See that method for example usage.
    #[must_use]
    pub fn clip_velocity(mut velocity: Vector, planes: &[Dir]) -> Vector {
        for normal in planes {
            velocity -=
                velocity.dot((*normal).adjust_precision()).min(0.0) * normal.adjust_precision();
        }
        velocity
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
    /// - `velocity`: The direction and magnitude of the movement. If this is [`Vector::ZERO`], this method can still return `Some(MoveHitData)` if the shape started off intersecting a collider.
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
    #[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
    #[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
    ///
    /// #[derive(Component)]
    /// struct CharacterController {
    ///     velocity: Vector,
    /// }
    ///
    /// fn perform_cast_move(
    ///     player: Single<(Entity, &Collider, &mut CharacterController, &mut Transform)>,
    ///     move_and_slide: MoveAndSlide,
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
    ///         transform.translation,
    ///         transform.rotation,
    ///         velocity,
    ///         MoveAndSlideConfig::default().skin_width,
    ///         &filter,
    ///     );
    ///     if let Some(hit) = hit {
    ///         // We collided with something on the way. Advance as much as possible
    ///         transform.translation += velocity.normalize_or_zero() * hit.distance;
    ///         // Then clip the velocity to make sure it no longer points towards the collision plane
    ///         controller.velocity =
    ///             MoveAndSlide::clip_velocity(velocity, &[Dir::new_unchecked(hit.normal1)])
    ///     } else {
    ///         // We traveled the full distance without colliding
    ///         transform.translation += velocity;
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
        velocity: Vector,
        skin_width: Scalar,
        filter: &SpatialQueryFilter,
    ) -> Option<MoveHitData> {
        let (direction, distance) = Dir::new_and_length(velocity.f32()).unwrap_or((Dir::X, 0.0));
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

    /// An [intersection test](spatial_query#intersection-tests) that finds all entities with a [`Collider`]
    /// that is closer to the given `shape` with a given position and rotation than `skin_width`.
    ///
    /// # Arguments
    ///
    /// - `shape`: The shape that intersections are tested against represented as a [`Collider`].
    /// - `shape_position`: The position of the shape.
    /// - `shape_rotation`: The rotation of the shape.
    /// - `filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query.
    /// - `skin_width`: An extra margin applied to the [`Collider`].
    ///
    /// # Example
    ///
    /// TODO: check if the character is allowed to stand up
    #[must_use]
    pub fn intersections(
        &self,
        shape: &Collider,
        shape_position: Vector,
        shape_rotation: RotationValue,
        skin_width: Scalar,
        filter: &SpatialQueryFilter,
    ) -> Vec<(Dir, Scalar)> {
        let mut intersections = Vec::new();
        let expanded_aabb = shape
            .aabb(shape_position, shape_rotation)
            .grow(Vector::splat(skin_width));
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
                skin_width,
                &mut manifolds,
            );
            for manifold in manifolds {
                let Some(deepest) = manifold.find_deepest_contact() else {
                    continue;
                };

                // penetration is positive if penetrating, negative if separated
                let dist = deepest.penetration + skin_width;
                let normal = Dir::new_unchecked(manifold.normal.f32());
                intersections.push((-normal, dist));
            }
        }
        intersections
    }

    /// Moves a collider so that it no longer intersects any other collider and keeps a minimum distance of [`MoveAndSlideConfig::skin_width`] to all.
    /// Depenetration is an iterative process that solves penetrations for all planes bit-by-bit,
    /// until we either reached [`MoveAndSlideConfig::move_and_slide_iterations`] or the accumulated error is less than [`MoveAndSlideConfig::max_depenetration_error`].
    /// If the max iterations were reached before the error was below the threshold, the current best attempt is returned, in which case the collider may still be intersecting with other colliders.
    ///
    /// # Arguments
    /// - `shape`: The shape being cast represented as a [`Collider`].
    /// - `shape_position`: Where the shape that needs to be moved is located.
    /// - `shape_rotation`: The rotation of the shape to be moved.
    /// - `config`: A [`DepenetrationConfig`] that determines the behavior of the depenetration. [`DepenetrationConfig::default()`] should be a good start for most cases.
    /// - `filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query. It is highly recommended to exclude the entity holding the collider itself,
    ///   otherwise the character will collide with itself.
    ///
    /// # Returns
    ///
    /// A displacement vector that can be added to the `shape_position` to resolve the intersections, or the best attempt if the max iterations were reached before a solution was found.
    ///
    /// # Example
    ///
    /// TODO: use
    ///
    /// See also [`MoveAndSlide::cast_move`] for a typical usage scenario.
    #[must_use]
    pub fn depenetrate(
        &self,
        shape: &Collider,
        shape_position: Vector,
        shape_rotation: RotationValue,
        config: &DepenetrationConfig,
        filter: &SpatialQueryFilter,
    ) -> Vector {
        let intersections = self.intersections(
            shape,
            shape_position,
            shape_rotation,
            config.skin_width,
            filter,
        );
        if intersections.is_empty() {
            return Vector::ZERO;
        }

        let mut fixup = Vector::ZERO;
        for _ in 0..config.depenetration_iterations {
            let mut total_error = 0.0;
            for (normal, dist) in &intersections {
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
}

/// Needed to not accidentally explode when `n.dot(dir)` happens to be very close to zero
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
    /// If you want to know the real distance to the next collision, use [`Self::raw_collision_distance`].
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
    /// If you want to know the real distance to the next collision, use [`Self::raw_collision_distance`].
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

    /// A small nudge to apply to the character when colliding with a plane for a second time. This is done to not get caught in a loop of repeated collisions.
    /// Reduce this value if you notice jittering.
    pub duplicate_plane_nudge: Scalar,
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
            max_planes: 5,
            duplicate_plane_nudge: {
                #[cfg(feature = "3d")]
                {
                    0.001
                }
                #[cfg(feature = "2d")]
                {
                    0.1
                }
            },
            skin_width: default_depen_cfg.skin_width,
        }
    }
}
