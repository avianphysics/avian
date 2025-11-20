//! Contains [`MoveAndSlide`] and related types. See that struct for more information.

use crate::{collision::collider::contact_query::contact_manifolds, prelude::*};
use bevy::{ecs::system::SystemParam, prelude::*};

/// A [`SystemParam`] for performing move and slide operations via the [`MoveAndSlide::move_and_slide`] method.
/// "Move and slide", a.k.a. "collide and slide" or "step slide", is the algorithm at the heart of kinematic character controllers.
/// In a nutshell, what it does is
/// This algorithm basically says
///- Please move in this direction
///- if you collide with anything, slide along it
///- make sure you're not intersecting with anything, and report everything you collide with
///
/// see the video [Collide and slide - Collision detection algorithm](https://www.youtube.com/watch?v=YR6Q7dUz2uk) for an in-depth explanation.
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
    #[must_use]
    pub fn move_and_slide(
        &self,
        shape: &Collider,
        shape_rotation: RotationValue,
        origin: Vector,
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
        let mut position = origin;
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
                self.depenetrate(shape, shape_rotation, position, filter, config);
            position += depenetration_offset;

            let hit = self.sweep(
                shape,
                position,
                shape_rotation,
                vel_dir,
                distance,
                config.skin_width,
                filter,
            );
            let Some(sweep_hit) = hit else {
                // moved the entire distance
                position += sweep;
                break;
            };
            if !on_hit(MoveAndSlideHitData {
                intersects: sweep_hit.intersects,
                entity: sweep_hit.entity,
                point1: sweep_hit.point1,
                point2: sweep_hit.point2,
                normal1: sweep_hit.normal1,
                normal2: sweep_hit.normal2,
                distance: sweep_hit.distance,
                safe_distance: sweep_hit.safe_distance,
                position,
                velocity,
            }) {
                velocity = Vector::ZERO;
                break 'outer;
            }
            if sweep_hit.intersects {
                // entity is completely trapped in another solid
                velocity = Vector::ZERO;
                break 'outer;
            }
            time_left -= time_left * (sweep_hit.safe_distance / distance);

            position += vel_dir.adjust_precision() * sweep_hit.safe_distance;

            // if this is the same plane we hit before, nudge velocity
            // out along it, which fixes some epsilon issues with
            // non-axial planes
            for plane in planes.iter().copied() {
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
                            .cross(planes[j].adjust_precision());
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
            self.depenetrate(shape, shape_rotation, position, filter, config);
        position += depenetration_offset;

        MoveAndSlideOutput {
            position,
            clipped_velocity: velocity,
        }
    }

    #[must_use]
    pub fn clip_velocity(mut velocity: Vector, planes: &[Dir]) -> Vector {
        for normal in planes {
            velocity -=
                velocity.dot((*normal).adjust_precision()).min(0.0) * normal.adjust_precision();
        }
        velocity
    }

    #[must_use]
    pub fn sweep(
        &self,
        shape: &Collider,
        origin: Vector,
        shape_rotation: RotationValue,
        direction: Dir,
        distance: Scalar,
        skin_width: Scalar,
        filter: &SpatialQueryFilter,
    ) -> Option<SweepHitData> {
        let shape_hit = self.query_pipeline.cast_shape(
            shape,
            origin,
            shape_rotation,
            direction,
            &ShapeCastConfig::from_max_distance(distance),
            filter,
        )?;
        let safe_distance = Self::pull_back(shape_hit, direction, skin_width);
        Some(SweepHitData {
            safe_distance,
            distance,
            intersects: shape_hit.distance == 0.0,
            entity: shape_hit.entity,
            point1: shape_hit.point1,
            point2: shape_hit.point2,
            normal1: shape_hit.normal1,
            normal2: shape_hit.normal2,
        })
    }

    #[must_use]
    fn pull_back(hit: ShapeHitData, dir: Dir, skin_width: Scalar) -> Scalar {
        let dot = dir.adjust_precision().dot(-hit.normal1).max(DOT_EPSILON);
        let skin_distance = skin_width / dot;
        (hit.distance - skin_distance).max(0.0)
    }

    #[must_use]
    pub fn intersections(
        &self,
        shape: &Collider,
        shape_rotation: RotationValue,
        origin: Vector,
        filter: &SpatialQueryFilter,
        skin_width: Scalar,
    ) -> Vec<(Dir, Scalar)> {
        let mut intersections = Vec::new();
        let expanded_aabb = shape
            .aabb(origin, shape_rotation)
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
                origin,
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

    #[must_use]
    pub fn depenetrate(
        &self,
        shape: &Collider,
        shape_rotation: RotationValue,
        origin: Vector,
        filter: &SpatialQueryFilter,
        config: &MoveAndSlideConfig,
    ) -> Vector {
        let intersections =
            self.intersections(shape, shape_rotation, origin, filter, config.skin_width);
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

// needed to not accidentally explode when `n.dot(dir)` happens to be very close to zero
const DOT_EPSILON: Scalar = 0.005;

#[derive(Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct MoveAndSlideHitData {
    /// The entity of the collider that was hit by the shape.
    pub entity: Entity,

    /// How far the shape travelled before the initial hit.
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
    pub safe_distance: Scalar,
    pub intersects: bool,
    pub position: Vector,
    pub velocity: Vector,
}

#[derive(Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct SweepHitData {
    /// The entity of the collider that was hit by the shape.
    pub entity: Entity,

    /// How far the shape travelled before the initial hit.
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
    pub safe_distance: Scalar,
    pub intersects: bool,
}

#[derive(Clone, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct MoveAndSlideConfig {
    pub move_and_slide_iterations: usize,
    pub depenetration_iterations: usize,
    pub max_depenetration_error: Scalar,
    pub skin_width: Scalar,
    pub planes: Vec<Dir>,
    pub max_planes: usize,
    pub duplicate_plane_nudge: Scalar,
}

#[derive(Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct MoveAndSlideOutput {
    pub position: Vector,
    pub clipped_velocity: Vector,
}

impl Default for MoveAndSlideConfig {
    fn default() -> Self {
        Self {
            move_and_slide_iterations: 4,
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
