use crate::{collision::collider::contact_query::contact_manifolds, prelude::*};
use bevy::{ecs::system::SystemParam, prelude::*};

#[derive(SystemParam)]
pub struct CollideAndSlide<'w, 's> {
    query_pipeline: Res<'w, SpatialQueryPipeline>,
    colliders: Query<
        'w,
        's,
        (
            &'static Collider,
            &'static Position,
            &'static Rotation,
            Option<&'static CollisionLayers>,
        ),
    >,
    time: Res<'w, Time>,
}

impl<'w, 's> CollideAndSlide<'w, 's> {
    /// High level overview:
    /// - for each iteration:
    ///   - Gauss-Seidel out of any penetration
    ///   - Shape cast to first hit
    ///   - Pull back the distance so were are skin_width away from the plane
    ///   - push collided plane
    ///   - change velocity according to colliding planes
    ///     - 2 planes: slide along in 3d, abort in 2d
    ///     - 3 planes: abort
    /// - perform final depenetration
    pub fn collide_and_slide(
        &self,
        shape: &Collider,
        shape_rotation: RotationValue,
        origin: Vector,
        mut velocity: Vector,
        config: &CollideAndSlideConfig,
        filter: &SpatialQueryFilter,
        mut on_hit: impl FnMut(CollideAndSlideHitData) -> bool,
    ) -> CollideAndSlideResult {
        let mut position = origin;
        let original_velocity = velocity;
        let mut time_left = self.time.delta_secs();
        let mut planes = config.planes.clone();

        'outer: for _ in 0..config.collide_and_slide_iterations {
            let sweep = time_left * velocity;
            let Some((vel_dir, speed)) = Dir::new_and_length(sweep).ok() else {
                // no more movement to go
                break;
            };
            const MIN_SPEED: f32 = 0.0001;
            if speed < MIN_SPEED {
                break;
            }

            let depenetration_offset =
                self.depenetrate(shape, shape_rotation, position, filter, config);
            position += depenetration_offset;

            let hit = self.query_pipeline.cast_shape(
                shape,
                position,
                shape_rotation,
                vel_dir,
                &ShapeCastConfig::from_max_distance(speed),
                filter,
            );
            let Some(hit) = hit else {
                // moved the entire distance
                position += sweep;
                break;
            };
            let safe_distance = Self::pull_back(hit, vel_dir, config.skin_width);
            if !on_hit(CollideAndSlideHitData {
                hit,
                position,
                velocity,
                safe_distance,
            }) {
                velocity = Vector::ZERO;
                break 'outer;
            }
            if hit.distance == 0.0 {
                // entity is completely trapped in another solid
                velocity = Vector::ZERO;
                break 'outer;
            }
            time_left -= time_left * (safe_distance / speed);

            position += vel_dir * safe_distance;

            // if this is the same plane we hit before, nudge velocity
            // out along it, which fixes some epsilon issues with
            // non-axial planes
            let mut i = 0;
            while i < planes.len() {
                if hit.normal1.dot(planes[i].into()) > (1.0 - DOT_EPSILON) {
                    velocity += hit.normal1 * config.duplicate_plane_nudge;
                    break;
                }
                i += 1;
            }
            if i < planes.len() {
                continue;
            }
            if planes.len() >= config.max_planes {
                velocity = Vector::ZERO;
                break 'outer;
            }
            planes.push(Dir::new_unchecked(hit.normal1));

            // modify velocity so it parallels all of the clip planes

            // find a plane that it enters
            for i in 0..planes.len() {
                let into = velocity.dot(planes[i].into());
                if into >= 0.0 {
                    // move doesn't interact with the plane
                    continue;
                }

                // slide along the plane
                #[cfg_attr(feature = "2d", expect(unused_mut, reason = "only used in 3D branch"))]
                let mut current_clip_velocity = Self::clip_velocity(velocity, &[planes[i]]);

                // see if there is a second plane that the new move enters
                for j in 0..planes.len() {
                    if j == i {
                        continue;
                    }
                    if current_clip_velocity.dot(planes[j].into()) >= DOT_EPSILON {
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
                        if current_clip_velocity.dot(planes[i].into()) >= DOT_EPSILON {
                            continue;
                        }

                        // slide the original velocity along the crease
                        let dir = planes[i].cross(planes[j].into());
                        let d = dir.dot(velocity);
                        current_clip_velocity = dir * d;

                        // see if there is a third plane the the new move enters
                        for k in 0..planes.len() {
                            if k == i || k == j {
                                continue;
                            }

                            if current_clip_velocity.dot(planes[k].into()) >= DOT_EPSILON {
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

        CollideAndSlideResult {
            position,
            internal_velocity: velocity,
        }
    }

    #[must_use]
    pub fn clip_velocity(mut velocity: Vector, planes: &[Dir]) -> Vector {
        for normal in planes {
            velocity -= velocity.dot((*normal).into()).min(0.0) * *normal;
        }
        velocity
    }

    #[must_use]
    fn pull_back(hit: ShapeHitData, dir: Dir, skin_width: Scalar) -> Scalar {
        let dot = dir.dot(-hit.normal1);
        if dot > DOT_EPSILON {
            let skin_distance = skin_width / dot;
            hit.distance - skin_distance
        } else {
            hit.distance
        }
        .max(0.0)
    }

    fn depenetrate(
        &self,
        shape: &Collider,
        shape_rotation: RotationValue,
        origin: Vector,
        filter: &SpatialQueryFilter,
        config: &CollideAndSlideConfig,
    ) -> Vector {
        let mut intersections = Vec::new();
        let expanded_aabb = shape
            .aabb(origin, shape_rotation)
            .grow(Vector::splat(config.skin_width));
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
                config.skin_width,
                &mut manifolds,
            );
            for manifold in manifolds {
                let Some(deepest) = manifold.find_deepest_contact() else {
                    continue;
                };

                // penetration is positive if penetrating, negative if separated
                let dist = deepest.penetration + config.skin_width;
                intersections.push((-manifold.normal, dist));
            }
        }
        if intersections.is_empty() {
            return Vector::ZERO;
        }

        let mut fixup = Vector::ZERO;
        for _ in 0..config.depenetration_iterations {
            let mut total_error = 0.0;
            for (normal, dist) in &intersections {
                let error = (dist - fixup.dot(*normal)).max(0.0);
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

pub struct CollideAndSlideHitData {
    pub hit: ShapeHitData,
    pub position: Vector,
    pub velocity: Vector,
    pub safe_distance: Scalar,
}

pub struct CollideAndSlideConfig {
    pub collide_and_slide_iterations: usize,
    pub depenetration_iterations: usize,
    pub max_depenetration_error: Scalar,
    pub skin_width: Scalar,
    pub planes: Vec<Dir>,
    pub max_planes: usize,
    pub duplicate_plane_nudge: Scalar,
}

pub struct CollideAndSlideResult {
    pub position: Vector,
    pub internal_velocity: Vector,
}

impl Default for CollideAndSlideConfig {
    fn default() -> Self {
        Self {
            collide_and_slide_iterations: 4,
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
