use std::f32;

use crate::{collision::collider::contact_query::contact, prelude::*};
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
    /// - push initial velocity as a plane, as if we were standing with our back to a wall
    ///   - This ensures we never slide back when trying to move forward
    /// - for each iteration:
    ///   - Shape cast to first hit
    ///   - Pull back the distance so were are skin_width away from the plane
    ///   - Gauss-Seidel out of any penetration
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
    ) -> CollideAndSlideResult {
        let mut position = origin;
        let mut time_left = self.time.delta_secs();
        let mut planes = config.planes.clone();

        for _ in 0..config.collide_and_slide_iterations {
            let sweep = time_left * velocity;
            let Some((vel_dir, speed)) = Dir::new_and_length(sweep).ok() else {
                // no more movement to go
                break;
            };
            if speed < EPSILON {
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
            let safe_dist = Self::pull_back(hit, vel_dir, config.skin_width);
            time_left -= time_left * safe_dist / speed;
            position += vel_dir * safe_dist;
            if planes.len() >= config.max_planes {
                return CollideAndSlideResult {
                    position,
                    velocity: Vector::ZERO,
                };
            }
            if hit.distance == 0.0 {
                // entity is completely trapped in another solid
                // TODO: call on_penetration callback here and let it decide whether to abort
                return CollideAndSlideResult {
                    position,
                    velocity: Vector::ZERO,
                };
            }
            // TODO: call on_hit callback here

            // if this is the same plane we hit before, nudge velocity
            // out along it, which fixes some epsilon issues with
            // non-axial planes
            let mut i = 0;
            while i < planes.len() {
                if hit.normal1.dot(planes[i].into()) > (1.0 - EPSILON) {
                    velocity += hit.normal1 * config.duplicate_plane_nudge;
                    break;
                }
                i += 1;
            }
            if i < planes.len() {
                continue;
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
                let mut current_clip_velocity = Self::clip_velocity(velocity, planes[i].into());

                // see if there is a second plane that the new move enters
                for j in 0..planes.len() {
                    if j == i {
                        continue;
                    }
                    if current_clip_velocity.dot(planes[j].into()) >= 0.0 {
                        // move doesn't interact with the plane
                        continue;
                    }
                    #[cfg(feature = "2d")]
                    {
                        // stop dead at a double plane interaction
                        return CollideAndSlideResult {
                            position,
                            velocity: Vector::ZERO,
                        };
                    }
                    #[cfg(feature = "3d")]
                    {
                        // try clipping the move to the plane
                        current_clip_velocity =
                            Self::clip_velocity(current_clip_velocity, planes[j]);

                        // see if it goes back into the first clip plane
                        if current_clip_velocity.dot(planes[i].into()) >= 0.0 {
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

                            if current_clip_velocity.dot(planes[k].into()) >= 0.0 {
                                // move doesn't interact with the plane
                                continue;
                            }

                            // stop dead at a triple plane interaction
                            return CollideAndSlideResult {
                                position,
                                velocity: Vector::ZERO,
                            };
                        }
                    }
                }
                // if we have fixed all interactions, try another move
                velocity = current_clip_velocity;
                break;
            }
        }

        let depenetration_offset =
            self.depenetrate(shape, shape_rotation, position, filter, config);
        position += depenetration_offset;

        CollideAndSlideResult { position, velocity }
    }

    #[must_use]
    // TODO: replace by Box2D's method, which accounts for multiple planes:
    // <https://github.com/erincatto/box2d/blob/3a4f0da8374af61293a03021c9a0b3ebcfe67948/src/mover.c#L57>
    // See also <https://blog.littlepolygon.com/posts/sliding/>
    pub fn clip_velocity(velocity: Vector, normal: Dir) -> Vector {
        const OVERCLIP: Scalar = 1.001;
        let backoff = velocity.dot(normal.into());
        let backoff = if backoff < 0.0 {
            backoff * OVERCLIP
        } else {
            backoff / OVERCLIP
        };
        velocity - normal * backoff
    }

    #[must_use]
    fn pull_back(hit: ShapeHitData, dir: Dir, skin_width: Scalar) -> Scalar {
        let dot = dir.dot(-hit.normal1);
        if dot.abs() > EPSILON {
            let skin_distance = skin_width / dot;
            hit.distance - skin_distance
        } else {
            hit.distance
        }
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
            let Ok(Some(contact)) = contact(
                shape,
                origin,
                shape_rotation,
                intersection_collider,
                *intersection_pos,
                *intersection_rot,
                config.skin_width,
            ) else {
                continue;
            };
            // penetration is positive is penetrating, negative if separated
            let dist = dbg!(contact.penetration) + config.skin_width;
            intersections.push((contact.global_normal1(intersection_rot), dist));
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
const EPSILON: Scalar = 0.005;

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
    pub velocity: Vector,
}

impl Default for CollideAndSlideConfig {
    fn default() -> Self {
        Self {
            collide_and_slide_iterations: 4,
            depenetration_iterations: 16,
            max_depenetration_error: 0.0001,
            planes: Vec::new(),
            max_planes: 5,
            duplicate_plane_nudge: 0.05,
            skin_width: 0.01,
        }
    }
}
