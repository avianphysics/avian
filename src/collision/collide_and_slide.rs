use crate::{collision::collider::contact_query::contact, prelude::*};
use bevy::{ecs::system::SystemParam, prelude::*};
use parry::shape::TypedShape;

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

        // never turn against original velocity
        let Ok(vel_dir) = Dir::new(velocity) else {
            return CollideAndSlideResult { position, velocity };
        };
        planes.push(vel_dir);

        for _ in 0..config.iterations {
            // calculate position we are trying to move to
            //
            // see if we can make it there
            let sweep = time_left * velocity;
            let hit = self.sweep_check(
                shape,
                shape_rotation,
                position,
                sweep,
                &config.sweep_check_config,
                filter,
            );
            let Some(hit) = hit else {
                // moved the entire distance
                position += sweep;
                break;
            };
            time_left -= time_left * hit.frac;
            position = hit.safe_pos;
            if planes.len() >= config.max_planes {
                return CollideAndSlideResult {
                    position,
                    velocity: Vector::ZERO,
                };
            }
            if hit.penetrating {
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
                if hit.shape_hit.normal1.dot(planes[i].into()) > 0.99 {
                    velocity += hit.shape_hit.normal1 * config.duplicate_plane_nudge;
                    break;
                }
                i += 1;
            }
            if i < planes.len() {
                continue;
            }
            planes.push(Dir::new_unchecked(hit.shape_hit.normal1));

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

        CollideAndSlideResult { position, velocity }
    }

    #[must_use]
    // TODO: replace by Box2D's method, which accounts for multiple planes:
    // <https://github.com/erincatto/box2d/blob/3a4f0da8374af61293a03021c9a0b3ebcfe67948/src/mover.c#L57>
    // See also <https://blog.littlepolygon.com/posts/sliding/>
    pub fn clip_velocity(velocity: Vector, normal: Dir) -> Vector {
        const OVERCLIP: f32 = 1.001;
        let backoff = velocity.dot(normal.into());
        let backoff = if backoff < 0.0 {
            backoff * OVERCLIP
        } else {
            backoff / OVERCLIP
        };
        velocity - normal * backoff
    }

    fn depenetrate(
        &self,
        shape: &Collider,
        shape_rotation: RotationValue,
        origin: Vector,
        filter: &SpatialQueryFilter,
        skin_width: f32,
    ) -> Vector {
        const ITERATIONS: usize = 16;
        const MAX_ERROR: f32 = 0.001;

        let mut intersections = Vec::new();
        let aabb_intersections = self
            .query_pipeline
            .aabb_intersections_with_aabb(shape.aabb(origin, shape_rotation));
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
                intersection_collider,
                *intersection_pos,
                *intersection_rot,
                shape,
                origin,
                shape_rotation,
                skin_width,
            ) else {
                continue;
            };
            // penetration is positive is penetrating, negative if separated
            let dist = contact.penetration + skin_width;
            intersections.push((contact.global_normal1(intersection_rot), dist));
        }

        let mut fixup = Vector::ZERO;
        for _ in 0..ITERATIONS {
            let mut total_error = 0.0;
            for (normal, dist) in &intersections {
                let error = (dist + EPSILON - fixup.dot(*normal)).max(0.0);
                total_error += error;
                fixup += error * normal;
            }
            if total_error < MAX_ERROR {
                break;
            }
        }
        origin + fixup
    }

    #[must_use]
    fn sweep_check(
        &self,
        shape: &Collider,
        shape_rotation: RotationValue,
        origin: Vector,
        velocity: Vector,
        config: &SweepCheckConfig,
        filter: &SpatialQueryFilter,
    ) -> Option<SweepHitData> {
        let (cast_dir, cast_len) = Dir::new_and_length(velocity).ok()?;
        let hit = self.query_pipeline.cast_shape(
            shape,
            origin,
            shape_rotation,
            cast_dir,
            &ShapeCastConfig::from_max_distance(cast_len),
            filter,
        )?;

        let n = hit.normal1;
        let dir: Vector = cast_dir.into();

        let safe_distance = if n.dot(dir).abs() > EPSILON {
            let skin_distance = config.skin_width / n.dot(-dir);
            hit.distance - skin_distance
        } else {
            hit.distance
        };

        Some(SweepHitData {
            shape_hit: hit,
            safe_distance,
            dir: cast_dir,
            safe_pos: origin + velocity * safe_distance,
            frac: safe_distance / cast_len,
            penetrating: hit.distance == 0.0,
        })
    }
}

// needed to not accidentally explode when `n.dot(-dir)` happens to be very close to zero
const EPSILON: f32 = 0.005;

pub struct SweepCheckConfig {
    pub skin_width: f32,
}

pub struct SweepHitData {
    pub shape_hit: ShapeHitData,
    pub safe_distance: f32,
    pub dir: Dir,
    pub safe_pos: Vector,
    pub frac: f32,
    pub penetrating: bool,
}

pub struct CollideAndSlideConfig {
    pub iterations: usize,
    pub sweep_check_config: SweepCheckConfig,
    pub planes: Vec<Dir>,
    pub max_planes: usize,
    pub duplicate_plane_nudge: f32,
}

pub struct CollideAndSlideResult {
    pub position: Vector,
    pub velocity: Vector,
}

impl Default for CollideAndSlideConfig {
    fn default() -> Self {
        Self {
            iterations: 4,
            planes: Vec::new(),
            max_planes: 5,
            duplicate_plane_nudge: 0.05,
            sweep_check_config: SweepCheckConfig { skin_width: 0.001 },
        }
    }
}
