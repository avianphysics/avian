use crate::prelude::*;
use bevy::{ecs::system::SystemParam, prelude::*};

#[derive(SystemParam)]
pub struct CollideAndSlide<'w> {
    query_pipeline: Res<'w, SpatialQueryPipeline>,
    time: Res<'w, Time>,
}

impl<'w> CollideAndSlide<'w> {
    pub fn collide_and_slide(
        &self,
        shape: &Collider,
        shape_rotation: RotationValue,
        origin: Vector,
        mut velocity: Vector,
        config: &CollideAndSlideConfig,
        filter: &SpatialQueryFilter,
    ) -> CollideAndSlideResult {
        let mut end_velocity = Vector::ZERO;
        let mut position = origin;

        if gravity {
            end_velocity = velocity;
            end_velocity.y -= ctx.dt * ctx.cfg.gravity;
            velocity.y = (velocity.y + end_velocity.y) * 0.5;
            if state.ground_plane
                && let Some(grounded) = state.grounded
            {
                // slide along the ground plane
                velocity = clip_velocity(velocity, grounded.normal1);
            }
        }

        let mut time_left = self.time.delta_secs();

        let mut planes = config.planes.clone();

        // never turn against original velocity
        let Ok(vel_dir) = Dir::new(velocity) else {
            return CollideAndSlideResult { position, velocity };
        };
        planes.push(vel_dir);

        let mut bump_count = 0;
        while bump_count < config.iterations {
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
                bump_count += 1;
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
                let mut current_clip_velocity = Self::clip_velocity(velocity, planes[i].into());
                let mut end_clip_velocity = Self::clip_velocity(end_velocity, planes[i].into());

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
                        end_clip_velocity = Self::clip_velocity(end_clip_velocity, planes[j]);

                        // see if it goes back into the first clip plane
                        if current_clip_velocity.dot(planes[i].into()) >= 0.0 {
                            continue;
                        }

                        // slide the original velocity along the crease
                        let dir = planes[i].cross(planes[j].into());
                        let d = dir.dot(velocity);
                        current_clip_velocity = dir * d;

                        let d = dir.dot(end_velocity);
                        end_clip_velocity = dir * d;

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
                end_velocity = end_clip_velocity;
                break;
            }

            bump_count += 1;
        }
        if gravity {
            velocity = end_velocity;
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

    #[must_use]
    pub fn sweep_check(
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
            &ShapeCastConfig {
                max_distance: cast_len,
                ignore_origin_penetration: true,
                compute_contact_on_penetration: false,
                ..default()
            },
            filter,
        )?;

        let n = hit.normal1;
        let dir: Vector = cast_dir.into();

        // needed to not accidentally explode when `n.dot(-dir)` happens to be very close to zero
        const EPSILON: f32 = 0.0001;
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
