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

        // Initial depenetration pass.
        let mut intersections = Vec::new();
        self.intersections_callback(
            shape,
            shape_rotation,
            position,
            filter,
            config.skin_width,
            |contact_point, normal| {
                // TODO: Should we call on_hit here?
                intersections.push((normal, contact_point.penetration + config.skin_width));
                true
            },
        );
        let depenetration_offset = self.depenetrate(config, &intersections);
        position += depenetration_offset;

        // Main move and slide loop.
        // 1. Sweep the shape along the velocity vector.
        // 2. If we hit something, move up to the hit point.
        // 3. Collect contact planes.
        // 4. Depenetrate based on penetrating intersections found.
        // 5. Clip velocity to be parallel to all contact planes.
        // 6. Repeat until we run out of iterations or time.
        'outer: for _ in 0..config.move_and_slide_iterations {
            let sweep = time_left * velocity;
            let Some((vel_dir, distance)) = Dir::new_and_length(sweep.f32()).ok() else {
                // no more movement to go
                break;
            };
            let distance = distance.adjust_precision();
            const MIN_DISTANCE: Scalar = 1e-4;
            if distance < MIN_DISTANCE {
                break;
            }

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
            if sweep_hit.intersects {
                // entity is completely trapped in another solid
                velocity = Vector::ZERO;
                break 'outer;
            }

            // Move up to the hit point.
            time_left -= time_left * (sweep_hit.safe_distance / distance);
            position += vel_dir.adjust_precision() * sweep_hit.safe_distance;

            // User-defined clipping planes.
            let mut planes = config.planes.clone();

            // Penetrating contacts.
            let mut intersections = Vec::new();

            // Collect contact planes.
            self.intersections_callback(
                shape,
                shape_rotation,
                position,
                filter,
                // Use a slightly larger skin width to ensure we catch all contacts for velocity clipping.
                // Depenetration still uses just the normal skin width.
                config.skin_width * 1.1,
                |contact_point, normal| {
                    if !on_hit(MoveAndSlideHitData {
                        intersects: false,
                        entity: sweep_hit.entity,
                        point: contact_point.point,
                        normal,
                        collision_distance: sweep_hit.collision_distance,
                        safe_distance: sweep_hit.safe_distance,
                        position,
                        velocity,
                    }) {
                        return false;
                    }
                    planes.push(normal);
                    intersections.push((normal, contact_point.penetration + config.skin_width));
                    true
                },
            );

            // Depenetrate based on intersections found.
            let depenetration_offset = self.depenetrate(config, &intersections);
            position += depenetration_offset;

            // Modify velocity so that it parallels all of the clip planes.
            velocity = project_velocity(velocity, &planes);

            // If the original velocity is against the original velocity, stop dead
            // to avoid tiny occilations in sloping corners.
            if velocity.dot(original_velocity) <= -DOT_EPSILON {
                velocity = Vector::ZERO;
                break 'outer;
            }
        }

        // Final depenetration pass.
        let mut intersections = Vec::new();
        self.intersections_callback(
            shape,
            shape_rotation,
            position,
            filter,
            config.skin_width,
            |contact_point, normal| {
                intersections.push((normal, contact_point.penetration + config.skin_width));
                true
            },
        );
        let depenetration_offset = self.depenetrate(config, &intersections);
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
            collision_distance: distance,
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

    /// Calls the provided callback for the deepest contact point of each contact manifold found
    /// between the provided shape and all colliders in the scene that pass the provided filter.
    pub fn intersections_callback(
        &self,
        shape: &Collider,
        shape_rotation: RotationValue,
        origin: Vector,
        filter: &SpatialQueryFilter,
        prediction_distance: Scalar,
        mut callback: impl FnMut(&ContactPoint, Dir) -> bool,
    ) {
        let expanded_aabb = shape
            .aabb(origin, shape_rotation)
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
                origin,
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

    #[must_use]
    pub fn depenetrate(
        &self,
        config: &MoveAndSlideConfig,
        intersections: &[(Dir, Scalar)],
    ) -> Vector {
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
    /// To move the shape, use [`Self::safe_distance`] instead.
    #[doc(alias = "time_of_impact")]
    pub collision_distance: Scalar,

    /// The hit point point on the shape that was hit, expressed in world space.
    pub point: Vector,

    /// The outward surface normal on the hit shape at `point`, expressed in world space.
    pub normal: Dir,

    /// A safe distance to move the shape without intersecting the hit collider.
    pub safe_distance: Scalar,

    /// Whether the current move and slide iteration started off with the collider intersecting another collider.
    pub intersects: bool,

    /// The position of the collider at the point of the move and slide iteration.
    pub position: Vector,

    /// The velocity of the collider at the point of the move and slide iteration.
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
    /// To move the shape, use [`Self::safe_distance`] instead.
    #[doc(alias = "time_of_impact")]
    pub collision_distance: Scalar,

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

    /// A safe distance to move the shape without intersecting the hit collider.
    pub safe_distance: Scalar,

    /// Whether the current move and slide iteration started off with the collider intersecting another collider.
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
    /// A minimal distance to always keep between the collider and any other colliders.
    /// This is here to ensure that the collider never intersects anything, even when numeric errors accumulate.
    /// Set this to a very small value.
    ///
    /// Increase the value if you notice your character getting stuck in geometry.
    /// Decrease it when you notice jittering, especially around V-shaped walls.
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
    /// The final position of the collider after move and slide. Set your [`Transform::translation`] to this value.
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

const EPS: f32 = 1e-6;

/// Projects input velocity `v` so it satisfies n_i · v' >= 0 for all contact normals n_i.
pub fn project_velocity(v: Vector, normals: &[Dir]) -> Vector {
    // Case 1: Check if v is inside the cone
    if normals.iter().all(|&n| v.dot(*n) >= -EPS) {
        return v;
    }

    // Best candidate so far
    let mut best_u = Vector::ZERO;
    let mut best_d2 = f32::INFINITY;

    // Helper to test halfspace validity
    let is_valid = |u: Vector| normals.iter().all(|&n| u.dot(*n) >= -EPS);

    // Case 2a: Face projections (single-plane active set)
    for &n in normals {
        let vd = v.dot(*n);
        if vd < 0.0 {
            // Only meaningful if v violates this plane
            let u = v - vd * n; // Project onto the plane

            // Check if better than previous best and valid
            let d2 = (v - u).length_squared();
            if d2 < best_d2 && is_valid(u) {
                best_d2 = d2;
                best_u = u;
            }
        }
    }

    // TODO: Does 2D still need something like this?
    // Case 2b: Edge projections (two-plane active set)
    // TODO: Can we optimize this from O(n^3) to O(n^2)?
    #[cfg(feature = "3d")]
    {
        let n = normals.len();
        for i in 0..n {
            for j in (i + 1)..n {
                let ni = *normals[i];
                let nj = *normals[j];

                // Compute edge direction e = ni x nj
                let e = ni.cross(nj);
                let e2 = e.length_squared();
                if e2 < EPS {
                    // Nearly parallel edge
                    continue;
                }

                // Project v onto the line spanned by e:
                // u = ((v·e) / |e|²) e
                let u = e * (v.dot(e) / e2);

                // Check if better than previous best and valid
                let d2 = (v - u).length_squared();
                if d2 < best_d2 && is_valid(u) {
                    best_d2 = d2;
                    best_u = u;
                }
            }
        }
    }

    // Case 3: If no candidate is found, the projection is at the apex (the origin)
    if best_d2.is_infinite() {
        Vector::ZERO
    } else {
        best_u
    }
}
