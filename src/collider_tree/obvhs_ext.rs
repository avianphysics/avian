use obvhs::{
    aabb::Aabb,
    bvh2::{Bvh2, node::Bvh2Node},
    fast_stack,
    faststack::FastStack,
    ray::{INVALID_ID, safe_inverse},
};
use obvhs_glam::Vec3A;

use crate::math::Ray;

/// A struct representing a sweep test, where an AABB is swept
/// along a velocity vector to test for intersection with other AABBs.
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct Sweep {
    /// The AABB of the collider being swept, in its starting position.
    pub aabb: Aabb,
    /// The velocity vector along which the AABB is swept.
    pub velocity: Vec3A,
    /// The inverse of the velocity vector components. Used to avoid division in sweep/aabb tests.
    pub inv_velocity: Vec3A,
    /// The minimum `t` (fraction) value for the sweep.
    pub tmin: f32,
    /// The maximum `t` (fraction) value for the sweep.
    pub tmax: f32,
}

impl Sweep {
    /// Creates a new `Sweep` with the given AABB, velocity, and `t` (fraction) range.
    pub fn new(aabb: Aabb, velocity: Vec3A, min: f32, max: f32) -> Self {
        let sweep = Sweep {
            aabb,
            velocity,
            inv_velocity: Vec3A::new(
                safe_inverse(velocity.x),
                safe_inverse(velocity.y),
                safe_inverse(velocity.z),
            ),
            tmin: min,
            tmax: max,
        };

        debug_assert!(sweep.inv_velocity.is_finite());
        debug_assert!(sweep.velocity.is_finite());
        debug_assert!(sweep.aabb.min.is_finite());
        debug_assert!(sweep.aabb.max.is_finite());

        sweep
    }
}

/// A hit record for a sweep test, containing the ID of the primitive that was hit
/// and the fraction along the sweep at which the hit occurred.
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct SweepHit {
    /// The ID of the primitive that was hit.
    pub primitive_id: u32,
    /// The fraction along the sweep at which the hit occurred.
    pub t: f32,
}

impl SweepHit {
    /// Creates a new `SweepHit` instance representing no hit.
    pub fn none() -> Self {
        Self {
            primitive_id: INVALID_ID,
            t: f32::INFINITY,
        }
    }
}

/// Extension trait for [`obvhs::bvh2::Bvh2`] to add additional traversal methods.
pub trait Bvh2Ext {
    /// Traverse the BVH by sweeping an AABB along a velocity vector. Returns the closest intersected primitive.
    ///
    /// # Arguments
    /// * `sweep` - The sweep to be tested for intersection.
    /// * `hit` - As `sweep_traverse_dynamic` intersects primitives, it will update `hit` with the closest.
    /// * `intersection_fn` - should take the given sweep and primitive index and return the distance to the intersection, if any.
    ///
    /// Note the primitive index should index first into `Bvh2::primitive_indices` then that will be index of original primitive.
    /// Various parts of the BVH building process might reorder the primitives. To avoid this indirection, reorder your
    /// original primitives per `primitive_indices`.
    fn sweep_traverse<F: FnMut(&Sweep, usize) -> f32>(
        &self,
        sweep: Sweep,
        hit: &mut SweepHit,
        intersection_fn: F,
    ) -> bool;

    /// Traverse the BVH by sweeping an AABB along a velocity vector. Returns true if the sweep missed all primitives.
    ///
    /// # Arguments
    /// * `sweep` - The sweep to be tested for intersection.
    /// * `intersection_fn` - should take the given sweep and primitive index and return the distance to the intersection, if any.
    ///
    /// Note the primitive index should index first into `Bvh2::primitive_indices` then that will be index of original primitive.
    /// Various parts of the BVH building process might reorder the primitives. To avoid this indirection, reorder your
    /// original primitives per `primitive_indices`.
    fn sweep_traverse_miss<F: FnMut(&Sweep, usize) -> f32>(
        &self,
        sweep: Sweep,
        intersection_fn: F,
    ) -> bool;

    /// Traverse the BVH by sweeping an AABB along a velocity vector. Intersects all primitives along the sweep
    /// and calls `intersection_fn` for each hit. The sweep is not updated, to allow for evaluating at every hit.
    ///
    /// # Arguments
    /// * `sweep` - The sweep to be tested for intersection.
    /// * `intersection_fn` - should take the given sweep and primitive index.
    ///
    /// Note the primitive index should index first into `Bvh2::primitive_indices` then that will be index of original primitive.
    /// Various parts of the BVH building process might reorder the primitives. To avoid this indirection, reorder your
    /// original primitives per `primitive_indices`.
    fn sweep_traverse_anyhit<F: FnMut(&Sweep, usize)>(&self, sweep: Sweep, intersection_fn: F);

    /// Traverse the BVH by sweeping an AABB along a velocity vector.
    ///
    /// Terminates when no hits are found or when `intersection_fn` returns false for a hit.
    ///
    /// # Arguments
    /// * `stack` - Stack for traversal state.
    /// * `sweep` - The sweep to be tested for intersection.
    /// * `hit` - As `sweep_traverse_dynamic` intersects primitives, it will update `hit` with the closest.
    /// * `intersection_fn` - should test the primitives in the given node, update the ray.tmax, and hit info. Return
    ///   false to halt traversal.
    ///
    /// Note the primitive index should index first into `Bvh2::primitive_indices` then that will be index of original primitive.
    /// Various parts of the BVH building process might reorder the primitives. To avoid this indirection, reorder your
    /// original primitives per `primitive_indices`.
    fn sweep_traverse_dynamic<
        F: FnMut(&Bvh2Node, &mut Sweep, &mut SweepHit) -> bool,
        Stack: FastStack<u32>,
    >(
        &self,
        stack: &mut Stack,
        sweep: Sweep,
        hit: &mut SweepHit,
        intersection_fn: F,
    );

    /// Traverse the BVH to find the closest leaf node to a point.
    /// Returns the primitive index and squared distance of the closest leaf, or `None` if no leaf is within `max_dist_sq`.
    ///
    /// # Arguments
    /// * `stack` - Stack for traversal state.
    /// * `point` - The query point.
    /// * `max_dist_sq` - Maximum squared distance to search (use `f32::INFINITY` for unlimited).
    /// * `closest_leaf` - Will be updated with the closest leaf node and distance found.
    /// * `visit_fn` - Called for each leaf node within range. Should take the given ray and primitive index and return the squared distance
    ///   to the primitive, if any.
    ///
    /// Note the primitive index should index first into `Bvh2::primitive_indices` then that will be index of original primitive.
    /// Various parts of the BVH building process might reorder the primitives. To avoid this indirection, reorder your
    /// original primitives per `primitive_indices`.
    fn squared_distance_traverse<F: FnMut(Vec3A, usize) -> f32>(
        &self,
        point: Vec3A,
        max_dist_sq: f32,
        visit_fn: F,
    ) -> Option<(u32, f32)>;

    /// Traverse the BVH with a point, calling `visit_fn` for each leaf node within `max_dist_sq` of the point.
    ///
    /// Terminates when all nodes within `max_dist_sq` have been visited or when `visit_fn` returns false for a node.
    ///
    /// # Arguments
    /// * `stack` - Stack for traversal state.
    /// * `point` - The query point.
    /// * `max_dist_sq` - Maximum squared distance to search (use `f32::INFINITY` for unlimited).
    /// * `closest_leaf` - Will be updated with the closest leaf node and distance found.
    /// * `visit_fn` - Called for each leaf node within range. Should update `max_dist_sq` and `closest_leaf`.
    ///   Return false to halt traversal early.
    ///
    /// Note the primitive index should index first into `Bvh2::primitive_indices` then that will be index of original primitive.
    /// Various parts of the BVH building process might reorder the primitives. To avoid this indirection, reorder your
    /// original primitives per `primitive_indices`.
    fn squared_distance_traverse_dynamic<
        F: FnMut(&Bvh2Node, &mut f32, &mut Option<(u32, f32)>) -> bool,
        Stack: FastStack<u32>,
    >(
        &self,
        stack: &mut Stack,
        point: Vec3A,
        max_dist_sq: f32,
        closest_leaf: &mut Option<(u32, f32)>,
        visit_fn: F,
    );
}

impl Bvh2Ext for Bvh2 {
    #[inline(always)]
    fn sweep_traverse<F: FnMut(&Sweep, usize) -> f32>(
        &self,
        sweep: Sweep,
        hit: &mut SweepHit,
        mut intersection_fn: F,
    ) -> bool {
        let mut intersect_prims = |node: &Bvh2Node, sweep: &mut Sweep, hit: &mut SweepHit| {
            (node.first_index..node.first_index + node.prim_count).for_each(|primitive_id| {
                let t = intersection_fn(sweep, primitive_id as usize);
                if t < sweep.tmax {
                    hit.primitive_id = primitive_id;
                    hit.t = t;
                    sweep.tmax = t;
                }
            });
            true
        };

        fast_stack!(u32, (96, 192), self.max_depth, stack, {
            Bvh2::sweep_traverse_dynamic(self, &mut stack, sweep, hit, &mut intersect_prims)
        });

        hit.t < sweep.tmax // Note this is valid since traverse_with_stack does not mutate the sweep
    }

    #[inline(always)]
    fn sweep_traverse_miss<F: FnMut(&Sweep, usize) -> f32>(
        &self,
        sweep: Sweep,
        mut intersection_fn: F,
    ) -> bool {
        let mut miss = true;
        let mut intersect_prims = |node: &Bvh2Node, sweep: &mut Sweep, _hit: &mut SweepHit| {
            for primitive_id in node.first_index..node.first_index + node.prim_count {
                let t = intersection_fn(sweep, primitive_id as usize);
                if t < sweep.tmax {
                    miss = false;
                    return false;
                }
            }
            true
        };

        fast_stack!(u32, (96, 192), self.max_depth, stack, {
            Bvh2::sweep_traverse_dynamic(
                self,
                &mut stack,
                sweep,
                &mut SweepHit::none(),
                &mut intersect_prims,
            )
        });

        miss
    }

    #[inline(always)]
    fn sweep_traverse_anyhit<F: FnMut(&Sweep, usize)>(&self, sweep: Sweep, mut intersection_fn: F) {
        let mut intersect_prims = |node: &Bvh2Node, sweep: &mut Sweep, _hit: &mut SweepHit| {
            for primitive_id in node.first_index..node.first_index + node.prim_count {
                intersection_fn(sweep, primitive_id as usize);
            }
            true
        };

        let mut hit = SweepHit::none();
        fast_stack!(u32, (96, 192), self.max_depth, stack, {
            self.sweep_traverse_dynamic(&mut stack, sweep, &mut hit, &mut intersect_prims)
        });
    }

    #[inline(always)]
    fn sweep_traverse_dynamic<
        F: FnMut(&Bvh2Node, &mut Sweep, &mut SweepHit) -> bool,
        Stack: FastStack<u32>,
    >(
        &self,
        stack: &mut Stack,
        mut sweep: Sweep,
        hit: &mut SweepHit,
        mut intersection_fn: F,
    ) {
        if self.nodes.is_empty() {
            return;
        }

        let root_node = &self.nodes[0];
        let root_aabb = root_node.aabb();
        let hit_root = root_aabb.intersect_sweep(&sweep) < sweep.tmax;
        if !hit_root {
            return;
        } else if root_node.is_leaf() {
            intersection_fn(root_node, &mut sweep, hit);
            return;
        };

        let mut current_node_index = root_node.first_index;
        loop {
            let right_index = current_node_index as usize + 1;
            assert!(right_index < self.nodes.len());
            let mut left_node = unsafe { self.nodes.get_unchecked(current_node_index as usize) };
            let mut right_node = unsafe { self.nodes.get_unchecked(right_index) };

            // TODO perf: could it be faster to intersect these at the same time with avx?
            let mut left_t = left_node.aabb().intersect_sweep(&sweep);
            let mut right_t = right_node.aabb().intersect_sweep(&sweep);

            if left_t > right_t {
                core::mem::swap(&mut left_t, &mut right_t);
                core::mem::swap(&mut left_node, &mut right_node);
            }

            let hit_left = left_t < sweep.tmax;

            let go_left = if hit_left && left_node.is_leaf() {
                if !intersection_fn(left_node, &mut sweep, hit) {
                    return;
                }
                false
            } else {
                hit_left
            };

            let hit_right = right_t < sweep.tmax;

            let go_right = if hit_right && right_node.is_leaf() {
                if !intersection_fn(right_node, &mut sweep, hit) {
                    return;
                }
                false
            } else {
                hit_right
            };

            match (go_left, go_right) {
                (true, true) => {
                    current_node_index = left_node.first_index;
                    stack.push(right_node.first_index);
                }
                (true, false) => current_node_index = left_node.first_index,
                (false, true) => current_node_index = right_node.first_index,
                (false, false) => {
                    let Some(next) = stack.pop() else {
                        hit.t = sweep.tmax;
                        return;
                    };
                    current_node_index = next;
                }
            }
        }
    }

    #[inline(always)]
    fn squared_distance_traverse<F: FnMut(Vec3A, usize) -> f32>(
        &self,
        point: Vec3A,
        max_dist_sq: f32,
        mut visit_fn: F,
    ) -> Option<(u32, f32)> {
        let mut closest_leaf = None;

        let mut visit_prims =
            |node: &Bvh2Node, max_dist_sq: &mut f32, closest_leaf: &mut Option<(u32, f32)>| {
                (node.first_index..node.first_index + node.prim_count).for_each(|primitive_id| {
                    let distance_sq = visit_fn(point, primitive_id as usize);
                    if distance_sq < *max_dist_sq {
                        *closest_leaf = Some((primitive_id, distance_sq));
                        *max_dist_sq = distance_sq;
                    }
                });
                true
            };

        fast_stack!(u32, (96, 192), self.max_depth, stack, {
            Bvh2::squared_distance_traverse_dynamic(
                self,
                &mut stack,
                point,
                max_dist_sq,
                &mut closest_leaf,
                &mut visit_prims,
            )
        });

        closest_leaf
    }

    #[inline(always)]
    fn squared_distance_traverse_dynamic<
        F: FnMut(&Bvh2Node, &mut f32, &mut Option<(u32, f32)>) -> bool,
        Stack: FastStack<u32>,
    >(
        &self,
        stack: &mut Stack,
        point: Vec3A,
        mut max_dist_sq: f32,
        closest_leaf: &mut Option<(u32, f32)>,
        mut visit_fn: F,
    ) {
        if self.nodes.is_empty() {
            return;
        }

        let root_node = &self.nodes[0];
        let root_dist_sq = root_node.aabb().distance_to_point_squared(point);

        if root_dist_sq > max_dist_sq {
            return;
        } else if root_node.is_leaf() {
            visit_fn(root_node, &mut max_dist_sq, closest_leaf);
            return;
        }

        let mut current_node_index = root_node.first_index;

        loop {
            let right_index = current_node_index as usize + 1;
            assert!(right_index < self.nodes.len());
            let mut left_node = unsafe { self.nodes.get_unchecked(current_node_index as usize) };
            let mut right_node = unsafe { self.nodes.get_unchecked(right_index) };

            // TODO perf: could it be faster to compute these at the same time with avx?
            let mut left_dist_sq = left_node.aabb().distance_to_point_squared(point);
            let mut right_dist_sq = right_node.aabb().distance_to_point_squared(point);

            // Sort by distance (closer first)
            if left_dist_sq > right_dist_sq {
                core::mem::swap(&mut left_dist_sq, &mut right_dist_sq);
                core::mem::swap(&mut left_node, &mut right_node);
            }

            let within_left = left_dist_sq <= max_dist_sq;

            let go_left = if within_left && left_node.is_leaf() {
                if !visit_fn(left_node, &mut max_dist_sq, closest_leaf) {
                    return;
                }
                false
            } else {
                within_left
            };

            let within_right = right_dist_sq <= max_dist_sq;

            let go_right = if within_right && right_node.is_leaf() {
                if !visit_fn(right_node, &mut max_dist_sq, closest_leaf) {
                    return;
                }
                false
            } else {
                within_right
            };

            match (go_left, go_right) {
                (true, true) => {
                    current_node_index = left_node.first_index;
                    stack.push(right_node.first_index);
                }
                (true, false) => current_node_index = left_node.first_index,
                (false, true) => current_node_index = right_node.first_index,
                (false, false) => {
                    let Some(next) = stack.pop() else {
                        return;
                    };
                    current_node_index = next;
                }
            }
        }
    }
}

pub trait ObvhsAabbExt {
    /// Computes the squared distance from a point to this AABB.
    fn distance_to_point_squared(&self, point: Vec3A) -> f32;

    /// Checks if this AABB intersects with a sweep and returns the fraction
    /// along the sweep at which the intersection occurs.
    ///
    /// Returns `f32::INFINITY` if there is no intersection.
    fn intersect_sweep(&self, sweep: &Sweep) -> f32;
}

impl ObvhsAabbExt for Aabb {
    #[inline(always)]
    fn distance_to_point_squared(&self, point: Vec3A) -> f32 {
        let point_min = self.min - point;
        let point_max = self.max - point;
        let zero = Vec3A::ZERO;
        let dist_min = point_min.max(zero);
        let dist_max = point_max.min(zero);
        dist_min.length_squared().min(dist_max.length_squared())
    }

    #[inline(always)]
    fn intersect_sweep(&self, sweep: &Sweep) -> f32 {
        let minkowski_sum_shift = -sweep.aabb.center();
        let minkowski_sum_margin = sweep.aabb.diagonal() * 0.5 + Vec3A::splat(sweep.tmin);

        let msum = Aabb {
            min: self.min + minkowski_sum_shift - minkowski_sum_margin,
            max: self.max + minkowski_sum_shift + minkowski_sum_margin,
        };

        // Now, we cast a ray from the origin along the velocity,
        // and intersect it with the Minkowski sum.
        let t1 = msum.min * sweep.inv_velocity;
        let t2 = msum.max * sweep.inv_velocity;

        let tmin = t1.min(t2);
        let tmax = t1.max(t2);

        let tmin_n = tmin.max_element();
        let tmax_n = tmax.min_element();

        if tmax_n >= tmin_n && tmax_n >= 0.0 {
            tmin_n
        } else {
            f32::INFINITY
        }
    }
}

#[inline(always)]
pub fn obvhs_ray(ray: &Ray, max_distance: f32) -> obvhs::ray::Ray {
    #[cfg(feature = "2d")]
    let origin = ray.origin.extend(0.0).to_array().into();
    #[cfg(feature = "3d")]
    let origin = ray.origin.to_array().into();
    #[cfg(feature = "2d")]
    let direction = ray.direction.extend(0.0).to_array().into();
    #[cfg(feature = "3d")]
    let direction = ray.direction.to_array().into();

    obvhs::ray::Ray::new(origin, direction, 0.0, max_distance)
}
