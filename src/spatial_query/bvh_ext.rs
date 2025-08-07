use bevy_math::Vec3A;
use obvhs::{
    aabb::Aabb,
    bvh2::{Bvh2, RayTraversal},
    heapstack::HeapStack,
    ray::{Ray, RayHit},
};

pub(crate) trait BvhExt {
    fn cast_traverse<F: FnMut(&Ray, usize) -> f32>(
        &self,
        ray: Ray,
        hit: &mut RayHit,
        shape: Aabb,
        intersection_fn: F,
    ) -> bool;

    fn cast_traverse_dynamic<F: FnMut(&Ray, usize) -> f32>(
        &self,
        state: &mut RayTraversal,
        hit: &mut RayHit,
        shape: Aabb,
        intersection_fn: F,
    ) -> bool;

    fn dist_traverse<F: FnMut(&Vec3A, usize) -> f32>(
        &self,
        pos: Vec3A,
        hit: &mut RayHit,
        dist_fn: F,
    ) -> bool;

    fn dist_traverse_dynamic<F: FnMut(&Vec3A, usize) -> f32>(
        &self,
        state: &mut DistTraversal,
        hit: &mut RayHit,
        dist_fn: F,
    ) -> bool;

    fn point_traverse_dynamic<F: FnMut(&Vec3A, usize) -> bool>(
        &self,
        state: &mut PointTraversal,
        hit: &mut RayHit,
        contains_fn: F,
    ) -> bool;
}

impl BvhExt for Bvh2 {
    fn cast_traverse<F: FnMut(&Ray, usize) -> f32>(
        &self,
        ray: Ray,
        hit: &mut RayHit,
        shape: Aabb,
        mut intersection_fn: F,
    ) -> bool {
        let mut state = self.new_ray_traversal(ray);
        while self.cast_traverse_dynamic(&mut state, hit, shape, &mut intersection_fn) {}
        hit.t < ray.tmax // Note this is valid since traverse_with_stack does not mutate the ray
    }

    fn cast_traverse_dynamic<F: FnMut(&Ray, usize) -> f32>(
        &self,
        state: &mut RayTraversal,
        hit: &mut RayHit,
        shape: Aabb,
        mut intersection_fn: F,
    ) -> bool {
        loop {
            while state.primitive_count > 0 {
                let primitive_id = state.current_primitive_index;
                state.current_primitive_index += 1;
                state.primitive_count -= 1;
                let t = intersection_fn(&state.ray, primitive_id as usize);
                if t < state.ray.tmax {
                    hit.primitive_id = primitive_id;
                    hit.t = t;
                    state.ray.tmax = t;
                    // Yield when we hit a primitive
                    return true;
                }
            }
            if let Some(current_node_index) = state.stack.pop() {
                let node = &self.nodes[*current_node_index as usize];
                let modified_aabb = Aabb {
                    min: node.aabb.min - shape.max,
                    max: node.aabb.max - shape.min,
                };
                if modified_aabb.intersect_ray(&state.ray) >= state.ray.tmax {
                    continue;
                }

                if node.is_leaf() {
                    state.primitive_count = node.prim_count;
                    state.current_primitive_index = node.first_index;
                } else {
                    state.stack.push(node.first_index);
                    state.stack.push(node.first_index + 1);
                }
            } else {
                // Returns false when there are no more primitives to test.
                // This doesn't mean we never hit one along the way though. (and yielded then)
                return false;
            }
        }
    }

    fn dist_traverse<F: FnMut(&Vec3A, usize) -> f32>(
        &self,
        pos: Vec3A,
        hit: &mut RayHit,
        mut dist_fn: F,
    ) -> bool {
        let mut stack = HeapStack::new_with_capacity(self.max_depth.unwrap_or(96));
        if !self.nodes.is_empty() {
            stack.push(0);
        }
        let mut state = DistTraversal {
            stack,
            point: pos,
            min_dist_sq: f32::INFINITY,
            current_primitive_index: 0,
            primitive_count: 0,
        };
        while self.dist_traverse_dynamic(&mut state, hit, &mut dist_fn) {}
        hit.t < f32::INFINITY
    }

    fn dist_traverse_dynamic<F: FnMut(&Vec3A, usize) -> f32>(
        &self,
        state: &mut DistTraversal,
        hit: &mut RayHit,
        mut dist_fn: F,
    ) -> bool {
        loop {
            while state.primitive_count > 0 {
                let primitive_id = state.current_primitive_index;
                state.current_primitive_index += 1;
                state.primitive_count -= 1;
                let d = dist_fn(&state.point, primitive_id as usize);
                let d_sq = d * d;
                if d_sq < state.min_dist_sq {
                    hit.primitive_id = primitive_id;
                    hit.t = d;
                    state.min_dist_sq = d_sq;
                    // Yield when we hit a primitive
                    return true;
                }
            }
            if let Some(current_node_index) = state.stack.pop() {
                let node = &self.nodes[*current_node_index as usize];
                if node.aabb.distance_squared(state.point) >= state.min_dist_sq {
                    continue;
                }

                if node.is_leaf() {
                    state.primitive_count = node.prim_count;
                    state.current_primitive_index = node.first_index;
                } else {
                    state.stack.push(node.first_index);
                    state.stack.push(node.first_index + 1);
                }
            } else {
                // Returns false when there are no more primitives to test.
                // This doesn't mean we never hit one along the way though. (and yielded then)
                return false;
            }
        }
    }

    fn point_traverse_dynamic<F: FnMut(&Vec3A, usize) -> bool>(
        &self,
        state: &mut PointTraversal,
        hit: &mut RayHit,
        mut contains_fn: F,
    ) -> bool {
        loop {
            while state.primitive_count > 0 {
                let primitive_id = state.current_primitive_index;
                state.current_primitive_index += 1;
                state.primitive_count -= 1;
                if contains_fn(&state.point, primitive_id as usize) {
                    hit.primitive_id = primitive_id;
                    // Yield when we hit a primitive
                    return true;
                }
            }
            if let Some(current_node_index) = state.stack.pop() {
                let node = &self.nodes[*current_node_index as usize];
                if !node.aabb.contains_point(state.point) {
                    continue;
                }

                if node.is_leaf() {
                    state.primitive_count = node.prim_count;
                    state.current_primitive_index = node.first_index;
                } else {
                    state.stack.push(node.first_index);
                    state.stack.push(node.first_index + 1);
                }
            } else {
                // Returns false when there are no more primitives to test.
                // This doesn't mean we never hit one along the way though. (and yielded then)
                return false;
            }
        }
    }
}

pub struct DistTraversal {
    pub stack: HeapStack<u32>,
    pub point: Vec3A,
    pub min_dist_sq: f32,
    pub current_primitive_index: u32,
    pub primitive_count: u32,
}

trait DistAabb {
    fn distance_squared(&self, p: Vec3A) -> f32;
}

impl DistAabb for Aabb {
    fn distance_squared(&self, p: Vec3A) -> f32 {
        let closest = p.clamp(self.min, self.max);
        return closest.distance_squared(p);
    }
}

pub struct PointTraversal {
    pub stack: HeapStack<u32>,
    pub point: Vec3A,
    pub current_primitive_index: u32,
    pub primitive_count: u32,
}

impl PointTraversal {
    pub fn new(bvh: &Bvh2, pos: Vec3A) -> Self {
        let mut stack = HeapStack::new_with_capacity(bvh.max_depth.unwrap_or(96));
        if !bvh.nodes.is_empty() {
            stack.push(0);
        }
        PointTraversal {
            stack,
            point: pos,
            current_primitive_index: 0,
            primitive_count: 0,
        }
    }
}

pub trait RayExt {
    fn transformed(self, pos: super::Position, rot: super::Rotation) -> Self;
}

impl RayExt for Ray {
    fn transformed(mut self, pos: super::Position, rot: super::Rotation) -> Self {
        let inv_rot = rot.inverse();
        self.direction = inv_rot * self.direction;
        self.inv_direction = inv_rot * self.inv_direction;
        #[cfg(feature = "2d")]
        let pos = pos.extend(0.);
        #[cfg(feature = "3d")]
        let pos = *pos;
        self.origin = inv_rot * (self.origin - Vec3A::from(pos));
        self
    }
}
