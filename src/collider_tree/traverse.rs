#![allow(clippy::unnecessary_cast)]

use obvhs::{aabb::Aabb, bvh2::node::Bvh2Node, fast_stack};

use crate::{
    collider_tree::{
        Bvh2Ext, ColliderTree, ProxyId,
        obvhs_ext::{Sweep, SweepHit},
        obvhs_ray,
    },
    math::{AsF32, Dir, Ray, Scalar, Vector},
};

impl ColliderTree {
    /// Traverses the tree for the closest intersection with the given ray.
    ///
    /// # Arguments
    ///
    /// - `ray`: The ray to be tested for intersection.
    /// - `max_distance`: The maximum distance along the ray to consider for intersections.
    /// - `intersection_fn`: A function that takes a proxy ID, and returns the distance to the intersection with that proxy.
    ///   This function is called for each potential intersection found during traversal.
    #[inline(always)]
    pub fn ray_traverse_closest<F: FnMut(ProxyId) -> Scalar>(
        &self,
        ray: Ray,
        max_distance: Scalar,
        mut intersection_fn: F,
    ) -> Option<(ProxyId, Scalar)> {
        let obvhs_ray = obvhs_ray(&ray, max_distance as f32);
        let mut hit = obvhs::ray::RayHit::none();

        let found_hit = self
            .bvh
            .ray_traverse(obvhs_ray, &mut hit, |_ray, primitive_id| {
                let proxy_id = ProxyId::new(self.bvh.primitive_indices[primitive_id]);
                intersection_fn(proxy_id) as f32
            });

        if found_hit {
            let proxy_id = ProxyId::new(self.bvh.primitive_indices[hit.primitive_id as usize]);
            Some((proxy_id, hit.t as Scalar))
        } else {
            None
        }
    }

    /// Traverses the tree for all intersections with the given ray.
    ///
    /// Terminates when all intersections within `max_distance` have been visited or when `intersection_fn` returns false for an intersection.
    ///
    /// # Arguments
    ///
    /// - `ray`: The ray to be tested for intersection.
    /// - `max_distance`: The maximum distance along the ray to consider for intersections.
    /// - `intersection_fn`: A function that takes a proxy ID, and is called for each potential intersection found during traversal.
    ///   Return false to halt traversal early.
    #[inline(always)]
    pub fn ray_traverse_all<F: FnMut(ProxyId) -> bool>(
        &self,
        ray: Ray,
        max_distance: Scalar,
        mut intersection_fn: F,
    ) {
        let obvhs_ray = obvhs_ray(&ray, max_distance as f32);

        self.bvh
            .ray_traverse_anyhit(obvhs_ray, |_ray, primitive_id| {
                let proxy_id = ProxyId::new(self.bvh.primitive_indices[primitive_id]);
                intersection_fn(proxy_id);
            });
    }

    /// Traverse the BVH by sweeping an AABB along a velocity vector, returning the closest hit.
    ///
    /// # Arguments
    ///
    /// - `aabb`: The axis-aligned bounding box to be swept.
    /// - `direction`: The direction along which to sweep the AABB.
    /// - `target_distance`: The separation distance at which a hit is still considered valid.
    /// - `max_distance`: The maximum distance along the sweep to consider for intersections.
    /// - `intersection_fn`: A function that takes a proxy ID, and returns the distance to the intersection with that proxy.
    ///   This function is called for each potential intersection found during traversal.
    #[inline(always)]
    pub fn sweep_traverse_closest<F: FnMut(ProxyId) -> Scalar>(
        &self,
        aabb: Aabb,
        direction: Dir,
        max_distance: Scalar,
        target_distance: Scalar,
        mut intersection_fn: F,
    ) -> Option<(ProxyId, Scalar)> {
        #[cfg(feature = "2d")]
        let direction = direction.extend(0.0).to_array().into();
        #[cfg(feature = "3d")]
        let direction = direction.to_array().into();
        let sweep = Sweep::new(aabb, direction, target_distance as f32, max_distance as f32);

        let mut hit = SweepHit::none();

        let found_hit = self
            .bvh
            .sweep_traverse(sweep, &mut hit, |_sweep, primitive_id| {
                let proxy_id = ProxyId::new(self.bvh.primitive_indices[primitive_id]);
                intersection_fn(proxy_id) as f32
            });

        if found_hit {
            let proxy_id = ProxyId::new(self.bvh.primitive_indices[hit.primitive_id as usize]);
            Some((proxy_id, hit.t as Scalar))
        } else {
            None
        }
    }

    /// Traverse the BVH by sweeping an AABB along a velocity vector, calling `intersection_fn` for each hit.
    ///
    /// # Arguments
    ///
    /// - `aabb`: The axis-aligned bounding box to be swept.
    /// - `direction`: The direction along which to sweep the AABB.
    /// - `target_distance`: The separation distance at which a hit is still considered valid.
    /// - `max_distance`: The maximum distance along the sweep to consider for intersections.
    /// - `intersection_fn`: A function that takes a proxy ID, and is called for each potential intersection found during traversal.
    ///   Return false to halt traversal early.
    #[inline(always)]
    pub fn sweep_traverse_all<F: FnMut(ProxyId) -> bool>(
        &self,
        aabb: Aabb,
        direction: Dir,
        target_distance: Scalar,
        max_distance: Scalar,
        mut intersection_fn: F,
    ) {
        #[cfg(feature = "2d")]
        let direction = direction.extend(0.0).to_array().into();
        #[cfg(feature = "3d")]
        let direction = direction.to_array().into();
        let sweep = Sweep::new(aabb, direction, target_distance as f32, max_distance as f32);

        let mut intersect_prims = |node: &Bvh2Node, _sweep: &mut Sweep, _hit: &mut SweepHit| {
            for primitive_id in node.first_index..node.first_index + node.prim_count {
                let proxy_id = ProxyId::new(self.bvh.primitive_indices[primitive_id as usize]);
                intersection_fn(proxy_id);
            }
            true
        };

        let mut hit = SweepHit::none();
        fast_stack!(u32, (96, 192), self.bvh.max_depth, stack, {
            self.bvh
                .sweep_traverse_dynamic(&mut stack, sweep, &mut hit, &mut intersect_prims)
        });
    }

    /// Traverse the BVH with a point, returning the closest proxy and its squared distance within `max_distance_squared`.
    ///
    /// # Arguments
    ///
    /// - `point`: The point to be tested for proximity.
    /// - `max_distance_squared`: The maximum distance from the point to consider for projections.
    /// - `eval`: A function that takes a proxy ID and returns the squared distance from the point to that proxy. This function is called for each potential projection found during traversal.
    #[inline(always)]
    pub fn squared_distance_traverse_closest<F: FnMut(ProxyId) -> Scalar>(
        &self,
        point: Vector,
        max_distance_squared: Scalar,
        mut eval: F,
    ) -> Option<(ProxyId, Scalar)> {
        #[cfg(feature = "2d")]
        let point = point.f32().extend(0.0).to_array().into();
        #[cfg(feature = "3d")]
        let point = point.f32().to_array().into();

        let closest_leaf = self.bvh.squared_distance_traverse(
            point,
            max_distance_squared as f32,
            |_point, primitive_id| {
                let proxy_id = ProxyId::new(self.bvh.primitive_indices[primitive_id]);
                eval(proxy_id) as f32
            },
        );

        if let Some((primitive_id, distance_squared)) = closest_leaf {
            let proxy_id = ProxyId::new(self.bvh.primitive_indices[primitive_id as usize]);
            Some((proxy_id, distance_squared as Scalar))
        } else {
            None
        }
    }

    /// Traverse the BVH with a point, calling `eval` for each intersection.
    ///
    /// # Arguments
    ///
    /// - `point`: The point to be tested for intersection.
    /// - `eval`: A function that takes a proxy ID and is called for each potential intersection found during traversal.
    ///   Return false to halt traversal early.
    #[inline(always)]
    pub fn point_traverse<F: FnMut(ProxyId) -> bool>(&self, point: Vector, mut eval: F) {
        #[cfg(feature = "2d")]
        let point = point.f32().extend(0.0).to_array().into();
        #[cfg(feature = "3d")]
        let point = point.f32().to_array().into();

        self.bvh.point_traverse(point, |bvh, node_index| {
            let node = &bvh.nodes[node_index as usize];
            let start = node.first_index as usize;
            let end = start + node.prim_count as usize;

            for primitive_id in start..end {
                let proxy_id = ProxyId::new(bvh.primitive_indices[primitive_id]);
                if !eval(proxy_id) {
                    return false;
                }
            }

            true
        });
    }

    /// Traverse the BVH with an AABB, calling `eval` for each intersection.
    ///
    /// # Arguments
    ///
    /// - `aabb`: The axis-aligned bounding box to be tested for intersection.
    /// - `eval`: A function that takes a proxy ID and is called for each potential intersection found during traversal.
    ///   Return false to halt traversal early.
    #[inline(always)]
    pub fn aabb_traverse<F: FnMut(ProxyId) -> bool>(&self, aabb: Aabb, mut eval: F) {
        self.bvh.aabb_traverse(aabb, |bvh, node_index| {
            let node = &bvh.nodes[node_index as usize];
            let start = node.first_index as usize;
            let end = start + node.prim_count as usize;

            for primitive_id in start..end {
                let proxy_id = ProxyId::new(bvh.primitive_indices[primitive_id]);
                if !eval(proxy_id) {
                    return false;
                }
            }

            true
        });
    }
}
