use obvhs::aabb::Aabb;

use crate::{
    collider_tree::{
        Bvh2Ext, ColliderTree, ProxyId,
        obvhs_ext::{Sweep, SweepHit},
        obvhs_ray,
    },
    math::{Dir, Ray, Vector},
};

impl ColliderTree {
    /// Traverses the tree for the closest intersection with the given ray.
    ///
    /// # Arguments
    ///
    /// - `ray`: The ray to be tested for intersection.
    /// - `max_distance`: The maximum distance along the ray to consider for intersections.
    /// - `intersection_fn`: A function that takes a ray and a proxy ID, and returns the distance to the intersection with that proxy.
    ///   This function is called for each potential intersection found during traversal.
    #[inline(always)]
    pub fn ray_traverse_closest<F: FnMut(&Ray, ProxyId) -> f32>(
        &self,
        ray: Ray,
        max_distance: f32,
        mut intersection_fn: F,
    ) -> Option<(ProxyId, f32)> {
        let obvhs_ray = obvhs_ray(&ray, max_distance);
        let mut hit = obvhs::ray::RayHit::none();

        let found_hit = self
            .bvh
            .ray_traverse(obvhs_ray, &mut hit, |_ray, primitive_id| {
                let proxy_id = ProxyId::new(self.bvh.primitive_indices[primitive_id]);
                intersection_fn(&ray, proxy_id)
            });

        if found_hit {
            let proxy_id = ProxyId::new(self.bvh.primitive_indices[hit.primitive_id as usize]);
            Some((proxy_id, hit.t))
        } else {
            None
        }
    }

    /// Traverses the tree for all intersections with the given ray.
    ///
    /// # Arguments
    ///
    /// - `ray`: The ray to be tested for intersection.
    /// - `max_distance`: The maximum distance along the ray to consider for intersections.
    /// - `intersection_fn`: A function that takes a ray and a proxy ID, and is called for each potential intersection found during traversal.
    #[inline(always)]
    pub fn ray_traverse_all<F: FnMut(&Ray, ProxyId)>(
        &self,
        ray: Ray,
        max_distance: f32,
        mut intersection_fn: F,
    ) {
        let obvhs_ray = obvhs_ray(&ray, max_distance);

        self.bvh
            .ray_traverse_anyhit(obvhs_ray, |_ray, primitive_id| {
                let proxy_id = ProxyId::new(self.bvh.primitive_indices[primitive_id]);
                intersection_fn(&ray, proxy_id);
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
    /// - `intersection_fn`: A function that takes a sweep and a proxy ID, and returns the distance to the intersection with that proxy.
    ///   This function is called for each potential intersection found during traversal.
    #[inline(always)]
    pub fn sweep_traverse_closest<F: FnMut(&Sweep, ProxyId) -> f32>(
        &self,
        aabb: Aabb,
        direction: Dir,
        max_distance: f32,
        target_distance: f32,
        mut intersection_fn: F,
    ) -> Option<(ProxyId, f32)> {
        #[cfg(feature = "2d")]
        let direction = direction.extend(0.0).to_array().into();
        #[cfg(feature = "3d")]
        let direction = direction.to_array().into();
        let sweep = Sweep::new(aabb, direction, target_distance, max_distance);

        let mut hit = SweepHit::none();

        let found_hit = self
            .bvh
            .sweep_traverse(sweep, &mut hit, |sweep, primitive_id| {
                let proxy_id = ProxyId::new(self.bvh.primitive_indices[primitive_id]);
                intersection_fn(sweep, proxy_id)
            });

        if found_hit {
            let proxy_id = ProxyId::new(self.bvh.primitive_indices[hit.primitive_id as usize]);
            Some((proxy_id, hit.t))
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
    /// - `intersection_fn`: A function that takes a sweep and a proxy ID, and is called for each potential intersection found during traversal.
    #[inline(always)]
    pub fn sweep_traverse_all<F: FnMut(&Sweep, ProxyId)>(
        &self,
        aabb: Aabb,
        direction: Dir,
        target_distance: f32,
        max_distance: f32,
        mut intersection_fn: F,
    ) {
        #[cfg(feature = "2d")]
        let direction = direction.extend(0.0).to_array().into();
        #[cfg(feature = "3d")]
        let direction = direction.to_array().into();
        let sweep = Sweep::new(aabb, direction, target_distance, max_distance);

        self.bvh
            .sweep_traverse_anyhit(sweep, |sweep, primitive_id| {
                let proxy_id = ProxyId::new(self.bvh.primitive_indices[primitive_id]);
                intersection_fn(sweep, proxy_id);
            });
    }

    /// Traverse the BVH with a point, returning the closest projection within `max_distance`.
    ///
    /// # Arguments
    ///
    /// - `point`: The point to be tested for proximity.
    /// - `max_distance`: The maximum distance from the point to consider for projections.
    /// - `eval`: A function that takes a proxy ID and returns the distance from the point to that proxy. This function is called for each potential projection found during traversal.
    #[inline(always)]
    pub fn point_projection_traverse_closest<F: FnMut(ProxyId) -> f32>(
        &self,
        point: Vector,
        max_distance: f32,
        mut eval: F,
    ) -> Option<(ProxyId, f32)> {
        let max_distance_squared = max_distance * max_distance;
        #[cfg(feature = "2d")]
        let point = point.extend(0.0).to_array().into();
        #[cfg(feature = "3d")]
        let point = point.to_array().into();

        let closest_leaf =
            self.bvh
                .distance_traverse(point, max_distance_squared, |_point, primitive_id| {
                    let proxy_id = ProxyId::new(self.bvh.primitive_indices[primitive_id]);
                    eval(proxy_id)
                });

        if let Some((primitive_id, distance_squared)) = closest_leaf {
            let proxy_id = ProxyId::new(self.bvh.primitive_indices[primitive_id as usize]);
            Some((proxy_id, distance_squared.sqrt().max(0.0)))
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
    #[inline(always)]
    pub fn point_traverse<F: FnMut(&Self, ProxyId) -> bool>(&self, point: Vector, mut eval: F) {
        #[cfg(feature = "2d")]
        let point = point.extend(0.0).to_array().into();
        #[cfg(feature = "3d")]
        let point = point.to_array().into();

        self.bvh.point_traverse(point, |bvh, node_index| {
            let node = &bvh.nodes[node_index as usize];
            let start = node.first_index as usize;
            let end = start + node.prim_count as usize;

            for primitive_id in start..end {
                let proxy_id = ProxyId::new(bvh.primitive_indices[primitive_id]);
                if !eval(self, proxy_id) {
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
    #[inline(always)]
    pub fn aabb_traverse<F: FnMut(&Self, ProxyId) -> bool>(&self, aabb: Aabb, mut eval: F) {
        self.bvh.aabb_traverse(aabb, |bvh, node_index| {
            let node = &bvh.nodes[node_index as usize];
            let start = node.first_index as usize;
            let end = start + node.prim_count as usize;

            for primitive_id in start..end {
                let proxy_id = ProxyId::new(bvh.primitive_indices[primitive_id]);
                if !eval(self, proxy_id) {
                    return false;
                }
            }

            true
        });
    }
}
