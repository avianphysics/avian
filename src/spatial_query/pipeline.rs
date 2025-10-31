use crate::{collision::collider::QueryShapeCastHit, prelude::*};
use bevy::prelude::*;
use obvhs::{
    BvhBuildParams,
    aabb::Aabb,
    bvh2::{Bvh2, builder::build_bvh2},
    ray::RayHit,
};

use spatial_query::bvh_ext::{BvhExt, PointTraversal};

/// A proxy for an entity used for the [`SpatialQueryPipeline`]
#[derive(Clone)]
pub struct BvhProxy {
    /// The entity the proxy is for
    pub entity: Entity,
    /// The collision layers of the entity
    pub layers: CollisionLayers,
    /// The AABB of the entity
    pub aabb: ColliderAabb,
}

#[cfg(feature = "3d")]
impl obvhs::Boundable for BvhProxy {
    fn aabb(&self) -> obvhs::aabb::Aabb {
        obvhs::aabb::Aabb {
            min: self.aabb.min.into(),
            max: self.aabb.max.into(),
        }
    }
}

#[cfg(feature = "2d")]
impl obvhs::Boundable for BvhProxy {
    fn aabb(&self) -> obvhs::aabb::Aabb {
        obvhs::aabb::Aabb {
            min: self.aabb.min.extend(-1.).into(),
            max: self.aabb.max.extend(1.).into(),
        }
    }
}

/// A resource for the spatial query pipeline.
///
/// The pipeline maintains a quaternary bounding volume hierarchy `Qbvh` of the world's colliders
/// as an acceleration structure for spatial queries.
#[derive(Resource, Clone)]
pub struct SpatialQueryPipeline {
    pub(crate) bvh: Bvh2,
    pub(crate) proxies: Vec<BvhProxy>,
}

impl Default for SpatialQueryPipeline {
    fn default() -> Self {
        Self {
            bvh: Bvh2::default(),
            proxies: Vec::default(),
        }
    }
}

impl SpatialQueryPipeline {
    /// Creates a new [`SpatialQueryPipeline`].
    pub fn new() -> SpatialQueryPipeline {
        SpatialQueryPipeline::default()
    }

    /// Get the BVH of the pipeline
    pub fn bvh(&self) -> &Bvh2 {
        &self.bvh
    }

    /// Get a specific proxy using an index returned from the BVH
    pub fn get_proxy(&self, bvh_idx: usize) -> &BvhProxy {
        &self.proxies[self.bvh.primitive_indices[bvh_idx] as usize]
    }

    /// Updates the associated acceleration structures with a new set of entities.
    pub fn update<'a>(
        &mut self,
        colliders: impl Iterator<Item = (Entity, Option<&'a CollisionLayers>, &'a ColliderAabb)>,
    ) {
        let colliders = colliders
            .filter(|&(_, _, &aabb)| aabb != ColliderAabb::INVALID)
            .map(|(entity, layers, &aabb)| BvhProxy {
                entity,
                layers: layers.map_or(CollisionLayers::default(), |layers| *layers),
                aabb,
            })
            .collect();

        self.update_internal(colliders);
    }

    fn update_internal(&mut self, proxies: Vec<BvhProxy>) {
        self.bvh = build_bvh2(
            &proxies,
            BvhBuildParams::fastest_build(),
            &mut core::time::Duration::default(),
        );

        self.proxies = proxies;
    }

    /// Casts a [ray](spatial_query#raycasting) and computes the closest [hit](RayHitData) with a collider.
    /// If there are no hits, `None` is returned.
    ///
    /// # Arguments
    ///
    /// - `origin`: Where the ray is cast from.
    /// - `direction`: What direction the ray is cast in.
    /// - `max_distance`: The maximum distance the ray can travel.
    /// - `filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query.
    /// - `f`: A function called on each entity hit by the ray. The ray keeps travelling until a valid distance is returned.
    ///
    /// # Related Methods
    ///
    /// - [`SpatialQueryPipeline::cast_ray`]
    /// - [`SpatialQueryPipeline::ray_hits`]
    pub fn cast_ray(
        &self,
        origin: Vector,
        direction: Dir,
        max_distance: Scalar,
        filter: &SpatialQueryFilter,
        mut f: impl FnMut(Entity, obvhs::ray::Ray) -> f32,
    ) -> Option<(Entity, f32)> {
        if self.bvh.nodes.is_empty() {
            return None;
        }

        #[cfg(feature = "2d")]
        let origin = origin.extend(0.);
        #[cfg(feature = "2d")]
        let direction = Dir3::new_unchecked(direction.extend(0.));
        let ray = obvhs::ray::Ray::new(origin.into(), (*direction).into(), 0., max_distance);

        let mut hit = RayHit::none();
        if !self.bvh.ray_traverse(ray, &mut hit, |&ray, idx| {
            let proxy = self.get_proxy(idx);
            if proxy.layers.memberships & filter.mask == LayerMask::NONE
                || filter.excluded_entities.contains(&proxy.entity)
            {
                return f32::INFINITY;
            }

            f(proxy.entity, ray)
        }) {
            return None;
        }

        let entity = self.get_proxy(hit.primitive_id as usize).entity;
        Some((entity, hit.t))
    }

    /// Casts a [ray](spatial_query#raycasting) and computes all [hits](RayHitData), calling the given `callback`
    /// for each hit. The raycast stops when `callback` returns false or all hits have been found.
    ///
    /// Note that the order of the results is not guaranteed.
    ///
    /// # Arguments
    ///
    /// - `origin`: Where the ray is cast from.
    /// - `direction`: What direction the ray is cast in.
    /// - `max_distance`: The maximum distance the ray can travel.
    /// - `filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query.
    /// - `callback`: A callback function called for each hit.
    ///
    /// # Related Methods
    ///
    /// - [`SpatialQueryPipeline::cast_ray`]
    /// - [`SpatialQueryPipeline::cast_ray_predicate`]
    pub fn ray_hits(
        &self,
        origin: Vector,
        direction: Dir,
        max_distance: Scalar,
        filter: &SpatialQueryFilter,
        mut f: impl FnMut(Entity, obvhs::ray::Ray) -> bool,
    ) {
        if self.bvh.nodes.is_empty() {
            return;
        }

        #[cfg(feature = "2d")]
        let origin = origin.extend(0.);
        #[cfg(feature = "2d")]
        let direction = Dir3::new_unchecked(direction.extend(0.));
        let ray = obvhs::ray::Ray::new(origin.into(), (*direction).into(), 0., max_distance);

        let mut traversal = self.bvh.new_ray_traversal(ray);
        let mut hit = RayHit::none();
        let mut c = true;
        while c
            && self
                .bvh
                .ray_traverse_dynamic(&mut traversal, &mut hit, |&ray, idx| {
                    let proxy = self.get_proxy(idx);
                    if proxy.layers.memberships & filter.mask == LayerMask::NONE
                        || filter.excluded_entities.contains(&proxy.entity)
                    {
                        return f32::INFINITY;
                    }

                    c = f(proxy.entity, ray);
                    return f32::INFINITY;
                })
        {}
    }

    /// Casts a [shape](spatial_query#shapecasting) with a given rotation and computes the closest [hit](ShapeHits)
    /// with a collider. If there are no hits, `None` is returned.
    ///
    /// For a more ECS-based approach, consider using the [`ShapeCaster`] component instead.
    ///
    /// # Arguments
    ///
    /// - `shape`: The shape being cast represented as a [`Collider`].
    /// - `origin`: Where the shape is cast from.
    /// - `shape_rotation`: The rotation of the shape being cast.
    /// - `direction`: What direction the shape is cast in.
    /// - `config`: A [`ShapeCastConfig`] that determines the behavior of the cast.
    /// - `filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query.
    ///
    /// # Related Methods
    ///
    /// - [`SpatialQueryPipeline::cast_shape_predicate`]
    /// - [`SpatialQueryPipeline::shape_hits`]
    /// - [`SpatialQueryPipeline::shape_hits_callback`]
    #[allow(clippy::too_many_arguments)]
    pub fn cast_aabb(
        &self,
        shape: ColliderAabb,
        origin: Vector,
        direction: Dir,
        config: &ShapeCastConfig,
        filter: &SpatialQueryFilter,
        mut f: impl FnMut(Entity, obvhs::ray::Ray) -> Option<QueryShapeCastHit>,
    ) -> Option<ShapeCastHit> {
        if self.bvh.nodes.is_empty() {
            return None;
        }

        #[cfg(feature = "2d")]
        let shape = Aabb {
            min: shape.min.extend(0.).into(),
            max: shape.max.extend(0.).into(),
        };
        #[cfg(feature = "3d")]
        let shape = Aabb {
            min: shape.min.into(),
            max: shape.max.into(),
        };

        #[cfg(feature = "2d")]
        let origin = origin.extend(0.);
        #[cfg(feature = "2d")]
        let direction = Dir3::new_unchecked(direction.extend(0.));
        let ray = obvhs::ray::Ray::new(origin.into(), (*direction).into(), 0., config.max_distance);

        let mut hit = RayHit::none();
        let mut shape_hit = None::<ShapeCastHit>;
        if !self.bvh.cast_traverse(ray, &mut hit, shape, |&ray, idx| {
            let proxy = self.get_proxy(idx);

            if proxy.layers.memberships & filter.mask == LayerMask::NONE
                || filter.excluded_entities.contains(&proxy.entity)
            {
                return f32::INFINITY;
            }

            let hit = f(proxy.entity, ray);
            if let Some(hit) = hit {
                shape_hit = Some(ShapeCastHit {
                    entity: proxy.entity,
                    distance: hit.distance,
                    point: hit.point,
                    normal: hit.normal,
                });
            }

            hit.map(|hit| hit.distance).unwrap_or(f32::INFINITY)
        }) {
            return None;
        }

        shape_hit
    }

    /// Casts a [shape](spatial_query#shapecasting) with a given rotation and computes the closest [hit](ShapeHits)
    /// with a collider. If there are no hits, `None` is returned.
    ///
    /// For a more ECS-based approach, consider using the [`ShapeCaster`] component instead.
    ///
    /// # Arguments
    ///
    /// - `shape`: The shape being cast represented as a [`Collider`].
    /// - `origin`: Where the shape is cast from.
    /// - `shape_rotation`: The rotation of the shape being cast.
    /// - `direction`: What direction the shape is cast in.
    /// - `config`: A [`ShapeCastConfig`] that determines the behavior of the cast.
    /// - `filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query.
    /// - `predicate`: A function called on each entity hit by the shape. The shape keeps travelling until the predicate returns `true`.
    ///
    /// # Related Methods
    ///
    /// - [`SpatialQueryPipeline::cast_shape`]
    /// - [`SpatialQueryPipeline::shape_hits`]
    /// - [`SpatialQueryPipeline::shape_hits_callback`]
    #[allow(clippy::too_many_arguments)]
    pub fn shape_hits(
        &self,
        shape: ColliderAabb,
        origin: Vector,
        direction: Dir,
        config: &ShapeCastConfig,
        filter: &SpatialQueryFilter,
        mut f: impl FnMut(Entity, obvhs::ray::Ray) -> bool,
    ) {
        if self.bvh.nodes.is_empty() {
            return;
        }

        #[cfg(feature = "2d")]
        let shape = Aabb {
            min: shape.min.extend(0.).into(),
            max: shape.max.extend(0.).into(),
        };
        #[cfg(feature = "3d")]
        let shape = Aabb {
            min: shape.min.into(),
            max: shape.max.into(),
        };

        #[cfg(feature = "2d")]
        let origin = origin.extend(0.);
        #[cfg(feature = "2d")]
        let direction = Dir3::new_unchecked(direction.extend(0.));
        let ray = obvhs::ray::Ray::new(origin.into(), (*direction).into(), 0., config.max_distance);

        let mut traversal = self.bvh.new_ray_traversal(ray);
        let mut hit = RayHit::none();
        let mut c = true;
        while c
            && self
                .bvh
                .cast_traverse_dynamic(&mut traversal, &mut hit, shape, |&ray, idx| {
                    let proxy = self.get_proxy(idx);
                    if proxy.layers.memberships & filter.mask == LayerMask::NONE
                        || filter.excluded_entities.contains(&proxy.entity)
                    {
                        return f32::INFINITY;
                    }

                    c = f(proxy.entity, ray);
                    f32::INFINITY
                })
        {}
    }

    /// Finds the [projection](spatial_query#point-projection) of a given point on the closest [collider](Collider).
    /// If one isn't found, `None` is returned.
    ///
    /// # Arguments
    ///
    /// - `point`: The point that should be projected.
    /// - `solid`: If true and the point is inside of a collider, the projection will be at the point.
    ///   Otherwise, the collider will be treated as hollow, and the projection will be at the collider's boundary.
    /// - `filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query.
    ///
    /// # Related Methods
    ///
    /// - [`SpatialQueryPipeline::project_point_predicate`]
    pub fn project_point(
        &self,
        point: Vector,
        filter: &SpatialQueryFilter,
        mut f: impl FnMut(Entity, Vector) -> f32,
    ) -> Option<PointProjection> {
        if self.bvh.nodes.is_empty() {
            return None;
        }

        #[cfg(feature = "2d")]
        let point = point.extend(0.).into();
        #[cfg(feature = "3d")]
        let point = point.into();

        let mut hit = RayHit::none();
        self.bvh.dist_traverse(point, &mut hit, |pos, idx| {
            let proxy = self.get_proxy(idx);

            if proxy.layers.memberships & filter.mask == LayerMask::NONE
                || filter.excluded_entities.contains(&proxy.entity)
            {
                return f32::INFINITY;
            }

            #[cfg(feature = "2d")]
            let pos = pos.xy();
            #[cfg(feature = "3d")]
            let pos = (*pos).into();

            f(proxy.entity, pos)
        });

        if hit.t == f32::INFINITY {
            return None;
        }

        let entity = self.get_proxy(hit.primitive_id as usize).entity;
        Some(PointProjection {
            entity,
            is_inside: hit.t.is_sign_negative(),
        })
    }

    /// An [intersection test](spatial_query#intersection-tests) that finds all entities with a [collider](Collider)
    /// that contains the given point.
    ///
    /// # Arguments
    ///
    /// - `point`: The point that intersections are tested against.
    /// - `filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query.
    ///
    /// # Related Methods
    ///
    /// - [`SpatialQueryPipeline::point_intersections_callback`]
    pub fn point_intersections(
        &self,
        point: Vector,
        filter: &SpatialQueryFilter,
        mut f: impl FnMut(Entity, Vector) -> (bool, bool),
    ) {
        if self.bvh.nodes.is_empty() {
            return;
        }

        #[cfg(feature = "2d")]
        let pos = point.extend(0.).into();
        #[cfg(feature = "3d")]
        let pos = point.into();

        let mut state = PointTraversal::new(&self.bvh, pos);
        let mut hit = RayHit::none();
        let mut c = true;
        while c
            && self
                .bvh
                .point_traverse_dynamic(&mut state, &mut hit, |pos, idx| {
                    let proxy = self.get_proxy(idx);

                    if proxy.layers.memberships & filter.mask == LayerMask::NONE
                        || filter.excluded_entities.contains(&proxy.entity)
                    {
                        return false;
                    }

                    #[cfg(feature = "2d")]
                    let pos = pos.xy();
                    #[cfg(feature = "3d")]
                    let pos = (*pos).into();

                    let hit: bool;
                    (hit, c) = f(proxy.entity, pos);
                    hit
                })
        {}
    }

    /// An [intersection test](spatial_query#intersection-tests) that finds all entities with a [`ColliderAabb`]
    /// that is intersecting the given `aabb`.
    ///
    /// # Related Methods
    ///
    /// - [`SpatialQueryPipeline::aabb_intersections_with_aabb_callback`]
    pub fn aabb_intersections_with_aabb(
        &self,
        aabb: ColliderAabb,
        filter: &SpatialQueryFilter,
        mut f: impl FnMut(Entity) -> bool,
    ) {
        if self.bvh.nodes.is_empty() {
            return;
        }

        #[cfg(feature = "2d")]
        let aabb = Aabb {
            min: aabb.min.extend(0.).into(),
            max: aabb.max.extend(0.).into(),
        };
        #[cfg(feature = "3d")]
        let aabb = Aabb {
            min: aabb.min.into(),
            max: aabb.max.into(),
        };

        self.bvh.aabb_traverse(aabb, |_, idx| {
            let first = self.bvh.nodes[idx as usize].first_index;
            let n = self.bvh.nodes[idx as usize].prim_count;
            for idx in first..(first + n) {
                let proxy = self.get_proxy(idx as usize);
                if proxy.layers.memberships & filter.mask == LayerMask::NONE
                    || filter.excluded_entities.contains(&proxy.entity)
                {
                    continue;
                }

                if !f(proxy.entity) {
                    return false;
                }
            }
            true
        })
    }
}

/// The result of a [point projection](spatial_query#point-projection) on a [collider](Collider).
#[derive(Clone, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct PointProjection {
    /// The entity of the collider that the point was projected onto.
    pub entity: Entity,
    /// True if the point was inside of the collider.
    pub is_inside: bool,
}
