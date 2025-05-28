use alloc::sync::Arc;

use crate::prelude::*;
use bevy::prelude::*;
use parry::{
    bounding_volume::Aabb,
    math::Isometry,
    partitioning::Qbvh,
    query::{
        details::{
            NormalConstraints, RayCompositeShapeToiAndNormalBestFirstVisitor,
            TOICompositeShapeShapeBestFirstVisitor,
        },
        point::PointCompositeShapeProjBestFirstVisitor,
        visitors::{
            BoundingVolumeIntersectionsVisitor, PointIntersectionsVisitor, RayIntersectionsVisitor,
        },
        DefaultQueryDispatcher, QueryDispatcher, ShapeCastOptions,
    },
    shape::{Shape, TypedSimdCompositeShape},
};

// TODO: It'd be nice not to store so much duplicate data.
//       Should we just query the ECS?
#[derive(Clone)]
pub(crate) struct QbvhProxyData {
    pub entity: Entity,
    pub isometry: Isometry<Scalar>,
    pub collider: Collider,
    pub layers: CollisionLayers,
}

/// A resource for the spatial query pipeline.
///
/// The pipeline maintains a quaternary bounding volume hierarchy `Qbvh` of the world's colliders
/// as an acceleration structure for spatial queries.
#[derive(Resource, Clone)]
pub struct SpatialQueryPipeline {
    pub(crate) qbvh: Qbvh<u32>,
    pub(crate) dispatcher: Arc<dyn QueryDispatcher>,
    // TODO: Store the proxies as `Qbvh` leaf data.
    pub(crate) proxies: Vec<QbvhProxyData>,
}

impl Default for SpatialQueryPipeline {
    fn default() -> Self {
        Self {
            qbvh: Qbvh::new(),
            dispatcher: Arc::new(DefaultQueryDispatcher),
            proxies: Vec::default(),
        }
    }
}

impl SpatialQueryPipeline {
    /// Creates a new [`SpatialQueryPipeline`].
    pub fn new() -> SpatialQueryPipeline {
        SpatialQueryPipeline::default()
    }

    pub(crate) fn as_composite_shape<'a>(
        &'a self,
        query_filter: &'a SpatialQueryFilter,
    ) -> QueryPipelineAsCompositeShape<'a> {
        QueryPipelineAsCompositeShape {
            pipeline: self,
            proxies: &self.proxies,
            query_filter,
        }
    }

    pub(crate) fn as_composite_shape_with_predicate<'a: 'b, 'b>(
        &'a self,
        query_filter: &'a SpatialQueryFilter,
        predicate: &'a dyn Fn(Entity) -> bool,
    ) -> QueryPipelineAsCompositeShapeWithPredicate<'a, 'b> {
        QueryPipelineAsCompositeShapeWithPredicate {
            pipeline: self,
            proxies: &self.proxies,
            query_filter,
            predicate,
        }
    }

    /// Updates the associated acceleration structures with a new set of entities.
    pub fn update<'a>(
        &mut self,
        colliders: impl Iterator<
            Item = (
                Entity,
                &'a Position,
                &'a Rotation,
                &'a Collider,
                &'a CollisionLayers,
            ),
        >,
    ) {
        self.update_internal(
            colliders.map(
                |(entity, position, rotation, collider, layers)| QbvhProxyData {
                    entity,
                    isometry: make_isometry(position.0, *rotation),
                    collider: collider.clone(),
                    layers: *layers,
                },
            ),
        )
    }

    // TODO: Incremental updates.
    fn update_internal(&mut self, proxies: impl Iterator<Item = QbvhProxyData>) {
        self.proxies.clear();
        self.proxies.extend(proxies);

        struct DataGenerator<'a>(&'a Vec<QbvhProxyData>);

        impl parry::partitioning::QbvhDataGenerator<u32> for DataGenerator<'_> {
            fn size_hint(&self) -> usize {
                self.0.len()
            }

            #[inline(always)]
            fn for_each(&mut self, mut f: impl FnMut(u32, parry::bounding_volume::Aabb)) {
                for (i, proxy) in self.0.iter().enumerate() {
                    // Compute and return AABB
                    let aabb = proxy.collider.shape_scaled().compute_aabb(&proxy.isometry);
                    f(i as u32, aabb)
                }
            }
        }

        self.qbvh
            .clear_and_rebuild(DataGenerator(&self.proxies), 0.01);
    }

    /// Casts a [ray](spatial_query#raycasting) and computes the closest [hit](RayHitData) with a collider.
    /// If there are no hits, `None` is returned.
    ///
    /// # Arguments
    ///
    /// - `origin`: Where the ray is cast from.
    /// - `direction`: What direction the ray is cast in.
    /// - `max_distance`: The maximum distance the ray can travel.
    /// - `solid`: If true *and* the ray origin is inside of a collider, the hit point will be the ray origin itself.
    ///   Otherwise, the collider will be treated as hollow, and the hit point will be at its boundary.
    /// - `filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query.
    ///
    /// # Related Methods
    ///
    /// - [`SpatialQueryPipeline::cast_ray_predicate`]
    /// - [`SpatialQueryPipeline::ray_hits`]
    /// - [`SpatialQueryPipeline::ray_hits_callback`]
    pub fn cast_ray(
        &self,
        origin: Vector,
        direction: Dir,
        max_distance: Scalar,
        solid: bool,
        filter: &SpatialQueryFilter,
    ) -> Option<RayHitData> {
        self.cast_ray_predicate(origin, direction, max_distance, solid, filter, &|_| true)
    }

    /// Casts a [ray](spatial_query#raycasting) and computes the closest [hit](RayHitData) with a collider.
    /// If there are no hits, `None` is returned.
    ///
    /// # Arguments
    ///
    /// - `origin`: Where the ray is cast from.
    /// - `direction`: What direction the ray is cast in.
    /// - `max_distance`: The maximum distance the ray can travel.
    /// - `solid`: If true *and* the ray origin is inside of a collider, the hit point will be the ray origin itself.
    ///   Otherwise, the collider will be treated as hollow, and the hit point will be at its boundary.
    /// - `predicate`: A function called on each entity hit by the ray. The ray keeps travelling until the predicate returns `true`.
    /// - `filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query.
    ///
    /// # Related Methods
    ///
    /// - [`SpatialQueryPipeline::cast_ray`]
    /// - [`SpatialQueryPipeline::ray_hits`]
    /// - [`SpatialQueryPipeline::ray_hits_callback`]
    pub fn cast_ray_predicate(
        &self,
        origin: Vector,
        direction: Dir,
        max_distance: Scalar,
        solid: bool,
        filter: &SpatialQueryFilter,
        predicate: &dyn Fn(Entity) -> bool,
    ) -> Option<RayHitData> {
        let pipeline_shape = self.as_composite_shape_with_predicate(filter, predicate);
        let ray = parry::query::Ray::new(origin.into(), direction.adjust_precision().into());
        let mut visitor = RayCompositeShapeToiAndNormalBestFirstVisitor::new(
            &pipeline_shape,
            &ray,
            max_distance,
            solid,
        );

        self.qbvh
            .traverse_best_first(&mut visitor)
            .map(|(_, (index, hit))| RayHitData {
                entity: self.proxies[index as usize].entity,
                distance: hit.time_of_impact,
                normal: hit.normal.into(),
            })
    }

    /// Casts a [ray](spatial_query#raycasting) and computes all [hits](RayHitData) until `max_hits` is reached.
    ///
    /// Note that the order of the results is not guaranteed, and if there are more hits than `max_hits`,
    /// some hits will be missed.
    ///
    /// # Arguments
    ///
    /// - `origin`: Where the ray is cast from.
    /// - `direction`: What direction the ray is cast in.
    /// - `max_distance`: The maximum distance the ray can travel.
    /// - `max_hits`: The maximum number of hits. Additional hits will be missed.
    /// - `solid`: If true *and* the ray origin is inside of a collider, the hit point will be the ray origin itself.
    ///   Otherwise, the collider will be treated as hollow, and the hit point will be at its boundary.
    /// - `filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query.
    ///
    /// # Related Methods
    ///
    /// - [`SpatialQueryPipeline::cast_ray`]
    /// - [`SpatialQueryPipeline::cast_ray_predicate`]
    /// - [`SpatialQueryPipeline::ray_hits_callback`]
    pub fn ray_hits(
        &self,
        origin: Vector,
        direction: Dir,
        max_distance: Scalar,
        max_hits: u32,
        solid: bool,
        filter: &SpatialQueryFilter,
    ) -> Vec<RayHitData> {
        let mut hits = Vec::with_capacity(10);
        self.ray_hits_callback(origin, direction, max_distance, solid, filter, |hit| {
            hits.push(hit);
            (hits.len() as u32) < max_hits
        });
        hits
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
    /// - `solid`: If true *and* the ray origin is inside of a collider, the hit point will be the ray origin itself.
    ///   Otherwise, the collider will be treated as hollow, and the hit point will be at its boundary.
    /// - `filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query.
    /// - `callback`: A callback function called for each hit.
    ///
    /// # Related Methods
    ///
    /// - [`SpatialQueryPipeline::cast_ray`]
    /// - [`SpatialQueryPipeline::cast_ray_predicate`]
    /// - [`SpatialQueryPipeline::ray_hits`]
    pub fn ray_hits_callback(
        &self,
        origin: Vector,
        direction: Dir,
        max_distance: Scalar,
        solid: bool,
        filter: &SpatialQueryFilter,
        mut callback: impl FnMut(RayHitData) -> bool,
    ) {
        let proxies = &self.proxies;

        let ray = parry::query::Ray::new(origin.into(), direction.adjust_precision().into());

        let mut leaf_callback = &mut |index: &u32| {
            if let Some(proxy) = proxies.get(*index as usize) {
                if filter.test(proxy.entity, proxy.layers) {
                    if let Some(hit) = proxy.collider.shape_scaled().cast_ray_and_get_normal(
                        &proxy.isometry,
                        &ray,
                        max_distance,
                        solid,
                    ) {
                        let hit = RayHitData {
                            entity: proxy.entity,
                            distance: hit.time_of_impact,
                            normal: hit.normal.into(),
                        };

                        return callback(hit);
                    }
                }
            }
            true
        };

        let mut visitor = RayIntersectionsVisitor::new(&ray, max_distance, &mut leaf_callback);
        self.qbvh.traverse_depth_first(&mut visitor);
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
    pub fn cast_shape(
        &self,
        shape: &Collider,
        origin: Vector,
        shape_rotation: RotationValue,
        direction: Dir,
        config: &ShapeCastConfig,
        filter: &SpatialQueryFilter,
    ) -> Option<ShapeHitData> {
        self.cast_shape_predicate(
            shape,
            origin,
            shape_rotation,
            direction,
            config,
            filter,
            &|_| true,
        )
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
    pub fn cast_shape_predicate(
        &self,
        shape: &Collider,
        origin: Vector,
        shape_rotation: RotationValue,
        direction: Dir,
        config: &ShapeCastConfig,
        filter: &SpatialQueryFilter,
        predicate: &dyn Fn(Entity) -> bool,
    ) -> Option<ShapeHitData> {
        let rotation: Rotation;
        #[cfg(feature = "2d")]
        {
            rotation = Rotation::radians(shape_rotation);
        }
        #[cfg(feature = "3d")]
        {
            rotation = Rotation::from(shape_rotation);
        }

        let shape_isometry = make_isometry(origin, rotation);
        let shape_direction = direction.adjust_precision().into();
        let pipeline_shape = self.as_composite_shape_with_predicate(filter, predicate);
        let mut visitor = TOICompositeShapeShapeBestFirstVisitor::new(
            &*self.dispatcher,
            &shape_isometry,
            &shape_direction,
            &pipeline_shape,
            shape.shape_scaled().as_ref(),
            ShapeCastOptions {
                max_time_of_impact: config.max_distance,
                stop_at_penetration: !config.ignore_origin_penetration,
                compute_impact_geometry_on_penetration: config.compute_contact_on_penetration,
                ..default()
            },
        );

        self.qbvh
            .traverse_best_first(&mut visitor)
            .map(|(_, (index, hit))| ShapeHitData {
                entity: self.proxies[index as usize].entity,
                distance: hit.time_of_impact,
                point1: hit.witness1.into(),
                point2: hit.witness2.into(),
                normal1: hit.normal1.into(),
                normal2: hit.normal2.into(),
            })
    }

    /// Casts a [shape](spatial_query#shapecasting) with a given rotation and computes computes all [hits](ShapeHitData)
    /// in the order of distance until `max_hits` is reached.
    ///
    /// # Arguments
    ///
    /// - `shape`: The shape being cast represented as a [`Collider`].
    /// - `origin`: Where the shape is cast from.
    /// - `shape_rotation`: The rotation of the shape being cast.
    /// - `direction`: What direction the shape is cast in.
    /// - `max_hits`: The maximum number of hits. Additional hits will be missed.
    /// - `config`: A [`ShapeCastConfig`] that determines the behavior of the cast.
    /// - `filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query.
    ///
    /// # Related Methods
    ///
    /// - [`SpatialQueryPipeline::cast_shape`]
    /// - [`SpatialQueryPipeline::cast_shape_predicate`]
    /// - [`SpatialQueryPipeline::shape_hits_callback`]
    #[allow(clippy::too_many_arguments)]
    pub fn shape_hits(
        &self,
        shape: &Collider,
        origin: Vector,
        shape_rotation: RotationValue,
        direction: Dir,
        max_hits: u32,
        config: &ShapeCastConfig,
        filter: &SpatialQueryFilter,
    ) -> Vec<ShapeHitData> {
        let mut hits = Vec::with_capacity(10);
        self.shape_hits_callback(
            shape,
            origin,
            shape_rotation,
            direction,
            config,
            filter,
            |hit| {
                hits.push(hit);
                (hits.len() as u32) < max_hits
            },
        );
        hits
    }

    /// Casts a [shape](spatial_query#shapecasting) with a given rotation and computes computes all [hits](ShapeHitData)
    /// in the order of distance, calling the given `callback` for each hit. The shapecast stops when
    /// `callback` returns false or all hits have been found.
    ///
    /// # Arguments
    ///
    /// - `shape`: The shape being cast represented as a [`Collider`].
    /// - `origin`: Where the shape is cast from.
    /// - `shape_rotation`: The rotation of the shape being cast.
    /// - `direction`: What direction the shape is cast in.
    /// - `config`: A [`ShapeCastConfig`] that determines the behavior of the cast.
    /// - `filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query.
    /// - `callback`: A callback function called for each hit.
    ///
    /// # Related Methods
    ///
    /// - [`SpatialQueryPipeline::cast_shape`]
    /// - [`SpatialQueryPipeline::cast_shape_predicate`]
    /// - [`SpatialQueryPipeline::shape_hits`]
    #[allow(clippy::too_many_arguments)]
    pub fn shape_hits_callback(
        &self,
        shape: &Collider,
        origin: Vector,
        shape_rotation: RotationValue,
        direction: Dir,
        config: &ShapeCastConfig,
        filter: &SpatialQueryFilter,
        mut callback: impl FnMut(ShapeHitData) -> bool,
    ) {
        // TODO: This clone is here so that the excluded entities in the original `query_filter` aren't modified.
        //       We could remove this if shapecasting could compute multiple hits without just doing casts in a loop.
        //       See https://github.com/Jondolf/avian/issues/403.
        let mut query_filter = filter.clone();

        let shape_cast_options = ShapeCastOptions {
            max_time_of_impact: config.max_distance,
            target_distance: config.target_distance,
            stop_at_penetration: !config.ignore_origin_penetration,
            compute_impact_geometry_on_penetration: config.compute_contact_on_penetration,
        };

        let rotation: Rotation;
        #[cfg(feature = "2d")]
        {
            rotation = Rotation::radians(shape_rotation);
        }
        #[cfg(feature = "3d")]
        {
            rotation = Rotation::from(shape_rotation);
        }

        let shape_isometry = make_isometry(origin, rotation);
        let shape_direction = direction.adjust_precision().into();

        loop {
            let pipeline_shape = self.as_composite_shape(&query_filter);
            let mut visitor = TOICompositeShapeShapeBestFirstVisitor::new(
                &*self.dispatcher,
                &shape_isometry,
                &shape_direction,
                &pipeline_shape,
                shape.shape_scaled().as_ref(),
                shape_cast_options,
            );

            if let Some(hit) =
                self.qbvh
                    .traverse_best_first(&mut visitor)
                    .map(|(_, (index, hit))| ShapeHitData {
                        entity: self.proxies[index as usize].entity,
                        distance: hit.time_of_impact,
                        point1: hit.witness1.into(),
                        point2: hit.witness2.into(),
                        normal1: hit.normal1.into(),
                        normal2: hit.normal2.into(),
                    })
            {
                query_filter.excluded_entities.insert(hit.entity);

                if !callback(hit) {
                    break;
                }
            } else {
                break;
            }
        }
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
        solid: bool,
        filter: &SpatialQueryFilter,
    ) -> Option<PointProjection> {
        self.project_point_predicate(point, solid, filter, &|_| true)
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
    /// - `predicate`: A function for filtering which entities are considered in the query. The projection will be on the closest collider for which the `predicate` returns `true`
    ///
    /// # Related Methods
    ///
    /// - [`SpatialQueryPipeline::project_point`]
    pub fn project_point_predicate(
        &self,
        point: Vector,
        solid: bool,
        filter: &SpatialQueryFilter,
        predicate: &dyn Fn(Entity) -> bool,
    ) -> Option<PointProjection> {
        let point = point.into();
        let pipeline_shape = self.as_composite_shape_with_predicate(filter, predicate);
        let mut visitor =
            PointCompositeShapeProjBestFirstVisitor::new(&pipeline_shape, &point, solid);

        self.qbvh
            .traverse_best_first(&mut visitor)
            .map(|(_, (projection, index))| PointProjection {
                entity: self.proxies[index as usize].entity,
                point: projection.point.into(),
                is_inside: projection.is_inside,
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
    pub fn point_intersections(&self, point: Vector, filter: &SpatialQueryFilter) -> Vec<Entity> {
        let mut intersections = vec![];
        self.point_intersections_callback(point, filter, |e| {
            intersections.push(e);
            true
        });
        intersections
    }

    /// An [intersection test](spatial_query#intersection-tests) that finds all entities with a [collider](Collider)
    /// that contains the given point, calling the given `callback` for each intersection.
    /// The search stops when `callback` returns `false` or all intersections have been found.
    ///
    /// # Arguments
    ///
    /// - `point`: The point that intersections are tested against.
    /// - `filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query.
    /// - `callback`: A callback function called for each intersection.
    ///
    /// # Related Methods
    ///
    /// - [`SpatialQueryPipeline::point_intersections`]
    pub fn point_intersections_callback(
        &self,
        point: Vector,
        filter: &SpatialQueryFilter,
        mut callback: impl FnMut(Entity) -> bool,
    ) {
        let point = point.into();

        let mut leaf_callback = &mut |index: &u32| {
            if let Some(proxy) = self.proxies.get(*index as usize) {
                if filter.test(proxy.entity, proxy.layers)
                    && proxy
                        .collider
                        .shape_scaled()
                        .contains_point(&proxy.isometry, &point)
                {
                    return callback(proxy.entity);
                }
            }
            true
        };

        let mut visitor = PointIntersectionsVisitor::new(&point, &mut leaf_callback);
        self.qbvh.traverse_depth_first(&mut visitor);
    }

    /// An [intersection test](spatial_query#intersection-tests) that finds all entities with a [`ColliderAabb`]
    /// that is intersecting the given `aabb`.
    ///
    /// # Related Methods
    ///
    /// - [`SpatialQueryPipeline::aabb_intersections_with_aabb_callback`]
    pub fn aabb_intersections_with_aabb(&self, aabb: ColliderAabb) -> Vec<Entity> {
        let mut intersections = vec![];
        self.aabb_intersections_with_aabb_callback(aabb, |e| {
            intersections.push(e);
            true
        });
        intersections
    }

    /// An [intersection test](spatial_query#intersection-tests) that finds all entities with a [`ColliderAabb`]
    /// that is intersecting the given `aabb`, calling `callback` for each intersection.
    /// The search stops when `callback` returns `false` or all intersections have been found.
    ///
    /// # Related Methods
    ///
    /// - [`SpatialQueryPipeline::aabb_intersections_with_aabb`]
    pub fn aabb_intersections_with_aabb_callback(
        &self,
        aabb: ColliderAabb,
        mut callback: impl FnMut(Entity) -> bool,
    ) {
        let mut leaf_callback = |index: &u32| {
            let entity = self.proxies[*index as usize].entity;
            callback(entity)
        };

        let mut visitor = BoundingVolumeIntersectionsVisitor::new(
            &Aabb {
                mins: aabb.min.into(),
                maxs: aabb.max.into(),
            },
            &mut leaf_callback,
        );
        self.qbvh.traverse_depth_first(&mut visitor);
    }

    /// An [intersection test](spatial_query#intersection-tests) that finds all entities with a [`Collider`]
    /// that is intersecting the given `shape` with a given position and rotation.
    ///
    /// # Arguments
    ///
    /// - `shape`: The shape that intersections are tested against represented as a [`Collider`].
    /// - `shape_position`: The position of the shape.
    /// - `shape_rotation`: The rotation of the shape.
    /// - `filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query.
    ///
    /// # Related Methods
    ///
    /// - [`SpatialQueryPipeline::shape_intersections_callback`]
    pub fn shape_intersections(
        &self,
        shape: &Collider,
        shape_position: Vector,
        shape_rotation: RotationValue,
        filter: &SpatialQueryFilter,
    ) -> Vec<Entity> {
        let mut intersections = vec![];
        self.shape_intersections_callback(shape, shape_position, shape_rotation, filter, |e| {
            intersections.push(e);
            true
        });
        intersections
    }

    /// An [intersection test](spatial_query#intersection-tests) that finds all entities with a [`Collider`]
    /// that is intersecting the given `shape` with a given position and rotation, calling `callback` for each
    /// intersection. The search stops when `callback` returns `false` or all intersections have been found.
    ///
    /// # Arguments
    ///
    /// - `shape`: The shape that intersections are tested against represented as a [`Collider`].
    /// - `shape_position`: The position of the shape.
    /// - `shape_rotation`: The rotation of the shape.
    /// - `filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query.
    /// - `callback`: A callback function called for each intersection.
    ///
    /// # Related Methods
    ///
    /// - [`SpatialQueryPipeline::shape_intersections`]
    pub fn shape_intersections_callback(
        &self,
        shape: &Collider,
        shape_position: Vector,
        shape_rotation: RotationValue,
        filter: &SpatialQueryFilter,
        mut callback: impl FnMut(Entity) -> bool,
    ) {
        let proxies = &self.proxies;
        let rotation: Rotation;
        #[cfg(feature = "2d")]
        {
            rotation = Rotation::radians(shape_rotation);
        }
        #[cfg(feature = "3d")]
        {
            rotation = Rotation::from(shape_rotation);
        }

        let shape_isometry = make_isometry(shape_position, rotation);
        let inverse_shape_isometry = shape_isometry.inverse();

        let dispatcher = &*self.dispatcher;

        let mut leaf_callback = &mut |index: &u32| {
            if let Some(proxy) = proxies.get(*index as usize) {
                if filter.test(proxy.entity, proxy.layers) {
                    let isometry = inverse_shape_isometry * proxy.isometry;

                    if dispatcher.intersection_test(
                        &isometry,
                        shape.shape_scaled().as_ref(),
                        proxy.collider.shape_scaled().as_ref(),
                    ) == Ok(true)
                    {
                        return callback(proxy.entity);
                    }
                }
            }
            true
        };

        let shape_aabb = shape.shape_scaled().compute_aabb(&shape_isometry);
        let mut visitor = BoundingVolumeIntersectionsVisitor::new(&shape_aabb, &mut leaf_callback);
        self.qbvh.traverse_depth_first(&mut visitor);
    }
}

pub(crate) struct QueryPipelineAsCompositeShape<'a> {
    proxies: &'a Vec<QbvhProxyData>,
    pipeline: &'a SpatialQueryPipeline,
    query_filter: &'a SpatialQueryFilter,
}

impl TypedSimdCompositeShape for QueryPipelineAsCompositeShape<'_> {
    type PartShape = dyn Shape;
    type PartNormalConstraints = dyn NormalConstraints;
    type PartId = u32;

    fn map_typed_part_at(
        &self,
        shape_id: Self::PartId,
        mut f: impl FnMut(
            Option<&Isometry<Scalar>>,
            &Self::PartShape,
            Option<&Self::PartNormalConstraints>,
        ),
    ) {
        if let Some(proxy) = self.proxies.get(shape_id as usize) {
            if self.query_filter.test(proxy.entity, proxy.layers) {
                f(
                    Some(&proxy.isometry),
                    proxy.collider.shape_scaled().as_ref(),
                    None,
                );
            }
        }
    }

    fn map_untyped_part_at(
        &self,
        shape_id: Self::PartId,
        f: impl FnMut(Option<&Isometry<Scalar>>, &dyn Shape, Option<&dyn NormalConstraints>),
    ) {
        self.map_typed_part_at(shape_id, f);
    }

    fn typed_qbvh(&self) -> &Qbvh<Self::PartId> {
        &self.pipeline.qbvh
    }
}

pub(crate) struct QueryPipelineAsCompositeShapeWithPredicate<'a, 'b> {
    proxies: &'a Vec<QbvhProxyData>,
    pipeline: &'a SpatialQueryPipeline,
    query_filter: &'a SpatialQueryFilter,
    predicate: &'b dyn Fn(Entity) -> bool,
}

impl TypedSimdCompositeShape for QueryPipelineAsCompositeShapeWithPredicate<'_, '_> {
    type PartShape = dyn Shape;
    type PartNormalConstraints = dyn NormalConstraints;
    type PartId = u32;

    fn map_typed_part_at(
        &self,
        shape_id: Self::PartId,
        mut f: impl FnMut(
            Option<&Isometry<Scalar>>,
            &Self::PartShape,
            Option<&Self::PartNormalConstraints>,
        ),
    ) {
        if let Some(proxy) = self.proxies.get(shape_id as usize) {
            if self.query_filter.test(proxy.entity, proxy.layers) && (self.predicate)(proxy.entity)
            {
                f(
                    Some(&proxy.isometry),
                    proxy.collider.shape_scaled().as_ref(),
                    None,
                );
            }
        }
    }

    fn map_untyped_part_at(
        &self,
        shape_id: Self::PartId,
        f: impl FnMut(Option<&Isometry<Scalar>>, &dyn Shape, Option<&dyn NormalConstraints>),
    ) {
        self.map_typed_part_at(shape_id, f);
    }

    fn typed_qbvh(&self) -> &Qbvh<Self::PartId> {
        &self.pipeline.qbvh
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
    /// The point where the point was projected.
    pub point: Vector,
    /// True if the point was inside of the collider.
    pub is_inside: bool,
}
