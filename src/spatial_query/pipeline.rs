use alloc::sync::Arc;

use crate::prelude::*;
use bevy::prelude::*;
use parry::{
    bounding_volume::{Aabb, BoundingVolume},
    math::Pose,
    partitioning::{Bvh, BvhBuildStrategy, BvhNode},
    query::{
        DefaultQueryDispatcher, QueryDispatcher, RayCast, ShapeCastOptions,
        details::NormalConstraints,
    },
    shape::{CompositeShape, CompositeShapeRef, Shape, TypedCompositeShape},
};

// TODO: It'd be nice not to store so much duplicate data.
//       Should we just query the ECS?
#[derive(Clone)]
pub(crate) struct BvhProxyData {
    pub entity: Entity,
    pub pose: Pose,
    pub collider: Collider,
    pub layers: CollisionLayers,
}

/// A resource for the spatial query pipeline.
///
/// The pipeline maintains a quaternary bounding volume hierarchy `Bvh` of the world's colliders
/// as an acceleration structure for spatial queries.
#[derive(Resource, Clone)]
pub struct SpatialQueryPipeline {
    pub(crate) bvh: Bvh,
    pub(crate) dispatcher: Arc<dyn QueryDispatcher>,
    // TODO: Store the proxies as `Bvh` leaf data.
    pub(crate) proxies: Vec<BvhProxyData>,
}

impl Default for SpatialQueryPipeline {
    fn default() -> Self {
        Self {
            bvh: Bvh::new(),
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

    pub(crate) fn as_composite_shape_internal<'a>(
        &'a self,
        query_filter: &'a SpatialQueryFilter,
    ) -> QueryPipelineAsCompositeShape<'a> {
        QueryPipelineAsCompositeShape {
            pipeline: self,
            query_filter,
        }
    }

    /// Creates a parry [`TypedCompositeShape`] for this pipeline.
    /// Can be used to implement custom spatial queries
    pub fn as_composite_shape<'a>(
        &'a self,
        query_filter: &'a SpatialQueryFilter,
    ) -> impl TypedCompositeShape {
        self.as_composite_shape_internal(query_filter)
    }

    pub(crate) fn as_composite_shape_with_predicate_internal<'a: 'b, 'b>(
        &'a self,
        query_filter: &'a SpatialQueryFilter,
        predicate: &'a dyn Fn(Entity) -> bool,
    ) -> QueryPipelineAsCompositeShapeWithPredicate<'a, 'b> {
        QueryPipelineAsCompositeShapeWithPredicate {
            pipeline: self,
            query_filter,
            predicate,
        }
    }

    /// Creates a parry [`TypedCompositeShape`] for this pipeline, with a predicate.
    /// Can be used to implement custom spatial queries
    pub fn as_composite_shape_with_predicate<'a>(
        &'a self,
        query_filter: &'a SpatialQueryFilter,
        predicate: &'a dyn Fn(Entity) -> bool,
    ) -> impl TypedCompositeShape {
        self.as_composite_shape_with_predicate_internal(query_filter, predicate)
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
                |(entity, position, rotation, collider, layers)| BvhProxyData {
                    entity,
                    pose: make_pose(position.0, *rotation),
                    collider: collider.clone(),
                    layers: *layers,
                },
            ),
        )
    }

    // TODO: Incremental updates.
    fn update_internal(&mut self, proxies: impl Iterator<Item = BvhProxyData>) {
        self.proxies.clear();
        self.proxies.extend(proxies);

        let aabbs = self
            .proxies
            .iter()
            .enumerate()
            .map(|(i, proxy)| (i, proxy.collider.shape_scaled().compute_aabb(&proxy.pose)));

        self.bvh = Bvh::from_iter(BvhBuildStrategy::Binned, aabbs);
    }

    /// Get the entity corresponding to a given index in the pipeline
    pub fn entity(&self, index: usize) -> Entity {
        self.proxies[index].entity
    }

    /// Get a dyn reference to the query dispatcher used in this pipeline
    pub fn dispatcher_ref(&self) -> &dyn QueryDispatcher {
        self.dispatcher.as_ref()
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
        let composite = self.as_composite_shape_with_predicate(filter, predicate);
        let pipeline_shape = CompositeShapeRef(&composite);
        let ray = parry::query::Ray::new(origin, direction.adjust_precision());

        pipeline_shape
            .cast_local_ray_and_get_normal(&ray, max_distance, solid)
            .map(|(index, hit)| RayHitData {
                entity: self.proxies[index as usize].entity,
                distance: hit.time_of_impact,
                normal: hit.normal,
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
        // TODO: Just return an iterator
        mut callback: impl FnMut(RayHitData) -> bool,
    ) {
        let proxies = &self.proxies;

        let ray = parry::query::Ray::new(origin, direction.adjust_precision());

        let hits = self
            .bvh
            .leaves(move |node: &BvhNode| node.aabb().intersects_local_ray(&ray, max_distance))
            .filter_map(move |leaf| {
                let proxy = proxies.get(leaf as usize)?;

                if !filter.test(proxy.entity, proxy.layers) {
                    return None;
                }

                let hit = proxy.collider.shape_scaled().cast_ray_and_get_normal(
                    &proxy.pose,
                    &ray,
                    max_distance,
                    solid,
                )?;

                Some(RayHitData {
                    entity: proxy.entity,
                    distance: hit.time_of_impact,
                    normal: hit.normal,
                })
            });

        for hit in hits {
            if !callback(hit) {
                break;
            }
        }
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

        let shape_isometry = make_pose(origin, rotation);
        let shape_direction = direction.adjust_precision();
        let composite = self.as_composite_shape_with_predicate(filter, predicate);
        let pipeline_shape = CompositeShapeRef(&composite);

        pipeline_shape
            .cast_shape(
                self.dispatcher.as_ref(),
                &shape_isometry,
                shape_direction,
                shape.shape_scaled().as_ref(),
                ShapeCastOptions {
                    max_time_of_impact: config.max_distance,
                    stop_at_penetration: !config.ignore_origin_penetration,
                    compute_impact_geometry_on_penetration: config.compute_contact_on_penetration,
                    ..default()
                },
            )
            .map(|(index, hit)| ShapeHitData {
                entity: self.proxies[index as usize].entity,
                distance: hit.time_of_impact,
                point1: hit.witness1,
                point2: hit.witness2,
                normal1: hit.normal1,
                normal2: hit.normal2,
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
        //       See https://github.com/avianphysics/avian/issues/403.
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

        let shape_isometry = make_pose(origin, rotation);
        let shape_direction = direction.adjust_precision();

        loop {
            let composite = self.as_composite_shape_internal(&query_filter);
            let pipeline_shape = CompositeShapeRef(&composite);

            let hit = pipeline_shape
                .cast_shape(
                    self.dispatcher.as_ref(),
                    &shape_isometry,
                    shape_direction,
                    shape.shape_scaled().as_ref(),
                    shape_cast_options,
                )
                .map(|(index, hit)| ShapeHitData {
                    entity: self.proxies[index as usize].entity,
                    distance: hit.time_of_impact,
                    point1: hit.witness1,
                    point2: hit.witness2,
                    normal1: hit.normal1,
                    normal2: hit.normal2,
                });

            if let Some(hit) = hit {
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
        if self.proxies.is_empty() {
            return None;
        }

        let composite = self.as_composite_shape_with_predicate(filter, predicate);
        let pipeline_shape = CompositeShapeRef(&composite);

        let (index, projection) = pipeline_shape.project_local_point(point, solid);

        Some(PointProjection {
            entity: self.proxies[index as usize].entity,
            point: projection.point,
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
        let intersecting_entities = self
            .bvh
            .leaves(|node: &BvhNode| node.aabb().contains_local_point(point))
            .filter_map(move |leaf| {
                let proxy = self.proxies.get(leaf as usize)?;

                if filter.test(proxy.entity, proxy.layers)
                    && proxy
                        .collider
                        .shape_scaled()
                        .contains_point(&proxy.pose, point)
                {
                    Some(proxy.entity)
                } else {
                    None
                }
            });

        for entity in intersecting_entities {
            if !callback(entity) {
                break;
            }
        }
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
        let aabb = Aabb {
            mins: aabb.min,
            maxs: aabb.max,
        };

        let intersecting_entities = self
            .bvh
            .leaves(move |node: &BvhNode| node.aabb().intersects(&aabb))
            .filter_map(move |leaf| self.proxies.get(leaf as usize).map(|p| p.entity));

        for entity in intersecting_entities {
            if !callback(entity) {
                break;
            }
        }
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

        let shape_isometry = make_pose(shape_position, rotation);
        let inverse_shape_isometry = shape_isometry.inverse();

        let dispatcher = &*self.dispatcher;

        let shape_aabb = shape.shape_scaled().compute_aabb(&shape_isometry);
        let entities = self
            .bvh
            .leaves(move |node: &BvhNode| node.aabb().intersects(&shape_aabb))
            .filter_map(move |leaf| {
                let proxy = proxies.get(leaf as usize)?;

                if !filter.test(proxy.entity, proxy.layers) {
                    return None;
                }

                let isometry = inverse_shape_isometry * proxy.pose;
                let intersects = dispatcher
                    .intersection_test(
                        &isometry,
                        shape.shape_scaled().as_ref(),
                        proxy.collider.shape_scaled().as_ref(),
                    )
                    .is_ok_and(|b| b);

                intersects.then_some(proxy.entity)
            });

        for entity in entities {
            if !callback(entity) {
                break;
            }
        }
    }
}

pub(crate) struct QueryPipelineAsCompositeShape<'a> {
    pipeline: &'a SpatialQueryPipeline,
    query_filter: &'a SpatialQueryFilter,
}

impl CompositeShape for QueryPipelineAsCompositeShape<'_> {
    fn map_part_at(
        &self,
        shape_id: u32,
        f: &mut dyn FnMut(Option<&Pose>, &dyn Shape, Option<&dyn NormalConstraints>),
    ) {
        self.map_untyped_part_at(shape_id, f);
    }

    fn bvh(&self) -> &Bvh {
        &self.pipeline.bvh
    }
}

impl TypedCompositeShape for QueryPipelineAsCompositeShape<'_> {
    type PartNormalConstraints = ();
    type PartShape = dyn Shape;

    fn map_typed_part_at<T>(
        &self,
        shape_id: u32,
        mut f: impl FnMut(Option<&Pose>, &Self::PartShape, Option<&Self::PartNormalConstraints>) -> T,
    ) -> Option<T> {
        let proxy = self.pipeline.proxies.get(shape_id as usize)?;

        if self.query_filter.test(proxy.entity, proxy.layers) {
            Some(f(
                Some(&proxy.pose),
                proxy.collider.shape_scaled().as_ref(),
                None,
            ))
        } else {
            None
        }
    }

    fn map_untyped_part_at<T>(
        &self,
        shape_id: u32,
        mut f: impl FnMut(Option<&Pose>, &dyn Shape, Option<&dyn NormalConstraints>) -> T,
    ) -> Option<T> {
        let proxy = self.pipeline.proxies.get(shape_id as usize)?;

        if self.query_filter.test(proxy.entity, proxy.layers) {
            Some(f(
                Some(&proxy.pose),
                proxy.collider.shape_scaled().as_ref(),
                None,
            ))
        } else {
            None
        }
    }
}

pub(crate) struct QueryPipelineAsCompositeShapeWithPredicate<'a, 'b> {
    pipeline: &'a SpatialQueryPipeline,
    query_filter: &'a SpatialQueryFilter,
    predicate: &'b dyn Fn(Entity) -> bool,
}

impl CompositeShape for QueryPipelineAsCompositeShapeWithPredicate<'_, '_> {
    fn map_part_at(
        &self,
        shape_id: u32,
        f: &mut dyn FnMut(Option<&Pose>, &dyn Shape, Option<&dyn NormalConstraints>),
    ) {
        self.map_untyped_part_at(shape_id, f);
    }

    fn bvh(&self) -> &Bvh {
        &self.pipeline.bvh
    }
}

impl TypedCompositeShape for QueryPipelineAsCompositeShapeWithPredicate<'_, '_> {
    type PartNormalConstraints = ();
    type PartShape = dyn Shape;

    fn map_typed_part_at<T>(
        &self,
        shape_id: u32,
        mut f: impl FnMut(Option<&Pose>, &Self::PartShape, Option<&Self::PartNormalConstraints>) -> T,
    ) -> Option<T> {
        if let Some(proxy) = self.pipeline.proxies.get(shape_id as usize)
            && self.query_filter.test(proxy.entity, proxy.layers)
            && (self.predicate)(proxy.entity)
        {
            Some(f(
                Some(&proxy.pose),
                proxy.collider.shape_scaled().as_ref(),
                None,
            ))
        } else {
            None
        }
    }

    fn map_untyped_part_at<T>(
        &self,
        shape_id: u32,
        mut f: impl FnMut(Option<&Pose>, &dyn Shape, Option<&dyn NormalConstraints>) -> T,
    ) -> Option<T> {
        if let Some(proxy) = self.pipeline.proxies.get(shape_id as usize)
            && self.query_filter.test(proxy.entity, proxy.layers)
            && (self.predicate)(proxy.entity)
        {
            Some(f(
                Some(&proxy.pose),
                proxy.collider.shape_scaled().as_ref(),
                None,
            ))
        } else {
            None
        }
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
