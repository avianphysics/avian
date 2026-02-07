use crate::{collider_tree::ColliderTrees, collision::collider::contact_query, prelude::*};
use bevy::{ecs::system::SystemParam, prelude::*};
use parry::query::ShapeCastOptions;

/// A system parameter for performing [spatial queries](spatial_query).
///
/// # Methods
///
/// - [Raycasting](spatial_query#raycasting): [`cast_ray`](SpatialQuery::cast_ray), [`cast_ray_predicate`](SpatialQuery::cast_ray_predicate),
///   [`ray_hits`](SpatialQuery::ray_hits), [`ray_hits_callback`](SpatialQuery::ray_hits_callback)
/// - [Shapecasting](spatial_query#shapecasting): [`cast_shape`](SpatialQuery::cast_shape), [`cast_shape_predicate`](SpatialQuery::cast_shape_predicate),
///   [`shape_hits`](SpatialQuery::shape_hits), [`shape_hits_callback`](SpatialQuery::shape_hits_callback)
/// - [Point projection](spatial_query#point-projection): [`project_point`](SpatialQuery::project_point) and [`project_point_predicate`](SpatialQuery::project_point_predicate)
/// - [Intersection tests](spatial_query#intersection-tests)
///     - Point intersections: [`point_intersections`](SpatialQuery::point_intersections),
///       [`point_intersections_callback`](SpatialQuery::point_intersections_callback)
///     - AABB intersections: [`aabb_intersections_with_aabb`](SpatialQuery::aabb_intersections_with_aabb),
///       [`aabb_intersections_with_aabb_callback`](SpatialQuery::aabb_intersections_with_aabb_callback)
///     - Shape intersections: [`shape_intersections`](SpatialQuery::shape_intersections)
///       [`shape_intersections_callback`](SpatialQuery::shape_intersections_callback)
///
/// For simple raycasts and shapecasts, consider using the [`RayCaster`] and [`ShapeCaster`] components that
/// provide a more ECS-based approach and perform casts on every frame.
///
/// # Example
///
/// ```
/// # #[cfg(feature = "2d")]
/// # use avian2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use avian3d::prelude::*;
/// use bevy::prelude::*;
///
/// # #[cfg(all(feature = "3d", feature = "f32"))]
/// fn print_hits(spatial_query: SpatialQuery) {
///     // Ray origin and direction
///     let origin = Vec3::ZERO;
///     let direction = Dir3::X;
///
///     // Configuration for the ray cast
///     let max_distance = 100.0;
///     let solid = true;
///     let filter = SpatialQueryFilter::default();
///
///     // Cast ray and print first hit
///     if let Some(first_hit) = spatial_query.cast_ray(origin, direction, max_distance, solid, &filter) {
///         println!("First hit: {:?}", first_hit);
///     }
///
///     // Cast ray and get up to 20 hits
///     let hits = spatial_query.ray_hits(origin, direction, max_distance, 20, solid, &filter);
///
///     // Print hits
///     for hit in hits.iter() {
///         println!("Hit: {:?}", hit);
///     }
/// }
/// ```
#[derive(SystemParam)]
pub struct SpatialQuery<'w, 's> {
    colliders: Query<'w, 's, (&'static Position, &'static Rotation, &'static Collider)>,
    collider_trees: ResMut<'w, ColliderTrees>,
}

impl SpatialQuery<'_, '_> {
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
    /// - `filter`: A [`SpatialQueryFilter`] that determines which entities are included in the cast.
    ///
    /// # Example
    ///
    /// ```
    /// # #[cfg(feature = "2d")]
    /// # use avian2d::prelude::*;
    /// # #[cfg(feature = "3d")]
    /// use avian3d::prelude::*;
    /// use bevy::prelude::*;
    ///
    /// # #[cfg(all(feature = "3d", feature = "f32"))]
    /// fn print_hits(spatial_query: SpatialQuery) {
    ///     // Ray origin and direction
    ///     let origin = Vec3::ZERO;
    ///     let direction = Dir3::X;
    ///
    ///     // Configuration for the ray cast
    ///     let max_distance = 100.0;
    ///     let solid = true;
    ///     let filter = SpatialQueryFilter::default();
    ///
    ///     // Cast ray and print first hit
    ///     if let Some(first_hit) = spatial_query.cast_ray(origin, direction, max_distance, solid, &filter) {
    ///         println!("First hit: {:?}", first_hit);
    ///     }
    /// }
    /// ```
    ///
    /// # Related Methods
    ///
    /// - [`SpatialQuery::cast_ray_predicate`]
    /// - [`SpatialQuery::ray_hits`]
    /// - [`SpatialQuery::ray_hits_callback`]
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
    /// - `filter`: A [`SpatialQueryFilter`] that determines which entities are included in the cast.
    /// - `predicate`: A function called on each entity hit by the ray. The ray keeps travelling until the predicate returns `false`.
    ///
    /// # Example
    ///
    /// ```
    /// # #[cfg(feature = "2d")]
    /// # use avian2d::prelude::*;
    /// # #[cfg(feature = "3d")]
    /// use avian3d::prelude::*;
    /// use bevy::prelude::*;
    ///
    /// #[derive(Component)]
    /// struct Invisible;
    ///
    /// # #[cfg(all(feature = "3d", feature = "f32"))]
    /// fn print_hits(spatial_query: SpatialQuery, query: Query<&Invisible>) {
    ///     // Ray origin and direction
    ///     let origin = Vec3::ZERO;
    ///     let direction = Dir3::X;
    ///
    ///     // Configuration for the ray cast
    ///     let max_distance = 100.0;
    ///     let solid = true;
    ///     let filter = SpatialQueryFilter::default();
    ///
    ///     // Cast ray and get the first hit that matches the predicate
    ///     let hit = spatial_query.cast_ray_predicate(origin, direction, max_distance, solid, &filter, &|entity| {
    ///         // Skip entities with the `Invisible` component.
    ///         !query.contains(entity)
    ///     });
    ///
    ///     // Print first hit
    ///     if let Some(first_hit) = hit {
    ///         println!("First hit: {:?}", first_hit);
    ///     }
    /// }
    /// ```
    ///
    /// # Related Methods
    ///
    /// - [`SpatialQuery::cast_ray`]
    /// - [`SpatialQuery::ray_hits`]
    /// - [`SpatialQuery::ray_hits_callback`]
    pub fn cast_ray_predicate(
        &self,
        origin: Vector,
        direction: Dir,
        mut max_distance: Scalar,
        solid: bool,
        filter: &SpatialQueryFilter,
        predicate: &dyn Fn(Entity) -> bool,
    ) -> Option<RayHitData> {
        let ray = Ray::new(origin.f32(), direction);

        let mut closest_hit: Option<RayHitData> = None;

        self.collider_trees.iter_trees().for_each(|tree| {
            tree.ray_traverse_closest(ray, max_distance, |proxy_id| {
                let proxy = tree.get_proxy(proxy_id).unwrap();
                if !filter.test(proxy.collider, proxy.layers) || !predicate(proxy.collider) {
                    return Scalar::MAX;
                }

                let Ok((position, rotation, collider)) = self.colliders.get(proxy.collider) else {
                    return Scalar::MAX;
                };

                let Some((distance, normal)) = collider.cast_ray(
                    position.0,
                    *rotation,
                    origin,
                    direction.adjust_precision(),
                    max_distance,
                    solid,
                ) else {
                    return Scalar::MAX;
                };

                if distance < max_distance {
                    max_distance = distance;
                    closest_hit = Some(RayHitData {
                        entity: proxy.collider,
                        normal,
                        distance,
                    });
                }

                distance
            });
        });

        closest_hit
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
    /// - `filter`: A [`SpatialQueryFilter`] that determines which entities are included in the cast.
    ///
    /// # Example
    ///
    /// ```
    /// # #[cfg(feature = "2d")]
    /// # use avian2d::prelude::*;
    /// # #[cfg(feature = "3d")]
    /// use avian3d::prelude::*;
    /// use bevy::prelude::*;
    ///
    /// # #[cfg(all(feature = "3d", feature = "f32"))]
    /// fn print_hits(spatial_query: SpatialQuery) {
    ///     // Ray origin and direction
    ///     let origin = Vec3::ZERO;
    ///     let direction = Dir3::X;
    ///
    ///     // Configuration for the ray cast
    ///     let max_distance = 100.0;
    ///     let solid = true;
    ///     let filter = SpatialQueryFilter::default();
    ///
    ///     // Cast ray and get up to 20 hits
    ///     let hits = spatial_query.ray_hits(origin, direction, max_distance, 20, solid, &filter);
    ///
    ///     // Print hits
    ///     for hit in hits.iter() {
    ///         println!("Hit: {:?}", hit);
    ///     }
    /// }
    /// ```
    ///
    /// # Related Methods
    ///
    /// - [`SpatialQuery::cast_ray`]
    /// - [`SpatialQuery::cast_ray_predicate`]
    /// - [`SpatialQuery::ray_hits_callback`]
    pub fn ray_hits(
        &self,
        origin: Vector,
        direction: Dir,
        max_distance: Scalar,
        max_hits: u32,
        solid: bool,
        filter: &SpatialQueryFilter,
    ) -> Vec<RayHitData> {
        let mut hits = Vec::new();

        self.ray_hits_callback(origin, direction, max_distance, solid, filter, |hit| {
            if hits.len() < max_hits as usize {
                hits.push(hit);
                true
            } else {
                false
            }
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
    /// - `filter`: A [`SpatialQueryFilter`] that determines which entities are included in the cast.
    /// - `callback`: A callback function called for each hit.
    ///
    /// # Example
    ///
    /// ```
    /// # #[cfg(feature = "2d")]
    /// # use avian2d::prelude::*;
    /// # #[cfg(feature = "3d")]
    /// use avian3d::prelude::*;
    /// use bevy::prelude::*;
    ///
    /// # #[cfg(all(feature = "3d", feature = "f32"))]
    /// fn print_hits(spatial_query: SpatialQuery) {
    ///     // Ray origin and direction
    ///     let origin = Vec3::ZERO;
    ///     let direction = Dir3::X;
    ///
    ///     // Configuration for the ray cast
    ///     let max_distance = 100.0;
    ///     let solid = true;
    ///     let filter = SpatialQueryFilter::default();
    ///
    ///     // Cast ray and get all hits
    ///     let mut hits = vec![];
    ///     spatial_query.ray_hits_callback(origin, direction, max_distance, 20, solid, &filter, |hit| {
    ///         hits.push(hit);
    ///         true
    ///     });
    ///
    ///     // Print hits
    ///     for hit in hits.iter() {
    ///         println!("Hit: {:?}", hit);
    ///     }
    /// }
    /// ```
    ///
    /// # Related Methods
    ///
    /// - [`SpatialQuery::cast_ray`]
    /// - [`SpatialQuery::cast_ray_predicate`]
    /// - [`SpatialQuery::ray_hits`]
    pub fn ray_hits_callback(
        &self,
        origin: Vector,
        direction: Dir,
        max_distance: Scalar,
        solid: bool,
        filter: &SpatialQueryFilter,
        mut callback: impl FnMut(RayHitData) -> bool,
    ) {
        let ray = Ray::new(origin.f32(), direction);

        self.collider_trees.iter_trees().for_each(|tree| {
            tree.ray_traverse_all(ray, max_distance, |proxy_id| {
                let proxy = tree.get_proxy(proxy_id).unwrap();

                if !filter.test(proxy.collider, proxy.layers) {
                    return true;
                }

                let Ok((position, rotation, collider)) = self.colliders.get(proxy.collider) else {
                    return true;
                };

                let Some((distance, normal)) = collider.cast_ray(
                    position.0,
                    *rotation,
                    origin,
                    direction.adjust_precision(),
                    max_distance,
                    solid,
                ) else {
                    return true;
                };

                callback(RayHitData {
                    entity: proxy.collider,
                    normal,
                    distance,
                })
            });
        });
    }

    /// Casts a [shape](spatial_query#shapecasting) with a given rotation and computes the closest [hit](ShapeHitData)
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
    /// - `filter`: A [`SpatialQueryFilter`] that determines which entities are included in the cast.
    ///
    /// # Example
    ///
    /// ```
    /// # #[cfg(feature = "2d")]
    /// # use avian2d::prelude::*;
    /// # #[cfg(feature = "3d")]
    /// use avian3d::prelude::*;
    /// use bevy::prelude::*;
    ///
    /// # #[cfg(all(feature = "3d", feature = "f32"))]
    /// fn print_hits(spatial_query: SpatialQuery) {
    ///     // Shape properties
    ///     let shape = Collider::sphere(0.5);
    ///     let origin = Vec3::ZERO;
    ///     let rotation = Quat::default();
    ///     let direction = Dir3::X;
    ///
    ///     // Configuration for the shape cast
    ///     let config = ShapeCastConfig::from_max_distance(100.0);
    ///     let filter = SpatialQueryFilter::default();
    ///
    ///     // Cast shape and print first hit
    ///     if let Some(first_hit) = spatial_query.cast_shape(&shape, origin, rotation, direction, &config, &filter)
    ///     {
    ///         println!("First hit: {:?}", first_hit);
    ///     }
    /// }
    /// ```
    ///
    /// # Related Methods
    ///
    /// - [`SpatialQuery::cast_shape_predicate`]
    /// - [`SpatialQuery::shape_hits`]
    /// - [`SpatialQuery::shape_hits_callback`]
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

    /// Casts a [shape](spatial_query#shapecasting) with a given rotation and computes the closest [hit](ShapeHitData)
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
    /// - `filter`: A [`SpatialQueryFilter`] that determines which entities are included in the cast.
    /// - `predicate`: A function called on each entity hit by the shape. The shape keeps travelling until the predicate returns `false`.
    ///
    /// # Example
    ///
    /// ```
    /// # #[cfg(feature = "2d")]
    /// # use avian2d::prelude::*;
    /// # #[cfg(feature = "3d")]
    /// use avian3d::prelude::*;
    /// use bevy::prelude::*;
    ///
    /// #[derive(Component)]
    /// struct Invisible;
    ///
    /// # #[cfg(all(feature = "3d", feature = "f32"))]
    /// fn print_hits(spatial_query: SpatialQuery, query: Query<&Invisible>) {
    ///     // Shape properties
    ///     let shape = Collider::sphere(0.5);
    ///     let origin = Vec3::ZERO;
    ///     let rotation = Quat::default();
    ///     let direction = Dir3::X;
    ///
    ///     // Configuration for the shape cast
    ///     let config = ShapeCastConfig::from_max_distance(100.0);
    ///     let filter = SpatialQueryFilter::default();
    ///
    ///     // Cast shape and get the first hit that matches the predicate
    ///     let hit = spatial_query.cast_shape(&shape, origin, rotation, direction, &config, &filter, &|entity| {
    ///        // Skip entities with the `Invisible` component.
    ///        !query.contains(entity)
    ///     });
    ///
    ///     // Print first hit
    ///     if let Some(first_hit) = hit {
    ///         println!("First hit: {:?}", first_hit);
    ///     }
    /// }
    /// ```
    ///
    /// # Related Methods
    ///
    /// - [`SpatialQuery::cast_ray`]
    /// - [`SpatialQuery::ray_hits`]
    /// - [`SpatialQuery::ray_hits_callback`]
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
        let mut closest_distance = config.max_distance;
        let mut closest_hit: Option<ShapeHitData> = None;

        let aabb = obvhs::aabb::Aabb::from(shape.aabb(origin, shape_rotation));

        self.collider_trees.iter_trees().for_each(|tree| {
            tree.sweep_traverse_closest(
                aabb,
                direction,
                closest_distance,
                config.target_distance,
                |proxy_id| {
                    let proxy = tree.get_proxy(proxy_id).unwrap();

                    if !filter.test(proxy.collider, proxy.layers) || !predicate(proxy.collider) {
                        return Scalar::MAX;
                    }

                    let Ok((position, rotation, collider)) = self.colliders.get(proxy.collider)
                    else {
                        return Scalar::MAX;
                    };

                    let pose1 = make_pose(position.0, *rotation);
                    let pose2 = make_pose(origin, shape_rotation);

                    let Ok(Some(hit)) = parry::query::cast_shapes(
                        &pose1,
                        Vector::ZERO,
                        collider.shape_scaled().as_ref(),
                        &pose2,
                        direction.adjust_precision(),
                        shape.shape_scaled().as_ref(),
                        ShapeCastOptions {
                            max_time_of_impact: config.max_distance,
                            target_distance: config.target_distance,
                            stop_at_penetration: !config.ignore_origin_penetration,
                            compute_impact_geometry_on_penetration: config
                                .compute_contact_on_penetration,
                        },
                    ) else {
                        return Scalar::MAX;
                    };
                    if hit.time_of_impact < closest_distance {
                        closest_distance = hit.time_of_impact;
                        closest_hit = Some(ShapeHitData {
                            entity: proxy.collider,
                            point1: pose1 * hit.witness1,
                            point2: pose2 * hit.witness2
                                + direction.adjust_precision() * hit.time_of_impact,
                            normal1: pose1.rotation * hit.normal1,
                            normal2: pose2.rotation * hit.normal2,
                            distance: hit.time_of_impact,
                        });
                    }

                    hit.time_of_impact
                },
            );
        });

        closest_hit
    }

    /// Casts a [shape](spatial_query#shapecasting) with a given rotation and computes computes all [hits](ShapeHitData)
    /// in the order of distance until `max_hits` is reached.
    ///
    /// Note that the order of the results is not guaranteed, and if there are more hits than `max_hits`,
    /// some hits will be missed.
    ///
    /// # Arguments
    ///
    /// - `shape`: The shape being cast represented as a [`Collider`].
    /// - `origin`: Where the shape is cast from.
    /// - `shape_rotation`: The rotation of the shape being cast.
    /// - `direction`: What direction the shape is cast in.
    /// - `max_hits`: The maximum number of hits. Additional hits will be missed.
    /// - `config`: A [`ShapeCastConfig`] that determines the behavior of the cast.
    /// - `filter`: A [`SpatialQueryFilter`] that determines which entities are included in the cast.
    /// - `callback`: A callback function called for each hit.
    ///
    /// # Example
    ///
    /// ```
    /// # #[cfg(feature = "2d")]
    /// # use avian2d::prelude::*;
    /// # #[cfg(feature = "3d")]
    /// use avian3d::prelude::*;
    /// use bevy::prelude::*;
    ///
    /// # #[cfg(all(feature = "3d", feature = "f32"))]
    /// fn print_hits(spatial_query: SpatialQuery) {
    ///     // Shape properties
    ///     let shape = Collider::sphere(0.5);
    ///     let origin = Vec3::ZERO;
    ///     let rotation = Quat::default();
    ///     let direction = Dir3::X;
    ///
    ///     // Configuration for the shape cast
    ///     let config = ShapeCastConfig::from_max_distance(100.0);
    ///     let filter = SpatialQueryFilter::default();
    ///
    ///     // Cast shape and get up to 20 hits
    ///     let hits = spatial_query.shape_hits(&shape, origin, rotation, direction, 20, &config, &filter);
    ///
    ///     // Print hits
    ///     for hit in hits.iter() {
    ///         println!("Hit: {:?}", hit);
    ///     }
    /// }
    /// ```
    ///
    /// # Related Methods
    ///
    /// - [`SpatialQuery::cast_shape`]
    /// - [`SpatialQuery::cast_shape_predicate`]
    /// - [`SpatialQuery::shape_hits_callback`]
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
        let mut hits = Vec::new();

        self.shape_hits_callback(
            shape,
            origin,
            shape_rotation,
            direction,
            config,
            filter,
            |hit| {
                if hits.len() < max_hits as usize {
                    hits.push(hit);
                    true
                } else {
                    false
                }
            },
        );

        hits
    }

    /// Casts a [shape](spatial_query#shapecasting) with a given rotation and computes computes all [hits](ShapeHitData)
    /// in the order of distance, calling the given `callback` for each hit. The shapecast stops when
    /// `callback` returns false or all hits have been found.
    ///
    /// Note that the order of the results is not guaranteed.
    ///
    /// # Arguments
    ///
    /// - `shape`: The shape being cast represented as a [`Collider`].
    /// - `origin`: Where the shape is cast from.
    /// - `shape_rotation`: The rotation of the shape being cast.
    /// - `direction`: What direction the shape is cast in.
    /// - `config`: A [`ShapeCastConfig`] that determines the behavior of the cast.
    /// - `filter`: A [`SpatialQueryFilter`] that determines which entities are included in the cast.
    /// - `callback`: A callback function called for each hit.
    ///
    /// # Example
    ///
    /// ```
    /// # #[cfg(feature = "2d")]
    /// # use avian2d::prelude::*;
    /// # #[cfg(feature = "3d")]
    /// use avian3d::prelude::*;
    /// use bevy::prelude::*;
    ///
    /// # #[cfg(all(feature = "3d", feature = "f32"))]
    /// fn print_hits(spatial_query: SpatialQuery) {
    ///     // Shape properties
    ///     let shape = Collider::sphere(0.5);
    ///     let origin = Vec3::ZERO;
    ///     let rotation = Quat::default();
    ///     let direction = Dir3::X;
    ///
    ///     // Configuration for the shape cast
    ///     let config = ShapeCastConfig::from_max_distance(100.0);
    ///     let filter = SpatialQueryFilter::default();
    ///
    ///     // Cast shape and get up to 20 hits
    ///     let mut hits = vec![];
    ///     spatial_query.shape_hits_callback(&shape, origin, rotation, direction, 20, &config, &filter, |hit| {
    ///         hits.push(hit);
    ///         true
    ///     });
    ///
    ///     // Print hits
    ///     for hit in hits.iter() {
    ///         println!("Hit: {:?}", hit);
    ///     }
    /// }
    /// ```
    ///
    /// # Related Methods
    ///
    /// - [`SpatialQuery::cast_shape`]
    /// - [`SpatialQuery::cast_shape_predicate`]
    /// - [`SpatialQuery::shape_hits`]
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
        let aabb = obvhs::aabb::Aabb::from(shape.aabb(origin, shape_rotation));

        self.collider_trees.iter_trees().for_each(|tree| {
            tree.sweep_traverse_all(
                aabb,
                direction,
                config.max_distance,
                config.target_distance,
                |proxy_id| {
                    let proxy = tree.get_proxy(proxy_id).unwrap();

                    if !filter.test(proxy.collider, proxy.layers) {
                        return true;
                    }

                    let Ok((position, rotation, collider)) = self.colliders.get(proxy.collider)
                    else {
                        return true;
                    };

                    let pose1 = make_pose(position.0, *rotation);
                    let pose2 = make_pose(origin, shape_rotation);

                    let Ok(Some(hit)) = parry::query::cast_shapes(
                        &pose1,
                        Vector::ZERO,
                        collider.shape_scaled().as_ref(),
                        &pose2,
                        direction.adjust_precision(),
                        shape.shape_scaled().as_ref(),
                        ShapeCastOptions {
                            max_time_of_impact: config.max_distance,
                            target_distance: config.target_distance,
                            stop_at_penetration: !config.ignore_origin_penetration,
                            compute_impact_geometry_on_penetration: config
                                .compute_contact_on_penetration,
                        },
                    ) else {
                        return true;
                    };

                    callback(ShapeHitData {
                        entity: proxy.collider,
                        point1: position.0 + rotation * hit.witness1,
                        point2: pose2 * hit.witness2
                            + direction.adjust_precision() * hit.time_of_impact,
                        normal1: pose1.rotation * hit.normal1,
                        normal2: pose2.rotation * hit.normal2,
                        distance: hit.time_of_impact,
                    })
                },
            );
        });
    }

    /// Finds the [projection](spatial_query#point-projection) of a given point on the closest [collider](Collider).
    /// If one isn't found, `None` is returned.
    ///
    /// # Arguments
    ///
    /// - `point`: The point that should be projected.
    /// - `solid`: If true and the point is inside of a collider, the projection will be at the point.
    ///   Otherwise, the collider will be treated as hollow, and the projection will be at the collider's boundary.
    /// - `query_filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query.
    ///
    /// # Example
    ///
    /// ```
    /// # #[cfg(feature = "2d")]
    /// # use avian2d::prelude::*;
    /// # #[cfg(feature = "3d")]
    /// use avian3d::prelude::*;
    /// use bevy::prelude::*;
    ///
    /// # #[cfg(all(feature = "3d", feature = "f32"))]
    /// fn print_point_projection(spatial_query: SpatialQuery) {
    ///     // Project a point and print the result
    ///     if let Some(projection) = spatial_query.project_point(
    ///         Vec3::ZERO,                    // Point
    ///         true,                          // Are colliders treated as "solid"
    ///         &SpatialQueryFilter::default(),// Query filter
    ///     ) {
    ///         println!("Projection: {:?}", projection);
    ///     }
    /// }
    /// ```
    ///
    /// # Related Methods
    ///
    /// - [`SpatialQuery::project_point_predicate`]
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
    /// - `predicate`: A function for filtering which entities are considered in the query. The projection will be on the closest collider that passes the predicate.
    ///
    /// # Example
    ///
    /// ```
    /// # #[cfg(feature = "2d")]
    /// # use avian2d::prelude::*;
    /// # #[cfg(feature = "3d")]
    /// use avian3d::prelude::*;
    /// use bevy::prelude::*;
    ///
    /// #[derive(Component)]
    /// struct Invisible;
    ///
    /// # #[cfg(all(feature = "3d", feature = "f32"))]
    /// fn print_point_projection(spatial_query: SpatialQuery, query: Query<&Invisible>) {
    ///     // Project a point and print the result
    ///     if let Some(projection) = spatial_query.project_point_predicate(
    ///         Vec3::ZERO,                    // Point
    ///         true,                          // Are colliders treated as "solid"
    ///         SpatialQueryFilter::default(), // Query filter
    ///         &|entity| {                    // Predicate
    ///             // Skip entities with the `Invisible` component.
    ///             !query.contains(entity)
    ///         }
    ///     ) {
    ///         println!("Projection: {:?}", projection);
    ///     }
    /// }
    /// ```
    ///
    /// # Related Methods
    ///
    /// - [`SpatialQuery::project_point`]
    pub fn project_point_predicate(
        &self,
        point: Vector,
        solid: bool,
        filter: &SpatialQueryFilter,
        predicate: &dyn Fn(Entity) -> bool,
    ) -> Option<PointProjection> {
        let mut closest_distance_squared = Scalar::INFINITY;
        let mut closest_projection: Option<PointProjection> = None;

        self.collider_trees.iter_trees().for_each(|tree| {
            tree.squared_distance_traverse_closest(point, Scalar::INFINITY, |proxy_id| {
                let proxy = tree.get_proxy(proxy_id).unwrap();
                if !filter.test(proxy.collider, proxy.layers) || !predicate(proxy.collider) {
                    return Scalar::INFINITY;
                }

                let Ok((position, rotation, collider)) = self.colliders.get(proxy.collider) else {
                    return Scalar::INFINITY;
                };

                let (projection, is_inside) =
                    collider.project_point(position.0, *rotation, point, solid);

                let distance_squared = (projection - point).length_squared();
                if distance_squared < closest_distance_squared {
                    closest_distance_squared = distance_squared;
                    closest_projection = Some(PointProjection {
                        entity: proxy.collider,
                        point: projection,
                        is_inside,
                    });
                }

                distance_squared
            });
        });

        closest_projection
    }

    /// An [intersection test](spatial_query#intersection-tests) that finds all entities with a [collider](Collider)
    /// that contains the given point.
    ///
    /// # Arguments
    ///
    /// - `point`: The point that intersections are tested against.
    /// - `filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query.
    ///
    /// # Example
    ///
    /// ```
    /// # #[cfg(feature = "2d")]
    /// # use avian2d::prelude::*;
    /// # #[cfg(feature = "3d")]
    /// use avian3d::prelude::*;
    /// use bevy::prelude::*;
    ///
    /// # #[cfg(all(feature = "3d", feature = "f32"))]
    /// fn print_point_intersections(spatial_query: SpatialQuery) {
    ///     let intersections =
    ///         spatial_query.point_intersections(Vec3::ZERO, &SpatialQueryFilter::default());
    ///
    ///     for entity in intersections.iter() {
    ///         println!("Entity: {}", entity);
    ///     }
    /// }
    /// ```
    ///
    /// # Related Methods
    ///
    /// - [`SpatialQuery::point_intersections_callback`]
    pub fn point_intersections(&self, point: Vector, filter: &SpatialQueryFilter) -> Vec<Entity> {
        let mut intersections = vec![];

        self.point_intersections_callback(point, filter, |entity| {
            intersections.push(entity);
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
    /// # Example
    ///
    /// ```
    /// # #[cfg(feature = "2d")]
    /// # use avian2d::prelude::*;
    /// # #[cfg(feature = "3d")]
    /// use avian3d::prelude::*;
    /// use bevy::prelude::*;
    ///
    /// # #[cfg(all(feature = "3d", feature = "f32"))]
    /// fn print_point_intersections(spatial_query: SpatialQuery) {
    ///     let mut intersections = vec![];
    ///     
    ///     spatial_query.point_intersections_callback(
    ///         Vec3::ZERO,                     // Point
    ///         &SpatialQueryFilter::default(), // Query filter
    ///         |entity| {                      // Callback function
    ///             intersections.push(entity);
    ///             true
    ///         },
    ///     );
    ///
    ///     for entity in intersections.iter() {
    ///         println!("Entity: {}", entity);
    ///     }
    /// }
    /// ```
    ///
    /// # Related Methods
    ///
    /// - [`SpatialQuery::point_intersections`]
    pub fn point_intersections_callback(
        &self,
        point: Vector,
        filter: &SpatialQueryFilter,
        mut callback: impl FnMut(Entity) -> bool,
    ) {
        self.collider_trees.iter_trees().for_each(|tree| {
            tree.point_traverse(point, |proxy_id| {
                let proxy = tree.get_proxy(proxy_id).unwrap();
                if filter.test(proxy.collider, proxy.layers) {
                    callback(proxy.collider)
                } else {
                    true
                }
            });
        });
    }

    /// An [intersection test](spatial_query#intersection-tests) that finds all entities with a [`ColliderAabb`]
    /// that is intersecting the given `aabb`.
    ///
    /// # Example
    ///
    /// ```
    /// # #[cfg(feature = "2d")]
    /// # use avian2d::prelude::*;
    /// # #[cfg(feature = "3d")]
    /// use avian3d::prelude::*;
    /// use bevy::prelude::*;
    ///
    /// # #[cfg(all(feature = "3d", feature = "f32"))]
    /// fn print_aabb_intersections(spatial_query: SpatialQuery) {
    ///     let aabb = Collider::sphere(0.5).aabb(Vec3::ZERO, Quat::default());
    ///     let intersections = spatial_query.aabb_intersections_with_aabb(aabb);
    ///
    ///     for entity in intersections.iter() {
    ///         println!("Entity: {}", entity);
    ///     }
    /// }
    /// ```
    ///
    /// # Related Methods
    ///
    /// - [`SpatialQuery::aabb_intersections_with_aabb_callback`]
    pub fn aabb_intersections_with_aabb(&self, aabb: ColliderAabb) -> Vec<Entity> {
        let mut intersections = vec![];

        self.aabb_intersections_with_aabb_callback(aabb, |entity| {
            intersections.push(entity);
            true
        });

        intersections
    }

    /// An [intersection test](spatial_query#intersection-tests) that finds all entities with a [`ColliderAabb`]
    /// that is intersecting the given `aabb`, calling `callback` for each intersection.
    /// The search stops when `callback` returns `false` or all intersections have been found.
    ///
    /// # Example
    ///
    /// ```
    /// # #[cfg(feature = "2d")]
    /// # use avian2d::prelude::*;
    /// # #[cfg(feature = "3d")]
    /// use avian3d::prelude::*;
    /// use bevy::prelude::*;
    ///
    /// # #[cfg(all(feature = "3d", feature = "f32"))]
    /// fn print_aabb_intersections(spatial_query: SpatialQuery) {
    ///     let mut intersections = vec![];
    ///
    ///     spatial_query.aabb_intersections_with_aabb_callback(
    ///         Collider::sphere(0.5).aabb(Vec3::ZERO, Quat::default()),
    ///         |entity| {
    ///             intersections.push(entity);
    ///             true
    ///         }
    ///     );
    ///
    ///     for entity in intersections.iter() {
    ///         println!("Entity: {}", entity);
    ///     }
    /// }
    /// ```
    ///
    /// # Related Methods
    ///
    /// - [`SpatialQuery::aabb_intersections_with_aabb`]
    pub fn aabb_intersections_with_aabb_callback(
        &self,
        aabb: ColliderAabb,
        mut callback: impl FnMut(Entity) -> bool,
    ) {
        let aabb = obvhs::aabb::Aabb::from(aabb);
        self.collider_trees.iter_trees().for_each(|tree| {
            tree.aabb_traverse(aabb, |proxy_id| {
                let proxy = tree.get_proxy(proxy_id).unwrap();
                // The proxy AABB is more tightly fitted to the collider than the AABB in the tree,
                // so we need to do an additional AABB intersection test here.
                if proxy.aabb.intersect_aabb(&aabb) {
                    callback(proxy.collider)
                } else {
                    true
                }
            });
        });
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
    /// # Example
    ///
    /// ```
    /// # #[cfg(feature = "2d")]
    /// # use avian2d::prelude::*;
    /// # #[cfg(feature = "3d")]
    /// use avian3d::prelude::*;
    /// use bevy::prelude::*;
    ///
    /// # #[cfg(all(feature = "3d", feature = "f32"))]
    /// fn print_shape_intersections(spatial_query: SpatialQuery) {
    ///     let intersections = spatial_query.shape_intersections(
    ///         &Collider::sphere(0.5),          // Shape
    ///         Vec3::ZERO,                      // Shape position
    ///         Quat::default(),                 // Shape rotation
    ///         &SpatialQueryFilter::default(),  // Query filter
    ///     );
    ///
    ///     for entity in intersections.iter() {
    ///         println!("Entity: {}", entity);
    ///     }
    /// }
    /// ```
    ///
    /// # Related Methods
    ///
    /// - [`SpatialQuery::shape_intersections_callback`]
    pub fn shape_intersections(
        &self,
        shape: &Collider,
        shape_position: Vector,
        shape_rotation: RotationValue,
        filter: &SpatialQueryFilter,
    ) -> Vec<Entity> {
        let mut intersections = vec![];

        self.shape_intersections_callback(
            shape,
            shape_position,
            shape_rotation,
            filter,
            |entity| {
                intersections.push(entity);
                true
            },
        );

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
    /// # Example
    ///
    /// ```
    /// # #[cfg(feature = "2d")]
    /// # use avian2d::prelude::*;
    /// # #[cfg(feature = "3d")]
    /// use avian3d::prelude::*;
    /// use bevy::prelude::*;
    ///
    /// # #[cfg(all(feature = "3d", feature = "f32"))]
    /// fn print_shape_intersections(spatial_query: SpatialQuery) {
    ///     let mut intersections = vec![];
    ///
    ///     spatial_query.shape_intersections_callback(
    ///         &Collider::sphere(0.5),          // Shape
    ///         Vec3::ZERO,                      // Shape position
    ///         Quat::default(),                 // Shape rotation
    ///         &SpatialQueryFilter::default(),  // Query filter
    ///         |entity| {                       // Callback function
    ///             intersections.push(entity);
    ///             true
    ///         },
    ///     );
    ///
    ///     for entity in intersections.iter() {
    ///         println!("Entity: {}", entity);
    ///     }
    /// }
    /// ```
    ///
    /// # Related Methods
    ///
    /// - [`SpatialQuery::shape_intersections`]
    pub fn shape_intersections_callback(
        &self,
        shape: &Collider,
        shape_position: Vector,
        shape_rotation: RotationValue,
        filter: &SpatialQueryFilter,
        mut callback: impl FnMut(Entity) -> bool,
    ) {
        let aabb = obvhs::aabb::Aabb::from(shape.aabb(shape_position, shape_rotation));

        self.collider_trees.iter_trees().for_each(|tree| {
            tree.aabb_traverse(aabb, |proxy_id| {
                let proxy = tree.get_proxy(proxy_id).unwrap();
                if !filter.test(proxy.collider, proxy.layers) {
                    return true;
                }

                let Ok((position, rotation, collider)) = self.colliders.get(proxy.collider) else {
                    return true;
                };

                if contact_query::intersection_test(
                    collider,
                    position.0,
                    *rotation,
                    shape,
                    shape_position,
                    shape_rotation,
                )
                .is_ok_and(|intersects| intersects)
                {
                    callback(proxy.collider)
                } else {
                    true
                }
            });
        });
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
