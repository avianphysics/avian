use crate::prelude::*;
use bevy::{
    ecs::{
        component::HookContext,
        entity::{EntityMapper, MapEntities},
        world::DeferredWorld,
    },
    prelude::*,
};
#[cfg(all(
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
use parry::query::{
    details::RayCompositeShapeToiAndNormalBestFirstVisitor, visitors::RayIntersectionsVisitor,
};

/// A component used for [raycasting](spatial_query#raycasting).
///
/// **Raycasting** is a type of [spatial query](spatial_query) that finds one or more hits
/// between a ray and a set of colliders.
///
/// Each ray is defined by a local `origin` and a `direction`. The [`RayCaster`] will find each hit
/// and add them to the [`RayHits`] component. Each hit has a `distance` property which refers to
/// how far the ray travelled, along with a `normal` for the point of intersection.
///
/// The [`RayCaster`] is the easiest way to handle simple raycasts. If you want more control and don't want to
/// perform raycasts every frame, consider using the [`SpatialQuery`] system parameter.
///
/// # Hit Count and Order
///
/// The results of a raycast are in an arbitrary order by default. You can iterate over them in the order of
/// distance with the [`RayHits::iter_sorted`] method.
///
/// You can configure the maximum amount of hits for a ray using `max_hits`. By default this is unbounded,
/// so you will get all hits. When the number or complexity of colliders is large, this can be very
/// expensive computationally. Set the value to whatever works best for your case.
///
/// Note that when there are more hits than `max_hits`, **some hits will be missed**.
/// To guarantee that the closest hit is included, you should set `max_hits` to one or a value that
/// is enough to contain all hits.
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
/// fn setup(mut commands: Commands) {
///     // Spawn a ray at the center going right
///     commands.spawn(RayCaster::new(Vec3::ZERO, Dir3::X));
///     // ...spawn colliders and other things
/// }
///
/// # #[cfg(all(feature = "3d", feature = "f32"))]
/// fn print_hits(query: Query<(&RayCaster, &RayHits)>) {
///     for (ray, hits) in &query {
///         // For the faster iterator that isn't sorted, use `.iter()`
///         for hit in hits.iter_sorted() {
///             println!(
///                 "Hit entity {} at {} with normal {}",
///                 hit.entity,
///                 ray.origin + *ray.direction * hit.distance,
///                 hit.normal,
///             );
///         }
///     }
/// }
/// ```
#[derive(Component, Clone, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
#[component(on_add = on_add_ray_caster)]
#[require(RayHits)]
pub struct RayCaster {
    /// Controls if the ray caster is enabled.
    pub enabled: bool,

    /// The local origin of the ray relative to the [`Position`] and [`Rotation`] of the ray entity or its parent.
    ///
    /// To get the global origin, use the `global_origin` method.
    pub origin: Vector,

    /// The global origin of the ray.
    global_origin: Vector,

    /// The local direction of the ray relative to the [`Rotation`] of the ray entity or its parent.
    ///
    /// To get the global direction, use the `global_direction` method.
    pub direction: Dir,

    /// The global direction of the ray.
    global_direction: Dir,

    /// The maximum number of hits allowed.
    ///
    /// When there are more hits than `max_hits`, **some hits will be missed**.
    /// To guarantee that the closest hit is included, you should set `max_hits` to one or a value that
    /// is enough to contain all hits.
    pub max_hits: u32,

    /// The maximum distance the ray can travel.
    ///
    /// By default this is infinite, so the ray will travel until all hits up to `max_hits` have been checked.
    #[doc(alias = "max_time_of_impact")]
    pub max_distance: Scalar,

    /// Controls how the ray behaves when the ray origin is inside of a [collider](Collider).
    ///
    /// If `true`, shapes will be treated as solid, and the ray cast will return with a distance of `0.0`
    /// if the ray origin is inside of the shape. Otherwise, shapes will be treated as hollow, and the ray
    /// will always return a hit at the shape's boundary.
    pub solid: bool,

    /// If true, the ray caster ignores hits against its own [`Collider`]. This is the default.
    pub ignore_self: bool,

    /// Rules that determine which colliders are taken into account in the ray cast.
    pub query_filter: SpatialQueryFilter,
}

impl Default for RayCaster {
    fn default() -> Self {
        Self {
            enabled: true,
            origin: Vector::ZERO,
            global_origin: Vector::ZERO,
            direction: Dir::X,
            global_direction: Dir::X,
            max_distance: Scalar::MAX,
            max_hits: u32::MAX,
            solid: true,
            ignore_self: true,
            query_filter: SpatialQueryFilter::default(),
        }
    }
}

impl From<Ray> for RayCaster {
    fn from(ray: Ray) -> Self {
        RayCaster::from_ray(ray)
    }
}

impl RayCaster {
    /// Creates a new [`RayCaster`] with a given origin and direction.
    pub fn new(origin: Vector, direction: Dir) -> Self {
        Self {
            origin,
            direction,
            ..default()
        }
    }

    /// Creates a new [`RayCaster`] from a ray.
    pub fn from_ray(ray: Ray) -> Self {
        Self {
            origin: ray.origin.adjust_precision(),
            direction: ray.direction,
            ..default()
        }
    }

    /// Sets the ray origin.
    pub fn with_origin(mut self, origin: Vector) -> Self {
        self.origin = origin;
        self
    }

    /// Sets the ray direction.
    pub fn with_direction(mut self, direction: Dir) -> Self {
        self.direction = direction;
        self
    }

    /// Controls how the ray behaves when the ray origin is inside of a [collider](Collider).
    ///
    /// If `true`, shapes will be treated as solid, and the ray cast will return with a distance of `0.0`
    /// if the ray origin is inside of the shape. Otherwise, shapes will be treated as hollow, and the ray
    /// will always return a hit at the shape's boundary.
    pub fn with_solidness(mut self, solid: bool) -> Self {
        self.solid = solid;
        self
    }

    /// Sets if the ray caster should ignore hits against its own [`Collider`].
    ///
    /// The default is `true`.
    pub fn with_ignore_self(mut self, ignore: bool) -> Self {
        self.ignore_self = ignore;
        self
    }

    /// Sets the maximum distance the ray can travel.
    pub fn with_max_distance(mut self, max_distance: Scalar) -> Self {
        self.max_distance = max_distance;
        self
    }

    /// Sets the maximum number of allowed hits.
    pub fn with_max_hits(mut self, max_hits: u32) -> Self {
        self.max_hits = max_hits;
        self
    }

    /// Sets the ray caster's [query filter](SpatialQueryFilter) that controls which colliders
    /// should be included or excluded by raycasts.
    pub fn with_query_filter(mut self, query_filter: SpatialQueryFilter) -> Self {
        self.query_filter = query_filter;
        self
    }

    /// Enables the [`RayCaster`].
    pub fn enable(&mut self) {
        self.enabled = true;
    }

    /// Disables the [`RayCaster`].
    pub fn disable(&mut self) {
        self.enabled = false;
    }

    /// Returns the global origin of the ray.
    pub fn global_origin(&self) -> Vector {
        self.global_origin
    }

    /// Returns the global direction of the ray.
    pub fn global_direction(&self) -> Dir {
        self.global_direction
    }

    /// Sets the global origin of the ray.
    pub(crate) fn set_global_origin(&mut self, global_origin: Vector) {
        self.global_origin = global_origin;
    }

    /// Sets the global direction of the ray.
    pub(crate) fn set_global_direction(&mut self, global_direction: Dir) {
        self.global_direction = global_direction;
    }

    #[cfg(all(
        feature = "default-collider",
        any(feature = "parry-f32", feature = "parry-f64")
    ))]
    pub(crate) fn cast(
        &mut self,
        caster_entity: Entity,
        hits: &mut RayHits,
        query_pipeline: &SpatialQueryPipeline,
    ) {
        if self.ignore_self {
            self.query_filter.excluded_entities.insert(caster_entity);
        } else {
            self.query_filter.excluded_entities.remove(&caster_entity);
        }

        hits.count = 0;

        if self.max_hits == 1 {
            let pipeline_shape = query_pipeline.as_composite_shape(&self.query_filter);
            let ray = parry::query::Ray::new(
                self.global_origin().into(),
                self.global_direction().adjust_precision().into(),
            );
            let mut visitor = RayCompositeShapeToiAndNormalBestFirstVisitor::new(
                &pipeline_shape,
                &ray,
                self.max_distance,
                self.solid,
            );

            if let Some(hit) =
                query_pipeline
                    .qbvh
                    .traverse_best_first(&mut visitor)
                    .map(|(_, (index, hit))| RayHitData {
                        entity: query_pipeline.proxies[index as usize].entity,
                        distance: hit.time_of_impact,
                        normal: hit.normal.into(),
                    })
            {
                if (hits.vector.len() as u32) < hits.count + 1 {
                    hits.vector.push(hit);
                } else {
                    hits.vector[0] = hit;
                }
                hits.count = 1;
            }
        } else {
            let ray = parry::query::Ray::new(
                self.global_origin().into(),
                self.global_direction().adjust_precision().into(),
            );

            let mut leaf_callback = &mut |index: &u32| {
                if let Some(proxy) = query_pipeline.proxies.get(*index as usize) {
                    if self.query_filter.test(proxy.entity, proxy.layers) {
                        if let Some(hit) = proxy.collider.shape_scaled().cast_ray_and_get_normal(
                            &proxy.isometry,
                            &ray,
                            self.max_distance,
                            self.solid,
                        ) {
                            if (hits.vector.len() as u32) < hits.count + 1 {
                                hits.vector.push(RayHitData {
                                    entity: proxy.entity,
                                    distance: hit.time_of_impact,
                                    normal: hit.normal.into(),
                                });
                            } else {
                                hits.vector[hits.count as usize] = RayHitData {
                                    entity: proxy.entity,
                                    distance: hit.time_of_impact,
                                    normal: hit.normal.into(),
                                };
                            }

                            hits.count += 1;

                            return hits.count < self.max_hits;
                        }
                    }
                }
                true
            };

            let mut visitor =
                RayIntersectionsVisitor::new(&ray, self.max_distance, &mut leaf_callback);
            query_pipeline.qbvh.traverse_depth_first(&mut visitor);
        }
    }
}

fn on_add_ray_caster(mut world: DeferredWorld, ctx: HookContext) {
    let ray_caster = world.get::<RayCaster>(ctx.entity).unwrap();
    let max_hits = if ray_caster.max_hits == u32::MAX {
        10
    } else {
        ray_caster.max_hits as usize
    };

    // Initialize capacity for hits
    world.get_mut::<RayHits>(ctx.entity).unwrap().vector = Vec::with_capacity(max_hits);
}

/// Contains the hits of a ray cast by a [`RayCaster`].
///
/// The maximum number of hits depends on the value of `max_hits` in [`RayCaster`].
///
/// # Order
///
/// By default, the order of the hits is not guaranteed.
///
/// You can iterate the hits in the order of distance with `iter_sorted`.
/// Note that this will create and sort a new vector instead of the original one.
///
/// **Note**: When there are more hits than `max_hits`, **some hits
/// will be missed**. If you want to guarantee that the closest hit is included, set `max_hits` to one.
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
/// fn print_hits(query: Query<&RayHits, With<RayCaster>>) {
///     for hits in &query {
///         // For the faster iterator that isn't sorted, use `.iter()`
///         for hit in hits.iter_sorted() {
///             println!("Hit entity {} with distance {}", hit.entity, hit.distance);
///         }
///     }
/// }
/// ```
#[derive(Debug, Component, Clone, Default, Reflect, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct RayHits {
    pub(crate) vector: Vec<RayHitData>,
    /// The number of hits.
    pub(crate) count: u32,
}

impl RayHits {
    /// Returns a slice over the ray hits.
    pub fn as_slice(&self) -> &[RayHitData] {
        &self.vector[0..self.count as usize]
    }

    /// Returns the number of hits.
    #[doc(alias = "count")]
    pub fn len(&self) -> usize {
        self.count as usize
    }

    /// Returns true if the number of hits is 0.
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// Clears the hits.
    pub fn clear(&mut self) {
        self.vector.clear();
        self.count = 0;
    }

    /// Returns an iterator over the hits in arbitrary order.
    ///
    /// If you want to get them sorted by distance, use `iter_sorted`.
    pub fn iter(&self) -> core::slice::Iter<'_, RayHitData> {
        self.as_slice().iter()
    }

    /// Returns an iterator over the hits, sorted in ascending order according to the distance.
    ///
    /// Note that this creates and sorts a new vector. If you don't need the hits in order, use `iter`.
    pub fn iter_sorted(&self) -> alloc::vec::IntoIter<RayHitData> {
        let mut vector = self.as_slice().to_vec();
        vector.sort_by(|a, b| a.distance.partial_cmp(&b.distance).unwrap());
        vector.into_iter()
    }
}

impl MapEntities for RayHits {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        for hit in &mut self.vector {
            hit.map_entities(entity_mapper);
        }
    }
}

/// Data related to a hit during a [raycast](spatial_query#raycasting).
#[derive(Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct RayHitData {
    /// The entity of the collider that was hit by the ray.
    pub entity: Entity,

    /// How far the ray travelled. This is the distance between the ray origin and the point of intersection.
    pub distance: Scalar,

    /// The normal at the point of intersection, expressed in world space.
    pub normal: Vector,
}

impl MapEntities for RayHitData {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.entity = entity_mapper.get_mapped(self.entity);
    }
}
