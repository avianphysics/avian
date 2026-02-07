use crate::prelude::*;
use bevy::{
    ecs::{
        entity::{EntityMapper, MapEntities},
        lifecycle::HookContext,
        world::DeferredWorld,
    },
    prelude::*,
};

/// A component used for [shapecasting](spatial_query#shapecasting).
///
/// **Shapecasting** is a type of [spatial query](spatial_query) where a shape travels along a straight
/// line and computes hits with colliders. This is often used to determine how far an object can move
/// in a direction before it hits something.
///
/// Each shapecast is defined by a `shape` (a [`Collider`]), its local `shape_rotation`, a local `origin` and
/// a local `direction`. The [`ShapeCaster`] will find each hit and add them to the [`ShapeHits`] component in
/// the order of distance.
///
/// Computing lots of hits can be expensive, especially against complex geometry, so the maximum number of hits
/// is one by default. This can be configured through the `max_hits` property.
///
/// The [`ShapeCaster`] is the easiest way to handle simple shapecasting. If you want more control and don't want
/// to perform shapecasts on every frame, consider using the [`SpatialQuery`] system parameter.
///
/// # Hit Count and Order
///
/// The results of a shapecast are in an arbitrary order by default. You can iterate over them in the order of
/// distance with the [`ShapeHits::iter_sorted`] method.
///
/// You can configure the maximum amount of hits for a shapecast using `max_hits`. By default this is unbounded,
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
///     // Spawn a shape caster with a ball shape moving right starting from the origin
///     commands.spawn(ShapeCaster::new(
#[cfg_attr(feature = "2d", doc = "        Collider::circle(0.5),")]
#[cfg_attr(feature = "3d", doc = "        Collider::sphere(0.5),")]
///         Vec3::ZERO,
///         Quat::default(),
///         Dir3::X,
///     ));
/// }
///
/// fn print_hits(query: Query<(&ShapeCaster, &ShapeHits)>) {
///     for (shape_caster, hits) in &query {
///         for hit in hits.iter() {
///             println!("Hit entity {}", hit.entity);
///         }
///     }
/// }
/// ```
#[derive(Component, Clone, Debug, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component)]
#[component(on_add = on_add_shape_caster)]
#[require(ShapeHits)]
pub struct ShapeCaster {
    /// Controls if the shape caster is enabled.
    pub enabled: bool,

    /// The shape being cast represented as a [`Collider`].
    #[reflect(ignore)]
    pub shape: Collider,

    /// The local origin of the shape relative to the [`Position`] and [`Rotation`]
    /// of the shape caster entity or its parent.
    ///
    /// To get the global origin, use the `global_origin` method.
    pub origin: Vector,

    /// The global origin of the shape.
    global_origin: Vector,

    /// The local rotation of the shape being cast relative to the [`Rotation`]
    /// of the shape caster entity or its parent. Expressed in radians.
    ///
    /// To get the global shape rotation, use the `global_shape_rotation` method.
    #[cfg(feature = "2d")]
    pub shape_rotation: Scalar,

    /// The local rotation of the shape being cast relative to the [`Rotation`]
    /// of the shape caster entity or its parent.
    ///
    /// To get the global shape rotation, use the `global_shape_rotation` method.
    #[cfg(feature = "3d")]
    pub shape_rotation: Quaternion,

    /// The global rotation of the shape.
    #[cfg(feature = "2d")]
    global_shape_rotation: Scalar,

    /// The global rotation of the shape.
    #[cfg(feature = "3d")]
    global_shape_rotation: Quaternion,

    /// The local direction of the shapecast relative to the [`Rotation`] of the shape caster entity or its parent.
    ///
    /// To get the global direction, use the `global_direction` method.
    pub direction: Dir,

    /// The global direction of the shapecast.
    global_direction: Dir,

    /// The maximum number of hits allowed. By default this is one and only the first hit is returned.
    pub max_hits: u32,

    /// The maximum distance the shape can travel.
    ///
    /// By default, this is infinite.
    #[doc(alias = "max_time_of_impact")]
    pub max_distance: Scalar,

    /// The separation distance at which the shapes will be considered as impacting.
    ///
    /// If the shapes are separated by a distance smaller than `target_distance` at the origin of the cast,
    /// the computed contact points and normals are only reliable if [`ShapeCaster::compute_contact_on_penetration`]
    /// is set to `true`.
    ///
    /// By default, this is `0.0`, so the shapes will only be considered as impacting when they first touch.
    pub target_distance: Scalar,

    /// If `true`, contact points and normals will be calculated even when the cast distance is `0.0`.
    ///
    /// The default is `true`.
    pub compute_contact_on_penetration: bool,

    /// If `true` *and* the shape is travelling away from the object that was hit,
    /// the cast will ignore any impact that happens at the cast origin.
    ///
    /// The default is `false`.
    pub ignore_origin_penetration: bool,

    /// If true, the shape caster ignores hits against its own [`Collider`]. This is the default.
    pub ignore_self: bool,

    /// Rules that determine which colliders are taken into account in the shape cast.
    pub query_filter: SpatialQueryFilter,
}

impl Default for ShapeCaster {
    fn default() -> Self {
        Self {
            enabled: true,
            #[cfg(feature = "2d")]
            shape: Collider::circle(0.0),
            #[cfg(feature = "3d")]
            shape: Collider::sphere(0.0),
            origin: Vector::ZERO,
            global_origin: Vector::ZERO,
            #[cfg(feature = "2d")]
            shape_rotation: 0.0,
            #[cfg(feature = "3d")]
            shape_rotation: Quaternion::IDENTITY,
            #[cfg(feature = "2d")]
            global_shape_rotation: 0.0,
            #[cfg(feature = "3d")]
            global_shape_rotation: Quaternion::IDENTITY,
            direction: Dir::X,
            global_direction: Dir::X,
            max_hits: 1,
            max_distance: Scalar::MAX,
            target_distance: 0.0,
            compute_contact_on_penetration: true,
            ignore_origin_penetration: false,
            ignore_self: true,
            query_filter: SpatialQueryFilter::default(),
        }
    }
}

impl ShapeCaster {
    /// Creates a new [`ShapeCaster`] with a given shape, origin, shape rotation and direction.
    #[cfg(feature = "2d")]
    pub fn new(
        shape: impl Into<Collider>,
        origin: Vector,
        shape_rotation: Scalar,
        direction: Dir,
    ) -> Self {
        Self {
            shape: shape.into(),
            origin,
            shape_rotation,
            direction,
            ..default()
        }
    }
    #[cfg(feature = "3d")]
    /// Creates a new [`ShapeCaster`] with a given shape, origin, shape rotation and direction.
    pub fn new(
        shape: impl Into<Collider>,
        origin: Vector,
        shape_rotation: Quaternion,
        direction: Dir,
    ) -> Self {
        Self {
            shape: shape.into(),
            origin,
            shape_rotation,
            direction,
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

    /// Sets the separation distance at which the shapes will be considered as impacting.
    ///
    /// If the shapes are separated by a distance smaller than `target_distance` at the origin of the cast,
    /// the computed contact points and normals are only reliable if [`ShapeCaster::compute_contact_on_penetration`]
    /// is set to `true`.
    ///
    /// By default, this is `0.0`, so the shapes will only be considered as impacting when they first touch.
    pub fn with_target_distance(mut self, target_distance: Scalar) -> Self {
        self.target_distance = target_distance;
        self
    }

    /// Sets if contact points and normals should be calculated even when the cast distance is `0.0`.
    ///
    /// The default is `true`.
    pub fn with_compute_contact_on_penetration(mut self, compute_contact: bool) -> Self {
        self.compute_contact_on_penetration = compute_contact;
        self
    }

    /// Controls how the shapecast behaves when the shape is already penetrating a [collider](Collider)
    /// at the shape origin.
    ///
    /// If set to `true` **and** the shape is being cast in a direction where it will eventually stop penetrating,
    /// the shapecast will not stop immediately, and will instead continue until another hit.\
    /// If set to false, the shapecast will stop immediately and return the hit. This is the default.
    pub fn with_ignore_origin_penetration(mut self, ignore: bool) -> Self {
        self.ignore_origin_penetration = ignore;
        self
    }

    /// Sets if the shape caster should ignore hits against its own [`Collider`].
    ///
    /// The default is `true`.
    pub fn with_ignore_self(mut self, ignore: bool) -> Self {
        self.ignore_self = ignore;
        self
    }

    /// Sets the maximum distance the shape can travel.
    pub fn with_max_distance(mut self, max_distance: Scalar) -> Self {
        self.max_distance = max_distance;
        self
    }

    /// Sets the maximum number of allowed hits.
    pub fn with_max_hits(mut self, max_hits: u32) -> Self {
        self.max_hits = max_hits;
        self
    }

    /// Sets the shape caster's [query filter](SpatialQueryFilter) that controls which colliders
    /// should be included or excluded by shapecasts.
    pub fn with_query_filter(mut self, query_filter: SpatialQueryFilter) -> Self {
        self.query_filter = query_filter;
        self
    }

    /// Enables the [`ShapeCaster`].
    pub fn enable(&mut self) {
        self.enabled = true;
    }

    /// Disables the [`ShapeCaster`].
    pub fn disable(&mut self) {
        self.enabled = false;
    }

    /// Returns the global origin of the ray.
    pub fn global_origin(&self) -> Vector {
        self.global_origin
    }

    /// Returns the global rotation of the shape.
    #[cfg(feature = "2d")]
    pub fn global_shape_rotation(&self) -> Scalar {
        self.global_shape_rotation
    }

    /// Returns the global rotation of the shape.
    #[cfg(feature = "3d")]
    pub fn global_shape_rotation(&self) -> Quaternion {
        self.global_shape_rotation
    }

    /// Returns the global direction of the ray.
    pub fn global_direction(&self) -> Dir {
        self.global_direction
    }

    /// Sets the global origin of the ray.
    pub(crate) fn set_global_origin(&mut self, global_origin: Vector) {
        self.global_origin = global_origin;
    }

    /// Sets the global rotation of the shape.
    #[cfg(feature = "2d")]
    pub(crate) fn set_global_shape_rotation(&mut self, global_rotation: Scalar) {
        self.global_shape_rotation = global_rotation;
    }

    /// Sets the global rotation of the shape.
    #[cfg(feature = "3d")]
    pub(crate) fn set_global_shape_rotation(&mut self, global_rotation: Quaternion) {
        self.global_shape_rotation = global_rotation;
    }

    /// Sets the global direction of the ray.
    pub(crate) fn set_global_direction(&mut self, global_direction: Dir) {
        self.global_direction = global_direction;
    }

    pub(crate) fn cast(
        &mut self,
        caster_entity: Entity,
        hits: &mut ShapeHits,
        spatial_query: &SpatialQuery,
    ) {
        if self.ignore_self {
            self.query_filter.excluded_entities.insert(caster_entity);
        } else {
            self.query_filter.excluded_entities.remove(&caster_entity);
        }

        hits.clear();

        let config = ShapeCastConfig {
            max_distance: self.max_distance,
            target_distance: self.target_distance,
            compute_contact_on_penetration: self.compute_contact_on_penetration,
            ignore_origin_penetration: self.ignore_origin_penetration,
        };

        if self.max_hits == 1 {
            let first_hit = spatial_query.cast_shape(
                &self.shape,
                self.global_origin,
                self.global_shape_rotation,
                self.global_direction,
                &config,
                &self.query_filter,
            );

            if let Some(hit) = first_hit {
                hits.push(hit);
            }
        } else {
            hits.extend(spatial_query.shape_hits(
                &self.shape,
                self.global_origin,
                self.global_shape_rotation,
                self.global_direction,
                self.max_hits,
                &config,
                &self.query_filter,
            ));
        }
    }
}

fn on_add_shape_caster(mut world: DeferredWorld, ctx: HookContext) {
    let shape_caster = world.get::<ShapeCaster>(ctx.entity).unwrap();
    let max_hits = if shape_caster.max_hits == u32::MAX {
        10
    } else {
        shape_caster.max_hits as usize
    };

    // Initialize capacity for hits
    world.get_mut::<ShapeHits>(ctx.entity).unwrap().0 = Vec::with_capacity(max_hits);
}

/// Configuration for a shape cast.
#[derive(Clone, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct ShapeCastConfig {
    /// The maximum distance the shape can travel.
    ///
    /// By default, this is infinite.
    #[doc(alias = "max_time_of_impact")]
    pub max_distance: Scalar,

    /// The separation distance at which the shapes will be considered as impacting.
    ///
    /// If the shapes are separated by a distance smaller than `target_distance` at the origin of the cast,
    /// the computed contact points and normals are only reliable if [`ShapeCastConfig::compute_contact_on_penetration`]
    /// is set to `true`.
    ///
    /// By default, this is `0.0`, so the shapes will only be considered as impacting when they first touch.
    pub target_distance: Scalar,

    /// If `true`, contact points and normals will be calculated even when the cast distance is `0.0`.
    ///
    /// The default is `true`.
    pub compute_contact_on_penetration: bool,

    /// If `true` *and* the shape is travelling away from the object that was hit,
    /// the cast will ignore any impact that happens at the cast origin.
    ///
    /// The default is `false`.
    pub ignore_origin_penetration: bool,
}

impl Default for ShapeCastConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl ShapeCastConfig {
    /// The default [`ShapeCastConfig`] configuration.
    pub const DEFAULT: Self = Self {
        max_distance: Scalar::MAX,
        target_distance: 0.0,
        compute_contact_on_penetration: true,
        ignore_origin_penetration: false,
    };

    /// Creates a new [`ShapeCastConfig`] with a given maximum distance the shape can travel.
    #[inline]
    pub const fn from_max_distance(max_distance: Scalar) -> Self {
        Self {
            max_distance,
            target_distance: 0.0,
            compute_contact_on_penetration: true,
            ignore_origin_penetration: false,
        }
    }

    /// Creates a new [`ShapeCastConfig`] with a given separation distance at which
    /// the shapes will be considered as impacting.
    #[inline]
    pub const fn from_target_distance(target_distance: Scalar) -> Self {
        Self {
            max_distance: Scalar::MAX,
            target_distance,
            compute_contact_on_penetration: true,
            ignore_origin_penetration: false,
        }
    }

    /// Sets the maximum distance the shape can travel.
    #[inline]
    pub const fn with_max_distance(mut self, max_distance: Scalar) -> Self {
        self.max_distance = max_distance;
        self
    }

    /// Sets the separation distance at which the shapes will be considered as impacting.
    #[inline]
    pub const fn with_target_distance(mut self, target_distance: Scalar) -> Self {
        self.target_distance = target_distance;
        self
    }
}

/// Contains the hits of a shape cast by a [`ShapeCaster`]. The hits are in the order of distance.
///
/// The maximum number of hits depends on the value of `max_hits` in [`ShapeCaster`]. By default only
/// one hit is computed, as shapecasting for many results can be expensive.
///
/// # Order
///
/// By default, the order of the hits is not guaranteed.
///
/// You can iterate the hits in the order of distance with `iter_sorted`.
/// Note that this will create and sort a new vector instead of iterating over the existing one.
///
/// **Note**: When there are more hits than `max_hits`, **some hits will be missed**.
/// If you want to guarantee that the closest hit is included, set `max_hits` to one.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// fn print_hits(query: Query<&ShapeHits, With<ShapeCaster>>) {
///     for hits in &query {
///         // For the faster iterator that isn't sorted, use `.iter()`.
///         for hit in hits.iter_sorted() {
///             println!("Hit entity {} with distance {}", hit.entity, hit.distance);
///         }
///     }
/// }
/// ```
#[derive(Component, Clone, Debug, Default, Deref, DerefMut, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
pub struct ShapeHits(pub Vec<ShapeHitData>);

impl ShapeHits {
    /// Returns an iterator over the hits, sorted in ascending order according to the distance.
    ///
    /// Note that this allocates a new vector. If you don't need the hits in order, use `iter`.
    pub fn iter_sorted(&self) -> alloc::vec::IntoIter<ShapeHitData> {
        let mut vector = self.as_slice().to_vec();
        vector.sort_by(|a, b| a.distance.partial_cmp(&b.distance).unwrap());
        vector.into_iter()
    }
}

impl IntoIterator for ShapeHits {
    type Item = ShapeHitData;
    type IntoIter = alloc::vec::IntoIter<ShapeHitData>;

    fn into_iter(self) -> Self::IntoIter {
        self.0.into_iter()
    }
}

impl<'a> IntoIterator for &'a ShapeHits {
    type Item = &'a ShapeHitData;
    type IntoIter = core::slice::Iter<'a, ShapeHitData>;

    fn into_iter(self) -> Self::IntoIter {
        self.0.iter()
    }
}

impl<'a> IntoIterator for &'a mut ShapeHits {
    type Item = &'a mut ShapeHitData;
    type IntoIter = core::slice::IterMut<'a, ShapeHitData>;

    fn into_iter(self) -> Self::IntoIter {
        self.0.iter_mut()
    }
}

impl MapEntities for ShapeHits {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        for hit in self {
            hit.map_entities(entity_mapper);
        }
    }
}

/// Data related to a hit during a [shapecast](spatial_query#shapecasting).
#[derive(Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct ShapeHitData {
    /// The entity of the collider that was hit by the shape.
    pub entity: Entity,

    /// How far the shape travelled before the initial hit.
    #[doc(alias = "time_of_impact")]
    pub distance: Scalar,

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
}

impl MapEntities for ShapeHitData {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.entity = entity_mapper.get_mapped(self.entity);
    }
}
