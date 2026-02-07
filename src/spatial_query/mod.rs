//! Functionality for performing ray casts, shape casts, and other spatial queries.
//!
//! Spatial queries query the world for geometric information about [`Collider`s](Collider)
//! and various types of intersections. Currently, four types of spatial queries are supported:
//!
//! - [Raycasts](#raycasting)
//! - [Shapecasts](#shapecasting)
//! - [Point projection](#point-projection)
//! - [Intersection tests](#intersection-tests)
//!
//! All spatial queries can be done using the various methods provided by the [`SpatialQuery`] system parameter.
//!
//! Raycasting and shapecasting can also be done with a component-based approach using the [`RayCaster`] and
//! [`ShapeCaster`] components. They enable performing casts every frame in a way that is often more convenient
//! than the normal [`SpatialQuery`] methods. See their documentation for more information.
//!
//! # Raycasting
//!
//! **Raycasting** is a spatial query that finds intersections between colliders and a half-line. This can be used for
//! a variety of things like getting information about the environment for character controllers and AI,
//! and even rendering using ray tracing.
//!
//! For each hit during raycasting, the hit entity, a distance, and a normal will be stored in [`RayHitData`].
//! The distance is the distance from the ray origin to the point of intersection, indicating how far the ray travelled.
//!
//! There are two ways to perform raycasts.
//!
//! 1. For simple raycasts, use the [`RayCaster`] component. It returns the results of the raycast
//!    in the [`RayHits`] component every frame. It uses local coordinates, so it will automatically follow the entity
//!    it's attached to or its parent.
//! 2. When you need more control or don't want to cast every frame, use the raycasting methods provided by
//!    [`SpatialQuery`], like [`cast_ray`](SpatialQuery::cast_ray), [`ray_hits`](SpatialQuery::ray_hits) or
//!    [`ray_hits_callback`](SpatialQuery::ray_hits_callback).
//!
//! See the documentation of the components and methods for more information.
//!
//! A simple example using the component-based method looks like this:
//!
//! ```
//! # #[cfg(feature = "2d")]
//! # use avian2d::prelude::*;
//! # #[cfg(feature = "3d")]
//! use avian3d::prelude::*;
//! use bevy::prelude::*;
//!
//! # #[cfg(all(feature = "3d", feature = "f32"))]
//! fn setup(mut commands: Commands) {
//!     // Spawn a ray caster at the center with the rays travelling right
//!     commands.spawn(RayCaster::new(Vec3::ZERO, Dir3::X));
//!     // ...spawn colliders and other things
//! }
//!
//! # #[cfg(all(feature = "3d", feature = "f32"))]
//! fn print_hits(query: Query<(&RayCaster, &RayHits)>) {
//!     for (ray, hits) in &query {
//!         // For the faster iterator that isn't sorted, use `.iter()`
//!         for hit in hits.iter_sorted() {
//!             println!(
//!                 "Hit entity {} at {} with normal {}",
//!                 hit.entity,
//!                 ray.origin + *ray.direction * hit.distance,
//!                 hit.normal,
//!             );
//!         }
//!     }
//! }
//! ```
//!
//! To specify which colliders should be considered in the query, use a [spatial query filter](`SpatialQueryFilter`).
//!
//! # Shapecasting
//!
//! **Shapecasting** or **sweep testing** is a spatial query that finds intersections between colliders and a shape
//! that is travelling along a half-line. It is very similar to [raycasting](#raycasting), but instead of a "point"
//! we have an entire shape travelling along a half-line. One use case is determining how far an object can move
//! before it hits the environment.
//!
//! For each hit during shapecasting, the hit entity, a distance, two world-space points of intersection and two world-space
//! normals will be stored in [`ShapeHitData`]. The distance refers to how far the shape travelled before the initial hit.
//!
//! There are two ways to perform shapecasts.
//!
//! 1. For simple shapecasts, use the [`ShapeCaster`] component. It returns the results of the shapecast
//!    in the [`ShapeHits`] component every frame. It uses local coordinates, so it will automatically follow the entity
//!    it's attached to or its parent.
//! 2. When you need more control or don't want to cast every frame, use the shapecasting methods provided by
//!    [`SpatialQuery`], like [`cast_shape`](SpatialQuery::cast_shape), [`shape_hits`](SpatialQuery::shape_hits) or
//!    [`shape_hits_callback`](SpatialQuery::shape_hits_callback).
//!
//! See the documentation of the components and methods for more information.
//!
//! A simple example using the component-based method looks like this:
//!
//! ```
//! # #[cfg(feature = "2d")]
//! # use avian2d::prelude::*;
//! # #[cfg(feature = "3d")]
//! use avian3d::prelude::*;
//! use bevy::prelude::*;
//!
//! # #[cfg(all(feature = "3d", feature = "f32"))]
//! fn setup(mut commands: Commands) {
//!     // Spawn a shape caster with a sphere shape at the center travelling right
//!     commands.spawn(ShapeCaster::new(
//!         Collider::sphere(0.5), // Shape
//!         Vec3::ZERO,            // Origin
//!         Quat::default(),       // Shape rotation
//!         Dir3::X                // Direction
//!     ));
//!     // ...spawn colliders and other things
//! }
//!
//! fn print_hits(query: Query<(&ShapeCaster, &ShapeHits)>) {
//!     for (shape_caster, hits) in &query {
//!         for hit in hits.iter() {
//!             println!("Hit entity {}", hit.entity);
//!         }
//!     }
//! }
//! ```
//!
//! To specify which colliders should be considered in the query, use a [spatial query filter](`SpatialQueryFilter`).
//!
//! # Point projection
//!
//! **Point projection** is a spatial query that projects a point on the closest collider. It returns the collider's
//! entity, the projected point, and whether the point is inside of the collider.
//!
//! Point projection can be done with the [`project_point`](SpatialQuery::project_point) method of the [`SpatialQuery`]
//! system parameter. See its documentation for more information.
//!
//! To specify which colliders should be considered in the query, use a [spatial query filter](`SpatialQueryFilter`).
//!
//! # Intersection tests
//!
//! **Intersection tests** are spatial queries that return the entities of colliders that are intersecting a given
//! shape or area.
//!
//! There are three types of intersection tests. They are all methods of the [`SpatialQuery`] system parameter,
//! and they all have callback variants that call a given callback on each intersection.
//!
//! - [`point_intersections`](SpatialQuery::point_intersections): Finds all entities with a collider that contains
//!   the given point.
//! - [`aabb_intersections_with_aabb`](SpatialQuery::aabb_intersections_with_aabb):
//!   Finds all entities with a [`ColliderAabb`] that is intersecting the given [`ColliderAabb`].
//! - [`shape_intersections`](SpatialQuery::shape_intersections): Finds all entities with a [collider](Collider)
//!   that is intersecting the given shape.
//!
//! See the documentation of the components and methods for more information.
//!
//! To specify which colliders should be considered in the query, use a [spatial query filter](`SpatialQueryFilter`).

mod query_filter;
mod ray_caster;
#[cfg(any(feature = "parry-f32", feature = "parry-f64"))]
mod shape_caster;
#[cfg(any(feature = "parry-f32", feature = "parry-f64"))]
mod system_param;

mod diagnostics;
pub use diagnostics::SpatialQueryDiagnostics;

pub use query_filter::*;
pub use ray_caster::*;
#[cfg(any(feature = "parry-f32", feature = "parry-f64"))]
pub use shape_caster::*;
#[cfg(any(feature = "parry-f32", feature = "parry-f64"))]
pub use system_param::*;

use crate::prelude::*;
use bevy::prelude::*;

/// Handles component-based [spatial queries](spatial_query) like [raycasting](spatial_query#raycasting)
/// and [shapecasting](spatial_query#shapecasting) with [`RayCaster`] and [`ShapeCaster`].
pub struct SpatialQueryPlugin;

impl Plugin for SpatialQueryPlugin {
    fn build(&self, app: &mut App) {
        let physics_schedule = app
            .get_schedule_mut(PhysicsSchedule)
            .expect("add PhysicsSchedule first");

        physics_schedule.add_systems(
            (
                update_ray_caster_positions,
                #[cfg(all(
                    feature = "default-collider",
                    any(feature = "parry-f32", feature = "parry-f64")
                ))]
                (update_shape_caster_positions, raycast, shapecast).chain(),
            )
                .chain()
                .in_set(PhysicsStepSystems::SpatialQuery),
        );
    }

    fn finish(&self, app: &mut App) {
        // Register timer diagnostics for spatial queries.
        app.register_physics_diagnostics::<SpatialQueryDiagnostics>();
    }
}

type RayCasterPositionQueryComponents = (
    &'static mut RayCaster,
    Option<&'static Position>,
    Option<&'static Rotation>,
    Option<&'static ChildOf>,
    Option<&'static GlobalTransform>,
);

#[allow(clippy::type_complexity)]
fn update_ray_caster_positions(
    mut rays: Query<RayCasterPositionQueryComponents>,
    parents: Query<
        (
            Option<&Position>,
            Option<&Rotation>,
            Option<&GlobalTransform>,
        ),
        With<Children>,
    >,
) {
    for (mut ray, position, rotation, parent, transform) in &mut rays {
        let origin = ray.origin;
        let direction = ray.direction;

        let global_position = position.copied().or(transform.map(Position::from));
        let global_rotation = rotation.copied().or(transform.map(Rotation::from));

        if let Some(global_position) = global_position {
            ray.set_global_origin(global_position.0 + rotation.map_or(origin, |rot| rot * origin));
        } else if parent.is_none() {
            ray.set_global_origin(origin);
        }

        if let Some(global_rotation) = global_rotation {
            let global_direction = global_rotation * ray.direction;
            ray.set_global_direction(global_direction);
        } else if parent.is_none() {
            ray.set_global_direction(direction);
        }

        if let Some(Ok((parent_position, parent_rotation, parent_transform))) =
            parent.map(|&ChildOf(parent)| parents.get(parent))
        {
            let parent_position = parent_position
                .copied()
                .or(parent_transform.map(Position::from));
            let parent_rotation = parent_rotation
                .copied()
                .or(parent_transform.map(Rotation::from));

            // Apply parent transformations
            if global_position.is_none()
                && let Some(position) = parent_position
            {
                let rotation = global_rotation.unwrap_or(parent_rotation.unwrap_or_default());
                ray.set_global_origin(position.0 + rotation * origin);
            }
            if global_rotation.is_none()
                && let Some(rotation) = parent_rotation
            {
                let global_direction = rotation * ray.direction;
                ray.set_global_direction(global_direction);
            }
        }
    }
}

#[cfg(any(feature = "parry-f32", feature = "parry-f64"))]
type ShapeCasterPositionQueryComponents = (
    &'static mut ShapeCaster,
    Option<&'static Position>,
    Option<&'static Rotation>,
    Option<&'static ChildOf>,
    Option<&'static GlobalTransform>,
);

#[cfg(any(feature = "parry-f32", feature = "parry-f64"))]
#[allow(clippy::type_complexity)]
fn update_shape_caster_positions(
    mut shape_casters: Query<ShapeCasterPositionQueryComponents>,
    parents: Query<
        (
            Option<&Position>,
            Option<&Rotation>,
            Option<&GlobalTransform>,
        ),
        With<Children>,
    >,
) {
    for (mut shape_caster, position, rotation, parent, transform) in &mut shape_casters {
        let origin = shape_caster.origin;
        let shape_rotation = shape_caster.shape_rotation;
        let direction = shape_caster.direction;

        let global_position = position.copied().or(transform.map(Position::from));
        let global_rotation = rotation.copied().or(transform.map(Rotation::from));

        if let Some(global_position) = global_position {
            shape_caster
                .set_global_origin(global_position.0 + rotation.map_or(origin, |rot| rot * origin));
        } else if parent.is_none() {
            shape_caster.set_global_origin(origin);
        }

        if let Some(global_rotation) = global_rotation {
            let global_direction = global_rotation * shape_caster.direction;
            shape_caster.set_global_direction(global_direction);
            #[cfg(feature = "2d")]
            {
                shape_caster
                    .set_global_shape_rotation(shape_rotation + global_rotation.as_radians());
            }
            #[cfg(feature = "3d")]
            {
                shape_caster.set_global_shape_rotation(shape_rotation * global_rotation.0);
            }
        } else if parent.is_none() {
            shape_caster.set_global_direction(direction);
            #[cfg(feature = "2d")]
            {
                shape_caster.set_global_shape_rotation(shape_rotation);
            }
            #[cfg(feature = "3d")]
            {
                shape_caster.set_global_shape_rotation(shape_rotation);
            }
        }

        if let Some(Ok((parent_position, parent_rotation, parent_transform))) =
            parent.map(|&ChildOf(parent)| parents.get(parent))
        {
            let parent_position = parent_position
                .copied()
                .or(parent_transform.map(Position::from));
            let parent_rotation = parent_rotation
                .copied()
                .or(parent_transform.map(Rotation::from));

            // Apply parent transformations
            if global_position.is_none()
                && let Some(position) = parent_position
            {
                let rotation = global_rotation.unwrap_or(parent_rotation.unwrap_or_default());
                shape_caster.set_global_origin(position.0 + rotation * origin);
            }
            if global_rotation.is_none()
                && let Some(rotation) = parent_rotation
            {
                let global_direction = rotation * shape_caster.direction;
                shape_caster.set_global_direction(global_direction);
                #[cfg(feature = "2d")]
                {
                    shape_caster.set_global_shape_rotation(shape_rotation + rotation.as_radians());
                }
                #[cfg(feature = "3d")]
                {
                    shape_caster.set_global_shape_rotation(shape_rotation * rotation.0);
                }
            }
        }
    }
}

#[cfg(any(feature = "parry-f32", feature = "parry-f64"))]
fn raycast(
    mut rays: Query<(Entity, &mut RayCaster, &mut RayHits)>,
    spatial_query: SpatialQuery,
    mut diagnostics: ResMut<SpatialQueryDiagnostics>,
) {
    let start = crate::utils::Instant::now();

    for (entity, mut ray_caster, mut hits) in &mut rays {
        if ray_caster.enabled {
            ray_caster.cast(entity, &mut hits, &spatial_query);
        } else if !hits.is_empty() {
            hits.clear();
        }
    }

    diagnostics.update_ray_casters = start.elapsed();
}

#[cfg(any(feature = "parry-f32", feature = "parry-f64"))]
fn shapecast(
    mut shape_casters: Query<(Entity, &mut ShapeCaster, &mut ShapeHits)>,
    spatial_query: SpatialQuery,
    mut diagnostics: ResMut<SpatialQueryDiagnostics>,
) {
    let start = crate::utils::Instant::now();

    for (entity, mut shape_caster, mut hits) in &mut shape_casters {
        if shape_caster.enabled {
            shape_caster.cast(entity, &mut hits, &spatial_query);
        } else if !hits.is_empty() {
            hits.clear();
        }
    }

    diagnostics.update_shape_casters = start.elapsed();
}
