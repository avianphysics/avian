use crate::prelude::*;
use bevy::{ecs::entity::hash_set::EntityHashSet, prelude::*, utils::Parallel};

/// A resource that tracks spatial queries performed during the frame
/// for debugging purposes.
// TODO: We could consider moving this out of the debug rendering plugin
//       and exposing it as a more general way to record spatial queries.
#[derive(Resource, Deref, DerefMut, Default)]
pub struct TrackedSpatialQueries {
    #[deref]
    queries: Parallel<Vec<TrackedSpatialQuery>>,
    /// Whether spatial queries should be tracked, mirroring [`GizmoConfig::enabled`]
    /// for [`PhysicsGizmos`].
    ///
    /// This is cached here so that spatial queries only need a single boolean check,
    /// rather than a [`GizmoConfigStore`] lookup on every call.
    pub enabled: bool,
}

/// A spatial query that has been tracked for debugging purposes.
pub enum TrackedSpatialQuery {
    Raycast {
        origin: RVector,
        direction: Dir,
        max_distance: f32,
        hits: Vec<RayHitData>,
    },
    Shapecast {
        shape: Collider,
        origin: RVector,
        rotation: Rot,
        direction: Dir,
        max_distance: f32,
        hits: Vec<ShapeHitData>,
    },
    PointProjection {
        point: RVector,
        projection: RVector,
    },
    ShapeIntersections {
        shape: Collider,
        position: RVector,
        rotation: Rot,
        hits: Vec<Entity>,
    },
}

/// A resource containing the collider entities that were hit by a [shape intersection]
/// test tracked during the frame.
///
/// [shape intersection]: SpatialQuery::shape_intersections
#[derive(Resource, Deref, DerefMut, Default)]
pub struct TrackedShapeIntersections(EntityHashSet);
