use crate::prelude::*;
use bevy::{prelude::*, utils::Parallel};

#[derive(Resource, Deref, DerefMut, Default)]
pub struct SpatialQueries(Parallel<Vec<DebugSpatialQuery>>);

pub struct DebugSpatialQuery {
    pub position: RVector,
    pub data: DebugSpatialQueryData,
}

pub enum DebugSpatialQueryData {
    Raycast {
        direction: Dir,
        max_distance: f32,
        hits: Vec<RayHitData>,
    },
    Shapecast {
        shape: Collider,
        direction: Dir,
        rotation: Rot,
        max_distance: f32,
        hits: Vec<ShapeHitData>,
    },
    PointProjection {
        projection: RVector,
    },
    ShapeIntersections {
        shape: Collider,
        rotation: Rot,
        hits: Vec<(Position, Rotation, Collider)>,
    },
}
