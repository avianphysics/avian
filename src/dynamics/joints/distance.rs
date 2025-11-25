use crate::{
    dynamics::joints::{EntityConstraint, JointSystems},
    prelude::*,
};
use bevy::{
    ecs::{
        entity::{EntityMapper, MapEntities},
        reflect::ReflectMapEntities,
    },
    prelude::*,
};

/// A distance [joint](dynamics::joints) maintains an upper and/or lower bound on the distance
/// between anchor points on two bodies.
///
/// This can be useful for things like springs, muscles, and mass-spring networks.
///
/// A distance joint is defined by a [`JointAnchor`] on each body, and a [`DistanceLimit`]. The joint aims to keep
/// the distance between the two anchor points within the specified limits.
///
#[doc = include_str!("./images/distance_joint.svg")]
#[derive(Component, Clone, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, MapEntities, PartialEq)]
pub struct DistanceJoint {
    /// The first body constrained by the joint.
    pub body1: Entity,
    /// The second body constrained by the joint.
    pub body2: Entity,
    /// The joint anchor point on the first body.
    pub anchor1: JointAnchor,
    /// The joint anchor point on the second body.
    pub anchor2: JointAnchor,
    /// The extents of the allowed relative translation between the attached bodies.
    pub limits: DistanceLimit,
    /// The joint's compliance, the inverse of stiffness (m / N).
    pub compliance: Scalar,
}

impl EntityConstraint<2> for DistanceJoint {
    fn entities(&self) -> [Entity; 2] {
        [self.body1, self.body2]
    }
}

impl DistanceJoint {
    /// Creates a new [`DistanceJoint`] between two entities.
    #[inline]
    pub const fn new(body1: Entity, body2: Entity) -> Self {
        Self {
            body1,
            body2,
            anchor1: JointAnchor::ZERO,
            anchor2: JointAnchor::ZERO,
            limits: DistanceLimit::ZERO,
            compliance: 0.0,
        }
    }

    /// Sets the global anchor point on both bodies.
    ///
    /// This configures the [`JointAnchor`] of each [`JointFrame`].
    #[inline]
    pub const fn with_anchor(mut self, anchor: Vector) -> Self {
        self.anchor1 = JointAnchor::FromGlobal(anchor);
        self.anchor2 = JointAnchor::FromGlobal(anchor);
        self
    }

    /// Sets the local anchor point on the first body.
    ///
    /// This configures the [`JointAnchor`] of the first [`JointFrame`].
    #[inline]
    pub const fn with_local_anchor1(mut self, anchor: Vector) -> Self {
        self.anchor1 = JointAnchor::Local(anchor);
        self
    }

    /// Sets the local anchor point on the second body.
    ///
    /// This configures the [`JointAnchor`] of the second [`JointFrame`].
    #[inline]
    pub const fn with_local_anchor2(mut self, anchor: Vector) -> Self {
        self.anchor2 = JointAnchor::Local(anchor);
        self
    }

    /// Returns the local anchor point on the first body.
    ///
    /// If the [`JointAnchor`] is set to [`FromGlobal`](JointAnchor::FromGlobal),
    /// and the local anchor has not yet been computed, this will return `None`.
    #[inline]
    pub const fn local_anchor1(&self) -> Option<Vector> {
        match self.anchor1 {
            JointAnchor::Local(anchor) => Some(anchor),
            _ => None,
        }
    }

    /// Returns the local anchor point on the second body.
    ///
    /// If the [`JointAnchor`] is set to [`FromGlobal`](JointAnchor::FromGlobal),
    /// and the local anchor has not yet been computed, this will return `None`.
    #[inline]
    pub const fn local_anchor2(&self) -> Option<Vector> {
        match self.anchor2 {
            JointAnchor::Local(anchor) => Some(anchor),
            _ => None,
        }
    }

    /// Sets the joint's rest length, or distance the bodies will be kept at.
    #[inline]
    #[deprecated(note = "Use `with_limits` instead.", since = "0.4.0")]
    pub const fn with_rest_length(mut self, rest_length: Scalar) -> Self {
        self.limits = DistanceLimit::new(rest_length, rest_length);
        self
    }

    /// Sets the minimum and maximum distance between the anchor points of the bodies.
    ///
    /// `min` and `max` should be non-negative, and `min` should be less than or equal to `max`.
    #[inline]
    pub const fn with_limits(mut self, min: Scalar, max: Scalar) -> Self {
        self.limits = DistanceLimit::new(min, max);
        self
    }

    /// Sets the minimum distance between the anchor points of the bodies.
    ///
    /// If `min` is larger than the current maximum distance, the maximum distance will be set to `min`.
    #[inline]
    pub const fn with_min_distance(mut self, min: Scalar) -> Self {
        self.limits.min = min;
        if self.limits.max < min {
            self.limits.max = min;
        }
        self
    }

    /// Sets the maximum distance between the anchor points of the bodies.
    ///
    /// If `max` is smaller than the current minimum distance, the minimum distance will be set to `max`.
    #[inline]
    pub const fn with_max_distance(mut self, max: Scalar) -> Self {
        self.limits.max = max;
        if self.limits.min > max {
            self.limits.min = max;
        }
        self
    }

    /// Sets the joint's compliance (inverse of stiffness, m / N).
    #[inline]
    pub const fn with_compliance(mut self, compliance: Scalar) -> Self {
        self.compliance = compliance;
        self
    }
}

impl MapEntities for DistanceJoint {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.body1 = entity_mapper.get_mapped(self.body1);
        self.body2 = entity_mapper.get_mapped(self.body2);
    }
}

pub(super) fn plugin(app: &mut App) {
    app.add_systems(
        PhysicsSchedule,
        update_local_anchors.in_set(JointSystems::PrepareLocalFrames),
    );
}

fn update_local_anchors(
    mut joints: Query<&mut DistanceJoint, Changed<DistanceJoint>>,
    bodies: Query<(&Position, &Rotation)>,
) {
    for mut joint in &mut joints {
        if matches!(joint.anchor1, JointAnchor::Local(_))
            && matches!(joint.anchor2, JointAnchor::Local(_))
        {
            continue;
        }

        let Ok([(pos1, rot1), (pos2, rot2)]) = bodies.get_many(joint.entities()) else {
            continue;
        };

        let [anchor1, anchor2] =
            JointAnchor::compute_local(joint.anchor1, joint.anchor2, pos1.0, pos2.0, rot1, rot2);
        joint.anchor1 = anchor1;
        joint.anchor2 = anchor2;
    }
}

#[cfg(feature = "debug-plugin")]
impl DebugRenderConstraint<2> for DistanceJoint {
    type Context = ();

    fn debug_render(
        &self,
        positions: [Vector; 2],
        rotations: [Rotation; 2],
        _context: &mut Self::Context,
        gizmos: &mut Gizmos<PhysicsGizmos>,
        config: &PhysicsGizmos,
    ) {
        let [pos1, pos2] = positions;
        let [rot1, rot2] = rotations;

        let JointAnchor::Local(local_anchor1) = self.anchor1 else {
            return;
        };
        let JointAnchor::Local(local_anchor2) = self.anchor2 else {
            return;
        };

        let anchor1 = pos1 + rot1 * local_anchor1;
        let anchor2 = pos2 + rot2 * local_anchor2;

        if let Some(anchor_color) = config.joint_anchor_color {
            gizmos.draw_line(pos1, anchor1, anchor_color);
            gizmos.draw_line(pos2, anchor2, anchor_color);
        }

        if let Some(color) = config.joint_separation_color {
            gizmos.draw_line(anchor1, anchor2, color);
        }
    }
}
