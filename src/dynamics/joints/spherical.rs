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

/// A spherical [joint](dynamics::joints) prevents any relative translation between two bodies,
/// while allowing limited rotation about all axes.
///
/// This is similar to the ball-and-socket joint in the human shoulder, and can be useful for things like ragdolls, robotic arms,
/// pendula, chains, and other mechanisms where you need to allow free rotation around a point.
///
/// Each spherical joint is defined by a [`JointFrame`] on each body. The joint aims to keep the anchor point of each frame aligned,
/// while allowing free rotation.
///
#[doc = include_str!("./images/point_constraint.svg")]
///
/// The relative rotation can optionally be limited to a circular cone defined by a [`twist_axis`](Self::twist_axis) about which the bodies can twist,
/// a [`twist_limit`](Self::twist_limit) that defines the extents of the allowed twisting, and a [`swing_limit`](Self::swing_limit) that defines the extents
/// of the allowed swing as a half-angle.
///
#[doc = include_str!("./images/swing_twist_limit.svg")]
#[derive(Component, Clone, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, MapEntities, PartialEq)]
pub struct SphericalJoint {
    /// The first body constrained by the joint.
    pub body1: Entity,
    /// The second body constrained by the joint.
    pub body2: Entity,
    /// The reference frame of the first body, defining the joint anchor and basis
    /// relative to the body transform.
    pub frame1: JointFrame,
    /// The reference frame of the second body, defining the joint anchor and basis
    /// relative to the body transform.
    pub frame2: JointFrame,
    /// The local axis about which the bodies can twist relative to each other.
    ///
    /// This is also the axis passing through the [`swing_limit`](Self::swing_limit) cone,
    /// defining which axes the bodies can swing around.
    ///
    /// By default, this is the y-axis.
    pub twist_axis: Vector,
    /// The extents of the allowed relative rotation of the bodies about a swing axis perpendicular to the [`twist_axis`](SphericalJoint::twist_axis).
    pub swing_limit: Option<AngleLimit>,
    /// The extents of the allowed relative rotation of the bodies about the [`twist_axis`](SphericalJoint::twist_axis).
    pub twist_limit: Option<AngleLimit>,
    /// The compliance of the point-to-point constraint (inverse of stiffness, m / N).
    pub point_compliance: Scalar,
    /// The compliance for swing (inverse of stiffness, N * m / rad).
    pub swing_compliance: Scalar,
    /// The compliance for twist (inverse of stiffness, N * m / rad).
    pub twist_compliance: Scalar,
}

impl EntityConstraint<2> for SphericalJoint {
    fn entities(&self) -> [Entity; 2] {
        [self.body1, self.body2]
    }
}

impl SphericalJoint {
    /// The default [`twist_axis`](Self::twist_axis) for a spherical joint.
    pub const DEFAULT_TWIST_AXIS: Vector = Vector::Y;

    /// Creates a new [`SphericalJoint`] between two entities.
    #[inline]
    pub const fn new(body1: Entity, body2: Entity) -> Self {
        Self {
            body1,
            body2,
            frame1: JointFrame::IDENTITY,
            frame2: JointFrame::IDENTITY,
            twist_axis: Self::DEFAULT_TWIST_AXIS,
            swing_limit: None,
            twist_limit: None,
            point_compliance: 0.0,
            swing_compliance: 0.0,
            twist_compliance: 0.0,
        }
    }

    /// Sets the [`twist_axis`](Self::twist_axis) about which the bodies can twist relative to each other.
    ///
    /// This is also the axis passing through the [`swing_limit`](Self::swing_limit) cone,
    /// defining which axes the bodies can swing around.
    ///
    /// By default, this is the y-axis.
    #[inline]
    pub const fn with_twist_axis(mut self, twist_axis: Vector) -> Self {
        self.twist_axis = twist_axis;
        self
    }

    /// Sets the local [`JointFrame`] of the first body, configuring both the [`JointAnchor`] and [`JointBasis`].
    #[inline]
    pub fn with_local_frame1(mut self, frame: impl Into<Isometry>) -> Self {
        self.frame1 = JointFrame::local(frame);
        self
    }

    /// Sets the local [`JointFrame`] of the second body, configuring both the [`JointAnchor`] and [`JointBasis`].
    #[inline]
    pub fn with_local_frame2(mut self, frame: impl Into<Isometry>) -> Self {
        self.frame2 = JointFrame::local(frame);
        self
    }

    /// Sets the global anchor point on both bodies.
    ///
    /// This configures the [`JointAnchor`] of each [`JointFrame`].
    #[inline]
    pub const fn with_anchor(mut self, anchor: Vector) -> Self {
        self.frame1.anchor = JointAnchor::FromGlobal(anchor);
        self.frame2.anchor = JointAnchor::FromGlobal(anchor);
        self
    }

    /// Sets the local anchor point on the first body.
    ///
    /// This configures the [`JointAnchor`] of the first [`JointFrame`].
    #[inline]
    pub const fn with_local_anchor1(mut self, anchor: Vector) -> Self {
        self.frame1.anchor = JointAnchor::Local(anchor);
        self
    }

    /// Sets the local anchor point on the second body.
    ///
    /// This configures the [`JointAnchor`] of the second [`JointFrame`].
    #[inline]
    pub const fn with_local_anchor2(mut self, anchor: Vector) -> Self {
        self.frame2.anchor = JointAnchor::Local(anchor);
        self
    }

    /// Sets the global basis for both bodies.
    ///
    /// This configures the [`JointBasis`] of each [`JointFrame`].
    #[inline]
    pub fn with_basis(mut self, basis: impl Into<Rot>) -> Self {
        let basis = basis.into();
        self.frame1.basis = JointBasis::FromGlobal(basis);
        self.frame2.basis = JointBasis::FromGlobal(basis);
        self
    }

    /// Sets the local basis for the first body.
    ///
    /// This configures the [`JointBasis`] of the first [`JointFrame`].
    #[inline]
    pub fn with_local_basis1(mut self, basis: impl Into<Rot>) -> Self {
        self.frame1.basis = JointBasis::Local(basis.into());
        self
    }

    /// Sets the local basis for the second body.
    ///
    /// This configures the [`JointBasis`] of the second [`JointFrame`].
    #[inline]
    pub fn with_local_basis2(mut self, basis: impl Into<Rot>) -> Self {
        self.frame2.basis = JointBasis::Local(basis.into());
        self
    }

    /// Returns the local [`JointFrame`] of the first body.
    ///
    /// If the [`JointAnchor`] is set to [`FromGlobal`](JointAnchor::FromGlobal),
    /// and the local anchor has not yet been computed, or the [`JointBasis`] is set to
    /// [`FromGlobal`](JointBasis::FromGlobal), and the local basis has not yet
    /// been computed, this will return `None`.
    #[inline]
    pub fn local_frame1(&self) -> Option<Isometry> {
        self.frame1.get_local_isometry()
    }

    /// Returns the local [`JointFrame`] of the second body.
    ///
    /// If the [`JointAnchor`] is set to [`FromGlobal`](JointAnchor::FromGlobal),
    /// and the local anchor has not yet been computed, or the [`JointBasis`] is set to
    /// [`FromGlobal`](JointBasis::FromGlobal), and the local basis has not yet
    /// been computed, this will return `None`.
    #[inline]
    pub fn local_frame2(&self) -> Option<Isometry> {
        self.frame2.get_local_isometry()
    }

    /// Returns the local anchor point on the first body.
    ///
    /// If the [`JointAnchor`] is set to [`FromGlobal`](JointAnchor::FromGlobal),
    /// and the local anchor has not yet been computed, this will return `None`.
    #[inline]
    pub const fn local_anchor1(&self) -> Option<Vector> {
        match self.frame1.anchor {
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
        match self.frame2.anchor {
            JointAnchor::Local(anchor) => Some(anchor),
            _ => None,
        }
    }

    /// Returns the local basis of the first body.
    ///
    /// If the [`JointBasis`] is set to [`FromGlobal`](JointBasis::FromGlobal),
    /// and the local basis has not yet been computed, this will return `None`.
    #[inline]
    pub const fn local_basis1(&self) -> Option<Rot> {
        match self.frame1.basis {
            JointBasis::Local(basis) => Some(basis),
            _ => None,
        }
    }

    /// Returns the local basis of the second body.
    ///
    /// If the [`JointBasis`] is set to [`FromGlobal`](JointBasis::FromGlobal),
    /// and the local basis has not yet been computed, this will return `None`.
    #[inline]
    pub const fn local_basis2(&self) -> Option<Rot> {
        match self.frame2.basis {
            JointBasis::Local(basis) => Some(basis),
            _ => None,
        }
    }

    /// Returns the local twist axis of the first body.
    ///
    /// This is equivalent to rotating the [`twist_axis`](Self::twist_axis)
    /// by the local basis of [`frame1`](Self::frame1).
    ///
    /// If the [`JointBasis`] is set to [`FromGlobal`](JointBasis::FromGlobal),
    /// and the local basis has not yet been computed, this will return `None`.
    #[inline]
    pub fn local_twist_axis1(&self) -> Option<Vector> {
        match self.frame1.basis {
            JointBasis::Local(basis) => Some(basis * self.twist_axis),
            _ => None,
        }
    }

    /// Returns the local twist axis of the second body.
    ///
    /// This is equivalent to rotating the [`twist_axis`](Self::twist_axis)
    /// by the local basis of [`frame2`](Self::frame2).
    ///
    /// If the [`JointBasis`] is set to [`FromGlobal`](JointBasis::FromGlobal),
    /// and the local basis has not yet been computed, this will return `None`.
    #[inline]
    pub fn local_twist_axis2(&self) -> Option<Vector> {
        match self.frame2.basis {
            JointBasis::Local(basis) => Some(basis * self.twist_axis),
            _ => None,
        }
    }

    /// Sets the limits of the allowed relative rotation about a swing axis perpendicular to the [`twist_axis`](Self::twist_axis).
    #[inline]
    pub const fn with_swing_limits(mut self, min: Scalar, max: Scalar) -> Self {
        self.swing_limit = Some(AngleLimit::new(min, max));
        self
    }

    /// Sets the limits of the allowed relative rotation about the [`twist_axis`](Self::twist_axis).
    #[inline]
    pub const fn with_twist_limits(mut self, min: Scalar, max: Scalar) -> Self {
        self.twist_limit = Some(AngleLimit::new(min, max));
        self
    }

    /// Sets the joint's compliance (inverse of stiffness, m / N).
    #[inline]
    #[deprecated(
        since = "0.4.0",
        note = "Use `with_point_compliance`, `with_swing_compliance`, and `with_twist_compliance` instead."
    )]
    pub const fn with_compliance(mut self, compliance: Scalar) -> Self {
        self.point_compliance = compliance;
        self.swing_compliance = compliance;
        self.twist_compliance = compliance;
        self
    }

    /// Sets the compliance of the axis alignment constraint (inverse of stiffness, m / N).
    #[inline]
    pub const fn with_point_compliance(mut self, compliance: Scalar) -> Self {
        self.point_compliance = compliance;
        self
    }

    /// Sets the compliance of the swing limit (inverse of stiffness, N * m / rad).
    #[inline]
    pub const fn with_swing_compliance(mut self, compliance: Scalar) -> Self {
        self.swing_compliance = compliance;
        self
    }

    /// Sets the compliance of the twist limit (inverse of stiffness, N * m / rad).
    #[inline]
    pub const fn with_twist_compliance(mut self, compliance: Scalar) -> Self {
        self.twist_compliance = compliance;
        self
    }
}

impl MapEntities for SphericalJoint {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.body1 = entity_mapper.get_mapped(self.body1);
        self.body2 = entity_mapper.get_mapped(self.body2);
    }
}

pub(super) fn plugin(app: &mut App) {
    app.add_systems(
        PhysicsSchedule,
        update_local_frames.in_set(JointSystems::PrepareLocalFrames),
    );
}

fn update_local_frames(
    mut joints: Query<&mut SphericalJoint, Changed<SphericalJoint>>,
    bodies: Query<(&Position, &Rotation)>,
) {
    for mut joint in &mut joints {
        if matches!(joint.frame1.anchor, JointAnchor::Local(_))
            && matches!(joint.frame2.anchor, JointAnchor::Local(_))
            && matches!(joint.frame1.basis, JointBasis::Local(_))
            && matches!(joint.frame2.basis, JointBasis::Local(_))
        {
            continue;
        }

        let Ok([(pos1, rot1), (pos2, rot2)]) = bodies.get_many(joint.entities()) else {
            continue;
        };

        let [frame1, frame2] =
            JointFrame::compute_local(joint.frame1, joint.frame2, pos1.0, pos2.0, rot1, rot2);
        joint.frame1 = frame1;
        joint.frame2 = frame2;
    }
}

#[cfg(feature = "debug-plugin")]
impl DebugRenderConstraint<2> for SphericalJoint {
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

        let Some(local_anchor1) = self.local_anchor1() else {
            return;
        };
        let Some(local_anchor2) = self.local_anchor2() else {
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
