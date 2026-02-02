use crate::{
    dynamics::joints::{EntityConstraint, JointSystems, motor::AngularMotor},
    prelude::*,
};
use bevy::{
    ecs::{
        entity::{EntityMapper, MapEntities},
        reflect::ReflectMapEntities,
    },
    prelude::*,
};

#[cfg_attr(
    feature = "2d",
    doc = "A revolute [joint](dynamics::joints) or hinge prevents any relative movement between two bodies, except for rotation about a pivot point defined by the joint anchor."
)]
#[cfg_attr(
    feature = "3d",
    doc = "A revolute [joint](dynamics::joints) or hinge prevents any relative movement between two bodies, except for rotation about the [`hinge_axis`](Self::hinge_axis) at a pivot point defined by the joint anchor."
)]
///
/// This can be useful for things like wheels, fans, doors, and other rotating mechanisms.
///
#[cfg_attr(
    feature = "2d",
    doc = "Each revolute joint is defined by a [`JointFrame`] on each body,"
)]
#[cfg_attr(
    feature = "3d",
    doc = "Each revolute joint is defined by a [`JointFrame`] on each body, a [`hinge_axis`](Self::hinge_axis) about which the bodies can rotate,"
)]
/// and an optional [`AngleLimit`] that defines the extents of the allowed rotation. The joint aims to keep the anchor point of each frame aligned,
#[cfg_attr(feature = "2d", doc = "while allowing rotation at the anchor point.")]
#[cfg_attr(
    feature = "3d",
    doc = "while allowing rotation about the hinge axis at the anchor point."
)]
///
#[doc = include_str!("./images/revolute_joint.svg")]
///
/// The joint can also include an [`AngularMotor`] for driving the rotation about the pivot point.
/// Use this to create wheels, fans, servos, or other rotating mechanisms.
#[derive(Component, Clone, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, MapEntities, PartialEq)]
#[doc(alias = "HingeJoint")]
pub struct RevoluteJoint {
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
    /// The local axis about which the bodies can rotate relative to each other.
    ///
    /// By default, this is the z-axis.
    #[cfg(feature = "3d")]
    pub hinge_axis: Vector,
    /// The extents of the allowed relative rotation of the bodies.
    pub angle_limit: Option<AngleLimit>,
    /// The compliance of the point-to-point constraint (inverse of stiffness, m / N).
    pub point_compliance: Scalar,
    /// The compliance used for aligning the bodies along the [`hinge_axis`](Self::hinge_axis) (inverse of stiffness, N * m / rad).
    #[cfg(feature = "3d")]
    pub align_compliance: Scalar,
    /// The compliance of the angle limit (inverse of stiffness, N * m / rad).
    pub limit_compliance: Scalar,
    /// A motor for driving the joint.
    pub motor: AngularMotor,
}

impl EntityConstraint<2> for RevoluteJoint {
    fn entities(&self) -> [Entity; 2] {
        [self.body1, self.body2]
    }
}

impl RevoluteJoint {
    /// The default [`hinge_axis`](Self::hinge_axis) for a revolute joint.
    #[cfg(feature = "3d")]
    pub const DEFAULT_HINGE_AXIS: Vector = Vector::Z;

    /// Creates a new [`RevoluteJoint`] between two entities.
    #[inline]
    pub const fn new(body1: Entity, body2: Entity) -> Self {
        Self {
            body1,
            body2,
            frame1: JointFrame::IDENTITY,
            frame2: JointFrame::IDENTITY,
            #[cfg(feature = "3d")]
            hinge_axis: Self::DEFAULT_HINGE_AXIS,
            angle_limit: None,
            point_compliance: 0.0,
            #[cfg(feature = "3d")]
            align_compliance: 0.0,
            limit_compliance: 0.0,
            motor: AngularMotor::new_disabled(MotorModel::DEFAULT),
        }
    }

    /// Sets the [`hinge_axis`](Self::hinge_axis) about which the bodies can rotate relative to each other.
    ///
    /// The axis should be a unit vector. By default, this is the z-axis.
    #[inline]
    #[cfg(feature = "3d")]
    pub const fn with_hinge_axis(mut self, axis: Vector) -> Self {
        self.hinge_axis = axis;
        self
    }

    /// Sets the [`hinge_axis`](Self::hinge_axis) about which the bodies can rotate relative to each other.
    ///
    /// The axis should be a unit vector. By default, this is the x-axis.
    ///
    /// This method is deprecated in favor of [`with_hinge_axis`](Self::with_hinge_axis).
    #[inline]
    #[deprecated(since = "0.4.0", note = "Use `with_hinge_axis` instead.")]
    #[cfg(feature = "3d")]
    pub const fn with_aligned_axis(self, axis: Vector) -> Self {
        self.with_hinge_axis(axis)
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
    pub fn local_basis1(&self) -> Option<Rot> {
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
    pub fn local_basis2(&self) -> Option<Rot> {
        match self.frame2.basis {
            JointBasis::Local(basis) => Some(basis),
            _ => None,
        }
    }

    /// Returns the local hinge axis of the first body.
    ///
    /// This is equivalent to rotating the [`hinge_axis`](Self::hinge_axis)
    /// by the local basis of [`frame1`](Self::frame1).
    ///
    /// If the [`JointBasis`] is set to [`FromGlobal`](JointBasis::FromGlobal),
    /// and the local basis has not yet been computed, this will return `None`.
    #[inline]
    #[cfg(feature = "3d")]
    pub fn local_hinge_axis1(&self) -> Option<Vector> {
        match self.frame1.basis {
            JointBasis::Local(basis) => Some(basis * self.hinge_axis),
            _ => None,
        }
    }

    /// Returns the local hinge axis of the second body.
    ///
    /// This is equivalent to rotating the [`hinge_axis`](Self::hinge_axis)
    /// by the local basis of [`frame2`](Self::frame2).
    ///
    /// If the [`JointBasis`] is set to [`FromGlobal`](JointBasis::FromGlobal),
    /// and the local basis has not yet been computed, this will return `None`.
    #[inline]
    #[cfg(feature = "3d")]
    pub fn local_hinge_axis2(&self) -> Option<Vector> {
        match self.frame2.basis {
            JointBasis::Local(basis) => Some(basis * self.hinge_axis),
            _ => None,
        }
    }

    /// Sets the limits of the allowed relative rotation.
    #[inline]
    pub const fn with_angle_limits(mut self, min: Scalar, max: Scalar) -> Self {
        self.angle_limit = Some(AngleLimit::new(min, max));
        self
    }

    /// Sets the joint's compliance (inverse of stiffness, m / N).
    #[inline]
    #[deprecated(
        since = "0.4.0",
        note = "Use `with_point_compliance`, `with_align_compliance`, and `with_limit_compliance` instead."
    )]
    pub const fn with_compliance(mut self, compliance: Scalar) -> Self {
        self.point_compliance = compliance;
        #[cfg(feature = "3d")]
        {
            self.align_compliance = compliance;
        }
        self.limit_compliance = compliance;
        self
    }

    /// Sets the compliance of the point-to-point constraint (inverse of stiffness, m / N).
    #[inline]
    pub const fn with_point_compliance(mut self, compliance: Scalar) -> Self {
        self.point_compliance = compliance;
        self
    }

    /// Sets the compliance of the axis alignment constraint (inverse of stiffness, N * m / rad).
    #[inline]
    #[cfg(feature = "3d")]
    pub const fn with_align_compliance(mut self, compliance: Scalar) -> Self {
        self.align_compliance = compliance;
        self
    }

    /// Sets the compliance of the angle limit (inverse of stiffness, N * m / rad).
    #[inline]
    pub const fn with_limit_compliance(mut self, compliance: Scalar) -> Self {
        self.limit_compliance = compliance;
        self
    }

    /// Sets the motor for the joint.
    #[inline]
    pub const fn with_motor(mut self, motor: AngularMotor) -> Self {
        self.motor = motor;
        self
    }
}

impl MapEntities for RevoluteJoint {
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
    mut joints: Query<&mut RevoluteJoint, Changed<RevoluteJoint>>,
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
impl DebugRenderConstraint<2> for RevoluteJoint {
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
