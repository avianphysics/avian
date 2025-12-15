//! Contact types and data structures used in the collision pipeline.

mod contact_graph;
mod feature_id;
mod system_param;

pub use contact_graph::{ContactGraph, ContactGraphInternal};
pub use feature_id::PackedFeatureId;
use smallvec::SmallVec;
pub use system_param::Collisions;

use crate::{
    data_structures::graph::EdgeIndex,
    dynamics::solver::{constraint_graph::ContactConstraintHandle, islands::IslandNode},
    prelude::*,
};
use bevy::prelude::*;

/// A stable identifier for a [`ContactEdge`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct ContactId(pub u32);

impl ContactId {
    /// A placeholder identifier for a [`ContactEdge`].
    ///
    /// Used as a temporary value before the contact pair is added to the [`ContactGraph`].
    pub const PLACEHOLDER: Self = Self(u32::MAX);
}

impl From<ContactId> for EdgeIndex {
    fn from(id: ContactId) -> Self {
        Self(id.0)
    }
}

impl From<EdgeIndex> for ContactId {
    fn from(id: EdgeIndex) -> Self {
        Self(id.0)
    }
}

impl core::fmt::Display for ContactId {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "ContactId({})", self.0)
    }
}

/// Cold contact data stored in the [`ContactGraph`]. Used as a persistent handle for a [`ContactPair`].
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct ContactEdge {
    /// The stable identifier of this contact edge.
    pub id: ContactId,

    /// The first collider entity in the contact.
    pub collider1: Entity,

    /// The second collider entity in the contact.
    pub collider2: Entity,

    /// The entity of the first body involved in the contact.
    pub body1: Option<Entity>,

    /// The entity of the second body involved in the contact.
    pub body2: Option<Entity>,

    /// The index of the [`ContactPair`] in the [`ContactGraph`].
    pub pair_index: usize,

    /// The handles to the constraints associated with this contact edge.
    #[cfg(feature = "2d")]
    pub constraint_handles: SmallVec<[ContactConstraintHandle; 2]>,

    /// The handles to the constraints associated with this contact edge.
    #[cfg(feature = "3d")]
    pub constraint_handles: SmallVec<[ContactConstraintHandle; 4]>,

    /// The [`IslandNode`] associated with this contact edge.
    pub island: Option<IslandNode<ContactId>>,

    /// Flags for the contact edge.
    pub flags: ContactEdgeFlags,
}

impl ContactEdge {
    /// Creates a new non-touching [`ContactEdge`] with the given entities.
    #[inline]
    pub fn new(collider1: Entity, collider2: Entity) -> Self {
        Self {
            // This gets set to a valid ID when the contact pair is added to the `ContactGraph`.
            id: ContactId::PLACEHOLDER,
            collider1,
            collider2,
            body1: None,
            body2: None,
            pair_index: 0,
            constraint_handles: SmallVec::new(),
            island: None,
            flags: ContactEdgeFlags::empty(),
        }
    }

    /// Returns `true` if the colliders are touching.
    #[inline]
    pub fn is_touching(&self) -> bool {
        self.flags.contains(ContactEdgeFlags::TOUCHING)
    }

    /// Returns `true` if the contact pair is between sleeping bodies.
    #[inline]
    pub fn is_sleeping(&self) -> bool {
        self.flags.contains(ContactEdgeFlags::SLEEPING)
    }

    /// Returns `true` if collision events are enabled for the contact.
    #[inline]
    pub fn events_enabled(&self) -> bool {
        self.flags.contains(ContactEdgeFlags::CONTACT_EVENTS)
    }
}

// These are stored separately from `ContactPairFlags` to avoid needing to fetch
// the `ContactPair` when for example querying for touching contacts.
/// Flags for a [`ContactEdge`].
#[repr(transparent)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[derive(Hash, Clone, Copy, PartialEq, Eq, Debug, Reflect)]
#[reflect(opaque, Hash, PartialEq, Debug)]
pub struct ContactEdgeFlags(u8);

bitflags::bitflags! {
    impl ContactEdgeFlags: u8 {
        /// Set if the colliders are touching, including sensors.
        const TOUCHING = 1 << 0;
        /// Set if the contact pair is between sleeping bodies.
        const SLEEPING = 1 << 1;
        /// Set if the contact pair should emit contact events or sensor events.
        const CONTACT_EVENTS = 1 << 2;
    }
}

/// A contact pair between two colliders.
///
/// Each contact pair has one or more [contact manifolds](ContactManifold),
/// which represent contact surfaces between the two colliders.
/// Each of these manifolds contains one or more [contact points](ContactPoint).
///
/// Contact pairs exist in the [`ContactGraph`] between colliders whose [`ColliderAabb`]s
/// are overlapping, even if the colliders themselves are not touching.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct ContactPair {
    /// The stable identifier of the [`ContactEdge`] in the [`ContactGraph`].
    pub contact_id: ContactId,
    /// The first collider entity in the contact.
    pub collider1: Entity,
    /// The second collider entity in the contact.
    pub collider2: Entity,
    /// The entity of the first body involved in the contact.
    pub body1: Option<Entity>,
    /// The entity of the second body involved in the contact.
    pub body2: Option<Entity>,
    /// A list of contact manifolds between two colliders.
    /// Each manifold contains one or more contact points, but each contact
    /// in a given manifold shares the same contact normal.
    pub manifolds: Vec<ContactManifold>,
    /// The number of manifolds that were added or removed during the current time step.
    ///
    /// This is used to manage contact constraint updates for persistent contacts in the [`ConstraintGraph`].
    ///
    /// [`ConstraintGraph`]: crate::dynamics::solver::constraint_graph::ConstraintGraph
    pub(crate) manifold_count_change: i16,
    /// Flag indicating the status and type of the contact pair.
    pub flags: ContactPairFlags,
}

/// Flags indicating the status and type of a [contact pair](ContactPair).
#[repr(transparent)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[derive(Hash, Clone, Copy, PartialEq, Eq, Debug, Reflect)]
#[reflect(opaque, Hash, PartialEq, Debug)]
pub struct ContactPairFlags(u16);

bitflags::bitflags! {
    impl ContactPairFlags: u16 {
        /// Set if the colliders are touching, including sensors.
        const TOUCHING = 1 << 0;
        /// Set if the AABBs of the colliders are no longer overlapping.
        const DISJOINT_AABB = 1 << 1;
        /// Set if the colliders are touching and were not touching previously.
        const STARTED_TOUCHING = 1 << 2;
        /// Set if the colliders are not touching and were touching previously.
        const STOPPED_TOUCHING = 1 << 3;
        /// Set if the contact pair should generate contact constraints.
        const GENERATE_CONSTRAINTS = 1 << 4;
        /// Set if the contact pair just started generating contact constraints,
        /// for example because a sensor became a normal collider or a collider was attached to a rigid body.
        const STARTED_GENERATING_CONSTRAINTS = 1 << 5;
        /// Set if the first rigid body is static.
        const STATIC1 = 1 << 6;
        /// Set if the second rigid body is static.
        const STATIC2 = 1 << 7;
        /// Set if the contact pair should have a custom contact modification hook applied.
        const MODIFY_CONTACTS = 1 << 8;
    }
}

impl ContactPair {
    /// Creates a new [`ContactPair`] with the given entities.
    #[inline]
    pub fn new(collider1: Entity, collider2: Entity, contact_id: ContactId) -> Self {
        Self {
            contact_id,
            collider1,
            collider2,
            body1: None,
            body2: None,
            manifolds: Vec::new(),
            manifold_count_change: 0,
            flags: ContactPairFlags::empty(),
        }
    }

    /// Computes the sum of all impulses applied along contact normals between the contact pair.
    ///
    /// To get the corresponding force, divide the impulse by the time step.
    #[inline]
    pub fn total_normal_impulse(&self) -> Vector {
        self.manifolds.iter().fold(Vector::ZERO, |acc, manifold| {
            acc + manifold.normal * manifold.total_normal_impulse()
        })
    }

    /// Computes the sum of the magnitudes of all impulses applied along contact normals between the contact pair.
    ///
    /// This is the sum of impulse magnitudes, *not* the magnitude of the [`total_normal_impulse`](Self::total_normal_impulse).
    ///
    /// To get the corresponding force, divide the impulse by the time step.
    #[inline]
    pub fn total_normal_impulse_magnitude(&self) -> Scalar {
        self.manifolds
            .iter()
            .fold(0.0, |acc, manifold| acc + manifold.total_normal_impulse())
    }

    // TODO: We could also return a reference to the whole manifold. Would that be useful?
    /// Finds the largest normal impulse between the contact pair, pointing along the world-space contact normal
    /// from the first shape to the second.
    ///
    /// To get the corresponding force, divide the impulse by the time step.
    #[inline]
    pub fn max_normal_impulse(&self) -> Vector {
        let mut magnitude: Scalar = Scalar::MIN;
        let mut normal = Vector::ZERO;

        for manifold in &self.manifolds {
            let impulse = manifold.max_normal_impulse();
            if impulse > magnitude {
                magnitude = impulse;
                normal = manifold.normal;
            }
        }
        normal * magnitude
    }

    /// Finds the magnitude of the largest normal impulse between the contact pair.
    ///
    /// To get the corresponding force, divide the impulse by the time step.
    #[inline]
    pub fn max_normal_impulse_magnitude(&self) -> Scalar {
        self.manifolds
            .iter()
            .fold(0.0, |acc, manifold| acc.max(manifold.max_normal_impulse()))
    }

    /// Returns `true` if the colliders are touching, including sensors.
    #[inline]
    pub fn is_touching(&self) -> bool {
        self.flags.contains(ContactPairFlags::TOUCHING)
    }

    /// Returns `true` if the AABBs of the colliders are no longer overlapping.
    #[inline]
    pub fn aabbs_disjoint(&self) -> bool {
        self.flags.contains(ContactPairFlags::DISJOINT_AABB)
    }

    /// Returns `true` if a collision started during the current frame.
    #[inline]
    pub fn collision_started(&self) -> bool {
        self.flags.contains(ContactPairFlags::STARTED_TOUCHING)
    }

    /// Returns `true` if a collision ended during the current frame.
    #[inline]
    pub fn collision_ended(&self) -> bool {
        self.flags.contains(ContactPairFlags::STOPPED_TOUCHING)
    }

    /// Returns `true` if the contact pair should generate contact constraints.
    ///
    /// This is typically `true` unless the contact pair involves a [`Sensor`] or a disabled rigid body.
    #[inline]
    pub fn generates_constraints(&self) -> bool {
        self.flags.contains(ContactPairFlags::GENERATE_CONSTRAINTS)
    }

    /// Returns the contact with the largest penetration depth.
    ///
    /// If the objects are separated but there is still a speculative contact,
    /// the penetration depth will be negative.
    ///
    /// If there are no contacts, `None` is returned.
    #[inline]
    pub fn find_deepest_contact(&self) -> Option<&ContactPoint> {
        self.manifolds
            .iter()
            .filter_map(|manifold| manifold.find_deepest_contact())
            .max_by(|a, b| {
                a.penetration
                    .partial_cmp(&b.penetration)
                    .unwrap_or(core::cmp::Ordering::Equal)
            })
    }
}

/// A contact manifold describing a contact surface between two colliders,
/// represented by a set of [contact points](ContactPoint) and surface properties.
///
/// A manifold can typically be a single point, a line segment, or a polygon formed by its contact points.
/// Each contact point in a manifold shares the same contact normal.
#[cfg_attr(
    feature = "2d",
    doc = "
In 2D, contact manifolds are limited to 2 points."
)]
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct ContactManifold {
    /// The contact points in this manifold. Limited to 2 points in 2D.
    ///
    /// Each point in a manifold shares the same `normal`.
    #[cfg(feature = "2d")]
    pub points: arrayvec::ArrayVec<ContactPoint, 2>,
    /// The contact points in this manifold.
    ///
    /// Each point in a manifold shares the same `normal`.
    #[cfg(feature = "3d")]
    // TODO: Store a maximum of 4 points in 3D in an `ArrayVec`.
    pub points: Vec<ContactPoint>,
    /// The unit contact normal in world space, pointing from the first shape to the second.
    ///
    /// The same normal is shared by all `points` in a manifold.
    pub normal: Vector,
    /// The effective coefficient of dynamic [friction](Friction) used for the contact surface.
    pub friction: Scalar,
    /// The effective coefficient of [restitution](Restitution) used for the contact surface.
    pub restitution: Scalar,
    /// The desired relative linear speed of the bodies along the surface,
    /// expressed in world space as `tangent_speed2 - tangent_speed1`.
    ///
    /// Defaults to zero. If set to a non-zero value, this can be used to simulate effects
    /// such as conveyor belts.
    #[cfg(feature = "2d")]
    pub tangent_speed: Scalar,
    // TODO: Jolt also supports a relative angular surface velocity, which can be used for making
    //       objects rotate on platforms. Would that be useful enough to warrant the extra memory usage?
    /// The desired relative linear velocity of the bodies along the surface,
    /// expressed in world space as `tangent_velocity2 - tangent_velocity1`.
    ///
    /// Defaults to zero. If set to a non-zero value, this can be used to simulate effects
    /// such as conveyor belts.
    #[cfg(feature = "3d")]
    pub tangent_velocity: Vector,
}

impl ContactManifold {
    /// Creates a new [`ContactManifold`] with the given contact points and surface normals,
    /// expressed in local space.
    ///
    /// `index` represents the index of the manifold in the collision.
    #[inline]
    pub fn new(points: impl IntoIterator<Item = ContactPoint>, normal: Vector) -> Self {
        Self {
            #[cfg(feature = "2d")]
            points: arrayvec::ArrayVec::from_iter(points),
            #[cfg(feature = "3d")]
            points: points.into_iter().collect(),
            normal,
            friction: 0.0,
            restitution: 0.0,
            #[cfg(feature = "2d")]
            tangent_speed: 0.0,
            #[cfg(feature = "3d")]
            tangent_velocity: Vector::ZERO,
        }
    }

    /// The sum of the impulses applied at the contact points in the manifold along the contact normal.
    #[inline]
    pub fn total_normal_impulse(&self) -> Scalar {
        self.points
            .iter()
            .fold(0.0, |acc, contact| acc + contact.normal_impulse)
    }

    /// The magnitude of the largest impulse applied at a contact point in the manifold along the contact normal.
    #[inline]
    pub fn max_normal_impulse(&self) -> Scalar {
        self.points
            .iter()
            .map(|contact| contact.normal_impulse)
            .max_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal))
            .unwrap_or(0.0)
    }

    /// Copies impulses from previous contacts to matching contacts in `self`.
    ///
    /// Contacts are first matched based on their [feature IDs](PackedFeatureId), and if they are unknown,
    /// matching is done based on contact positions using the given `distance_threshold`
    /// for determining if points are too far away from each other to be considered matching.
    #[inline]
    pub fn match_contacts(
        &mut self,
        previous_contacts: &[ContactPoint],
        distance_threshold: Scalar,
    ) {
        // The squared maximum distance for two contact points to be considered matching.
        let distance_threshold_squared = distance_threshold.powi(2);

        for contact in self.points.iter_mut() {
            for previous_contact in previous_contacts.iter() {
                // If the feature IDs match, copy the contact impulses over for warm starting.
                if (contact.feature_id1 == previous_contact.feature_id1
                    && contact.feature_id2 == previous_contact.feature_id2) ||
                    // we have to check both directions because the entities are sorted in order
                    // of aabb.min.x, which could have changed even the two objects in contact are the same
                    (contact.feature_id2 == previous_contact.feature_id1
                    && contact.feature_id1 == previous_contact.feature_id2)
                {
                    contact.warm_start_normal_impulse = previous_contact.warm_start_normal_impulse;
                    contact.warm_start_tangent_impulse =
                        previous_contact.warm_start_tangent_impulse;
                    break;
                }

                let unknown_features = contact.feature_id1 == PackedFeatureId::UNKNOWN
                    || contact.feature_id2 == PackedFeatureId::UNKNOWN;

                // If the feature IDs are unknown and the contact positions match closely enough,
                // copy the contact impulses over for warm starting.
                if unknown_features
                    && (contact.anchor1.distance_squared(previous_contact.anchor1)
                        < distance_threshold_squared
                        && contact.anchor2.distance_squared(previous_contact.anchor2)
                            < distance_threshold_squared)
                    || (contact.anchor1.distance_squared(previous_contact.anchor2)
                        < distance_threshold_squared
                        && contact.anchor2.distance_squared(previous_contact.anchor1)
                            < distance_threshold_squared)
                {
                    contact.warm_start_normal_impulse = previous_contact.warm_start_normal_impulse;
                    contact.warm_start_tangent_impulse =
                        previous_contact.warm_start_tangent_impulse;
                    break;
                }
            }
        }
    }

    /// Prunes the contact points in the manifold to a maximum of 4 points.
    /// This is done to improve performance and stability.
    #[inline]
    #[cfg(feature = "3d")]
    pub fn prune_points(&mut self) {
        // Based on `PruneContactPoints` in Jolt by Jorrit Rouwe.
        // https://github.com/jrouwe/JoltPhysics/blob/f3dbdd2dadac4a5510391f103f264c0427d55c50/Jolt/Physics/Collision/ManifoldBetweenTwoFaces.cpp#L16

        debug_assert!(
            self.points.len() > 4,
            "`ContactManifold::prune_points` called on a manifold with 4 or fewer points"
        );

        // We use a heuristic of `distance_to_com * penetration` to find the points we should keep.
        // Neither of these should become zero, so we clamp the minimum distance.
        const MIN_DISTANCE_SQUARED: Scalar = 1e-6;

        // Project the contact points onto the contact normal and compute the squared penetration depths.
        let (projected, penetrations_squared): (Vec<Vector>, Vec<Scalar>) = self
            .points
            .iter()
            .map(|point| {
                (
                    point.anchor1.reject_from_normalized(self.normal),
                    (point.penetration * point.penetration).max(MIN_DISTANCE_SQUARED),
                )
            })
            .unzip();

        // Find the point that is farthest from the center of mass, as its torque has the largest influence,
        // while also taking into account the penetration depth heuristic.
        let mut point1_index = 0;
        let mut value = Scalar::MIN;
        for (i, point) in projected.iter().enumerate() {
            let v = point.length_squared().max(MIN_DISTANCE_SQUARED) * penetrations_squared[i];
            if v > value {
                value = v;
                point1_index = i;
            }
        }
        let point1 = projected[point1_index];

        // Find the point farthest from the first point, forming a line segment.
        // Also consider the penetration depth heuristic.
        let mut point2_index = usize::MAX;
        let mut max_distance = Scalar::MIN;
        for (i, point) in projected.iter().enumerate() {
            if i == point1_index {
                continue;
            }
            let v =
                point.distance_squared(point1).max(MIN_DISTANCE_SQUARED) * penetrations_squared[i];
            if v > max_distance {
                max_distance = v;
                point2_index = i;
            }
        }
        debug_assert!(point2_index != usize::MAX, "No second point found");
        let point2 = projected[point2_index];

        // Find the farthest points on both sides of the line segment in order to maximize the area.
        let mut point3_index = usize::MAX;
        let mut point4_index = usize::MAX;
        let mut min_value = 0.0;
        let mut max_value = 0.0;
        let perp = (point2 - point1).cross(self.normal);
        for (i, point) in projected.iter().enumerate() {
            if i == point1_index || i == point2_index {
                continue;
            }
            let v = perp.dot(point - point1);
            if v < min_value {
                min_value = v;
                point3_index = i;
            } else if v > max_value {
                max_value = v;
                point4_index = i;
            }
        }

        // Construct the manifold, ensuring the order is correct to form a convex polygon.
        // TODO: The points in the manifold should be in an `ArrayVec`, and the input points should be separate.
        let points = core::mem::take(&mut self.points);
        self.points.push(points[point1_index]);
        if point3_index != usize::MAX {
            self.points.push(points[point3_index]);
        }
        self.points.push(points[point2_index]);
        if point4_index != usize::MAX {
            debug_assert_ne!(point3_index, point4_index);
            self.points.push(points[point4_index]);
        }
    }

    /// Returns the contact point with the largest penetration depth.
    ///
    /// If the objects are separated but there is still a speculative contact,
    /// the penetration depth will be negative.
    ///
    /// If there are no contacts, `None` is returned.
    #[inline]
    pub fn find_deepest_contact(&self) -> Option<&ContactPoint> {
        self.points.iter().max_by(|a, b| {
            a.penetration
                .partial_cmp(&b.penetration)
                .unwrap_or(core::cmp::Ordering::Equal)
        })
    }

    /// Retains only the elements specified by the predicate.
    #[inline]
    pub(crate) fn retain_points_mut<F>(&mut self, f: F)
    where
        F: FnMut(&mut ContactPoint) -> bool,
    {
        #[cfg(feature = "2d")]
        {
            self.points.retain(f);
        }
        #[cfg(feature = "3d")]
        {
            self.points.retain_mut(f);
        }
    }
}

/// Data associated with a contact point in a [`ContactManifold`].
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct ContactPoint {
    /// The world-space contact point on the first shape relative to the center of mass.
    pub anchor1: Vector,
    /// The world-space contact point on the second shape relative to the center of mass.
    pub anchor2: Vector,
    /// The contact point in world space.
    ///
    /// This is the midpoint between the closest points on the surfaces of the two shapes.
    ///
    /// Note that because the contact point is expressed in world space,
    /// it is subject to precision loss at large coordinates.
    pub point: Vector,
    /// The penetration depth.
    ///
    /// Can be negative if the objects are separated and [speculative collision] is enabled.
    ///
    /// [speculative collision]: crate::dynamics::ccd#speculative-collision
    pub penetration: Scalar,
    /// The total normal impulse applied to the first body at this contact point.
    /// The unit is typically N⋅s or kg⋅m/s.
    ///
    /// This can be used to determine how "strong" a contact is. To compute the corresponding
    /// contact force, divide the impulse by the time step.
    pub normal_impulse: Scalar,
    /// The relative velocity of the bodies at the contact point along the contact normal.
    /// If negative, the bodies are approaching. The unit is typically m/s.
    ///
    /// This is computed before the solver, and can be used as an impact velocity to determine
    /// how "strong" the contact is in a mass-independent way.
    ///
    /// Internally, the `normal_speed` is used for restitution.
    pub normal_speed: Scalar,
    /// The normal impulse used to warm start the contact solver.
    ///
    /// This corresponds to the clamped accumulated impulse from the last substep
    /// of the previous time step.
    pub warm_start_normal_impulse: Scalar,
    /// The frictional impulse used to warm start the contact solver.
    ///
    /// This corresponds to the clamped accumulated impulse from the last substep
    /// of the previous time step.
    #[cfg(feature = "2d")]
    #[doc(alias = "warm_start_friction_impulse")]
    pub warm_start_tangent_impulse: Scalar,
    /// The frictional impulse used to warm start the contact solver.
    ///
    /// This corresponds to the clamped accumulated impulse from the last substep
    /// of the previous time step.
    #[cfg(feature = "3d")]
    #[doc(alias = "warm_start_friction_impulse")]
    pub warm_start_tangent_impulse: Vector2,
    /// The contact feature ID on the first shape. This indicates the ID of
    /// the vertex, edge, or face of the contact, if one can be determined.
    pub feature_id1: PackedFeatureId,
    /// The contact feature ID on the second shape. This indicates the ID of
    /// the vertex, edge, or face of the contact, if one can be determined.
    pub feature_id2: PackedFeatureId,
}

impl ContactPoint {
    /// Creates a new [`ContactPoint`] with the given world-space contact points
    /// relative to the centers of mass of the two shapes, the world-space contact point,
    /// and the penetration depth.
    ///
    /// [Feature IDs](PackedFeatureId) can be specified for the contact points using [`with_feature_ids`](Self::with_feature_ids).
    #[allow(clippy::too_many_arguments)]
    pub fn new(anchor1: Vector, anchor2: Vector, world_point: Vector, penetration: Scalar) -> Self {
        Self {
            anchor1,
            anchor2,
            point: world_point,
            penetration,
            normal_impulse: 0.0,
            normal_speed: 0.0,
            warm_start_normal_impulse: 0.0,
            #[cfg(feature = "2d")]
            warm_start_tangent_impulse: 0.0,
            #[cfg(feature = "3d")]
            warm_start_tangent_impulse: Vector2::ZERO,
            feature_id1: PackedFeatureId::UNKNOWN,
            feature_id2: PackedFeatureId::UNKNOWN,
        }
    }

    /// Sets the [feature IDs](PackedFeatureId) of the contact points.
    #[inline]
    pub fn with_feature_ids(mut self, id1: PackedFeatureId, id2: PackedFeatureId) -> Self {
        self.feature_id1 = id1;
        self.feature_id2 = id2;
        self
    }

    /// Flips the contact data, swapping the points and feature IDs,
    /// and negating the impulses.
    #[inline]
    pub fn flip(&mut self) {
        core::mem::swap(&mut self.anchor1, &mut self.anchor2);
        core::mem::swap(&mut self.feature_id1, &mut self.feature_id2);
        self.normal_impulse = -self.normal_impulse;
        self.warm_start_normal_impulse = -self.warm_start_normal_impulse;
        self.warm_start_tangent_impulse = -self.warm_start_tangent_impulse;
    }

    /// Returns a flipped copy of the contact data, swapping the points and feature IDs,
    /// and negating the impulses.
    #[inline]
    pub fn flipped(&self) -> Self {
        Self {
            anchor1: self.anchor2,
            anchor2: self.anchor1,
            point: self.point,
            penetration: self.penetration,
            normal_impulse: -self.normal_impulse,
            normal_speed: self.normal_speed,
            warm_start_normal_impulse: -self.warm_start_normal_impulse,
            warm_start_tangent_impulse: -self.warm_start_tangent_impulse,
            feature_id1: self.feature_id2,
            feature_id2: self.feature_id1,
        }
    }
}

/// Data related to a single contact between two bodies.
///
/// If you want a contact that belongs to a [contact manifold](ContactManifold) and has more data,
/// see [`ContactPoint`].
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct SingleContact {
    /// The contact point on the first shape in local space.
    pub local_point1: Vector,
    /// The contact point on the second shape in local space.
    pub local_point2: Vector,
    /// The contact normal expressed in the local space of the first shape.
    pub local_normal1: Vector,
    /// The contact normal expressed in the local space of the second shape.
    pub local_normal2: Vector,
    /// Penetration depth.
    pub penetration: Scalar,
}

impl SingleContact {
    /// Creates a new [`SingleContact`]. The contact points and normals should be given in local space.
    #[inline]
    pub fn new(
        local_point1: Vector,
        local_point2: Vector,
        local_normal1: Vector,
        local_normal2: Vector,
        penetration: Scalar,
    ) -> Self {
        Self {
            local_point1,
            local_point2,
            local_normal1,
            local_normal2,
            penetration,
        }
    }

    /// Returns the global contact point on the first shape,
    /// transforming the local point by the given position and rotation.
    #[inline]
    pub fn global_point1(&self, position: &Position, rotation: &Rotation) -> Vector {
        position.0 + rotation * self.local_point1
    }

    /// Returns the global contact point on the second shape,
    /// transforming the local point by the given position and rotation.
    #[inline]
    pub fn global_point2(&self, position: &Position, rotation: &Rotation) -> Vector {
        position.0 + rotation * self.local_point2
    }

    /// Returns the world-space contact normal pointing from the first shape to the second.
    #[inline]
    pub fn global_normal1(&self, rotation: &Rotation) -> Vector {
        rotation * self.local_normal1
    }

    /// Returns the world-space contact normal pointing from the second shape to the first.
    #[inline]
    pub fn global_normal2(&self, rotation: &Rotation) -> Vector {
        rotation * self.local_normal2
    }

    /// Flips the contact data, swapping the points and normals.
    #[inline]
    pub fn flip(&mut self) {
        core::mem::swap(&mut self.local_point1, &mut self.local_point2);
        core::mem::swap(&mut self.local_normal1, &mut self.local_normal2);
    }

    /// Returns a flipped copy of the contact data, swapping the points and normals.
    #[inline]
    pub fn flipped(&self) -> Self {
        Self {
            local_point1: self.local_point2,
            local_point2: self.local_point1,
            local_normal1: self.local_normal2,
            local_normal2: self.local_normal1,
            penetration: self.penetration,
        }
    }
}
