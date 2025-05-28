//! Contact types and data structures used in the collision pipeline.

mod contact_graph;
mod feature_id;
mod system_param;

pub use contact_graph::ContactGraph;
pub use feature_id::PackedFeatureId;
pub use system_param::Collisions;

use crate::prelude::*;
use bevy::prelude::*;

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
    /// Flag indicating the status and type of the contact pair.
    pub flags: ContactPairFlags,
}

/// Flags indicating the status and type of a [contact pair](ContactPair).
#[repr(transparent)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[derive(Hash, Clone, Copy, PartialEq, Eq, Debug, Reflect)]
#[reflect(opaque, Hash, PartialEq, Debug)]
pub struct ContactPairFlags(u8);

bitflags::bitflags! {
    impl ContactPairFlags: u8 {
        /// Set if the colliders are touching, including sensors.
        const TOUCHING = 0b0000_0001;
        /// Set if the AABBs of the colliders are no longer overlapping.
        const DISJOINT_AABB = 0b0000_0010;
        /// Set if the colliders are touching and were not touching previously.
        const STARTED_TOUCHING = 0b0000_0100;
        /// Set if the colliders are not touching and were touching previously.
        const STOPPED_TOUCHING = 0b0000_1000;
        /// Set if at least one of the colliders is a sensor.
        const SENSOR = 0b0001_0000;
        /// Set if the contact pair should emit contact events or sensor events.
        const CONTACT_EVENTS = 0b0010_0000;
        /// Set if the contact pair should have a custom contact modification hook applied.
        const MODIFY_CONTACTS = 0b0100_0000;
    }
}

impl ContactPair {
    /// Creates a new [`ContactPair`] with the given entities.
    #[inline]
    pub fn new(collider1: Entity, collider2: Entity) -> Self {
        Self {
            collider1,
            collider2,
            body1: None,
            body2: None,
            manifolds: Vec::new(),
            flags: ContactPairFlags::empty(),
        }
    }

    /// Computes the sum of all impulses applied along contact normals between the contact pair.
    ///
    /// To get the corresponding force, divide the impulse by `Time::<Substeps>::delta_secs()`.
    pub fn total_normal_impulse(&self) -> Vector {
        self.manifolds.iter().fold(Vector::ZERO, |acc, manifold| {
            acc + manifold.normal * manifold.total_normal_impulse()
        })
    }

    /// Computes the sum of the magnitudes of all impulses applied along contact normals between the contact pair.
    ///
    /// This is the sum of impulse magnitudes, *not* the magnitude of the [`total_normal_impulse`](Self::total_normal_impulse).
    ///
    /// To get the corresponding force, divide the impulse by `Time::<Substeps>::delta_secs()`.
    pub fn total_normal_impulse_magnitude(&self) -> Scalar {
        self.manifolds
            .iter()
            .fold(0.0, |acc, manifold| acc + manifold.total_normal_impulse())
    }

    // TODO: We could also return a reference to the whole manifold. Would that be useful?
    /// Finds the largest impulse between the contact pair, and the associated world-space contact normal,
    /// pointing from the first shape to the second.
    ///
    /// To get the corresponding force, divide the impulse by `Time::<Substeps>::delta_secs()`.
    pub fn max_normal_impulse(&self) -> (Scalar, Vector) {
        let mut magnitude: Scalar = 0.0;
        let mut normal = Vector::ZERO;

        for manifold in &self.manifolds {
            let impulse = manifold.max_normal_impulse();
            if impulse.abs() > magnitude.abs() {
                magnitude = impulse;
                normal = manifold.normal;
            }
        }

        (magnitude, normal)
    }

    /// The force corresponding to the total normal impulse applied over `delta_time`.
    ///
    /// Because contacts are solved over several substeps, `delta_time` should
    /// typically use `Time<Substeps>::delta_secs()`.
    #[deprecated(
        note = "Use `total_normal_impulse` instead, and divide it by `Time<Substeps>::delta_secs()`",
        since = "0.3.0"
    )]
    pub fn total_normal_force(&self, delta_time: Scalar) -> Scalar {
        self.total_normal_impulse_magnitude() / delta_time
    }

    /// Returns `true` if at least one of the colliders is a [`Sensor`].
    pub fn is_sensor(&self) -> bool {
        self.flags.contains(ContactPairFlags::SENSOR)
    }

    /// Returns `true` if the colliders are touching, including sensors.
    pub fn is_touching(&self) -> bool {
        self.flags.contains(ContactPairFlags::TOUCHING)
    }

    /// Returns `true` if the AABBs of the colliders are no longer overlapping.
    pub fn aabbs_disjoint(&self) -> bool {
        self.flags.contains(ContactPairFlags::DISJOINT_AABB)
    }

    /// Returns `true` if a collision started during the current frame.
    pub fn collision_started(&self) -> bool {
        self.flags.contains(ContactPairFlags::STARTED_TOUCHING)
    }

    /// Returns `true` if a collision ended during the current frame.
    pub fn collision_ended(&self) -> bool {
        self.flags.contains(ContactPairFlags::STOPPED_TOUCHING)
    }

    /// Returns `true` if collision events are enabled for the contact pair.
    pub fn events_enabled(&self) -> bool {
        self.flags.contains(ContactPairFlags::CONTACT_EVENTS)
    }

    /// Returns the contact with the largest penetration depth.
    ///
    /// If the objects are separated but there is still a speculative contact,
    /// the penetration depth will be negative.
    ///
    /// If there are no contacts, `None` is returned.
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
    /// The index of the manifold in the collision.
    pub index: usize,
}

impl ContactManifold {
    /// Creates a new [`ContactManifold`] with the given contact points and surface normals,
    /// expressed in local space.
    ///
    /// `index` represents the index of the manifold in the collision.
    pub fn new(
        points: impl IntoIterator<Item = ContactPoint>,
        normal: Vector,
        index: usize,
    ) -> Self {
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
            index,
        }
    }

    /// The sum of the impulses applied at the contact points in the manifold along the contact normal.
    fn total_normal_impulse(&self) -> Scalar {
        self.points
            .iter()
            .fold(0.0, |acc, contact| acc + contact.normal_impulse)
    }

    /// The magnitude of the largest impulse applied at a contact point in the manifold along the contact normal.
    fn max_normal_impulse(&self) -> Scalar {
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
                    contact.normal_impulse = previous_contact.normal_impulse;
                    contact.tangent_impulse = previous_contact.tangent_impulse;
                    break;
                }

                let unknown_features = contact.feature_id1 == PackedFeatureId::UNKNOWN
                    || contact.feature_id2 == PackedFeatureId::UNKNOWN;

                // If the feature IDs are unknown and the contact positions match closely enough,
                // copy the contact impulses over for warm starting.
                if unknown_features
                    && (contact
                        .local_point1
                        .distance_squared(previous_contact.local_point1)
                        < distance_threshold_squared
                        && contact
                            .local_point2
                            .distance_squared(previous_contact.local_point2)
                            < distance_threshold_squared)
                    || (contact
                        .local_point1
                        .distance_squared(previous_contact.local_point2)
                        < distance_threshold_squared
                        && contact
                            .local_point2
                            .distance_squared(previous_contact.local_point1)
                            < distance_threshold_squared)
                {
                    contact.normal_impulse = previous_contact.normal_impulse;
                    contact.tangent_impulse = previous_contact.tangent_impulse;
                    break;
                }
            }
        }
    }

    /// Returns the contact point with the largest penetration depth.
    ///
    /// If the objects are separated but there is still a speculative contact,
    /// the penetration depth will be negative.
    ///
    /// If there are no contacts, `None` is returned.
    pub fn find_deepest_contact(&self) -> Option<&ContactPoint> {
        self.points.iter().max_by(|a, b| {
            a.penetration
                .partial_cmp(&b.penetration)
                .unwrap_or(core::cmp::Ordering::Equal)
        })
    }
}

/// Data associated with a contact point in a [`ContactManifold`].
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct ContactPoint {
    /// The contact point on the first shape in local space.
    pub local_point1: Vector,
    /// The contact point on the second shape in local space.
    pub local_point2: Vector,
    /// The penetration depth.
    ///
    /// Can be negative if the objects are separated and [speculative collision] is enabled.
    ///
    /// [speculative collision]: crate::dynamics::ccd#speculative-collision
    pub penetration: Scalar,
    /// The impulse applied to the first body along the contact normal.
    ///
    /// To get the corresponding force, divide the impulse by `Time<Substeps>::delta_secs()`.
    pub normal_impulse: Scalar,
    /// The impulse applied to the first body along the contact tangent. This corresponds to the impulse caused by friction.
    ///
    /// To get the corresponding force, divide the impulse by `Time<Substeps>::delta_secs()`.
    #[cfg(feature = "2d")]
    #[doc(alias = "friction_impulse")]
    pub tangent_impulse: Scalar,
    /// The impulse applied to the first body along the contact tangent. This corresponds to the impulse caused by friction.
    ///
    /// To get the corresponding force, divide the impulse by `Time<Substeps>::delta_secs()`.
    #[cfg(feature = "3d")]
    #[doc(alias = "friction_impulse")]
    pub tangent_impulse: Vector2,
    /// The contact feature ID on the first shape. This indicates the ID of
    /// the vertex, edge, or face of the contact, if one can be determined.
    pub feature_id1: PackedFeatureId,
    /// The contact feature ID on the second shape. This indicates the ID of
    /// the vertex, edge, or face of the contact, if one can be determined.
    pub feature_id2: PackedFeatureId,
}

impl ContactPoint {
    /// Creates a new [`ContactPoint`] with the given points expressed in the local space
    /// of the first and second shape respectively.
    ///
    /// [Feature IDs](PackedFeatureId) can be specified for the contact points using [`with_feature_ids`](Self::with_feature_ids).
    #[allow(clippy::too_many_arguments)]
    pub fn new(local_point1: Vector, local_point2: Vector, penetration: Scalar) -> Self {
        Self {
            local_point1,
            local_point2,
            penetration,
            normal_impulse: 0.0,
            tangent_impulse: default(),
            feature_id1: PackedFeatureId::UNKNOWN,
            feature_id2: PackedFeatureId::UNKNOWN,
        }
    }

    /// Sets the [feature IDs](PackedFeatureId) of the contact points.
    pub fn with_feature_ids(mut self, id1: PackedFeatureId, id2: PackedFeatureId) -> Self {
        self.feature_id1 = id1;
        self.feature_id2 = id2;
        self
    }

    /// The force corresponding to the normal impulse applied over `delta_time`.
    ///
    /// Because contacts are solved over several substeps, `delta_time` should
    /// typically use `Time<Substeps>::delta_secs()`.
    pub fn normal_force(&self, delta_time: Scalar) -> Scalar {
        self.normal_impulse / delta_time
    }

    /// The force corresponding to the tangent impulse applied over `delta_time`.
    ///
    /// Because contacts are solved over several substeps, `delta_time` should
    /// typically use `Time<Substeps>::delta_secs()`.
    #[cfg(feature = "2d")]
    #[doc(alias = "friction_force")]
    pub fn tangent_force(&self, delta_time: Scalar) -> Scalar {
        self.tangent_impulse / delta_time
    }

    /// The force corresponding to the tangent impulse applied over `delta_time`.
    ///
    /// Because contacts are solved over several substeps, `delta_time` should
    /// typically use `Time<Substeps>::delta_secs()`.
    #[cfg(feature = "3d")]
    #[doc(alias = "friction_force")]
    pub fn tangent_force(&self, delta_time: Scalar) -> Vector2 {
        self.tangent_impulse / delta_time
    }

    /// Returns the global contact point on the first shape,
    /// transforming the local point by the given position and rotation.
    pub fn global_point1(&self, position: &Position, rotation: &Rotation) -> Vector {
        position.0 + rotation * self.local_point1
    }

    /// Returns the global contact point on the second shape,
    /// transforming the local point by the given position and rotation.
    pub fn global_point2(&self, position: &Position, rotation: &Rotation) -> Vector {
        position.0 + rotation * self.local_point2
    }

    /// Flips the contact data, swapping the points and feature IDs,
    /// and negating the impulses.
    pub fn flip(&mut self) {
        core::mem::swap(&mut self.local_point1, &mut self.local_point2);
        core::mem::swap(&mut self.feature_id1, &mut self.feature_id2);
        self.normal_impulse = -self.normal_impulse;
        self.tangent_impulse = -self.tangent_impulse;
    }

    /// Returns a flipped copy of the contact data, swapping the points and feature IDs,
    /// and negating the impulses.
    pub fn flipped(&self) -> Self {
        Self {
            local_point1: self.local_point2,
            local_point2: self.local_point1,
            penetration: self.penetration,
            normal_impulse: -self.normal_impulse,
            tangent_impulse: -self.tangent_impulse,
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
    pub fn global_point1(&self, position: &Position, rotation: &Rotation) -> Vector {
        position.0 + rotation * self.local_point1
    }

    /// Returns the global contact point on the second shape,
    /// transforming the local point by the given position and rotation.
    pub fn global_point2(&self, position: &Position, rotation: &Rotation) -> Vector {
        position.0 + rotation * self.local_point2
    }

    /// Returns the world-space contact normal pointing from the first shape to the second.
    pub fn global_normal1(&self, rotation: &Rotation) -> Vector {
        rotation * self.local_normal1
    }

    /// Returns the world-space contact normal pointing from the second shape to the first.
    pub fn global_normal2(&self, rotation: &Rotation) -> Vector {
        rotation * self.local_normal2
    }

    /// Flips the contact data, swapping the points and normals.
    pub fn flip(&mut self) {
        core::mem::swap(&mut self.local_point1, &mut self.local_point2);
        core::mem::swap(&mut self.local_normal1, &mut self.local_normal2);
    }

    /// Returns a flipped copy of the contact data, swapping the points and normals.
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
