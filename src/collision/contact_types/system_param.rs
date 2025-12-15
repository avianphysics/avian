use crate::data_structures::pair_key::PairKey;
use bevy::{ecs::system::SystemParam, prelude::*};

use super::{ContactGraph, ContactPair};

/// A [`SystemParam`] for accessing and querying collision data.
///
/// This is a wrapper around the [`ContactGraph`] resource that provides a convenient API
/// for querying touching [`ContactPair`]s between entities. If you need more lower-level control
/// and access to non-touching contact pairs, consider using the [`ContactGraph`] directly.
///
/// # Usage
///
/// The following methods can be used for querying collisions:
///
/// - [`get`](Self::get)
/// - [`iter`](Self::iter)
/// - [`contains`](Self::contains)
/// - [`collisions_with`](Self::collisions_with)
/// - [`entities_colliding_with`](Self::entities_colliding_with)
///
/// For example, to iterate over all collisions with a given entity:
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// #[derive(Component)]
/// struct PressurePlate;
///
/// fn activate_pressure_plates(mut query: Query<Entity, With<PressurePlate>>, collisions: Collisions) {
///     for pressure_plate in &query {
///         // Compute the total impulse applied to the pressure plate.
///         let mut total_impulse = 0.0;
///
///         for contact_pair in collisions.collisions_with(pressure_plate) {
///             total_impulse += contact_pair.total_normal_impulse_magnitude();
///         }
///
///         if total_impulse > 5.0 {
///             println!("Pressure plate activated!");
///         }
///     }
/// }
/// ```
///
/// Contact modification and filtering should be done using [`CollisionHooks`].
/// See the documentation for more information.
///
/// [`CollisionHooks`]: crate::collision::hooks::CollisionHooks
#[derive(SystemParam)]
pub struct Collisions<'w> {
    /// The [`ContactGraph`] that stores all contact edges.
    contact_graph: ResMut<'w, ContactGraph>,
}

impl Collisions<'_> {
    /// Returns a reference to the internal [`ContactGraph`].
    ///
    /// Note that unlike [`Collisions`], which only provides touching contacts,
    /// the contact graph includes both touching and non-touching contacts.
    #[inline]
    pub fn graph(&self) -> &ContactGraph {
        &self.contact_graph
    }

    /// Returns a touching contact pair between two entities.
    /// If the pair does not exist, `None` is returned.
    #[inline]
    pub fn get(&self, entity1: Entity, entity2: Entity) -> Option<&ContactPair> {
        self.contact_graph
            .get(entity1, entity2)
            .map(|(_edge, pair)| pair)
    }

    /// Returns `true` if the given entities have a touching contact pair.
    #[inline]
    pub fn contains(&self, entity1: Entity, entity2: Entity) -> bool {
        self.contact_graph.contains(entity1, entity2)
    }

    /// Returns `true` if the given pair key matches a touching contact pair.
    ///
    /// The pair key should be equivalent to `PairKey::new(entity1.index(), entity2.index())`.
    ///
    /// This method can be useful to avoid constructing a new `PairKey` when the key is already known.
    /// If the key is not available, consider using [`contains`](Self::contains) instead.
    #[inline]
    pub fn contains_key(&self, pair_key: &PairKey) -> bool {
        self.contact_graph.contains_key(pair_key)
    }

    /// Returns an iterator yielding immutable access to all touching contact pairs.
    #[inline]
    pub fn iter(&self) -> impl Iterator<Item = &ContactPair> {
        self.contact_graph
            .iter_active_touching()
            .chain(self.contact_graph.iter_sleeping_touching())
    }

    /// Returns an iterator yielding immutable access to all touching contact pairs
    /// involving the given entity.
    #[inline]
    pub fn collisions_with(&self, entity: Entity) -> impl Iterator<Item = &ContactPair> {
        self.contact_graph
            .contact_pairs_with(entity)
            .filter(|contact_pair| contact_pair.is_touching())
    }

    /// Returns an iterator yielding immutable access to all entities
    /// that have a touching contact pair with the given entity.
    #[inline]
    pub fn entities_colliding_with(&self, entity: Entity) -> impl Iterator<Item = Entity> + '_ {
        // TODO: Can we do this more efficiently?
        self.collisions_with(entity).map(move |contacts| {
            if contacts.collider1 == entity {
                contacts.collider2
            } else {
                contacts.collider1
            }
        })
    }
}
