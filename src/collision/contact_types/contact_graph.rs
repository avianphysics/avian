use crate::collision::contact_types::ContactEdgeFlags;
use crate::prelude::*;
use crate::{
    data_structures::{
        graph::NodeIndex, pair_key::PairKey, sparse_secondary_map::SparseSecondaryEntityMap,
        stable_graph::StableUnGraph,
    },
    dynamics::solver::constraint_graph::ContactManifoldHandle,
};
use bevy::{platform::collections::HashSet, prelude::*};

use super::{ContactEdge, ContactId};

/// A resource that stores all [`ContactEdge`]s in the physics world in an [undirected graph](StableUnGraph),
/// and their corresponding [`ContactPair`]s.
///
/// Contact pairs exist between [colliders](Collider) that have intersecting [AABBs](ColliderAabb),
/// even if the shapes themselves are not yet touching. Internally, pairs for active (non-sleeping) bodies
/// are stored separately from pairs for sleeping bodies for optimization purposes.
///
/// For a simpler API that abstracts over this complexity, consider using the [`Collisions`]
/// system parameter.
///
/// # Usage
///
/// The following methods can be used for querying collisions:
///
/// - [`get`](Self::get) and [`get_mut`](Self::get_mut)
/// - [`iter_active`](Self::iter_active) and [`iter_active_mut`](Self::iter_active_mut)
/// - [`iter_sleeping`](Self::iter_sleeping) and [`iter_sleeping_mut`](Self::iter_sleeping_mut)
/// - [`contains`](Self::contains)
/// - [`contact_pairs_with`](Self::contact_pairs_with)
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
/// fn activate_pressure_plates(mut query: Query<Entity, With<PressurePlate>>, contact_graph: Res<ContactGraph>) {
///     for pressure_plate in &query {
///         // Compute the total impulse applied to the pressure plate.
///         let mut total_impulse = 0.0;
///
///         for contact_pair in contact_graph.contact_pairs_with(pressure_plate) {
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
/// While mutable access is allowed, contact modification and filtering should typically
/// be done using [`CollisionHooks`]. See the documentation for more information.
///
/// For advanced usage, there are also methods such as [`get_edge`](Self::get_edge) and [`get_edge_mut`](Self::get_edge_mut)
/// methods to access the [`ContactEdge`]s directly, along with variants that take a [`ContactId`] to access edges by their ID.
///
/// # Warning
///
/// For users, this resource is primarily for querying and reading collision data.
///
/// Directly adding, modifying, or removing contact pairs using this resource will *not* trigger any collision events,
/// wake up the entities involved, or perform any other cleanup. Only make structural modifications if you know what you are doing.
///
/// For filtering and modifying collisions, consider using [`CollisionHooks`] instead.
#[derive(Resource, Clone, Debug, Default)]
pub struct ContactGraph {
    // TODO: Can and should edges be between bodies instead of colliders?
    // TODO: We could have a separate intersection graph for sensors.
    // TODO: Make the fields private, but expose a public API through methods.
    /// The internal undirected graph where nodes are entities and edges are contact pairs.
    pub edges: ContactGraphInternal,

    /// Contact pairs for awake dynamic and kinematic bodies.
    pub(crate) active_pairs: Vec<ContactPair>,

    /// Contact pairs for sleeping dynamic bodies.
    pub(crate) sleeping_pairs: Vec<ContactPair>,

    /// A set of all contact pairs for fast lookup.
    ///
    /// The [`PairKey`] is a unique pair of entity indices, sorted in ascending order,
    /// concatenated into a single `u64` key.
    ///
    /// Two entities have a contact pair if they have intersecting AABBs.
    pub(crate) pair_set: HashSet<PairKey>,

    /// A map from entities to their corresponding node indices in the contact graph.
    entity_to_node: SparseSecondaryEntityMap<NodeIndex>,
}

/// An undirected graph where each node is an entity and each edge is a [`ContactEdge`].
///
/// The graph maintains stable indices for nodes and edges even when items are removed.
///
/// This is stored internally in the [`ContactGraph`] resource, which provides a higher-level API
/// for working with contact pairs.
pub type ContactGraphInternal = StableUnGraph<Entity, ContactEdge>;

impl ContactGraph {
    /// Returns the [`NodeIndex`] of the given entity in the contact graph.
    ///
    /// If the entity is not in the graph, `None` is returned.
    #[inline]
    pub fn entity_to_node(&self, entity: Entity) -> Option<NodeIndex> {
        self.entity_to_node.get(entity).copied()
    }

    /// Returns the [`ContactEdge`] between two entities.
    /// If the edge does not exist, `None` is returned.
    ///
    /// A contact edge exists between two entities if their [`ColliderAabb`]s intersect.
    /// Use [`ContactEdge::is_touching`] to determine if the actual collider shapes are touching.
    #[inline]
    pub fn get_edge(&self, entity1: Entity, entity2: Entity) -> Option<&ContactEdge> {
        let (Some(index1), Some(index2)) =
            (self.entity_to_node(entity1), self.entity_to_node(entity2))
        else {
            return None;
        };

        self.edges
            .find_edge(index1, index2)
            .and_then(|edge| self.get_edge_by_id(edge.into()))
    }

    /// Returns the [`ContactEdge`] between two entities based on their IDs.
    /// If the edge does not exist, `None` is returned.
    ///
    /// A contact edge exists between two entities if their [`ColliderAabb`]s intersect.
    /// Use [`ContactEdge::is_touching`] to determine if the actual collider shapes are touching.
    #[inline]
    pub fn get_edge_by_id(&self, id: ContactId) -> Option<&ContactEdge> {
        self.edges.edge_weight(id.into())
    }

    /// Returns a mutable reference to the [`ContactEdge`] between two entities.
    /// If the edge does not exist, `None` is returned.
    ///
    /// A contact edge exists between two entities if their [`ColliderAabb`]s intersect.
    /// Use [`ContactEdge::is_touching`] to determine if the actual collider shapes are touching.
    #[inline]
    pub fn get_edge_mut(&mut self, entity1: Entity, entity2: Entity) -> Option<&mut ContactEdge> {
        let (Some(index1), Some(index2)) =
            (self.entity_to_node(entity1), self.entity_to_node(entity2))
        else {
            return None;
        };

        self.edges
            .find_edge(index1, index2)
            .and_then(|edge| self.get_edge_mut_by_id(edge.into()))
    }

    /// Returns a mutable reference to the [`ContactEdge`] between two entities based on their IDs.
    /// If the edge does not exist, `None` is returned.
    ///
    /// A contact edge exists between two entities if their [`ColliderAabb`]s intersect.
    /// Use [`ContactEdge::is_touching`] to determine if the actual collider shapes are touching.
    #[inline]
    pub fn get_edge_mut_by_id(&mut self, id: ContactId) -> Option<&mut ContactEdge> {
        self.edges.edge_weight_mut(id.into())
    }

    /// Returns the [`ContactEdge`] and [`ContactPair`] between two entities.
    /// If the pair does not exist, `None` is returned.
    ///
    /// A contact pair exists between two entities if their [`ColliderAabb`]s intersect.
    /// Use [`ContactPair::is_touching`] to determine if the actual collider shapes are touching.
    #[inline]
    pub fn get(&self, entity1: Entity, entity2: Entity) -> Option<(&ContactEdge, &ContactPair)> {
        self.get_edge(entity1, entity2)
            .and_then(|edge| self.get_pair_by_edge(edge).map(|pair| (edge, pair)))
    }

    /// Returns the [`ContactEdge`] and [`ContactPair`] between two entities based on the contact ID.
    /// If the pair does not exist, `None` is returned.
    ///
    /// A contact pair exists between two entities if their [`ColliderAabb`]s intersect.
    /// Use [`ContactPair::is_touching`] to determine if the actual collider shapes are touching.
    #[inline]
    pub fn get_by_id(&self, id: ContactId) -> Option<(&ContactEdge, &ContactPair)> {
        self.get_edge_by_id(id)
            .and_then(|edge| self.get_pair_by_edge(edge).map(|pair| (edge, pair)))
    }

    /// Returns a mutable reference to the [`ContactEdge`] and [`ContactPair`] between two entities.
    /// If the pair does not exist, `None` is returned.
    ///
    /// A contact pair exists between two entities if their [`ColliderAabb`]s intersect.
    /// Use [`ContactPair::is_touching`] to determine if the actual collider shapes are touching.
    #[inline]
    pub fn get_mut(
        &mut self,
        entity1: Entity,
        entity2: Entity,
    ) -> Option<(&mut ContactEdge, &mut ContactPair)> {
        let (Some(index1), Some(index2)) =
            (self.entity_to_node(entity1), self.entity_to_node(entity2))
        else {
            return None;
        };

        self.edges
            .find_edge(index1, index2)
            .and_then(|edge| self.get_mut_by_id(edge.into()))
    }

    /// Returns a mutable reference to the [`ContactEdge`] and [`ContactPair`] between two entities based on the contact ID.
    /// If the pair does not exist, `None` is returned.
    ///
    /// A contact pair exists between two entities if their [`ColliderAabb`]s intersect.
    /// Use [`ContactPair::is_touching`] to determine if the actual collider shapes are touching.
    #[inline]
    pub fn get_mut_by_id(&mut self, id: ContactId) -> Option<(&mut ContactEdge, &mut ContactPair)> {
        self.edges.edge_weight_mut(id.into()).and_then(|edge| {
            if edge.is_sleeping() {
                self.sleeping_pairs
                    .get_mut(edge.pair_index)
                    .map(|pair| (edge, pair))
            } else {
                self.active_pairs
                    .get_mut(edge.pair_index)
                    .map(|pair| (edge, pair))
            }
        })
    }

    /// Returns the [`ContactPair`] between two entities based on the [`ContactEdge`].
    /// If the pair does not exist, `None` is returned.
    ///
    /// A contact pair exists between two entities if their [`ColliderAabb`]s intersect.
    /// Use [`ContactPair::is_touching`] to determine if the actual collider shapes are touching.
    #[inline]
    pub fn get_pair_by_edge(&self, edge: &ContactEdge) -> Option<&ContactPair> {
        if edge.is_sleeping() {
            self.sleeping_pairs.get(edge.pair_index)
        } else {
            self.active_pairs.get(edge.pair_index)
        }
    }

    /// Returns a mutable reference to the [`ContactPair`] between two entities based on the [`ContactEdge`].
    /// If the pair does not exist, `None` is returned.
    ///
    /// A contact pair exists between two entities if their [`ColliderAabb`]s intersect.
    /// Use [`ContactPair::is_touching`] to determine if the actual collider shapes are touching.
    #[inline]
    pub fn get_pair_mut_by_edge(&mut self, edge: &ContactEdge) -> Option<&mut ContactPair> {
        if edge.is_sleeping() {
            self.sleeping_pairs.get_mut(edge.pair_index)
        } else {
            self.active_pairs.get_mut(edge.pair_index)
        }
    }

    /// Returns a [`ContactManifold`] of a contact pair based on the [`ContactManifoldHandle`].
    /// If the manifold does not exist, `None` is returned.
    #[inline]
    pub fn get_manifold(&self, handle: ContactManifoldHandle) -> Option<&ContactManifold> {
        let contact_pair = self.get_by_id(handle.contact_id)?.1;
        contact_pair.manifolds.get(handle.manifold_index)
    }

    /// Returns a mutable reference to a [`ContactManifold`] of a contact pair based on the [`ContactManifoldHandle`].
    /// If the manifold does not exist, `None` is returned.
    #[inline]
    pub fn get_manifold_mut(
        &mut self,
        handle: ContactManifoldHandle,
    ) -> Option<&mut ContactManifold> {
        let contact_pair = self.get_mut_by_id(handle.contact_id)?.1;
        contact_pair.manifolds.get_mut(handle.manifold_index)
    }

    /// Returns `true` if the given entities have a contact pair.
    ///
    /// A contact pair exists between two entities if their [`ColliderAabb`]s intersect,
    /// even if the shapes themselves are not yet touching.
    #[inline]
    pub fn contains(&self, entity1: Entity, entity2: Entity) -> bool {
        self.contains_key(&PairKey::new(entity1.index_u32(), entity2.index_u32()))
    }

    /// Returns `true` if the given pair key is in the contact graph.
    ///
    /// The pair key should be equivalent to `PairKey::new(entity1.index_u32(), entity2.index_u32())`.
    ///
    /// This method can be useful to avoid constructing a new `PairKey` when the key is already known.
    /// If the key is not available, consider using [`contains`](Self::contains) instead.
    #[inline]
    pub fn contains_key(&self, pair_key: &PairKey) -> bool {
        self.pair_set.contains(pair_key)
    }

    /// Returns a slice over all active (non-sleeping) contact pairs.
    #[inline]
    pub fn active_pairs(&self) -> &[ContactPair] {
        &self.active_pairs
    }

    /// Returns a slice over all sleeping contact pairs.
    #[inline]
    pub fn sleeping_pairs(&self) -> &[ContactPair] {
        &self.sleeping_pairs
    }

    /// Returns a mutable slice over all active (non-sleeping) contact pairs.
    #[inline]
    pub fn active_pairs_mut(&mut self) -> &mut [ContactPair] {
        &mut self.active_pairs
    }

    /// Returns a mutable slice over all sleeping contact pairs.
    #[inline]
    pub fn sleeping_pairs_mut(&mut self) -> &mut [ContactPair] {
        &mut self.sleeping_pairs
    }

    /// Returns an iterator yielding immutable access to all active (non-sleeping) contact pairs.
    ///
    /// A contact pair exists between two entities if their [`ColliderAabb`]s intersect,
    /// even if the shapes themselves are not yet touching.
    ///
    /// If you only want touching contacts, use [`iter_active_touching`](Self::iter_active_touching) instead.
    #[inline]
    pub fn iter_active(&self) -> impl Iterator<Item = &ContactPair> {
        self.active_pairs.iter()
    }

    /// Returns an iterator yielding immutable access to all active (non-sleeping) contact pairs
    /// that are currently touching.
    ///
    /// This is a subset of [`iter_active`](Self::iter_active) that only includes pairs where the colliders are touching.
    #[inline]
    pub fn iter_active_touching(&self) -> impl Iterator<Item = &ContactPair> {
        self.iter_active()
            .filter(|contacts| contacts.flags.contains(ContactPairFlags::TOUCHING))
    }

    /// Returns a iterator yielding mutable access to all contact pairs.
    ///
    /// A contact pair exists between two entities if their [`ColliderAabb`]s intersect,
    /// even if the shapes themselves are not yet touching.
    ///
    /// If you only want touching contacts, use [`iter_active_touching_mut`](Self::iter_active_touching_mut) instead.
    #[inline]
    pub fn iter_active_mut(&mut self) -> impl Iterator<Item = &mut ContactPair> {
        self.active_pairs.iter_mut()
    }

    /// Returns an iterator yielding mutable access to all active (non-sleeping) contact pairs
    /// that are currently touching.
    ///
    /// This is a subset of [`iter_active_mut`](Self::iter_active_mut) that only includes pairs where the colliders are touching.
    #[inline]
    pub fn iter_active_touching_mut(&mut self) -> impl Iterator<Item = &mut ContactPair> {
        self.iter_active_mut()
            .filter(|contacts| contacts.flags.contains(ContactPairFlags::TOUCHING))
    }

    /// Returns an iterator yielding immutable access to all sleeping contact pairs.
    ///
    /// A contact pair exists between two entities if their [`ColliderAabb`]s intersect,
    /// even if the shapes themselves are not yet touching.
    ///
    /// If you only want touching contacts, use [`iter_sleeping_touching`](Self::iter_sleeping_touching) instead.
    #[inline]
    pub fn iter_sleeping(&self) -> impl Iterator<Item = &ContactPair> {
        self.sleeping_pairs.iter()
    }

    /// Returns an iterator yielding immutable access to all sleeping contact pairs
    /// that are currently touching.
    ///
    /// This is a subset of [`iter_sleeping`](Self::iter_sleeping) that only includes pairs where the colliders are touching.
    #[inline]
    pub fn iter_sleeping_touching(&self) -> impl Iterator<Item = &ContactPair> {
        self.iter_sleeping()
            .filter(|contacts| contacts.flags.contains(ContactPairFlags::TOUCHING))
    }

    /// Returns an iterator yielding mutable access to all sleeping contact pairs.
    ///
    /// A contact pair exists between two entities if their [`ColliderAabb`]s intersect,
    /// even if the shapes themselves are not yet touching.
    ///
    /// If you only want touching contacts, use [`iter_sleeping_touching_mut`](Self::iter_sleeping_touching_mut) instead.
    #[inline]
    pub fn iter_sleeping_mut(&mut self) -> impl Iterator<Item = &mut ContactPair> {
        self.sleeping_pairs.iter_mut()
    }

    /// Returns an iterator yielding mutable access to all sleeping contact pairs
    /// that are currently touching.
    ///
    /// This is a subset of [`iter_sleeping_mut`](Self::iter_sleeping_mut) that only includes pairs where the colliders are touching.
    #[inline]
    pub fn iter_sleeping_touching_mut(&mut self) -> impl Iterator<Item = &mut ContactPair> {
        self.iter_sleeping_mut()
            .filter(|contacts| contacts.flags.contains(ContactPairFlags::TOUCHING))
    }

    /// Returns an iterator yielding immutable access to all contact edges involving the given entity.
    #[inline]
    pub fn contact_edges_with(&self, entity: Entity) -> impl Iterator<Item = &ContactEdge> {
        let index = self.entity_to_node(entity);
        if let Some(index) = index {
            itertools::Either::Left(self.edges.edge_weights(index))
        } else {
            itertools::Either::Right(core::iter::empty())
        }
    }

    /// Returns an iterator yielding mutable access to all contact edges involving the given entity.
    #[inline]
    pub fn contact_edges_with_mut(
        &mut self,
        entity: Entity,
    ) -> impl Iterator<Item = &mut ContactEdge> {
        let index = self.entity_to_node(entity);
        if let Some(index) = index {
            itertools::Either::Left(self.edges.edge_weights_mut(index))
        } else {
            itertools::Either::Right(core::iter::empty())
        }
    }

    /// Returns an iterator yielding immutable access to all contact pairs involving the given entity.
    ///
    /// A contact pair exists between two entities if their [`ColliderAabb`]s intersect,
    /// even if the shapes themselves are not yet touching.
    ///
    /// Use [`ContactEdge::is_touching`](ContactEdge::is_touching) to determine if the actual collider shapes are touching.
    // TODO: A mutable version of this could be useful, but it would probably require some `unsafe`.
    #[inline]
    pub fn contact_pairs_with(&self, entity: Entity) -> impl Iterator<Item = &ContactPair> {
        self.contact_edges_with(entity)
            .filter_map(move |edge| self.get_pair_by_edge(edge))
    }

    /// Returns an iterator yielding immutable access to all entities that have a contact pair with the given entity.
    ///
    /// A contact pair exists between two entities if their [`ColliderAabb`]s intersect,
    /// even if the shapes themselves are not yet touching.
    #[inline]
    pub fn entities_colliding_with(&self, entity: Entity) -> impl Iterator<Item = Entity> + '_ {
        self.entity_to_node
            .get(entity)
            .into_iter()
            .flat_map(move |&index| {
                self.edges
                    .neighbors(index)
                    .map(|index| *self.edges.node_weight(index).unwrap())
            })
    }

    /// Creates a [`ContactEdge`] between two entities and adds an associated [`ContactPair`]
    /// to the list of active pairs.
    ///
    /// Returns the ID of the contact edge if it was created, or `None` if the edge already exists.
    ///
    /// # Warning
    ///
    /// Creating a contact edge with this method will *not* trigger any collision events
    /// or wake up the entities involved. Only use this method if you know what you are doing.
    #[inline]
    pub fn add_edge(&mut self, contact_edge: ContactEdge) -> Option<ContactId> {
        self.add_edge_with(contact_edge, |_| {})
    }

    /// Creates a [`ContactEdge`] between two entities, calling the provided callback
    /// to initialize the associated [`ContactPair`] in the list of active pairs.
    ///
    /// Returns the ID of the contact edge if it was created, or `None` if the edge already exists.
    ///
    /// # Warning
    ///
    /// Creating a contact edge with this method will *not* trigger any collision events
    /// or wake up the entities involved. Only use this method if you know what you are doing.
    #[inline]
    pub fn add_edge_with<F>(
        &mut self,
        contact_edge: ContactEdge,
        pair_callback: F,
    ) -> Option<ContactId>
    where
        F: FnMut(&mut ContactPair),
    {
        let pair_key = PairKey::new(
            contact_edge.collider1.index_u32(),
            contact_edge.collider2.index_u32(),
        );
        self.add_edge_and_key_with(contact_edge, pair_key, pair_callback)
    }

    /// Creates a [`ContactEdge`] between two entities with the given pair key, calling the provided callback
    /// to initialize the associated [`ContactPair`] in the list of active pairs.
    ///
    /// The key must be equivalent to `PairKey::new(contacts.entity1.index_u32(), contacts.entity2.index_u32())`.
    ///
    /// Returns the ID of the contact edge if it was created, or `None` if the edge already exists.
    ///
    /// This method can be useful to avoid constructing a new `PairKey` when the key is already known.
    /// If the key is not available, consider using [`add_edge`](Self::add_edge) or [`add_edge_with`](Self::add_edge_with) instead.
    ///
    /// # Warning
    ///
    /// Creating a contact edge with this method will *not* trigger any collision events
    /// or wake up the entities involved. Only use this method if you know what you are doing.
    #[inline]
    pub fn add_edge_and_key_with<F>(
        &mut self,
        contact_edge: ContactEdge,
        pair_key: PairKey,
        mut pair_callback: F,
    ) -> Option<ContactId>
    where
        F: FnMut(&mut ContactPair),
    {
        // Add the pair to the pair set for fast lookup.
        if !self.pair_set.insert(pair_key) {
            // The pair already exists.
            return None;
        }

        let collider1 = contact_edge.collider1;
        let collider2 = contact_edge.collider2;

        // Get the indices of the entities in the graph.
        let index1 = self
            .entity_to_node
            .get_or_insert_with(collider1, || self.edges.add_node(collider1));
        let index2 = self
            .entity_to_node
            .get_or_insert_with(collider2, || self.edges.add_node(collider2));

        // Add the edge to the graph.
        let edge_id = ContactId(self.edges.add_edge(index1, index2, contact_edge).0);

        // Create the contact pair. Contacts are created as active.
        let mut pair = ContactPair::new(collider1, collider2, edge_id);

        // Call the pair callback to allow for custom initialization.
        pair_callback(&mut pair);

        let pair_index = self.active_pairs.len();
        self.active_pairs.push(pair);

        // Set the contact ID and pair index of the edge.
        let edge = self.edges.edge_weight_mut(edge_id.into()).unwrap();
        edge.id = edge_id;
        edge.pair_index = pair_index;

        Some(edge_id)
    }

    /// Removes a [`ContactEdge`] between two entites and returns its value.
    ///
    /// # Warning
    ///
    /// Removing a contact edge with this method will *not* trigger any collision events
    /// or wake up the entities involved. Only use this method if you know what you are doing.
    ///
    /// For filtering and modifying collisions, consider using [`CollisionHooks`] instead.
    #[inline]
    pub fn remove_edge(&mut self, entity1: Entity, entity2: Entity) -> Option<ContactEdge> {
        let (Some(index1), Some(index2)) =
            (self.entity_to_node(entity1), self.entity_to_node(entity2))
        else {
            return None;
        };

        // Remove the edge from the graph.
        self.edges.find_edge(index1, index2).and_then(|edge_id| {
            let pair_key = PairKey::new(entity1.index_u32(), entity2.index_u32());
            self.remove_edge_by_id(&pair_key, edge_id.into())
        })
    }

    /// Removes a [`ContactEdge`] based on its pair key and ID and returns its value.
    ///
    /// # Warning
    ///
    /// Removing a contact edge with this method will *not* trigger any collision events
    /// or wake up the entities involved. Only use this method if you know what you are doing.
    ///
    /// For filtering and modifying collisions, consider using [`CollisionHooks`] instead.
    #[inline]
    pub fn remove_edge_by_id(&mut self, pair_key: &PairKey, id: ContactId) -> Option<ContactEdge> {
        // Remove the pair from the pair set.
        self.pair_set.remove(pair_key);

        // Remove the edge from the graph.
        let edge = self.edges.remove_edge(id.into())?;
        let edge_count = self.edges.raw_edges().len();

        // Remove the contact pair from the active or sleeping pairs.
        let pair_index = edge.pair_index;
        let contact_pairs = if edge.is_sleeping() {
            &mut self.sleeping_pairs
        } else {
            &mut self.active_pairs
        };

        let moved_index = contact_pairs.len() - 1;
        contact_pairs.swap_remove(pair_index);

        if moved_index != pair_index {
            // Fix moved pair index.
            let moved_contact_id = contact_pairs[pair_index].contact_id;
            let moved_edge = self
                    .get_edge_mut_by_id(moved_contact_id)
                    .unwrap_or_else(|| {
                        panic!("error when removing contact pair {id:?} with pair index {pair_index:?}: moved edge {moved_contact_id:?} not found in contact graph with {edge_count} edges and {moved_index} sleeping pairs")
                    });
            debug_assert!(moved_edge.pair_index == moved_index);
            moved_edge.pair_index = pair_index;
        }

        Some(edge)
    }

    /// Removes the collider of the given entity from the contact graph, calling the given callback
    /// for each [`ContactEdge`] right before it is removed.
    ///
    /// # Warning
    ///
    /// Removing a collider with this method will *not* trigger any collision events
    /// or wake up the entities involved. Only use this method if you know what you are doing.
    #[inline]
    pub fn remove_collider_with<F>(&mut self, entity: Entity, mut edge_callback: F)
    where
        F: FnMut(&mut StableUnGraph<Entity, ContactEdge>, ContactId),
    {
        // Remove the entity from the entity-to-node mapping,
        // and get the index of the node in the graph.
        let Some(index) = self.entity_to_node.remove(entity) else {
            return;
        };

        // Remove the entity from the graph.
        self.edges.remove_node_with(index, |graph, edge_index| {
            let contact_id: ContactId = edge_index.into();
            let contact_edge = graph.edge_weight(edge_index).unwrap_or_else(|| {
                panic!("contact edge {contact_id:?} not found in contact graph")
            });
            let pair_key = PairKey::new(
                contact_edge.collider1.index_u32(),
                contact_edge.collider2.index_u32(),
            );
            let pair_index = contact_edge.pair_index;
            let contact_pairs = if contact_edge.is_sleeping() {
                &mut self.sleeping_pairs
            } else {
                &mut self.active_pairs
            };

            edge_callback(graph, contact_id);

            // Remove the contact pair from the active or sleeping pairs.
            let moved_index = contact_pairs.len() - 1;
            let _ = contact_pairs.swap_remove(pair_index);

            // Fix moved pair index.
            if moved_index != pair_index {
                let moved_contact_id = contact_pairs[pair_index].contact_id;
                let moved_edge = graph
                    .edge_weight_mut(moved_contact_id.into())
                    .unwrap_or_else(|| {
                        panic!("moved edge {moved_contact_id:?} not found in contact graph")
                    });
                debug_assert!(moved_edge.pair_index == moved_index);
                moved_edge.pair_index = pair_index;
            }

            // Remove the pair from the pair set.
            self.pair_set.remove(&pair_key);
        });

        // Removing the node swapped the last node to its place,
        // so we need to remap the entity-to-node mapping of the swapped node.
        if let Some(swapped) = self.edges.node_weight(index).copied() {
            let swapped_index = self
                .entity_to_node
                .get_mut(swapped)
                // This should never panic.
                .expect("swapped entity has no entity-to-node mapping");
            *swapped_index = index;
        }
    }

    /// Transfers touching contacts of a body from the sleeping contacts to the active contacts,
    /// calling the given callback for each [`ContactPair`] that is moved to active contacts.
    #[inline]
    pub fn wake_entity_with<F>(&mut self, entity: Entity, mut pair_callback: F)
    where
        F: FnMut(&mut ContactGraph, &ContactPair),
    {
        // Get the index of the entity in the graph.
        let Some(index) = self.entity_to_node(entity) else {
            return;
        };

        // Find all edges connected to the entity.
        let contact_ids: Vec<ContactId> = self
            .edges
            .edge_weights(index)
            .filter_map(|edge| edge.is_sleeping().then_some(edge.id))
            .collect();

        // Iterate over the edges and move sleeping contacts to active contacts.
        for id in contact_ids {
            let edge = self
                .edges
                .edge_weight_mut(id.into())
                .expect("edge should exist");

            if !edge.is_touching() {
                // Only wake touching contacts.
                continue;
            }

            let pair_index = edge.pair_index;

            // Remove the sleeping contact pair.
            let moved_index = self.sleeping_pairs.len() - 1;
            let transferred_pair = self.sleeping_pairs.swap_remove(pair_index);

            // Update the pair index of the edge.
            edge.pair_index = self.active_pairs.len();

            // Set the edge to active.
            edge.flags.set(ContactEdgeFlags::SLEEPING, false);

            // Call the pair callback.
            pair_callback(self, &transferred_pair);

            // Add the transferred pair to the active pairs.
            self.active_pairs.push(transferred_pair);

            // Fix moved edge.
            if moved_index != pair_index {
                let moved_contact_id = self.sleeping_pairs[pair_index].contact_id;
                let moved_edge = self
                    .get_edge_mut_by_id(moved_contact_id)
                    .unwrap_or_else(|| {
                        panic!("error when waking contact pair {moved_contact_id:?} (other {id:?}) with pair index {pair_index:?}: moved edge not found in contact graph")
                    });
                debug_assert!(moved_edge.pair_index == moved_index);
                moved_edge.pair_index = pair_index;
            }
        }
    }

    /// Transfers touching contacts of a body from the active contacts to the sleeping contacts,
    /// calling the given callback for each [`ContactPair`] that is moved to sleeping contacts.
    #[inline]
    pub fn sleep_entity_with<F>(&mut self, entity: Entity, mut pair_callback: F)
    where
        F: FnMut(&mut ContactGraph, &ContactPair),
    {
        // Get the index of the entity in the graph.
        let Some(index) = self.entity_to_node(entity) else {
            return;
        };

        // Find all edges connected to the entity.
        let contact_ids: Vec<ContactId> = self
            .edges
            .edge_weights(index)
            .filter_map(|edge| (!edge.is_sleeping()).then_some(edge.id))
            .collect();

        // Iterate over the edges and move active contacts to sleeping contacts.
        for id in contact_ids {
            let edge = self
                .edges
                .edge_weight_mut(id.into())
                .expect("edge should exist");

            if !edge.is_touching() {
                // Only sleep touching contacts.
                continue;
            }

            let pair_index = edge.pair_index;

            // Remove the active contact pair.
            let moved_index = self.active_pairs.len() - 1;
            let transferred_pair = self.active_pairs.swap_remove(pair_index);

            // Update the pair index of the edge.
            edge.pair_index = self.sleeping_pairs.len();

            // Set the edge to sleeping.
            edge.flags.set(ContactEdgeFlags::SLEEPING, true);

            // Call the edge callback.
            pair_callback(self, &transferred_pair);

            // Add the transferred pair to the sleeping pairs.
            self.sleeping_pairs.push(transferred_pair);

            // Fix moved edge.
            if moved_index != pair_index {
                let moved_contact_id = self.active_pairs[pair_index].contact_id;
                let moved_edge = self
                    .get_edge_mut_by_id(moved_contact_id)
                    .unwrap_or_else(|| {
                        panic!("error when sleeping contact pair {moved_contact_id:?} with pair index {pair_index:?}: moved edge not found in contact graph")
                    });
                debug_assert!(moved_edge.pair_index == moved_index);
                moved_edge.pair_index = pair_index;
            }
        }
    }

    /// Clears all contact pairs and the contact graph.
    ///
    /// # Warning
    ///
    /// Clearing contact pairs with this method will *not* trigger any collision events
    /// or wake up the entities involved. Only use this method if you know what you are doing.
    ///
    /// Additionally, this does *not* clear the [`ConstraintGraph`]! You should additionally
    /// call [`ConstraintGraph::clear`].
    ///
    /// [`ConstraintGraph`]: crate::dynamics::solver::constraint_graph::ConstraintGraph
    /// [`ConstraintGraph::clear`]: crate::dynamics::solver::constraint_graph::ConstraintGraph::clear
    #[inline]
    pub fn clear(&mut self) {
        self.edges.clear();
        self.active_pairs.clear();
        self.sleeping_pairs.clear();
        self.pair_set.clear();
        self.entity_to_node.clear();
    }
}
