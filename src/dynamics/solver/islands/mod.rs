//! Persistent simulation islands for sleeping and waking.
//!
//! Islands are retained across time steps. Each dynamic body starts with its own island,
//! and when a constraint between two dynamic bodies is created, the islands are merged with union find.
//!
//! Splitting is deferred and done using depth-first search (DFS). Only one island is split per time step,
//! choosing the sleepiest island with one or more constraints removed as the candidate.
//!
//! Islands are only used for sleeping and waking. Solver parallelism is achieved with [graph coloring](super::constraint_graph)
//! using the [`ConstraintGraph`](super::constraint_graph::ConstraintGraph).
//!
//! If simulation islands are not needed, the [`IslandPlugin`] can be omitted.
//!
//! # References
//!
//! - [Box2D - Simulation Islands] by [Erin Catto]
//!
//! [Box2D - Simulation Islands]: https://box2d.org/posts/2023/10/simulation-islands/
//! [Erin Catto]: https://github.com/erincatto

// Options:
//
// 1. Depth-first search (DFS): Islands are built from scratch with DFS every timestep.
//    - Looks for awake bodies to be the seeds of islands, and traverses connected constraints.
//    - Traversal mark management can be expensive.
//    - Waking is cheap and straightforward.
// 2. Union-find (UF): Each body starts in its own island. As constraints are added, the islands are merged.
//    - Every island has a unique root body that is used to identify the island.
//    - Reuires additional care to handle waking for connected bodies.
//    - Can be parallelized at the cost of determinism, unless performing constraint sorting with quicksort,
//      which is expensive for large islands.
// 3. Persistent islands: Retain islands across time steps, and merge or split them as constraints are added or removed.
//    - Each dynamic body starts with its own island. When a constraint between two dynamic bodies is created,
//      the islands are merged with union find. When constraints between two bodies are removed, their islands
//      are marked as candidates for splitting.
//    - Splitting can be deferred and done in parallel, using union find, DFS, or any other island-finding algorithm.
//    - Fast and deterministic!
//
// Avian uses persistent islands. See Erin Catto's "Simulation Islands" article for more details.
// https://box2d.org/posts/2023/10/simulation-islands/
//
// The implementation is largely based on Box2D:
// https://github.com/erincatto/box2d/blob/df9787b59e4480135fbd73d275f007b5d931a83f/src/island.c#L57

mod sleeping;
pub use sleeping::{IslandSleepingPlugin, SleepBody, SleepIslands, WakeBody, WakeIslands};

use bevy::{
    ecs::{entity_disabling::Disabled, lifecycle::HookContext, world::DeferredWorld},
    prelude::*,
};
use slab::Slab;

use crate::{
    collision::contact_types::{ContactGraphInternal, ContactId},
    dynamics::solver::{
        joint_graph::{JointGraph, JointId},
        solver_body::SolverBody,
    },
    prelude::{
        ContactGraph, PhysicsSchedule, RigidBody, RigidBodyColliders, RigidBodyDisabled,
        SolverSystems,
    },
};

/// A plugin for managing [`PhysicsIsland`]s.
///
/// See the [module-level documentation](self) for more information.
pub struct IslandPlugin;

impl Plugin for IslandPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<PhysicsIslands>();

        // Insert `BodyIslandNode` for each `SolverBody`.
        app.register_required_components::<SolverBody, BodyIslandNode>();

        // Add `BodyIslandNode` for each dynamic and kinematic rigid body
        // when the associated rigid body is enabled.
        app.add_observer(
            |trigger: On<Replace, RigidBodyDisabled>,
             rb_query: Query<&RigidBody>,
             mut commands: Commands| {
                let Ok(rb) = rb_query.get(trigger.entity) else {
                    return;
                };
                if rb.is_dynamic() || rb.is_kinematic() {
                    commands
                        .entity(trigger.entity)
                        .try_insert(BodyIslandNode::default());
                }
            },
        );
        app.add_observer(
            |trigger: On<Replace, Disabled>,
             rb_query: Query<
                &RigidBody,
                (
                    // The body still has `Disabled` at this point,
                    // and we need to include in the query to match against the entity.
                    With<Disabled>,
                    Without<RigidBodyDisabled>,
                ),
            >,
             mut commands: Commands| {
                let Ok(rb) = rb_query.get(trigger.entity) else {
                    return;
                };
                if rb.is_dynamic() || rb.is_kinematic() {
                    commands
                        .entity(trigger.entity)
                        .try_insert(BodyIslandNode::default());
                }
            },
        );

        // Remove `BodyIslandNode` when any of the following happens:
        //
        // 1. `RigidBody` is removed.
        // 2. `Disabled` or `RigidBodyDisabled` is added to the body.
        // 3. The body becomes `RigidBody::Static`.
        app.add_observer(|trigger: On<Remove, RigidBody>, mut commands: Commands| {
            commands
                .entity(trigger.entity)
                .try_remove::<BodyIslandNode>();
        });
        app.add_observer(
            |trigger: On<Insert, (Disabled, RigidBodyDisabled)>,
             query: Query<&RigidBody, Or<(With<Disabled>, Without<Disabled>)>>,
             mut commands: Commands| {
                if query.contains(trigger.entity) {
                    commands
                        .entity(trigger.entity)
                        .try_remove::<BodyIslandNode>();
                }
            },
        );
        app.add_observer(
            |trigger: On<Insert, RigidBody>, query: Query<&RigidBody>, mut commands: Commands| {
                if let Ok(rb) = query.get(trigger.entity)
                    && rb.is_static()
                {
                    commands
                        .entity(trigger.entity)
                        .try_remove::<BodyIslandNode>();
                }
            },
        );

        app.add_systems(
            PhysicsSchedule,
            split_island.in_set(SolverSystems::Finalize),
        );
    }
}

fn split_island(
    mut islands: ResMut<PhysicsIslands>,
    mut body_islands: Query<&mut BodyIslandNode, Or<(With<Disabled>, Without<Disabled>)>>,
    body_colliders: Query<&RigidBodyColliders>,
    mut contact_graph: ResMut<ContactGraph>,
    mut joint_graph: ResMut<JointGraph>,
) {
    // Splitting is only done when bodies want to sleep.
    if let Some(island_id) = islands.split_candidate {
        islands.split_island(
            island_id,
            &mut body_islands,
            &body_colliders,
            &mut contact_graph,
            &mut joint_graph,
        );
    }
}

/// A stable identifier for a [`PhysicsIsland`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct IslandId(pub u32);

impl IslandId {
    /// A placeholder identifier for a [`PhysicsIsland`].
    pub const PLACEHOLDER: Self = Self(u32::MAX);
}

impl core::fmt::Display for IslandId {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "IslandId({})", self.0)
    }
}

/// A [simulation island](self) that contains bodies, contacts, and joints. Used for sleeping and waking.
///
/// Islands are retained across time steps. Each dynamic body starts with its own island,
/// and when a constraint between two dynamic bodies is created, the islands are merged with union find.
///
/// Splitting is deferred and done using depth-first search (DFS). Only one island is split per time step,
/// choosing the sleepiest island with one or more constraints removed as the candidate.
///
/// Bodies, contacts, and joints are linked to islands using linked lists for efficient addition, removal,
/// and merging. Each island stores the head and tail of each list, while each [`BodyIslandNode`], [`ContactEdge`],
/// and [`JointGraphEdge`] stores an [`IslandNode`] that links it to the next and previous item in the list.
///
/// [`ContactEdge`]: crate::collision::contact_types::ContactEdge
/// [`JointGraphEdge`]: crate::dynamics::solver::joint_graph::JointGraphEdge
#[derive(Clone, Debug, PartialEq)]
pub struct PhysicsIsland {
    pub(crate) id: IslandId,

    pub(crate) head_body: Option<Entity>,
    pub(crate) tail_body: Option<Entity>,
    pub(crate) body_count: u32,

    pub(crate) head_contact: Option<ContactId>,
    pub(crate) tail_contact: Option<ContactId>,
    pub(crate) contact_count: u32,

    pub(crate) head_joint: Option<JointId>,
    pub(crate) tail_joint: Option<JointId>,
    pub(crate) joint_count: u32,

    pub(crate) sleep_timer: f32,
    pub(crate) is_sleeping: bool,

    pub(crate) constraints_removed: u32,
}

impl PhysicsIsland {
    /// Creates a new [`PhysicsIsland`] with the given ID.
    #[inline]
    pub const fn new(id: IslandId) -> Self {
        Self {
            id,
            head_body: None,
            tail_body: None,
            body_count: 0,
            head_contact: None,
            tail_contact: None,
            contact_count: 0,
            head_joint: None,
            tail_joint: None,
            joint_count: 0,
            sleep_timer: 0.0,
            is_sleeping: false,
            constraints_removed: 0,
        }
    }

    /// Returns the island ID.
    #[inline]
    pub const fn id(&self) -> IslandId {
        self.id
    }

    /// Returns the number of bodies in the island.
    #[inline]
    pub const fn body_count(&self) -> u32 {
        self.body_count
    }

    /// Returns the number of contacts in the island.
    #[inline]
    pub const fn contact_count(&self) -> u32 {
        self.contact_count
    }

    /// Returns the number of joints in the island.
    #[inline]
    pub const fn joint_count(&self) -> u32 {
        self.joint_count
    }

    /// Returns `true` if the island is sleeping.
    #[inline]
    pub const fn is_sleeping(&self) -> bool {
        self.is_sleeping
    }

    /// Returns the number of constraints that have been removed from the island,
    #[inline]
    pub const fn constraints_removed(&self) -> u32 {
        self.constraints_removed
    }

    // TODO: Use errors rather than panics.
    /// Validates the island.
    #[inline]
    pub fn validate(
        &self,
        body_islands: &Query<&BodyIslandNode, Or<(With<Disabled>, Without<Disabled>)>>,
        contact_graph: &ContactGraphInternal,
        joint_graph: &JointGraph,
    ) {
        self.validate_bodies(body_islands);
        self.validate_contacts(contact_graph);
        self.validate_joints(joint_graph);
    }

    /// Validates the body linked list.
    pub fn validate_bodies(
        &self,
        body_islands: &Query<&BodyIslandNode, Or<(With<Disabled>, Without<Disabled>)>>,
    ) {
        if self.head_body.is_none() {
            assert!(self.tail_body.is_none());
            assert_eq!(self.body_count, 0);
            return;
        }

        assert!(self.tail_body.is_some());
        assert!(self.body_count > 0);

        if self.body_count > 1 {
            assert_ne!(self.head_body, self.tail_body);
        }

        let mut count = 0;
        let mut body_id = self.head_body;

        while let Some(entity) = body_id {
            let body = body_islands.get(entity).unwrap();
            assert_eq!(body.island_id, self.id);

            count += 1;

            if count == self.body_count {
                assert_eq!(body_id, self.tail_body);
            }

            body_id = body.next;
        }

        assert_eq!(count, self.body_count);
    }

    /// Validates the contact linked list.
    pub fn validate_contacts(&self, contact_graph: &ContactGraphInternal) {
        if self.head_contact.is_none() {
            assert!(self.tail_contact.is_none());
            assert_eq!(self.contact_count, 0);
            return;
        }

        assert!(self.tail_contact.is_some());
        assert!(self.contact_count > 0);

        if self.contact_count > 1 {
            assert_ne!(self.head_contact, self.tail_contact);
        }

        let mut count = 0;
        let mut contact_id = self.head_contact;

        while let Some(id) = contact_id {
            let contact = contact_graph.edge_weight(id.into()).unwrap();
            let contact_island = contact
                .island
                .as_ref()
                .unwrap_or_else(|| panic!("Contact {id:?} has no island"));
            assert_eq!(contact_island.island_id, self.id);

            count += 1;

            if count == self.contact_count {
                assert_eq!(contact_id, self.tail_contact);
            }

            contact_id = contact_island.next;
        }

        assert_eq!(count, self.contact_count);
    }

    /// Validates the joint linked list.
    pub fn validate_joints(&self, joint_graph: &JointGraph) {
        if self.head_joint.is_none() {
            assert!(self.tail_joint.is_none());
            assert_eq!(self.joint_count, 0);
            return;
        }

        assert!(self.tail_joint.is_some());
        assert!(self.joint_count > 0);

        if self.joint_count > 1 {
            assert_ne!(self.head_joint, self.tail_joint);
        }

        let mut count = 0;
        let mut joint_id = self.head_joint;

        while let Some(id) = joint_id {
            let joint = joint_graph.get_by_id(id).unwrap();
            let joint_island = &joint.island;
            assert_eq!(joint_island.island_id, self.id);

            count += 1;

            if count == self.joint_count {
                assert_eq!(joint_id, self.tail_joint);
            }

            joint_id = joint_island.next;
        }

        assert_eq!(count, self.joint_count);
    }
}

/// A resource for the [`PhysicsIsland`]s in the simulation.
#[derive(Resource, Debug, Default, Clone)]
pub struct PhysicsIslands {
    /// The list of islands.
    islands: Slab<PhysicsIsland>,
    /// The current island candidate for splitting.
    ///
    /// This is chosen based on which island with one or more constraints removed
    /// has the largest sleep timer.
    pub split_candidate: Option<IslandId>,
    /// The largest [`SleepTimer`] of the split candidate.
    ///
    /// [`SleepTimer`]: crate::dynamics::rigid_body::sleeping::SleepTimer
    pub split_candidate_sleep_timer: f32,
}

impl PhysicsIslands {
    /// Creates a new [`PhysicsIsland`], calling the given `init` function before pushing the island to the list.
    #[inline]
    pub fn create_island_with<F>(&mut self, init: F) -> IslandId
    where
        F: FnOnce(&mut PhysicsIsland),
    {
        // Create a new island.
        let island_id = self.next_id();
        let mut island = PhysicsIsland::new(island_id);

        // Initialize the island with the provided function.
        init(&mut island);

        // Add the island to the list.
        IslandId(self.islands.insert(island) as u32)
    }

    /// Removes a [`PhysicsIsland`] with the given ID. The island is assumed to be empty,
    ///
    /// # Panics
    ///
    /// Panics if `island_id` is out of bounds.
    #[inline]
    pub fn remove_island(&mut self, island_id: IslandId) -> PhysicsIsland {
        if self.split_candidate == Some(island_id) {
            self.split_candidate = None;
        }

        // Assume the island is empty, and remove it.
        self.islands.remove(island_id.0 as usize)
    }

    /// Returns the next available island ID.
    #[inline]
    pub fn next_id(&self) -> IslandId {
        IslandId(self.islands.vacant_key() as u32)
    }

    /// Returns a reference to the [`PhysicsIsland`] with the given ID.
    #[inline]
    pub fn get(&self, island_id: IslandId) -> Option<&PhysicsIsland> {
        self.islands.get(island_id.0 as usize)
    }

    /// Returns a mutable reference to the [`PhysicsIsland`] with the given ID.
    #[inline]
    pub fn get_mut(&mut self, island_id: IslandId) -> Option<&mut PhysicsIsland> {
        self.islands.get_mut(island_id.0 as usize)
    }

    /// Returns an iterator over all [`PhysicsIsland`]s.
    #[inline]
    pub fn iter(&self) -> impl Iterator<Item = &PhysicsIsland> {
        self.islands.iter().map(|(_, island)| island)
    }

    /// Returns a mutable iterator over all [`PhysicsIsland`]s.
    #[inline]
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut PhysicsIsland> {
        self.islands.iter_mut().map(|(_, island)| island)
    }

    /// Returns the number of [`PhysicsIsland`]s.
    #[inline]
    pub fn len(&self) -> usize {
        self.islands.len()
    }

    /// Returns `true` if there are no [`PhysicsIsland`]s.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.islands.is_empty()
    }

    /// Adds a contact to the island manager. Returns a reference to the island that the contact was added to.
    ///
    /// This will merge the islands of the bodies involved in the contact,
    /// and link the contact to the resulting island.
    ///
    /// Called when a touching contact is created between two bodies.
    pub fn add_contact(
        &mut self,
        contact_id: ContactId,
        body_islands: &mut Query<&mut BodyIslandNode, Or<(With<Disabled>, Without<Disabled>)>>,
        contact_graph: &mut ContactGraph,
        joint_graph: &mut JointGraph,
    ) -> Option<&PhysicsIsland> {
        let contact = contact_graph.get_edge_mut_by_id(contact_id).unwrap();

        debug_assert!(contact.island.is_none());
        debug_assert!(contact.is_touching());

        let (Some(body1), Some(body2)) = (contact.body1, contact.body2) else {
            return None;
        };

        // Merge the islands.
        let island_id = self.merge_islands(body1, body2, body_islands, contact_graph, joint_graph);

        // Link the contact to the island.
        let island = self
            .islands
            .get_mut(island_id.0 as usize)
            .unwrap_or_else(|| {
                panic!("Island {island_id} does not exist");
            });

        let mut contact_island = IslandNode {
            island_id: island.id,
            prev: None,
            next: None,
            is_visited: false,
        };

        if let Some(head_contact_id) = island.head_contact {
            // Link the new contact to the head of the island.
            contact_island.next = Some(head_contact_id);

            let head_contact = contact_graph.get_edge_mut_by_id(head_contact_id).unwrap();
            let head_contact_island = head_contact
                .island
                .as_mut()
                .unwrap_or_else(|| panic!("Head contact {head_contact_id:?} has no island"));
            head_contact_island.prev = Some(contact_id);
        }

        island.head_contact = Some(contact_id);

        if island.tail_contact.is_none() {
            island.tail_contact = island.head_contact;
        }

        // TODO: Can we avoid this second lookup?
        let contact = contact_graph.get_edge_mut_by_id(contact_id).unwrap();
        contact.island = Some(contact_island);

        island.contact_count += 1;

        #[cfg(feature = "validate")]
        {
            // Validate the island.
            island.validate(
                &body_islands.as_readonly(),
                &contact_graph.edges,
                joint_graph,
            );
        }

        Some(island)
    }

    /// Removes a contact from the island manager. Returns a reference to the island that the contact was removed from.
    ///
    /// This will unlink the contact from the island and update the island's contact list.
    /// The [`PhysicsIsland::constraints_removed`] counter is incremented.
    ///
    /// Called when a contact is destroyed or no longer touching.
    #[expect(
        unused_variables,
        reason = "additional parameters are needed with the `validate` feature"
    )]
    pub fn remove_contact(
        &mut self,
        contact_id: ContactId,
        body_islands: &mut Query<&mut BodyIslandNode, Or<(With<Disabled>, Without<Disabled>)>>,
        contact_graph: &mut ContactGraphInternal,
        joint_graph: &JointGraph,
    ) -> &PhysicsIsland {
        let contact = contact_graph.edge_weight_mut(contact_id.into()).unwrap();

        debug_assert!(contact.island.is_some());

        // Remove the island from the contact edge.
        let contact_island = contact.island.take().unwrap();

        // Remove the contact from the island.
        if let Some(prev_contact_id) = contact_island.prev {
            let prev_contact = contact_graph
                .edge_weight_mut(prev_contact_id.into())
                .unwrap();
            let prev_contact_island = prev_contact
                .island
                .as_mut()
                .expect("Previous contact has no island");
            debug_assert!(prev_contact_island.next == Some(contact_id));
            prev_contact_island.next = contact_island.next;
        }

        if let Some(next_contact_id) = contact_island.next {
            let next_contact = contact_graph
                .edge_weight_mut(next_contact_id.into())
                .unwrap();
            let next_contact_island = next_contact
                .island
                .as_mut()
                .expect("Next contact has no island");
            debug_assert!(next_contact_island.prev == Some(contact_id));
            next_contact_island.prev = contact_island.prev;
        }

        let island = self
            .islands
            .get_mut(contact_island.island_id.0 as usize)
            .unwrap_or_else(|| {
                panic!("Island {} does not exist", contact_island.island_id);
            });

        if island.head_contact == Some(contact_id) {
            // The contact is the head of the island.
            island.head_contact = contact_island.next;
        }

        if island.tail_contact == Some(contact_id) {
            // The contact is the tail of the island.
            island.tail_contact = contact_island.prev;
        }

        debug_assert!(island.contact_count > 0);
        island.contact_count -= 1;
        island.constraints_removed += 1;

        #[cfg(feature = "validate")]
        {
            // Validate the island.
            island.validate(&body_islands.as_readonly(), contact_graph, joint_graph);
        }

        island
    }

    /// Adds a joint to the island manager. Returns a reference to the island that the joint was added to.
    ///
    /// This will merge the islands of the bodies connected by the joint,
    /// and link the joint to the resulting island.
    ///
    /// Called when a joint is created between two bodies.
    pub fn add_joint(
        &mut self,
        joint_id: JointId,
        body_islands: &mut Query<&mut BodyIslandNode, Or<(With<Disabled>, Without<Disabled>)>>,
        contact_graph: &mut ContactGraph,
        joint_graph: &mut JointGraph,
    ) -> Option<&PhysicsIsland> {
        let joint = joint_graph.get_mut_by_id(joint_id).unwrap();

        // Merge the islands.
        let island_id = self.merge_islands(
            joint.body1,
            joint.body2,
            body_islands,
            contact_graph,
            joint_graph,
        );

        // Link the joint to the island.
        let island = self
            .islands
            .get_mut(island_id.0 as usize)
            .unwrap_or_else(|| {
                panic!("Island {island_id} does not exist");
            });

        let mut joint_island = IslandNode {
            island_id: island.id,
            prev: None,
            next: None,
            is_visited: false,
        };

        if let Some(head_joint_id) = island.head_joint {
            // Link the new joint to the head of the island.
            joint_island.next = Some(head_joint_id);

            let head_joint = joint_graph.get_mut_by_id(head_joint_id).unwrap();
            let head_island = &mut head_joint.island;
            head_island.prev = Some(joint_id);
        }

        island.head_joint = Some(joint_id);

        if island.tail_joint.is_none() {
            island.tail_joint = island.head_joint;
        }

        // TODO: Can we avoid this second lookup?
        let joint = joint_graph.get_mut_by_id(joint_id).unwrap();
        joint.island = joint_island;

        island.joint_count += 1;

        #[cfg(feature = "validate")]
        {
            // Validate the island.
            island.validate(
                &body_islands.as_readonly(),
                &contact_graph.edges,
                joint_graph,
            );
        }

        Some(island)
    }

    /// Removes a joint from the island manager. Returns a reference to the island
    /// that the joint was removed from, if the island still exists.
    ///
    /// This will unlink the joint from the island and update the island's joint list.
    /// The [`PhysicsIsland::constraints_removed`] counter is incremented.
    ///
    /// The joint should be removed from the [`JointGraph`] in concert with calling this method.
    ///
    /// Called when a joint is destroyed or no longer connected to the bodies.
    #[expect(
        unused_variables,
        reason = "additional parameters are needed with the `validate` feature"
    )]
    pub fn remove_joint(
        &mut self,
        joint_id: JointId,
        body_islands: &mut Query<&mut BodyIslandNode, Or<(With<Disabled>, Without<Disabled>)>>,
        contact_graph: &ContactGraph,
        joint_graph: &mut JointGraph,
    ) -> Option<&PhysicsIsland> {
        let joint = joint_graph.get_mut_by_id(joint_id).unwrap();

        debug_assert!(joint.island.island_id != IslandId::PLACEHOLDER);

        // Remove the island from the joint edge.
        let joint_island = joint.island;

        // Remove the joint from the island.
        if let Some(prev_joint_id) = joint_island.prev {
            let prev_joint = joint_graph.get_mut_by_id(prev_joint_id).unwrap();
            let prev_island = &mut prev_joint.island;
            debug_assert!(prev_island.next == Some(joint_id));
            prev_island.next = joint_island.next;
        }

        if let Some(next_joint_id) = joint_island.next {
            let next_joint = joint_graph.get_mut_by_id(next_joint_id).unwrap();
            let next_island = &mut next_joint.island;
            debug_assert!(next_island.prev == Some(joint_id));
            next_island.prev = joint_island.prev;
        }

        let island = self.islands.get_mut(joint_island.island_id.0 as usize)?;

        if island.head_joint == Some(joint_id) {
            // The joint is the head of the island.
            island.head_joint = joint_island.next;
        }

        if island.tail_joint == Some(joint_id) {
            // The joint is the tail of the island.
            island.tail_joint = joint_island.prev;
        }

        debug_assert!(island.joint_count > 0);
        island.joint_count -= 1;
        island.constraints_removed += 1;

        #[cfg(feature = "validate")]
        {
            // Validate the island.
            island.validate(
                &body_islands.as_readonly(),
                &contact_graph.edges,
                joint_graph,
            );
        }

        Some(island)
    }

    /// Merges the [`PhysicsIsland`]s associated with the given bodies. Returns the ID of the resulting island.
    ///
    /// If an awake island is merged with a sleeping island, the resulting island will remain sleeping.
    /// It is up to the caller to wake up the resulting island if needed.
    ///
    /// The bodies and contacts of the smaller island are transferred to the larger island,
    /// and the smaller island is removed.
    pub fn merge_islands(
        &mut self,
        body1: Entity,
        body2: Entity,
        body_islands: &mut Query<&mut BodyIslandNode, Or<(With<Disabled>, Without<Disabled>)>>,
        contact_graph: &mut ContactGraph,
        joint_graph: &mut JointGraph,
    ) -> IslandId {
        let Ok(island_id1) = body_islands.get(body1).map(|island| island.island_id) else {
            let island2 = body_islands.get(body2).unwrap_or_else(|_| {
                panic!("Neither body {body1:?} nor {body2:?} is in an island");
            });
            return island2.island_id;
        };

        let Ok(island_id2) = body_islands.get(body2).map(|island| island.island_id) else {
            let island1 = body_islands.get(body1).unwrap_or_else(|_| {
                panic!("Neither body {body1:?} nor {body2:?} is in an island");
            });
            return island1.island_id;
        };

        if island_id1 == island_id2 {
            // Merging an island with itself is a no-op.
            return island_id1;
        }

        // Get the islands to merge.
        // Keep the bigger island to reduce cache misses.
        let [mut big, mut small] = self
            .islands
            .get_disjoint_mut([island_id1.0 as usize, island_id2.0 as usize])
            .unwrap();
        if big.body_count < small.body_count {
            core::mem::swap(&mut big, &mut small);
        }

        // 1. Remap IDs such that the bodies and constraints in `small` are moved to `big`.

        // Bodies
        let mut body_entity = small.head_body;
        while let Some(entity) = body_entity {
            let mut body_island = body_islands.get_mut(entity).unwrap();
            body_island.island_id = big.id;
            body_entity = body_island.next;
        }

        // Contacts
        let mut contact_id = small.head_contact;
        while let Some(id) = contact_id {
            let contact = contact_graph.get_edge_mut_by_id(id).unwrap();
            let contact_island = contact.island.as_mut().expect("Contact has no island");
            contact_island.island_id = big.id;
            contact_id = contact_island.next;
        }

        // Joints
        let mut joint_id = small.head_joint;
        while let Some(id) = joint_id {
            let joint = joint_graph.get_mut_by_id(id).unwrap();
            joint.island.island_id = big.id;
            joint_id = joint.island.next;
        }

        // 2. Connect body and constraint lists such that `small` is appended to `big`.

        // Bodies
        let tail_body_id = big.tail_body.expect("Island {island_id1} has no tail body");
        let head_body_id = small
            .head_body
            .expect("Island {island_id2} has no head body");
        let [mut tail_body, mut head_body] = body_islands
            .get_many_mut([tail_body_id, head_body_id])
            .unwrap();
        tail_body.next = small.head_body;
        head_body.prev = big.tail_body;

        big.tail_body = small.tail_body;
        big.body_count += small.body_count;

        // Contacts
        if big.head_contact.is_none() {
            // The big island has no contacts.
            debug_assert!(big.tail_contact.is_none() && big.contact_count == 0);

            big.head_contact = small.head_contact;
            big.tail_contact = small.tail_contact;
            big.contact_count = small.contact_count;
        } else if small.head_contact.is_some() {
            // Both islands have contacts.
            debug_assert!(big.contact_count > 0);
            debug_assert!(small.contact_count > 0);

            let tail_contact_id = big.tail_contact.expect("Root island has no tail contact");
            let head_contact_id = small.head_contact.expect("Island has no head contact");

            // Connect the tail of the big island to the head of the small island.
            let tail_contact = contact_graph.get_edge_mut_by_id(tail_contact_id).unwrap();
            let tail_contact_island = tail_contact
                .island
                .as_mut()
                .expect("Tail contact has no island");
            debug_assert!(tail_contact_island.next.is_none());
            tail_contact_island.next = small.head_contact;

            // Connect the head of the small island to the tail of the big island.
            let head_contact = contact_graph.get_edge_mut_by_id(head_contact_id).unwrap();
            let head_contact_island = head_contact
                .island
                .as_mut()
                .expect("Head contact has no island");
            debug_assert!(head_contact_island.prev.is_none());
            head_contact_island.prev = big.tail_contact;

            big.tail_contact = small.tail_contact;
            big.contact_count += small.contact_count;
        }

        // Joints
        if big.head_joint.is_none() {
            // The big island has no joints.
            debug_assert!(big.tail_joint.is_none() && big.joint_count == 0);

            big.head_joint = small.head_joint;
            big.tail_joint = small.tail_joint;
            big.joint_count = small.joint_count;
        } else if small.head_joint.is_some() {
            // Both islands have joints.
            debug_assert!(big.joint_count > 0);
            debug_assert!(small.joint_count > 0);

            let tail_joint_id = big.tail_joint.expect("Root island has no tail joint");
            let head_joint_id = small.head_joint.expect("Island has no head joint");

            // Connect the tail of the big island to the head of the small island.
            let tail_joint = joint_graph.get_mut_by_id(tail_joint_id).unwrap();
            let tail_island = &mut tail_joint.island;
            debug_assert!(tail_island.next.is_none());
            tail_island.next = small.head_joint;

            // Connect the head of the small island to the tail of the big island.
            let head_joint = joint_graph.get_mut_by_id(head_joint_id).unwrap();
            let head_island = &mut head_joint.island;
            debug_assert!(head_island.prev.is_none());
            head_island.prev = big.tail_joint;

            big.tail_joint = small.tail_joint;
            big.joint_count += small.joint_count;
        }

        // Track removed constraints.
        big.constraints_removed += small.constraints_removed;

        // 3. Update the sleep state of the big island.
        if small.is_sleeping {
            // If the small island is sleeping, the big island will remain sleeping.
            big.is_sleeping = true;
            big.sleep_timer = small.sleep_timer.max(big.sleep_timer);
        }

        #[cfg(feature = "validate")]
        {
            // Validate the big island.
            big.validate(
                &body_islands.as_readonly(),
                &contact_graph.edges,
                joint_graph,
            );
        }

        // 4. Remove the small island.
        let big_id = big.id;
        let small_id = small.id;
        self.remove_island(small_id);

        big_id
    }

    /// Splits the [`PhysicsIsland`] associated with the given ID.
    ///
    /// Unlike merging, splitting can be deferred and done in parallel with other work.
    pub fn split_island(
        &mut self,
        island_id: IslandId,
        body_islands: &mut Query<&mut BodyIslandNode, Or<(With<Disabled>, Without<Disabled>)>>,
        body_colliders: &Query<&RigidBodyColliders>,
        contact_graph: &mut ContactGraph,
        joint_graph: &mut JointGraph,
    ) {
        let island = self.islands.get_mut(island_id.0 as usize).unwrap();

        if island.is_sleeping {
            // Only awake islands can be split.
            return;
        }

        if island.constraints_removed == 0 {
            // No constraints have been removed, so no need to split the island.
            return;
        }

        #[cfg(feature = "validate")]
        {
            // Validate the island before splitting.
            island.validate(
                &body_islands.as_readonly(),
                &contact_graph.edges,
                joint_graph,
            );
        }

        let body_count = island.body_count;

        // Build a `Vec` of all body IDs in the base island.
        // These are seeds for the depth-first search (DFS).
        //
        // Also clear visited flags for bodies.
        let mut body_ids = Vec::with_capacity(body_count as usize);
        let mut next_body = island.head_body;

        while let Some(body_id) = next_body {
            body_ids.push(body_id);

            let mut body_island = body_islands.get_mut(body_id).unwrap();

            // Clear visited flag.
            body_island.is_visited = false;

            next_body = body_island.next;
        }

        debug_assert_eq!(body_ids.len(), body_count as usize);

        // Clear visited flags for contacts.
        let mut next_contact = island.head_contact;
        while let Some(contact_id) = next_contact {
            let contact = contact_graph.get_edge_mut_by_id(contact_id).unwrap();
            let contact_island = contact
                .island
                .as_mut()
                .unwrap_or_else(|| panic!("Contact {contact_id:?} has no island"));

            // Clear visited flag.
            contact_island.is_visited = false;

            next_contact = contact_island.next;
        }

        // Clear visited flags for joints.
        let mut next_joint = island.head_joint;
        while let Some(joint_id) = next_joint {
            let joint = joint_graph.get_mut_by_id(joint_id).unwrap();
            let joint_island = &mut joint.island;

            // Clear visited flag.
            joint_island.is_visited = false;

            next_joint = joint_island.next;
        }

        // Destroy the base island.
        self.remove_island(island_id);

        // Split the island using a depth-first search (DFS) starting from the seeds.
        let mut stack = Vec::with_capacity(body_ids.len());
        for seed_id in body_ids.into_iter() {
            let mut seed = body_islands.get_mut(seed_id).unwrap();

            if seed.is_visited {
                // The body has already been visited.
                continue;
            }

            seed.is_visited = true;

            // Create a new island.
            let island_id = self.next_id();
            let mut island = PhysicsIsland::new(island_id);

            // Initialize the stack with the seed body.
            stack.push(seed_id);

            // Traverse the constraint graph using a depth-first search (DFS).
            while let Some(body) = stack.pop() {
                let mut body_island = unsafe { body_islands.get_unchecked(body).unwrap() };

                debug_assert!(body_island.is_visited);

                // Add the body to the new island.
                body_island.island_id = island_id;

                if let Some(tail_body_id) = island.tail_body {
                    debug_assert_ne!(tail_body_id, body);
                    unsafe {
                        body_islands.get_unchecked(tail_body_id).unwrap().next = Some(body);
                    }
                }

                body_island.prev = island.tail_body;
                body_island.next = None;
                island.tail_body = Some(body);

                if island.head_body.is_none() {
                    island.head_body = Some(body);
                }

                island.body_count += 1;

                // Traverse the contacts of the body.
                // TODO: Avoid collecting here and only iterate once.
                let contact_edges: Vec<(ContactId, Entity)> = body_colliders
                    .get(body)
                    .iter()
                    .flat_map(|colliders| {
                        colliders.into_iter().flat_map(|collider| {
                            contact_graph
                                .contact_edges_with(collider)
                                .filter_map(|contact_edge| {
                                    if contact_edge.island.is_some_and(|island| island.is_visited) {
                                        // Only consider contacts that have not been visited yet.
                                        return None;
                                    }

                                    if contact_edge.constraint_handles.is_empty() {
                                        // Only consider touching contacts.
                                        return None;
                                    }

                                    // TODO: Remove this once the contact graph is reworked to only have rigid body collisions.
                                    let (Some(body1), Some(body2)) =
                                        (contact_edge.body1, contact_edge.body2)
                                    else {
                                        // Only consider contacts with two bodies.
                                        return None;
                                    };

                                    Some((
                                        contact_edge.id,
                                        if body1 == body { body2 } else { body1 },
                                    ))
                                })
                        })
                    })
                    .collect();

                for (contact_id, other_body) in contact_edges {
                    // Maybe add the other body to the stack.
                    if let Ok(mut other_body_island) = body_islands.get_mut(other_body)
                        && !other_body_island.is_visited
                    {
                        stack.push(other_body);
                        other_body_island.is_visited = true;
                    }

                    // Add the contact to the new island.
                    if let Some(tail_contact_id) = island.tail_contact {
                        let tail_contact =
                            contact_graph.get_edge_mut_by_id(tail_contact_id).unwrap();
                        let tail_contact_island =
                            tail_contact.island.as_mut().unwrap_or_else(|| {
                                panic!("Tail contact {tail_contact_id:?} has no island")
                            });
                        tail_contact_island.next = Some(contact_id);
                    }

                    let contact_edge = contact_graph.get_edge_mut_by_id(contact_id).unwrap();
                    let contact_island = contact_edge
                        .island
                        .as_mut()
                        .unwrap_or_else(|| panic!("Contact {contact_id:?} has no island"));

                    contact_island.is_visited = true;
                    contact_island.island_id = island_id;
                    contact_island.prev = island.tail_contact;
                    contact_island.next = None;

                    island.tail_contact = Some(contact_id);

                    if island.head_contact.is_none() {
                        island.head_contact = Some(contact_id);
                    }

                    island.contact_count += 1;
                }

                // Traverse the joints of the body.
                let joint_edges: Vec<(JointId, Entity)> = joint_graph
                    .joints_of(body)
                    .filter_map(|joint_edge| {
                        if joint_edge.island.is_visited {
                            // Only consider joints that have not been visited yet.
                            return None;
                        }

                        Some((
                            joint_edge.id,
                            if joint_edge.body1 == body {
                                joint_edge.body2
                            } else {
                                joint_edge.body1
                            },
                        ))
                    })
                    .collect();

                for (joint_id, other_body) in joint_edges {
                    // Maybe add the other body to the stack.
                    if let Ok(mut other_body_island) = body_islands.get_mut(other_body)
                        && !other_body_island.is_visited
                    {
                        stack.push(other_body);
                        other_body_island.is_visited = true;
                    }

                    // Add the joint to the new island.
                    if let Some(tail_joint_id) = island.tail_joint {
                        let tail_joint = joint_graph.get_mut_by_id(tail_joint_id).unwrap();
                        let tail_island = &mut tail_joint.island;
                        tail_island.next = Some(joint_id);
                    }

                    let joint = joint_graph.get_mut_by_id(joint_id).unwrap();
                    let joint_island = &mut joint.island;

                    joint_island.is_visited = true;
                    joint_island.island_id = island_id;
                    joint_island.prev = island.tail_joint;
                    joint_island.next = None;

                    island.tail_joint = Some(joint_id);

                    if island.head_joint.is_none() {
                        island.head_joint = Some(joint_id);
                    }

                    island.joint_count += 1;
                }
            }

            #[cfg(feature = "validate")]
            {
                // Validate the new island.
                island.validate(
                    &body_islands.as_readonly(),
                    &contact_graph.edges,
                    joint_graph,
                );
            }

            // Add the new island to the list.
            self.islands.insert(island);
        }
    }
}

/// A node in a linked list in a [`PhysicsIsland`].
#[derive(Clone, Debug, PartialEq, Eq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct IslandNode<Id> {
    /// The ID of the island that the node belongs to.
    pub(crate) island_id: IslandId,
    /// The ID of the previous node in the linked list.
    pub(crate) prev: Option<Id>,
    /// The ID of the next node in the linked list.
    pub(crate) next: Option<Id>,
    /// A flag to mark the node as visited during depth-first traversal (DFS) for island splitting.
    pub(crate) is_visited: bool,
}

impl<Id> IslandNode<Id> {
    /// A placeholder [`IslandNode`] that has not been initialized yet.
    pub const PLACEHOLDER: Self = Self::new(IslandId::PLACEHOLDER);

    /// Creates a new [`IslandNode`] with the given island ID.
    #[inline]
    pub const fn new(island_id: IslandId) -> Self {
        Self {
            island_id,
            prev: None,
            next: None,
            is_visited: false,
        }
    }

    /// Returns the [`IslandId`] of the island that the node belongs to.
    #[inline]
    pub const fn island_id(&self) -> IslandId {
        self.island_id
    }
}

impl<Id> Default for IslandNode<Id> {
    fn default() -> Self {
        Self::PLACEHOLDER
    }
}

impl<Id: Copy> Copy for IslandNode<Id> {}

/// A component that stores [`PhysicsIsland`] connectivity data for a rigid body.
#[derive(Component, Clone, Debug, Default, Deref, DerefMut, PartialEq, Eq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[component(on_add = BodyIslandNode::on_add, on_remove = BodyIslandNode::on_remove)]
pub struct BodyIslandNode(IslandNode<Entity>);

impl BodyIslandNode {
    /// Creates a new [`BodyIslandNode`] with the given island ID.
    #[inline]
    pub const fn new(island_id: IslandId) -> Self {
        Self(IslandNode::new(island_id))
    }

    // Initialize a new island when `BodyIslandNode` is added to a body.
    fn on_add(mut world: DeferredWorld, ctx: HookContext) {
        // Create a new island for the body.
        let mut islands = world.resource_mut::<PhysicsIslands>();
        let island_id = islands.create_island_with(|island| {
            island.head_body = Some(ctx.entity);
            island.tail_body = Some(ctx.entity);
            island.body_count = 1;
        });

        // Set the island ID for the body.
        let mut body_island = world.get_mut::<BodyIslandNode>(ctx.entity).unwrap();
        body_island.island_id = island_id;
    }

    // Remove the body from the island when `BodyIslandNode` is removed.
    fn on_remove(mut world: DeferredWorld, ctx: HookContext) {
        let body_island = world.get::<BodyIslandNode>(ctx.entity).unwrap();
        let island_id = body_island.island_id;
        let prev_body_entity = body_island.prev;
        let next_body_entity = body_island.next;

        // Fix the linked list of bodies in the island.
        if let Some(entity) = prev_body_entity {
            let mut prev_body_island = world.get_mut::<BodyIslandNode>(entity).unwrap();
            prev_body_island.next = next_body_entity;
        }
        if let Some(entity) = next_body_entity {
            let mut next_body_island = world.get_mut::<BodyIslandNode>(entity).unwrap();
            next_body_island.prev = prev_body_entity;
        }

        let mut islands = world.resource_mut::<PhysicsIslands>();
        let island = islands
            .get_mut(island_id)
            .unwrap_or_else(|| panic!("Island {island_id} does not exist"));

        debug_assert!(island.body_count > 0);
        island.body_count -= 1;

        #[cfg(feature = "validate")]
        let mut island_removed = false;

        if island.head_body == Some(ctx.entity) {
            island.head_body = next_body_entity;

            if island.head_body.is_none() {
                // The island is empty. Remove it.
                debug_assert!(island.tail_body == Some(ctx.entity));
                debug_assert!(island.body_count == 0);
                debug_assert!(island.contact_count == 0);

                world
                    .resource_mut::<PhysicsIslands>()
                    .remove_island(island_id);

                #[cfg(feature = "validate")]
                {
                    island_removed = true;
                }
            }
        } else if island.tail_body == Some(ctx.entity) {
            island.tail_body = prev_body_entity;
        }

        #[cfg(feature = "validate")]
        if !island_removed {
            // Validate the island.
            world.commands().queue(move |world: &mut World| {
                use bevy::ecs::system::RunSystemOnce;
                // TODO: This is probably quite inefficient.
                let _ = world.run_system_once(
                    move |bodies: Query<
                        &BodyIslandNode,
                        Or<(With<Disabled>, Without<Disabled>)>,
                    >,
                          islands: Res<PhysicsIslands>,
                          contact_graph: Res<ContactGraph>,
                          joint_graph: Res<JointGraph>| {
                        let island = islands
                            .get(island_id)
                            .unwrap_or_else(|| panic!("Island {island_id} does not exist"));
                        island.validate(&bodies, &contact_graph.edges, &joint_graph);
                    },
                );
            });
        }
    }
}
