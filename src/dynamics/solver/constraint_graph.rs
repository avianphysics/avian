//! A [`ConstraintGraph`] with [graph coloring] for solving constraints in parallel.
//!
//! The constraint graph implements a greedy edge coloring algorithm that assigns
//! constraints to colors such that no adjacent edges in the graph share the same color.
//! This ensures that each body only appears in a given color once, allowing constraints
//! within the same color to be solved in parallel without race conditions.
//!
//! See [`ConstraintGraph`] for more details on how the graph is structured.
//!
//! [graph coloring]: https://en.wikipedia.org/wiki/Graph_coloring
//!
//! # References
//!
//! - [High-Performance Physical Simulations on Next-Generation Architecture with Many Cores][Intel paper]
//! - [Box2D - SIMD Matters] by [Erin Catto]
//!
//! [Intel paper]: http://web.eecs.umich.edu/~msmelyan/papers/physsim_onmanycore_itj.pdf
//! [Box2D - SIMD Matters]: https://box2d.org/posts/2024/08/simd-matters/
//! [Erin Catto]: https://github.com/erincatto

#[cfg(feature = "serialize")]
use bevy::reflect::{ReflectDeserialize, ReflectSerialize};
use bevy::{
    ecs::{entity::Entity, resource::Resource},
    reflect::Reflect,
};

use smallvec::SmallVec;

use crate::{
    collision::contact_types::ContactId,
    data_structures::bit_vec::BitVec,
    prelude::{ContactPair, ContactPairFlags},
};

use super::contact::ContactConstraint;

/// The [`ContactConstraintHandle`]s associated with a single contact, with one handle
/// per contact manifold that generates constraints.
///
/// Stored inline for up to 2 (in 2D) or 4 (in 3D) manifolds before spilling to the heap.
#[cfg(feature = "2d")]
type ManifoldConstraintHandles = SmallVec<[ContactConstraintHandle; 2]>;

/// The [`ContactConstraintHandle`]s associated with a single contact, with one handle
/// per contact manifold that generates constraints.
///
/// Stored inline for up to 2 (in 2D) or 4 (in 3D) manifolds before spilling to the heap.
#[cfg(feature = "3d")]
type ManifoldConstraintHandles = SmallVec<[ContactConstraintHandle; 4]>;

/// The maximum number of [`GraphColor`]s in the [`ConstraintGraph`].
/// Constraints that cannot find a color are added to the overflow set,
/// which is solved on a single thread.
pub const GRAPH_COLOR_COUNT: usize = 24;

/// The index of the overflow color in the graph, used for constraints that don't fit
/// the graph color limit. This can happen when a single body is interacting with many other bodies.
pub const COLOR_OVERFLOW_INDEX: usize = GRAPH_COLOR_COUNT - 1;

/// The number of colors with constraints involving two non-static bodies.
/// Leaving constraints involving static bodies to later colors gives higher priority
/// to those constraints, reducing tunneling through static geometry.
pub const DYNAMIC_COLOR_COUNT: usize = GRAPH_COLOR_COUNT - 4;

/// A color in the [`ConstraintGraph`]. Each color is a set of bodies and constraints
/// that can be solved in parallel without race conditions.
///
/// Only awake dynamic and kinematic bodies are included in graph coloring. For static bodies,
/// the solver uses a dummy [`SolverBody`].
///
/// Kinematic bodies cannot use a dummy [`SolverBody`], because we cannot access the same body
/// from multiple threads efficiently, since the (to-be-implemented) SIMD solver body scatter
/// would write to the same body from multiple threads. Even if these writes didn't actually modify
/// the body, they would still cause horrible cache stalls.
///
/// [`SolverBody`]: crate::dynamics::solver::solver_body::SolverBody
#[derive(Clone, Debug, Default, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug)]
pub struct GraphColor {
    /// A bit vector representing the [`SolverBody`]s that are part of this color.
    ///
    /// The bit vector is indexed by the body index, so it also contains static and sleeping bodies.
    /// However, these bits are never iterated over, and the bit count is not used,
    /// so it should not be a problem.
    ///
    /// [`SolverBody`]: crate::dynamics::solver::solver_body::SolverBody
    pub body_set: BitVec,
    /// The handles of the contact manifolds in this color.
    pub manifold_handles: Vec<ContactManifoldHandle>,
    /// The contact constraints in this color.
    pub contact_constraints: Vec<ContactConstraint>,
    // TODO: Joints
}

/// A handle to a contact manifold in the [`ContactGraph`].
///
/// [`ContactGraph`]: crate::collision::contact_types::ContactGraph
#[derive(Clone, Copy, Debug, PartialEq, Eq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct ContactManifoldHandle {
    /// The stable identifier of the contact pair in the [`ContactGraph`].
    ///
    /// [`ContactGraph`]: crate::collision::contact_types::ContactGraph
    pub contact_id: ContactId,
    /// The index of the manifold in the [`ContactPair`]. This is not stable
    /// and may change when the contact pair is updated.
    pub manifold_index: usize,
}

/// A handle to a contact constraint in the [`ConstraintGraph`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct ContactConstraintHandle {
    /// The index of the [`GraphColor`] of the constraint in the [`ConstraintGraph`].
    pub color_index: u8,
    /// The index of the constraint in the [`GraphColor`]. This is not stable
    /// and may change when the contact pair is updated.
    pub local_index: usize,
}

/// The constraint graph used for graph coloring to solve constraints in parallel.
///
/// The graph holds up to [`GRAPH_COLOR_COUNT`] colors, where each [`GraphColor`] is a set of bodies and constraints
/// that can be solved in parallel without race conditions. Each body can appear in a given color only once.
///
/// The last color, [`COLOR_OVERFLOW_INDEX`], is used for constraints that cannot find a color.
/// They are solved serially on a single thread.
///
/// Each [`ContactManifold`] is considered to be a separate edge, because each manifold has its own [`ContactConstraint`].
///
/// See the [module-level documentation](self) for more general information about graph coloring.
///
/// [`ContactManifold`]: crate::collision::contact_types::ContactManifold
#[derive(Resource, Clone, Debug, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug)]
pub struct ConstraintGraph {
    /// The colors in the graph.
    pub colors: Vec<GraphColor>,

    /// The [`ContactConstraintHandle`]s associated with each contact, indexed by [`ContactId`].
    /// Each contact has one handle per contact manifold that generates constraints.
    ///
    /// Entries persist as empty lists once allocated, since [`ContactId`]s are reused by the
    /// [`ContactGraph`].
    ///
    /// [`ContactGraph`]: crate::collision::contact_types::ContactGraph
    #[reflect(ignore)]
    contact_handles: Vec<ManifoldConstraintHandles>,
}

impl Default for ConstraintGraph {
    fn default() -> Self {
        Self::new(16)
    }
}

impl ConstraintGraph {
    /// Creates a new constraint graph with the given number of colors.
    pub fn new(body_capacity: usize) -> Self {
        let mut colors = Vec::with_capacity(GRAPH_COLOR_COUNT);

        let bit_capacity = body_capacity.max(8);

        // Initialize graph color bit vectors.
        for _ in 0..GRAPH_COLOR_COUNT {
            let mut body_set = BitVec::new(bit_capacity);
            body_set.set_bit_count_and_clear(bit_capacity);
            colors.push(GraphColor {
                body_set,
                // TODO: What's a good initial capacity for contacts and constraints?
                manifold_handles: Vec::with_capacity(bit_capacity),
                contact_constraints: Vec::with_capacity(bit_capacity),
            });
        }

        Self {
            colors,
            contact_handles: Vec::new(),
        }
    }

    /// Returns the [`ContactConstraintHandle`]s associated with the given contact,
    /// with one handle per contact manifold that generates constraints.
    ///
    /// Returns an empty slice if the contact has no associated constraints.
    #[inline]
    pub fn contact_handles(&self, contact_id: ContactId) -> &[ContactConstraintHandle] {
        self.contact_handles
            .get(contact_id.0 as usize)
            .map_or(&[], |handles| handles.as_slice())
    }

    /// Returns the number of contact manifold constraints associated with the given contact.
    #[inline]
    pub fn manifold_count(&self, contact_id: ContactId) -> usize {
        self.contact_handles(contact_id).len()
    }

    /// Adds a manifold to the graph, associating it with the given contact pair.
    pub fn push_manifold(&mut self, contact_pair: &ContactPair) {
        // TODO: These shouldn't be `Option`s.
        let (Some(body1), Some(body2)) = (contact_pair.body1, contact_pair.body2) else {
            return;
        };

        let is_static1 = contact_pair.flags.contains(ContactPairFlags::STATIC1);
        let is_static2 = contact_pair.flags.contains(ContactPairFlags::STATIC2);

        debug_assert!(!contact_pair.manifolds.is_empty());
        debug_assert!(!is_static1 || !is_static2);

        let mut color_index = COLOR_OVERFLOW_INDEX;

        // TODO: We could allow forcing overflow by making this optional.
        if !is_static1 && !is_static2 {
            // Constraints involving only non-static bodies cannot be in colors reserved
            // for constraints involving static bodies. This helps reduce tunneling through
            // static geometry by solving static contacts last.
            for i in 0..DYNAMIC_COLOR_COUNT {
                let color = &mut self.colors[i];
                if color.body_set.get(body1.index_u32() as usize)
                    || color.body_set.get(body2.index_u32() as usize)
                {
                    continue;
                }

                color.body_set.set_and_grow(body1.index_u32() as usize);
                color.body_set.set_and_grow(body2.index_u32() as usize);
                color_index = i;
                break;
            }
        } else if !is_static1 {
            // Build static colors from the end to give them higher priority.
            for i in (1..COLOR_OVERFLOW_INDEX).rev() {
                let color = &mut self.colors[i];
                if color.body_set.get(body1.index_u32() as usize) {
                    continue;
                }

                color.body_set.set_and_grow(body1.index_u32() as usize);
                color_index = i;
                break;
            }
        } else if !is_static2 {
            // Build static colors from the end to give them higher priority.
            for i in (1..COLOR_OVERFLOW_INDEX).rev() {
                let color = &mut self.colors[i];
                if color.body_set.get(body2.index_u32() as usize) {
                    continue;
                }

                color.body_set.set_and_grow(body2.index_u32() as usize);
                color_index = i;
                break;
            }
        }

        // Ensure there is a slot for this contact's constraint handles.
        let contact_index = contact_pair.contact_id.0 as usize;
        if contact_index >= self.contact_handles.len() {
            self.contact_handles
                .resize_with(contact_index + 1, ManifoldConstraintHandles::new);
        }

        // Add a constraint handle for the contact.
        let handles = &mut self.contact_handles[contact_index];
        let manifold_index = handles.len();
        let color = &mut self.colors[color_index];
        handles.push(ContactConstraintHandle {
            color_index: color_index as u8,
            local_index: color.manifold_handles.len(),
        });

        // Add the handle of the contact manifold to the color.
        color.manifold_handles.push(ContactManifoldHandle {
            contact_id: contact_pair.contact_id,
            manifold_index,
        });
    }

    /// Removes the [`ContactConstraintHandle`] corresponding to the last [`ContactManifold`]
    /// associated with the given contact, updating the color's body set and manifold handles
    /// accordingly.
    ///
    /// Returns the removed [`ContactConstraintHandle`] if it exists.
    ///
    /// To remove *all* constraints associated with a contact, use [`remove_contact`](Self::remove_contact).
    ///
    /// [`ContactManifold`]: crate::collision::contact_types::ContactManifold
    pub fn pop_manifold(
        &mut self,
        contact_id: ContactId,
        body1: Entity,
        body2: Entity,
    ) -> Option<ContactConstraintHandle> {
        // Remove a constraint handle associated with the contact.
        let handles = self.contact_handles.get_mut(contact_id.0 as usize)?;
        let contact_constraint_handle = handles.pop()?;

        let color_index = contact_constraint_handle.color_index as usize;
        let local_index = contact_constraint_handle.local_index;
        debug_assert!(color_index < GRAPH_COLOR_COUNT);

        let color = &mut self.colors[color_index];

        if color_index != COLOR_OVERFLOW_INDEX {
            // Remove the bodies from the color's body set.
            color.body_set.unset(body1.index_u32() as usize);
            color.body_set.unset(body2.index_u32() as usize);
        }

        // Remove the manifold handle from the color.
        let moved_index = color.manifold_handles.len() - 1;
        color.manifold_handles.swap_remove(local_index);

        if moved_index != local_index {
            // Fix moved manifold handle.
            let moved_handle = color.manifold_handles[local_index];

            // Update the local index of the constraint handle associated with the moved manifold handle.
            let constraint_handle = &mut self.contact_handles[moved_handle.contact_id.0 as usize]
                [moved_handle.manifold_index];
            debug_assert!(constraint_handle.color_index == color_index as u8);
            debug_assert!(constraint_handle.local_index == moved_index);
            constraint_handle.local_index = local_index;
        }

        // Return the constraint handle that was removed.
        Some(contact_constraint_handle)
    }

    /// Removes all [`ContactConstraintHandle`]s associated with the given contact from the graph,
    /// updating the colors' body sets and manifold handles accordingly.
    ///
    /// This is equivalent to calling [`pop_manifold`](Self::pop_manifold) until no handles remain.
    pub fn remove_contact(&mut self, contact_id: ContactId, body1: Entity, body2: Entity) {
        while self.pop_manifold(contact_id, body1, body2).is_some() {}
    }

    /// Clears the constraint graph, removing all colors and their contents.
    ///
    /// # Warning
    ///
    /// This does *not* clear the [`ContactGraph`]! You should additionally
    /// call [`ContactGraph::clear`].
    ///
    /// [`ContactGraph`]: crate::collision::contact_types::ContactGraph
    /// [`ContactGraph::clear`]: crate::collision::contact_types::ContactGraph::clear
    pub fn clear(&mut self) {
        for color in &mut self.colors {
            color.body_set.clear();
            color.manifold_handles.clear();
            color.contact_constraints.clear();
        }
        self.contact_handles.clear();
    }
}
