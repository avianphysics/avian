#[cfg(feature = "parallel")]
use core::cell::RefCell;

use crate::{
    collision::contact_types::{ContactEdgeFlags, ContactId},
    data_structures::{bit_vec::BitVec, pair_key::PairKey},
    dynamics::solver::{
        constraint_graph::ConstraintGraph,
        islands::{BodyIslandNode, IslandId, PhysicsIslands, WakeIslands},
        joint_graph::JointGraph,
    },
    prelude::*,
};
use bevy::{
    ecs::{
        entity_disabling::Disabled,
        query::QueryData,
        system::{SystemParam, SystemParamItem, lifetimeless::Read},
    },
    prelude::*,
};
#[cfg(feature = "parallel")]
use thread_local::ThreadLocal;

#[derive(QueryData)]
struct ColliderQuery<C: AnyCollider> {
    entity: Entity,
    of: Option<Read<ColliderOf>>,
    shape: Read<C>,
    aabb: Read<ColliderAabb>,
    position: Read<Position>,
    rotation: Read<Rotation>,
    transform: Option<Read<ColliderTransform>>,
    layers: Read<CollisionLayers>,
    friction: Option<Read<Friction>>,
    restitution: Option<Read<Restitution>>,
    collision_margin: Option<Read<CollisionMargin>>,
    speculative_margin: Option<Read<SpeculativeMargin>>,
    is_sensor: Has<Sensor>,
}

#[derive(QueryData)]
struct RigidBodyQuery {
    entity: Entity,
    rb: Read<RigidBody>,
    position: Read<Position>,
    rotation: Read<Rotation>,
    center_of_mass: Read<ComputedCenterOfMass>,
    linear_velocity: Read<LinearVelocity>,
    angular_velocity: Read<AngularVelocity>,
    // TODO: We should define these as purely collider components and not query for them here.
    friction: Option<Read<Friction>>,
    restitution: Option<Read<Restitution>>,
    collision_margin: Option<Read<CollisionMargin>>,
    speculative_margin: Option<Read<SpeculativeMargin>>,
}

/// A system parameter for managing the narrow phase.
///
/// Responsibilities:
///
/// - Updates each active [`ContactPair`] in the [`ContactGraph`].
/// - Sends [collision events](crate::collision::collision_events) when colliders start or stop touching.
/// - Removes contact pairs from the [`ContactGraph`] when AABBs stop overlapping.
/// - Adds [`ContactManifold`]s to the [`ConstraintGraph`] when they are created.
/// - Removes [`ContactManifold`]s from the [`ConstraintGraph`] when they are destroyed.
#[derive(SystemParam)]
#[expect(missing_docs)]
pub struct NarrowPhase<'w, 's, C: AnyCollider> {
    collider_query: Query<'w, 's, ColliderQuery<C>, Without<ColliderDisabled>>,
    colliding_entities_query: Query<'w, 's, &'static mut CollidingEntities>,
    body_query: Query<'w, 's, RigidBodyQuery, Without<RigidBodyDisabled>>,
    body_islands:
        Query<'w, 's, &'static mut BodyIslandNode, Or<(With<Disabled>, Without<Disabled>)>>,
    pub contact_graph: ResMut<'w, ContactGraph>,
    pub joint_graph: ResMut<'w, JointGraph>,
    pub constraint_graph: ResMut<'w, ConstraintGraph>,
    pub islands: Option<ResMut<'w, PhysicsIslands>>,
    contact_status_bits: ResMut<'w, ContactStatusBits>,
    #[cfg(feature = "parallel")]
    thread_local_contact_status_bits: ResMut<'w, ThreadLocalContactStatusBits>,
    pub config: Res<'w, NarrowPhaseConfig>,
    default_friction: Res<'w, DefaultFriction>,
    default_restitution: Res<'w, DefaultRestitution>,
    length_unit: Res<'w, PhysicsLengthUnit>,
    // These are scaled by the length unit.
    default_speculative_margin: Local<'s, Scalar>,
    contact_tolerance: Local<'s, Scalar>,
}

/// A bit vector for tracking contact status changes.
/// Set bits correspond to contact pairs that were either added or removed.
#[derive(Resource, Default, Deref, DerefMut)]
pub(super) struct ContactStatusBits(pub BitVec);

// TODO: We could just combine all the bit vectors into the first one
//       instead of having a separate `ContactStatusBits` resource.
/// A thread-local bit vector for tracking contact status changes.
/// Set bits correspond to contact pairs that were either added or removed.
///
/// The thread-local bit vectors are combined with the global [`ContactStatusBits`].
#[cfg(feature = "parallel")]
#[derive(Resource, Default, Deref, DerefMut)]
pub(super) struct ThreadLocalContactStatusBits(pub ThreadLocal<RefCell<BitVec>>);

impl<C: AnyCollider> NarrowPhase<'_, '_, C> {
    /// Updates the narrow phase.
    ///
    /// - Updates each active [`ContactPair`] in the [`ContactGraph`].
    /// - Sends [collision events](crate::collision::collision_events) when colliders start or stop touching.
    /// - Removes contact pairs from the [`ContactGraph`] when AABBs stop overlapping.
    /// - Adds [`ContactManifold`]s to the [`ConstraintGraph`] when they are created.
    /// - Removes [`ContactManifold`]s from the [`ConstraintGraph`] when they are destroyed.
    pub fn update<H: CollisionHooks>(
        &mut self,
        collision_started_writer: &mut MessageWriter<CollisionStart>,
        collision_ended_writer: &mut MessageWriter<CollisionEnd>,
        delta_secs: Scalar,
        hooks: &SystemParamItem<H>,
        context: &SystemParamItem<C::Context>,
        commands: &mut ParallelCommands,
    ) where
        for<'w, 's> SystemParamItem<'w, 's, H>: CollisionHooks,
    {
        // Cache default margins scaled by the length unit.
        if self.config.is_changed() {
            *self.default_speculative_margin =
                self.length_unit.0 * self.config.default_speculative_margin;
            *self.contact_tolerance = self.length_unit.0 * self.config.contact_tolerance;
        }

        // Update contacts for all contact pairs.
        self.update_contacts::<H>(delta_secs, hooks, context, commands);

        let mut islands_to_wake: Vec<IslandId> = Vec::with_capacity(128);

        // Process contact status changes, iterating over set bits serially to maintain determinism.
        //
        // Iterating over set bits is done efficiently with the "count trailing zeros" method:
        // https://lemire.me/blog/2018/02/21/iterating-over-set-bits-quickly/
        for (i, mut bits) in self.contact_status_bits.blocks().enumerate() {
            while bits != 0 {
                let trailing_zeros = bits.trailing_zeros();
                let contact_id = ContactId(i as u32 * 64 + trailing_zeros);

                let (contact_edge, contact_pair) = self
                    .contact_graph
                    .get_mut_by_id(contact_id)
                    .unwrap_or_else(|| panic!("Contact pair not found for {contact_id:?}"));

                // Three options:
                // 1. The AABBs are no longer overlapping, and the contact pair should be removed.
                // 2. The colliders started touching, and a collision started event should be sent.
                // 3. The colliders stopped touching, and a collision ended event should be sent.
                if contact_pair.aabbs_disjoint() {
                    // Send a collision ended event if the contact pair was touching.
                    let send_event = contact_edge
                        .flags
                        .contains(ContactEdgeFlags::TOUCHING | ContactEdgeFlags::CONTACT_EVENTS);
                    if send_event {
                        collision_ended_writer.write(CollisionEnd {
                            collider1: contact_pair.collider1,
                            collider2: contact_pair.collider2,
                            body1: contact_pair.body1,
                            body2: contact_pair.body2,
                        });
                    }

                    // Remove from `CollidingEntities`.
                    Self::remove_colliding_entities(
                        &mut self.colliding_entities_query,
                        contact_pair.collider1,
                        contact_pair.collider2,
                    );

                    let pair_key = PairKey::new(
                        contact_pair.collider1.index_u32(),
                        contact_pair.collider2.index_u32(),
                    );

                    if contact_pair.generates_constraints()
                        && let (Some(body1), Some(body2)) = (contact_pair.body1, contact_pair.body2)
                    {
                        let has_island = contact_edge.island.is_some();

                        // Remove the contact pair from the constraint graph.
                        for _ in 0..contact_edge.constraint_handles.len() {
                            self.constraint_graph.pop_manifold(
                                &mut self.contact_graph.edges,
                                contact_id,
                                body1,
                                body2,
                            );
                        }

                        // Unlink the contact pair from its island.
                        if has_island && let Some(islands) = &mut self.islands {
                            islands.remove_contact(
                                contact_id,
                                &mut self.body_islands,
                                &mut self.contact_graph.edges,
                                &self.joint_graph,
                            );
                        }
                    }

                    // Remove the contact edge from the contact graph.
                    self.contact_graph.remove_edge_by_id(&pair_key, contact_id);
                } else if contact_pair.collision_started() {
                    // Send collision started event.
                    if contact_edge.events_enabled() {
                        collision_started_writer.write(CollisionStart {
                            collider1: contact_pair.collider1,
                            collider2: contact_pair.collider2,
                            body1: contact_pair.body1,
                            body2: contact_pair.body2,
                        });
                    }

                    // Add to `CollidingEntities`.
                    Self::add_colliding_entities(
                        &mut self.colliding_entities_query,
                        contact_pair.collider1,
                        contact_pair.collider2,
                    );

                    debug_assert!(
                        !contact_pair.manifolds.is_empty(),
                        "Manifolds should not be empty when colliders start touching"
                    );

                    contact_edge.flags.set(ContactEdgeFlags::TOUCHING, true);
                    contact_pair
                        .flags
                        .set(ContactPairFlags::STARTED_TOUCHING, false);

                    if contact_pair.generates_constraints() {
                        // Add the contact pair to the constraint graph.
                        for _ in contact_pair.manifolds.iter() {
                            self.constraint_graph
                                .push_manifold(contact_edge, contact_pair);
                        }

                        // Link the contact pair to an island.
                        if let Some(islands) = &mut self.islands {
                            let island = islands.add_contact(
                                contact_id,
                                &mut self.body_islands,
                                &mut self.contact_graph,
                                &mut self.joint_graph,
                            );

                            if let Some(island) = island
                                && island.is_sleeping
                            {
                                // Wake up the island if it was previously sleeping.
                                islands_to_wake.push(island.id);
                            }
                        }
                    }
                } else if contact_pair
                    .flags
                    .contains(ContactPairFlags::STOPPED_TOUCHING)
                {
                    // Send collision ended event.
                    if contact_edge.events_enabled() {
                        collision_ended_writer.write(CollisionEnd {
                            collider1: contact_pair.collider1,
                            collider2: contact_pair.collider2,
                            body1: contact_pair.body1,
                            body2: contact_pair.body2,
                        });
                    }

                    // Remove from `CollidingEntities`.
                    Self::remove_colliding_entities(
                        &mut self.colliding_entities_query,
                        contact_pair.collider1,
                        contact_pair.collider2,
                    );

                    debug_assert!(
                        contact_pair.manifolds.is_empty(),
                        "Manifolds should be empty when colliders stopped touching"
                    );

                    contact_edge.flags.set(ContactEdgeFlags::TOUCHING, false);
                    contact_pair
                        .flags
                        .set(ContactPairFlags::STOPPED_TOUCHING, false);

                    // Remove the contact pair from the constraint graph.
                    if contact_pair.generates_constraints()
                        && !contact_edge.constraint_handles.is_empty()
                        && let (Some(body1), Some(body2)) = (contact_pair.body1, contact_pair.body2)
                    {
                        for _ in 0..contact_edge.constraint_handles.len() {
                            self.constraint_graph.pop_manifold(
                                &mut self.contact_graph.edges,
                                contact_id,
                                body1,
                                body2,
                            );
                        }

                        // Unlink the contact pair from its island.
                        if let Some(islands) = &mut self.islands {
                            let island = islands.remove_contact(
                                contact_id,
                                &mut self.body_islands,
                                &mut self.contact_graph.edges,
                                &self.joint_graph,
                            );

                            // TODO: Do we need this?
                            if island.is_sleeping {
                                // Wake up the island if it was previously sleeping.
                                islands_to_wake.push(island.id);
                            }
                        }
                    }
                } else if contact_pair.is_touching()
                    && contact_pair
                        .flags
                        .contains(ContactPairFlags::STARTED_GENERATING_CONSTRAINTS)
                {
                    // The contact pair should start generating constraints.
                    contact_pair
                        .flags
                        .set(ContactPairFlags::STARTED_GENERATING_CONSTRAINTS, false);

                    // Add the contact pair to the constraint graph.
                    for _ in contact_pair.manifolds.iter() {
                        self.constraint_graph
                            .push_manifold(contact_edge, contact_pair);
                    }

                    // Link the contact pair to an island.
                    if let Some(islands) = &mut self.islands {
                        let island = islands.add_contact(
                            contact_id,
                            &mut self.body_islands,
                            &mut self.contact_graph,
                            &mut self.joint_graph,
                        );

                        if let Some(island) = island
                            && island.is_sleeping
                        {
                            // Wake up the island if it was previously sleeping.
                            islands_to_wake.push(island.id);
                        }
                    }
                } else if contact_pair.is_touching()
                    && contact_pair.generates_constraints()
                    && contact_pair.manifold_count_change > 0
                {
                    // The contact pair is still touching, but the manifold count has increased.
                    // Add the new manifolds to the constraint graph.
                    for _ in 0..contact_pair.manifold_count_change {
                        self.constraint_graph
                            .push_manifold(contact_edge, contact_pair);
                    }
                    contact_pair.manifold_count_change = 0;
                } else if contact_pair.is_touching()
                    && contact_pair.generates_constraints()
                    && contact_pair.manifold_count_change < 0
                {
                    // The contact pair is still touching, but the manifold count has decreased.
                    // Remove the excess manifolds from the constraint graph.
                    let removal_count = contact_pair.manifold_count_change.unsigned_abs() as usize;
                    contact_pair.manifold_count_change = 0;

                    if let (Some(body1), Some(body2)) = (contact_pair.body1, contact_pair.body2) {
                        for _ in 0..removal_count {
                            self.constraint_graph.pop_manifold(
                                &mut self.contact_graph.edges,
                                contact_id,
                                body1,
                                body2,
                            );
                        }
                    }
                }

                // Clear the least significant set bit.
                bits &= bits - 1;
            }
        }

        if !islands_to_wake.is_empty() {
            islands_to_wake.sort_unstable();
            islands_to_wake.dedup();

            // Wake up the islands that were previously sleeping.
            commands.command_scope(|mut commands| {
                commands.queue(WakeIslands(islands_to_wake));
            });
        }
    }

    /// Adds the colliding entities to their respective [`CollidingEntities`] components.
    fn add_colliding_entities(
        query: &mut Query<&mut CollidingEntities>,
        entity1: Entity,
        entity2: Entity,
    ) {
        if let Ok(mut colliding_entities1) = query.get_mut(entity1) {
            colliding_entities1.insert(entity2);
        }
        if let Ok(mut colliding_entities2) = query.get_mut(entity2) {
            colliding_entities2.insert(entity1);
        }
    }

    /// Removes the colliding entities from their respective [`CollidingEntities`] components.
    fn remove_colliding_entities(
        query: &mut Query<&mut CollidingEntities>,
        entity1: Entity,
        entity2: Entity,
    ) {
        if let Ok(mut colliding_entities1) = query.get_mut(entity1) {
            colliding_entities1.remove(&entity2);
        }
        if let Ok(mut colliding_entities2) = query.get_mut(entity2) {
            colliding_entities2.remove(&entity1);
        }
    }

    /// Updates contacts for all contact pairs in the [`ContactGraph`].
    ///
    /// Also updates the [`ContactStatusBits`] resource to track status changes for each contact pair.
    /// Set bits correspond to contact pairs that were either added or removed,
    /// while unset bits correspond to contact pairs that were persisted.
    ///
    /// The order of contact pairs is preserved.
    fn update_contacts<H: CollisionHooks>(
        &mut self,
        delta_secs: Scalar,
        hooks: &SystemParamItem<H>,
        collider_context: &SystemParamItem<C::Context>,
        par_commands: &mut ParallelCommands,
    ) where
        for<'w, 's> SystemParamItem<'w, 's, H>: CollisionHooks,
    {
        // Contact bit vecs must be sized based on the full contact capacity,
        // not the number of active contact pairs, because pair indices
        // are unstable and can be invalidated when pairs are removed.
        let bit_count = self.contact_graph.edges.raw_edges().len();

        // Clear the bit vector used to track status changes for each contact pair.
        self.contact_status_bits.set_bit_count_and_clear(bit_count);

        #[cfg(feature = "parallel")]
        self.thread_local_contact_status_bits
            .iter_mut()
            .for_each(|context| {
                let bit_vec_mut = &mut context.borrow_mut();
                bit_vec_mut.set_bit_count_and_clear(bit_count);
            });

        // Compute contacts for all contact pairs in parallel or serially
        // based on the `parallel` feature.
        //
        // Constraints are also generated for each contact pair that is touching
        // or expected to start touching.
        //
        // If parallelism is used, status changes are written to thread-local bit vectors,
        // where set bits correspond to contact pairs that were added or removed.
        // At the end, the bit vectors are combined using bit-wise OR.
        //
        // If parallelism is not used, status changes are instead written
        // directly to the global bit vector.
        //
        // TODO: An alternative to thread-local bit vectors could be to have one larger bit vector
        //       and to chunk it into smaller bit vectors for each thread. Might not be any faster though.
        crate::utils::par_for_each(self.contact_graph.active_pairs_mut(), 64, |_i, contacts| {
            let contact_id = contacts.contact_id.0 as usize;

            #[cfg(not(feature = "parallel"))]
            let status_change_bits = &mut self.contact_status_bits;

            // TODO: Move this out of the chunk iteration? Requires refactoring `par_for_each!`.
            #[cfg(feature = "parallel")]
            // Get the thread-local narrow phase context.
            let mut thread_context = self
                .thread_local_contact_status_bits
                .get_or(|| {
                    // No thread-local bit vector exists for this thread yet.
                    // Create a new one with the same capacity as the global bit vector.
                    let mut contact_status_bits = BitVec::new(bit_count);
                    contact_status_bits.set_bit_count_and_clear(bit_count);
                    RefCell::new(contact_status_bits)
                })
                .borrow_mut();
            #[cfg(feature = "parallel")]
            let status_change_bits = &mut *thread_context;

            // Get the colliders for the contact pair.
            let Ok([collider1, collider2]) = self
                .collider_query
                .get_many([contacts.collider1, contacts.collider2])
            else {
                return;
            };

            // Check if the AABBs of the colliders still overlap and the contact pair is valid.
            let overlap = collider1.aabb.intersects(collider2.aabb);

            // Also check if the collision layers are still compatible and the contact pair is valid.
            // TODO: Ideally, we would have fine-grained change detection for `CollisionLayers`
            //       rather than checking it for every pair here.
            if !overlap || !collider1.layers.interacts_with(*collider2.layers) {
                // The AABBs no longer overlap. The contact pair should be removed.
                contacts.flags.set(ContactPairFlags::DISJOINT_AABB, true);
                status_change_bits.set(contact_id);
            } else {
                // The AABBs overlap. Compute contacts.

                let body1_bundle = collider1
                    .of
                    .and_then(|&ColliderOf { body }| self.body_query.get(body).ok());
                let body2_bundle = collider2
                    .of
                    .and_then(|&ColliderOf { body }| self.body_query.get(body).ok());

                // The rigid body's friction, restitution, collision margin, and speculative margin
                // will be used if the collider doesn't have them specified.
                let (
                    is_static1,
                    collider_offset1,
                    world_com1,
                    mut lin_vel1,
                    ang_vel1,
                    rb_friction1,
                    rb_collision_margin1,
                    rb_speculative_margin1,
                ) = body1_bundle
                    .as_ref()
                    .map(|body| {
                        (
                            body.rb.is_static(),
                            collider1.position.0 - body.position.0,
                            body.rotation * body.center_of_mass.0,
                            body.linear_velocity.0,
                            body.angular_velocity.0,
                            body.friction,
                            body.collision_margin,
                            body.speculative_margin,
                        )
                    })
                    .unwrap_or_default();
                let (
                    is_static2,
                    collider_offset2,
                    world_com2,
                    mut lin_vel2,
                    ang_vel2,
                    rb_friction2,
                    rb_collision_margin2,
                    rb_speculative_margin2,
                ) = body2_bundle
                    .as_ref()
                    .map(|body| {
                        (
                            body.rb.is_static(),
                            collider2.position.0 - body.position.0,
                            body.rotation * body.center_of_mass.0,
                            body.linear_velocity.0,
                            body.angular_velocity.0,
                            body.friction,
                            body.collision_margin,
                            body.speculative_margin,
                        )
                    })
                    .unwrap_or_default();

                // Store these to avoid having to query for the bodies
                // when processing status changes for the constraint graph.
                contacts.flags.set(ContactPairFlags::STATIC1, is_static1);
                contacts.flags.set(ContactPairFlags::STATIC2, is_static2);

                // If either collider is a sensor or either body is disabled, no constraints should be generated.
                let is_disabled = body1_bundle.is_none()
                    || body2_bundle.is_none()
                    || collider1.is_sensor
                    || collider2.is_sensor;

                if !is_disabled && !contacts.generates_constraints() {
                    // This can happen when a sensor is removed, or when a collider is attached to a body.
                    contacts
                        .flags
                        .set(ContactPairFlags::STARTED_GENERATING_CONSTRAINTS, true);
                    status_change_bits.set(contact_id);
                }

                contacts
                    .flags
                    .set(ContactPairFlags::GENERATE_CONSTRAINTS, !is_disabled);

                // Get combined friction and restitution coefficients of the colliders
                // or the bodies they are attached to. Fall back to the global defaults.
                let friction = collider1
                    .friction
                    .or(rb_friction1)
                    .copied()
                    .unwrap_or(self.default_friction.0)
                    .combine(
                        collider2
                            .friction
                            .or(rb_friction2)
                            .copied()
                            .unwrap_or(self.default_friction.0),
                    )
                    .dynamic_coefficient;
                let restitution = collider1
                    .restitution
                    .copied()
                    .unwrap_or(self.default_restitution.0)
                    .combine(
                        collider2
                            .restitution
                            .copied()
                            .unwrap_or(self.default_restitution.0),
                    )
                    .coefficient;

                // Use the collider's own collision margin if specified, and fall back to the body's
                // collision margin.
                //
                // The collision margin adds artificial thickness to colliders for performance
                // and stability. See the `CollisionMargin` documentation for more details.
                let collision_margin1 = collider1
                    .collision_margin
                    .or(rb_collision_margin1)
                    .map_or(0.0, |margin| margin.0);
                let collision_margin2 = collider2
                    .collision_margin
                    .or(rb_collision_margin2)
                    .map_or(0.0, |margin| margin.0);
                let collision_margin_sum = collision_margin1 + collision_margin2;

                // Use the collider's own speculative margin if specified, and fall back to the body's
                // speculative margin.
                //
                // The speculative margin is used to predict contacts that might happen during the frame.
                // This is used for speculative collision. See the CCD and `SpeculativeMargin` documentation
                // for more details.
                let speculative_margin1 = collider1
                    .speculative_margin
                    .map_or(rb_speculative_margin1.map(|margin| margin.0), |margin| {
                        Some(margin.0)
                    });
                let speculative_margin2 = collider2
                    .speculative_margin
                    .map_or(rb_speculative_margin2.map(|margin| margin.0), |margin| {
                        Some(margin.0)
                    });

                let relative_linear_velocity: Vector;

                // Compute the effective speculative margin, clamping it based on velocities and the maximum bound.
                let effective_speculative_margin = {
                    let speculative_margin1 =
                        speculative_margin1.unwrap_or(*self.default_speculative_margin);
                    let speculative_margin2 =
                        speculative_margin2.unwrap_or(*self.default_speculative_margin);
                    let inv_delta_secs = delta_secs.recip();

                    // Clamp velocities to the maximum speculative margins.
                    if speculative_margin1 < Scalar::MAX {
                        lin_vel1 = lin_vel1.clamp_length_max(speculative_margin1 * inv_delta_secs);
                    }
                    if speculative_margin2 < Scalar::MAX {
                        lin_vel2 = lin_vel2.clamp_length_max(speculative_margin2 * inv_delta_secs);
                    }

                    // Compute the effective margin based on how much the bodies
                    // are expected to move relative to each other.
                    relative_linear_velocity = lin_vel2 - lin_vel1;
                    delta_secs * relative_linear_velocity.length()
                };

                // The maximum distance at which contacts are detected.
                // At least as large as the contact tolerance.
                let max_contact_distance = effective_speculative_margin
                    .max(*self.contact_tolerance)
                    + collision_margin_sum;

                let was_touching = contacts.flags.contains(ContactPairFlags::TOUCHING);

                // Save the old manifolds for warm starting.
                let old_manifolds = contacts.manifolds.clone();

                // TODO: It'd be good to persist the manifolds and let Parry match contacts.
                //       This isn't currently done because it requires using Parry's contact manifold type.
                // Compute the contact manifolds using the effective speculative margin.
                let context = ContactManifoldContext::new(
                    collider1.entity,
                    collider2.entity,
                    collider_context,
                );
                collider1.shape.contact_manifolds_with_context(
                    collider2.shape,
                    collider1.position.0,
                    *collider1.rotation,
                    collider2.position.0,
                    *collider2.rotation,
                    max_contact_distance,
                    &mut contacts.manifolds,
                    context,
                );

                // Transform and prune contact data.
                contacts.manifolds.retain_mut(|manifold| {
                    // Set the initial surface properties.
                    manifold.friction = friction;
                    manifold.restitution = restitution;
                    #[cfg(feature = "2d")]
                    {
                        manifold.tangent_speed = 0.0;
                    }
                    #[cfg(feature = "3d")]
                    {
                        manifold.tangent_velocity = Vector::ZERO;
                    }

                    let normal = manifold.normal;

                    // Transform and prune contact points.
                    manifold.retain_points_mut(|point| {
                        // Transform contact points to be relative to the centers of mass of the bodies.
                        point.anchor1 = point.anchor1 + collider_offset1 - world_com1;
                        point.anchor2 = point.anchor2 + collider_offset2 - world_com2;

                        // Add the collision margin to the penetration depth.
                        point.penetration += collision_margin_sum;

                        // Compute the relative velocity along the contact normal.
                        #[cfg(feature = "2d")]
                        let relative_velocity = relative_linear_velocity
                            + ang_vel2 * point.anchor2.perp()
                            - ang_vel1 * point.anchor1.perp();
                        #[cfg(feature = "3d")]
                        let relative_velocity = relative_linear_velocity
                            + ang_vel2.cross(point.anchor2)
                            - ang_vel1.cross(point.anchor1);
                        let normal_speed = relative_velocity.dot(normal);
                        point.normal_speed = normal_speed;

                        // Keep the contact if (1) the separation distance is below the required threshold,
                        // or if (2) the bodies are expected to come into contact within the next time step.
                        -point.penetration < effective_speculative_margin || {
                            let delta_distance = normal_speed * delta_secs;
                            delta_distance - point.penetration < effective_speculative_margin
                        }
                    });

                    // Prune extra contact points.
                    #[cfg(feature = "3d")]
                    if manifold.points.len() > 4 {
                        manifold.prune_points();
                    }

                    !manifold.points.is_empty()
                });

                // Check if the colliders are now touching.
                let mut touching = !contacts.manifolds.is_empty();

                if touching && contacts.flags.contains(ContactPairFlags::MODIFY_CONTACTS) {
                    par_commands.command_scope(|mut commands| {
                        touching = hooks.modify_contacts(contacts, &mut commands);
                    });
                    if !touching {
                        contacts.manifolds.clear();
                    }
                }

                contacts.flags.set(ContactPairFlags::TOUCHING, touching);

                // TODO: Manifold reduction

                // TODO: This condition is pretty arbitrary, mainly to skip dense trimeshes.
                //       If we let Parry handle contact matching, this wouldn't be needed.
                if contacts.manifolds.len() <= 4 && self.config.match_contacts {
                    // TODO: Cache this?
                    let distance_threshold = 0.1 * self.length_unit.0;

                    for manifold in contacts.manifolds.iter_mut() {
                        for previous_manifold in old_manifolds.iter() {
                            manifold.match_contacts(&previous_manifold.points, distance_threshold);
                        }
                    }
                }

                // Record how many manifolds were added or removed.
                // This is used to add or remove contact constraints in the `ConstraintGraph` for persistent contacts.
                contacts.manifold_count_change =
                    (contacts.manifolds.len() as i32 - old_manifolds.len() as i32) as i16;

                // TODO: For unmatched contacts, apply any leftover impulses from the previous frame.

                if touching && !was_touching {
                    // The colliders started touching.
                    contacts.flags.set(ContactPairFlags::STARTED_TOUCHING, true);
                    status_change_bits.set(contact_id);
                } else if !touching && was_touching {
                    // The colliders stopped touching.
                    contacts.flags.set(ContactPairFlags::STOPPED_TOUCHING, true);
                    status_change_bits.set(contact_id);
                } else if contacts.manifold_count_change != 0 {
                    // The manifold count changed, but the colliders are still touching.
                    // This is used to add or remove contact constraints in the `ConstraintGraph`.
                    status_change_bits.set(contact_id);
                }
            };
        });

        #[cfg(feature = "parallel")]
        {
            // Combine the thread-local bit vectors serially using bit-wise OR.
            self.thread_local_contact_status_bits
                .iter_mut()
                .for_each(|context| {
                    let contact_status_bits = context.borrow();
                    self.contact_status_bits.or(&contact_status_bits);
                });
        }
    }
}
