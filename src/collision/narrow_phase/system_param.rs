#[cfg(feature = "parallel")]
use core::cell::RefCell;
use core::cmp::Ordering;

use crate::{
    collision::collider::ColliderQuery,
    data_structures::{bit_vec::BitVec, graph::EdgeIndex},
    dynamics::solver::{contact::ContactConstraint, ContactSoftnessCoefficients},
    prelude::*,
};
use bevy::{
    ecs::system::{SystemParam, SystemParamItem},
    prelude::*,
};
use dynamics::solver::{
    contact::{ContactConstraintPoint, ContactNormalPart, ContactTangentPart},
    ContactConstraints,
};
#[cfg(feature = "parallel")]
use thread_local::ThreadLocal;

/// A system parameter for managing the narrow phase.
///
/// Responsibilities:
///
/// - Updates contacts for each contact pair in the [`ContactGraph`].
/// - Sends collision events when colliders start or stop touching.
/// - Removes contact pairs from the [`ContactGraph`] when AABBs stop overlapping.
/// - Generates contact constraints for each contact pair that is touching or expected to start touching.
#[derive(SystemParam)]
#[expect(missing_docs)]
pub struct NarrowPhase<'w, 's, C: AnyCollider> {
    pub collider_query: Query<'w, 's, ColliderQuery<C>, Without<ColliderDisabled>>,
    pub colliding_entities_query: Query<'w, 's, &'static mut CollidingEntities>,
    pub body_query: Query<
        'w,
        's,
        (
            RigidBodyQueryReadOnly,
            Option<&'static CollisionMargin>,
            Option<&'static SpeculativeMargin>,
        ),
        Without<RigidBodyDisabled>,
    >,
    pub contact_graph: ResMut<'w, ContactGraph>,
    contact_status_bits: ResMut<'w, ContactStatusBits>,
    pub contact_constraints: ResMut<'w, ContactConstraints>,
    #[cfg(feature = "parallel")]
    thread_locals: ResMut<'w, NarrowPhaseThreadLocals>,
    pub config: Res<'w, NarrowPhaseConfig>,
    default_friction: Res<'w, DefaultFriction>,
    default_restitution: Res<'w, DefaultRestitution>,
    contact_softness: Res<'w, ContactSoftnessCoefficients>,
    length_unit: Res<'w, PhysicsLengthUnit>,
    // These are scaled by the length unit.
    default_speculative_margin: Local<'s, Scalar>,
    contact_tolerance: Local<'s, Scalar>,
}

/// A bit vector for tracking contact status changes.
/// Set bits correspond to contact pairs that were either added or removed.
#[derive(Resource, Default, Deref, DerefMut)]
pub(super) struct ContactStatusBits(pub BitVec);

/// A resource storing thread-local context for the narrow phase.
#[cfg(feature = "parallel")]
#[derive(Resource, Default, Deref, DerefMut)]
pub(super) struct NarrowPhaseThreadLocals(pub ThreadLocal<RefCell<NarrowPhaseThreadContext>>);

/// A thread-local context for the narrow phase.
#[cfg(feature = "parallel")]
#[derive(Default)]
pub(super) struct NarrowPhaseThreadContext {
    /// A bit vector for tracking contact status changes.
    /// Set bits correspond to contact pairs that were either added or removed.
    ///
    /// The thread-local bit vectors are combined with the global [`ContactStatusBits`].
    pub contact_status_bits: BitVec,
    /// A vector for storing generated contact constraints.
    ///
    /// The thread-local constraints are combined with the global [`ContactConstraints`].
    pub contact_constraints: Vec<ContactConstraint>,
}

impl<C: AnyCollider> NarrowPhase<'_, '_, C> {
    /// Updates the narrow phase.
    ///
    /// - Updates contacts for each contact pair in the [`ContactGraph`].
    /// - Sends collision events when colliders start or stop touching.
    /// - Removes pairs from the [`ContactGraph`] when AABBs stop overlapping.
    pub fn update<H: CollisionHooks>(
        &mut self,
        collision_started_event_writer: &mut EventWriter<CollisionStarted>,
        collision_ended_event_writer: &mut EventWriter<CollisionEnded>,
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

        // Contact pairs that should be removed.
        // TODO: This is needed because removing pairs while iterating over the bit vec can invalidate indices.
        //       With a stable mapping between contact pair indices and bits, we could remove this.
        // TODO: Pre-allocate this with some reasonable capacity?
        let mut pairs_to_remove = Vec::<(Entity, Entity)>::new();

        // Process contact status changes, iterating over set bits serially to maintain determinism.
        //
        // Iterating over set bits is done efficiently with the "count trailing zeros" method:
        // https://lemire.me/blog/2018/02/21/iterating-over-set-bits-quickly/
        for (i, mut bits) in self.contact_status_bits.blocks().enumerate() {
            while bits != 0 {
                let trailing_zeros = bits.trailing_zeros();
                let pair_index = EdgeIndex(i as u32 * 64 + trailing_zeros);

                let contact_pair = self
                    .contact_graph
                    .internal
                    .edge_weight_mut(pair_index)
                    .unwrap_or_else(|| panic!("Contact pair not found for {pair_index:?}"));

                // Three options:
                // 1. The AABBs are no longer overlapping, and the contact pair should be removed.
                // 2. The colliders started touching, and a collision started event should be sent.
                // 3. The colliders stopped touching, and a collision ended event should be sent.
                if contact_pair.aabbs_disjoint() {
                    // Send a collision ended event if the contact pair was touching.
                    let send_event = contact_pair
                        .flags
                        .contains(ContactPairFlags::TOUCHING | ContactPairFlags::CONTACT_EVENTS);
                    if send_event {
                        collision_ended_event_writer.write(CollisionEnded(
                            contact_pair.collider1,
                            contact_pair.collider2,
                        ));
                    }

                    // Remove from `CollidingEntities`.
                    Self::remove_colliding_entities(
                        &mut self.colliding_entities_query,
                        contact_pair.collider1,
                        contact_pair.collider2,
                    );

                    // Wake up the bodies.
                    // TODO: When we have simulation islands, this will be more efficient.
                    commands.command_scope(|mut commands| {
                        commands.queue(WakeUpBody(
                            contact_pair.body1.unwrap_or(contact_pair.collider1),
                        ));
                        commands.queue(WakeUpBody(
                            contact_pair.body2.unwrap_or(contact_pair.collider2),
                        ));
                    });

                    // Queue the contact pair for removal.
                    pairs_to_remove.push((contact_pair.collider1, contact_pair.collider2));
                } else if contact_pair.collision_started() {
                    // Send collision started event.
                    if contact_pair.events_enabled() {
                        collision_started_event_writer.write(CollisionStarted(
                            contact_pair.collider1,
                            contact_pair.collider2,
                        ));
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

                    contact_pair
                        .flags
                        .set(ContactPairFlags::STARTED_TOUCHING, false);
                } else if contact_pair
                    .flags
                    .contains(ContactPairFlags::STOPPED_TOUCHING)
                {
                    // Send collision ended event.
                    if contact_pair.events_enabled() {
                        collision_ended_event_writer.write(CollisionEnded(
                            contact_pair.collider1,
                            contact_pair.collider2,
                        ));
                    }

                    // Remove from `CollidingEntities`.
                    Self::remove_colliding_entities(
                        &mut self.colliding_entities_query,
                        contact_pair.collider1,
                        contact_pair.collider2,
                    );

                    // Wake up the bodies.
                    // TODO: When we have simulation islands, this will be more efficient.
                    commands.command_scope(|mut commands| {
                        commands.queue(WakeUpBody(
                            contact_pair.body1.unwrap_or(contact_pair.collider1),
                        ));
                        commands.queue(WakeUpBody(
                            contact_pair.body2.unwrap_or(contact_pair.collider2),
                        ));
                    });

                    debug_assert!(
                        contact_pair.manifolds.is_empty(),
                        "Manifolds should be empty when colliders stopped touching"
                    );

                    contact_pair
                        .flags
                        .set(ContactPairFlags::STOPPED_TOUCHING, false);
                }

                // Clear the least significant set bit.
                bits &= bits - 1;
            }
        }

        // Remove the contact pairs that were marked for removal.
        for pair_index in pairs_to_remove.drain(..) {
            self.contact_graph.remove_pair(pair_index.0, pair_index.1);
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
        let contact_pair_count = self.contact_graph.internal.edge_count();

        // Clear the bit vector used to track status changes for each contact pair.
        self.contact_status_bits
            .set_bit_count_and_clear(contact_pair_count);

        #[cfg(feature = "parallel")]
        self.thread_locals.iter_mut().for_each(|context| {
            let bit_vec_mut = &mut context.borrow_mut().contact_status_bits;
            bit_vec_mut.set_bit_count_and_clear(contact_pair_count);
        });

        // Clear the contact constraints.
        self.contact_constraints.clear();

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
        crate::utils::par_for_each!(
            self.contact_graph.internal.raw_edges_mut(),
            |contact_index, contacts| {
                #[cfg(not(feature = "parallel"))]
                let status_change_bits = &mut self.contact_status_bits;
                #[cfg(not(feature = "parallel"))]
                let constraints = &mut self.contact_constraints;

                // TODO: Move this out of the chunk iteration? Requires refactoring `par_for_each!`.
                #[cfg(feature = "parallel")]
                // Get the thread-local narrow phase context.
                let mut thread_context = self
                    .thread_locals
                    .get_or(|| {
                        // No thread-local bit vector exists for this thread yet.
                        // Create a new one with the same capacity as the global bit vector.
                        let mut contact_status_bits = BitVec::new(contact_pair_count);
                        contact_status_bits.set_bit_count_and_clear(contact_pair_count);
                        RefCell::new(NarrowPhaseThreadContext {
                            contact_status_bits,
                            contact_constraints: Vec::new(),
                        })
                    })
                    .borrow_mut();
                #[cfg(feature = "parallel")]
                let NarrowPhaseThreadContext {
                    contact_status_bits: status_change_bits,
                    contact_constraints: constraints,
                } = &mut *thread_context;

                let contacts = &mut contacts.weight;

                // Get the colliders for the contact pair.
                let Ok([collider1, collider2]) = self
                    .collider_query
                    .get_many([contacts.collider1, contacts.collider2])
                else {
                    return;
                };

                // Check if the AABBs of the colliders still overlap and the contact pair is valid.
                let overlap = collider1.aabb.intersects(&collider2.aabb);

                // Also check if the collision layers are still compatible and the contact pair is valid.
                // TODO: Ideally, we would have fine-grained change detection for `CollisionLayers`
                //       rather than checking it for every pair here.
                if !overlap || !collider1.layers.interacts_with(*collider2.layers) {
                    // The AABBs no longer overlap. The contact pair should be removed.
                    contacts.flags.set(ContactPairFlags::DISJOINT_AABB, true);
                    status_change_bits.set(contact_index);
                } else {
                    // The AABBs overlap. Compute contacts.

                    let body1_bundle = collider1
                        .body()
                        .and_then(|body| self.body_query.get(body).ok());
                    let body2_bundle = collider2
                        .body()
                        .and_then(|body| self.body_query.get(body).ok());

                    // The rigid body's friction, restitution, collision margin, and speculative margin
                    // will be used if the collider doesn't have them specified.
                    let (mut lin_vel1, rb_friction1, rb_collision_margin1, rb_speculative_margin1) =
                        body1_bundle
                            .as_ref()
                            .map(|(body, collision_margin, speculative_margin)| {
                                (
                                    body.linear_velocity.0,
                                    body.friction,
                                    *collision_margin,
                                    *speculative_margin,
                                )
                            })
                            .unwrap_or_default();
                    let (mut lin_vel2, rb_friction2, rb_collision_margin2, rb_speculative_margin2) =
                        body2_bundle
                            .as_ref()
                            .map(|(body, collision_margin, speculative_margin)| {
                                (
                                    body.linear_velocity.0,
                                    body.friction,
                                    *collision_margin,
                                    *speculative_margin,
                                )
                            })
                            .unwrap_or_default();

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
                            lin_vel1 =
                                lin_vel1.clamp_length_max(speculative_margin1 * inv_delta_secs);
                        }
                        if speculative_margin2 < Scalar::MAX {
                            lin_vel2 =
                                lin_vel2.clamp_length_max(speculative_margin2 * inv_delta_secs);
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

                    // Set the initial surface properties.
                    // TODO: This could be done in `contact_manifolds` to avoid the extra iteration.
                    contacts.manifolds.iter_mut().for_each(|manifold| {
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

                    // TODO: This condition is pretty arbitrary, mainly to skip dense trimeshes.
                    //       If we let Parry handle contact matching, this wouldn't be needed.
                    if contacts.manifolds.len() <= 4 && self.config.match_contacts {
                        // TODO: Cache this?
                        let distance_threshold = 0.1 * self.length_unit.0;

                        for manifold in contacts.manifolds.iter_mut() {
                            for previous_manifold in old_manifolds.iter() {
                                manifold
                                    .match_contacts(&previous_manifold.points, distance_threshold);
                            }
                        }
                    }

                    // TODO: For unmatched contacts, apply any leftover impulses from the previous frame.

                    if touching && !was_touching {
                        // The colliders started touching.
                        contacts.flags.set(ContactPairFlags::STARTED_TOUCHING, true);
                        status_change_bits.set(contact_index);
                    } else if !touching && was_touching {
                        // The colliders stopped touching.
                        contacts.flags.set(ContactPairFlags::STOPPED_TOUCHING, true);
                        status_change_bits.set(contact_index);
                    }

                    // Now, we generate contact constraints for the contact pair.
                    let Some(((body1, _, _), (body2, _, _))) = body1_bundle.zip(body2_bundle)
                    else {
                        return;
                    };

                    let inactive1 = body1.rb.is_static() || body1.is_sleeping;
                    let inactive2 = body2.rb.is_static() || body2.is_sleeping;

                    // No collision response if both bodies are static or sleeping
                    // or if either of the colliders is a sensor collider.
                    if (inactive1 && inactive2) || contacts.is_sensor() {
                        return;
                    }

                    // When an active body collides with a sleeping body, wake up the sleeping body.
                    par_commands.command_scope(|mut commands| {
                        if body1.is_sleeping {
                            commands.queue(WakeUpBody(body1.entity));
                        } else if body2.is_sleeping {
                            commands.queue(WakeUpBody(body2.entity));
                        }
                    });

                    // Compute the relative dominance of the bodies.
                    let relative_dominance = body1.dominance() - body2.dominance();

                    // Compute the inverse mass and angular inertia, taking into account the relative dominance.
                    // TODO: How should we properly take locked axes into account for mass?
                    let (inv_mass1, i1, inv_mass2, i2) = match relative_dominance.cmp(&0) {
                        Ordering::Equal => (
                            body1.mass().inverse(),
                            body1.effective_global_angular_inertia(),
                            body2.mass().inverse(),
                            body2.effective_global_angular_inertia(),
                        ),
                        Ordering::Greater => (
                            0.0,
                            ComputedAngularInertia::INFINITY,
                            body2.mass().inverse(),
                            body2.effective_global_angular_inertia(),
                        ),
                        Ordering::Less => (
                            body1.mass().inverse(),
                            body1.effective_global_angular_inertia(),
                            0.0,
                            ComputedAngularInertia::INFINITY,
                        ),
                    };

                    let contact_softness = if inactive1 || inactive2 {
                        self.contact_softness.non_dynamic
                    } else {
                        self.contact_softness.dynamic
                    };

                    let inverse_mass_sum = inv_mass1 + inv_mass2;

                    // Generate a contact constraint for each contact manifold.
                    for (manifold_index, manifold) in contacts.manifolds.iter_mut().enumerate() {
                        let mut constraint = ContactConstraint {
                            body1: body1.entity,
                            body2: body2.entity,
                            collider1: collider1.entity,
                            collider2: collider2.entity,
                            relative_dominance,
                            friction: manifold.friction,
                            restitution: manifold.restitution,
                            #[cfg(feature = "2d")]
                            tangent_speed: manifold.tangent_speed,
                            #[cfg(feature = "3d")]
                            tangent_velocity: manifold.tangent_velocity,
                            normal: manifold.normal,
                            points: Vec::with_capacity(manifold.points.len()),
                            #[cfg(feature = "parallel")]
                            pair_index: contact_index,
                            manifold_index,
                        };

                        let tangents = constraint
                            .tangent_directions(body1.linear_velocity.0, body2.linear_velocity.0);

                        for mut contact in manifold.points.iter().copied() {
                            // Transform contact points from collider-space to body-space.
                            if let Some(transform) = collider1.transform.copied() {
                                contact.local_point1 = transform.rotation * contact.local_point1
                                    + transform.translation;
                            }
                            if let Some(transform) = collider2.transform.copied() {
                                contact.local_point2 = transform.rotation * contact.local_point2
                                    + transform.translation;
                            }

                            contact.penetration += collision_margin_sum;

                            let effective_distance = -contact.penetration;

                            let local_anchor1 = contact.local_point1 - body1.center_of_mass.0;
                            let local_anchor2 = contact.local_point2 - body2.center_of_mass.0;

                            // Store fixed world-space anchors.
                            // This improves rolling behavior for shapes like balls and capsules.
                            let r1 = *body1.rotation * local_anchor1;
                            let r2 = *body2.rotation * local_anchor2;

                            // Relative velocity at the contact point.
                            // body2.velocity_at_point(r2) - body1.velocity_at_point(r1)
                            #[cfg(feature = "2d")]
                            let relative_velocity = relative_linear_velocity
                                + body2.angular_velocity.0 * r2.perp()
                                - body1.angular_velocity.0 * r1.perp();
                            #[cfg(feature = "3d")]
                            let relative_velocity = relative_linear_velocity
                                + body2.angular_velocity.0.cross(r2)
                                - body1.angular_velocity.0.cross(r1);

                            // Keep the contact if (1) the separation distance is below the required threshold,
                            // or if (2) the bodies are expected to come into contact within the next frame.
                            let normal_speed = relative_velocity.dot(constraint.normal);
                            let keep_contact = effective_distance < effective_speculative_margin
                                || {
                                    let delta_distance = normal_speed * delta_secs;
                                    effective_distance + delta_distance
                                        < effective_speculative_margin
                                };

                            if !keep_contact {
                                continue;
                            }

                            let point = ContactConstraintPoint {
                                // TODO: Apply warm starting scale here instead of in `warm_start`?
                                normal_part: ContactNormalPart::generate(
                                    inverse_mass_sum,
                                    i1,
                                    i2,
                                    r1,
                                    r2,
                                    constraint.normal,
                                    self.config.match_contacts.then_some(contact.normal_impulse),
                                    contact_softness,
                                ),
                                // There should only be a friction part if the coefficient of friction is non-negative.
                                tangent_part: (friction > 0.0).then_some(
                                    ContactTangentPart::generate(
                                        inverse_mass_sum,
                                        i1,
                                        i2,
                                        r1,
                                        r2,
                                        tangents,
                                        self.config
                                            .match_contacts
                                            .then_some(contact.tangent_impulse),
                                    ),
                                ),
                                max_normal_impulse: 0.0,
                                anchor1: r1,
                                anchor2: r2,
                                normal_speed,
                                initial_separation: -contact.penetration
                                    - (r2 - r1).dot(constraint.normal),
                            };

                            constraint.points.push(point);
                        }

                        if !constraint.points.is_empty() {
                            constraints.push(constraint);
                        }
                    }
                };
            }
        );

        #[cfg(feature = "parallel")]
        {
            // Combine the thread-local bit vectors serially using bit-wise OR,
            // and drain the thread-local contact constraints into the global contact constraints.
            self.thread_locals.iter_mut().for_each(|context| {
                let mut context_mut = context.borrow_mut();
                self.contact_status_bits
                    .or(&context_mut.contact_status_bits);
                self.contact_constraints
                    .extend(context_mut.contact_constraints.drain(..));
            });

            // Sort the contact constraints by pair index to maintain determinism.
            // NOTE: `sort_by_key` is faster than `sort_unstable_by_key` here,
            //       because the constraints within chunks are already sorted.
            // TODO: We should figure out an approach that doesn't require sorting.
            self.contact_constraints
                .sort_by_key(|constraint| constraint.pair_index);
        }
    }
}
