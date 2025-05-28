//! Finds pairs of entities with overlapping [`ColliderAabb`]s to reduce
//! the number of potential contacts for the [narrow phase](crate::narrow_phase).
//!
//! See [`BroadPhasePlugin`].

mod tree;
use obvhs::{aabb::Aabb, heapstack::HeapStack};
pub use tree::*;

use core::marker::PhantomData;

use crate::{
    data_structures::pair_key::PairKey, dynamics::solver::solver_body::SolverBody, prelude::*,
};
use bevy::{ecs::system::SystemParamItem, prelude::*, utils::Parallel};

use super::CollisionDiagnostics;

/// Finds pairs of entities with overlapping [`ColliderAabb`]s to reduce
/// the number of potential contacts for the [narrow phase](crate::narrow_phase).
///
/// A contact pair is created in the [`Collisions`] resource for each pair found.
/// Removing and updating these pairs is left to the [narrow phase](crate::narrow_phase).
///
/// The broad phase systems run in [`PhysicsStepSet::BroadPhase`].
///
/// [`CollisionHooks`] can be provided with generics to apply custom filtering for collision pairs.
pub struct BroadPhasePlugin<H: CollisionHooks = ()>(PhantomData<H>);

impl<H: CollisionHooks> Default for BroadPhasePlugin<H> {
    fn default() -> Self {
        Self(PhantomData)
    }
}

impl<H: CollisionHooks + 'static> Plugin for BroadPhasePlugin<H>
where
    for<'w, 's> SystemParamItem<'w, 's, H>: CollisionHooks,
{
    fn build(&self, app: &mut App) {
        app.add_plugins(ColliderTreePlugin::<Collider>::default());

        app.configure_sets(
            PhysicsSchedule,
            (
                BroadPhaseSet::First,
                BroadPhaseSet::UpdateStructures,
                BroadPhaseSet::CollectCollisions,
                BroadPhaseSet::Last,
            )
                .chain()
                .in_set(PhysicsStepSet::BroadPhase),
        );

        let physics_schedule = app
            .get_schedule_mut(PhysicsSchedule)
            .expect("add PhysicsSchedule first");

        physics_schedule.add_systems(find_collision_pairs.in_set(BroadPhaseSet::CollectCollisions));
    }

    fn finish(&self, app: &mut App) {
        // Register timer and counter diagnostics for collision detection.
        app.register_physics_diagnostics::<CollisionDiagnostics>();
    }
}

/// System sets for systems running in [`PhysicsStepSet::BroadPhase`].
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum BroadPhaseSet {
    /// Runs at the start of the broad phase. Empty by default.
    First,
    /// Updates acceleration structures and other data needed for broad phase collision detection.
    UpdateStructures,
    /// Finds pairs of entities with overlapping [`ColliderAabb`]s
    /// and creates contact pairs for them in [`Collisions`].
    CollectCollisions,
    /// Runs at the end of the broad phase. Empty by default.
    Last,
}

fn find_collision_pairs(
    tree: ResMut<ColliderTrees>,
    body_query: Query<
        (
            Entity,
            &BroadPhaseProxyIndex,
            &ColliderAabb,
            Option<&CollisionLayers>,
        ),
        With<SolverBody>,
    >,
    aabb_query: Query<&ColliderAabb>,
    collider_of_query: Query<&ColliderOf>,
    mut collisions: ResMut<ContactGraph>,
    mut diagnostics: ResMut<CollisionDiagnostics>,
) {
    let start = crate::utils::Instant::now();

    let mut broad_collision_pairs = Vec::new();

    let mut parallel = Parallel::<Vec<(Entity, Entity)>>::default();

    // Perform tree queries for all moving proxies.

    body_query.par_iter().for_each_init(
        || parallel.borrow_local_mut(),
        |pairs, (entity, &BroadPhaseProxyIndex(query_proxy_index), aabb, layers)| {
            let layers = layers.map_or(CollisionLayers::default(), |layers| *layers);

            // Find potential collisions against moving proxies.
            let mut stack = HeapStack::new_with_capacity(256);
            tree.dynamic_tree.bvh.aabb_traverse(
                &mut stack,
                Aabb::from(*aabb),
                |bvh, node_index| {
                    let node = bvh.nodes[node_index as usize];
                    let start = node.first_index as usize;
                    let end = start + node.prim_count as usize;

                    for node_primitive_index in start..end {
                        let proxy_index =
                            tree.dynamic_tree.bvh.primitive_indices[node_primitive_index] as usize;

                        let Some(Some(proxy)) = tree.dynamic_tree.proxies.get(proxy_index) else {
                            error!(
                                "Proxy index {} exceeds the number of proxies {}",
                                node.first_index,
                                tree.dynamic_tree.proxies.len()
                            );
                            continue;
                        };

                        let entity1 = entity;
                        let entity2 = proxy.entity;

                        // A proxy cannot collide with itself.
                        if proxy.entity == entity {
                            continue;
                        }

                        let Ok(proxy_aabb) = aabb_query.get(proxy.entity) else {
                            continue;
                        };

                        // Check if the AABBs intersect.
                        if !Aabb::from(*aabb).intersect_aabb(&Aabb::from(*proxy_aabb)) {
                            continue;
                        }

                        // Check if the layers interact.
                        if !layers.interacts_with(proxy.layers) {
                            continue;
                        }

                        // Avoid duplicate pairs for moving proxies.
                        if (proxy_index as u32) < query_proxy_index {
                            continue;
                        }

                        // No collisions between bodies that haven't moved or colliders with incompatible layers
                        // or colliders with the same parent.
                        /*if flags1
                            .intersection(*flags2)
                            .contains(AabbIntervalFlags::IS_INACTIVE)
                            || !layers1.interacts_with(*layers2)
                            || parent1 == parent2
                        {
                            continue;
                        }*/

                        // Avoid duplicate pairs.
                        let pair_key = PairKey::new(entity1.index(), entity2.index());
                        if collisions.contains_key(&pair_key) {
                            continue;
                        }

                        pairs.push((entity1, entity2));

                        // Apply user-defined filter.
                        /*if flags1
                            .union(*flags2)
                            .contains(AabbIntervalFlags::CUSTOM_FILTER)
                        {
                            let should_collide = hooks.filter_pairs(*entity1, *entity2, commands);
                            if !should_collide {
                                continue;
                            }
                        }*/

                        /*contacts.flags.set(
                            ContactPairFlags::SENSOR,
                            flags1.union(*flags2).contains(AabbIntervalFlags::IS_SENSOR),
                        );
                        contacts.flags.set(
                            ContactPairFlags::CONTACT_EVENTS,
                            flags1
                                .union(*flags2)
                                .contains(AabbIntervalFlags::CONTACT_EVENTS),
                        );
                        contacts.flags.set(
                            ContactPairFlags::MODIFY_CONTACTS,
                            flags1
                                .union(*flags2)
                                .contains(AabbIntervalFlags::MODIFY_CONTACTS),
                        );*/

                        // TODO: On the same body?
                        // TODO: Both sensors?
                        // TODO: Joint disables collision?
                        // TODO: Custom collision filter using hook defined in `BroadPhasePlugin`
                    }

                    true
                },
            );

            // Find potential collisions against static or sleeping proxies.
            tree.static_tree
                .bvh
                .aabb_traverse(&mut stack, Aabb::from(*aabb), |bvh, node_index| {
                    let node = bvh.nodes[node_index as usize];
                    let start = node.first_index as usize;
                    let end = start + node.prim_count as usize;

                    for node_primitive_index in start..end {
                        let proxy_index =
                            tree.static_tree.bvh.primitive_indices[node_primitive_index] as usize;

                        let Some(Some(proxy)) = tree.static_tree.proxies.get(proxy_index) else {
                            error!(
                                "Proxy index {} exceeds the number of proxies {}",
                                proxy_index,
                                tree.static_tree.proxies.len()
                            );
                            continue;
                        };

                        let Ok(proxy_aabb) = aabb_query.get(proxy.entity) else {
                            continue;
                        };

                        // Check if the AABBs intersect.
                        if !Aabb::from(*aabb).intersect_aabb(&Aabb::from(*proxy_aabb)) {
                            continue;
                        }

                        let entity1 = entity;
                        let entity2 = proxy.entity;

                        // Note: Unlike with the dynamic tree, we don't need to check for self-collision or duplicate pairs.

                        // Check if the layers interact.
                        if !layers.interacts_with(proxy.layers) {
                            return true;
                        }

                        // No collisions between bodies that haven't moved or colliders with incompatible layers
                        // or colliders with the same parent.
                        /*if flags1
                            .intersection(*flags2)
                            .contains(AabbIntervalFlags::IS_INACTIVE)
                            || !layers1.interacts_with(*layers2)
                            || parent1 == parent2
                        {
                            return;
                        }*/

                        // Avoid duplicate pairs.
                        let pair_key = PairKey::new(entity1.index(), entity2.index());
                        if collisions.contains_key(&pair_key) {
                            return true;
                        }

                        pairs.push((entity1, entity2));
                    }

                    true
                });
        },
    );

    parallel.drain_into(&mut broad_collision_pairs);

    for (entity1, entity2) in broad_collision_pairs {
        let pair_key = PairKey::new(entity1.index(), entity2.index());

        // Create a new contact pair as non-touching.
        // The narrow phase will determine if the entities are touching and compute contact data.
        let mut contacts = ContactPair::new(entity1, entity2);

        if let (Ok(collider_of1), Ok(collider_of2)) = (
            collider_of_query.get(entity1),
            collider_of_query.get(entity2),
        ) {
            contacts.body1 = Some(collider_of1.body);
            contacts.body2 = Some(collider_of2.body);
        }

        // Initialize flags and other data for the contact pair.

        // Add the contact pair to the contact graph.
        collisions.add_pair_with_key(contacts, pair_key);
    }

    diagnostics.broad_phase += start.elapsed();
}
