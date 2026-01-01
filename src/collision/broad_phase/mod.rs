//! Finds pairs of entities with overlapping [`ColliderAabb`]s to reduce
//! the number of potential contacts for the [narrow phase](crate::narrow_phase).
//!
//! See [`BroadPhasePlugin`].

mod diagnostics;
pub use diagnostics::BroadPhaseDiagnostics;

mod tree;
pub use tree::*;

use core::marker::PhantomData;

use crate::{data_structures::pair_key::PairKey, prelude::*};
use bevy::{
    ecs::system::SystemParamItem,
    prelude::*,
    tasks::{ComputeTaskPool, ParallelSlice},
};

/// Finds pairs of entities with overlapping [`ColliderAabb`]s to reduce
/// the number of potential contacts for the [narrow phase](crate::narrow_phase).
///
/// A contact pair is created in the [`Collisions`] resource for each pair found.
/// Removing and updating these pairs is left to the [narrow phase](crate::narrow_phase).
///
/// The broad phase systems run in [`PhysicsStepSystems::BroadPhase`].
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

        app.init_resource::<ContactGraph>();

        app.configure_sets(
            PhysicsSchedule,
            (
                BroadPhaseSystems::First,
                BroadPhaseSystems::CollectCollisions,
                BroadPhaseSystems::Last,
            )
                .chain()
                .in_set(PhysicsStepSystems::BroadPhase),
        );

        app.configure_sets(
            PhysicsSchedule,
            BroadPhaseSystems::BeginOptimize.in_set(NarrowPhaseSystems::Update),
        );

        app.configure_sets(
            PhysicsSchedule,
            BroadPhaseSystems::EndOptimize
                .after(PhysicsStepSystems::Solver)
                .before(PhysicsStepSystems::Sleeping),
        );

        app.add_systems(
            PhysicsSchedule,
            find_collision_pairs.in_set(BroadPhaseSystems::CollectCollisions),
        );
    }

    fn finish(&self, app: &mut App) {
        // Register timer diagnostics for the broad phase.
        app.register_physics_diagnostics::<BroadPhaseDiagnostics>();
    }
}

/// System sets for systems running in [`PhysicsStepSystems::BroadPhase`].
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum BroadPhaseSystems {
    /// Runs at the start of the broad phase. Empty by default.
    First,
    /// Begins optimizing acceleration structures to keep their query performance good.
    ///
    /// This runs concurrently with the simulation step as an async task.
    BeginOptimize,
    /// Finds pairs of entities with overlapping [`ColliderAabb`]s
    /// and creates contact pairs for them in [`Collisions`].
    CollectCollisions,
    /// Completes the optimization of acceleration structures started in [`BroadPhaseSystems::BeginOptimize`].
    ///
    /// This runs at the end of the simulation step.
    EndOptimize,
    /// Runs at the end of the broad phase. Empty by default.
    Last,
}

fn find_collision_pairs(
    tree: ResMut<ColliderTrees>,
    moved_proxies: Res<MovedProxies>,
    collider_of_query: Query<&ColliderOf>,
    mut collisions: ResMut<ContactGraph>,
    mut diagnostics: ResMut<BroadPhaseDiagnostics>,
) {
    let start = crate::utils::Instant::now();

    let mut broad_collision_pairs = Vec::new();

    // Perform tree queries for all moving proxies.
    let pairs = moved_proxies.proxies().par_splat_map(
        ComputeTaskPool::get(),
        None,
        |_chunk_index, proxies| {
            let mut pairs = Vec::new();

            for &proxy_index1 in proxies {
                let proxy1 = tree
                    .dynamic_tree
                    .proxies
                    .get(proxy_index1 as usize)
                    .unwrap();

                // Find potential collisions against moving proxies.
                tree.dynamic_tree
                    .bvh
                    .aabb_traverse(proxy1.aabb, |bvh, node_index| {
                        let node = bvh.nodes[node_index as usize];
                        let start = node.first_index as usize;
                        let end = start + node.prim_count as usize;

                        for node_primitive_index in start..end {
                            let proxy_index2 =
                                tree.dynamic_tree.bvh.primitive_indices[node_primitive_index];

                            // Avoid duplicate pairs for moving proxies.
                            if proxy_index2 < proxy_index1 {
                                // TODO: Check move set first.
                                continue;
                            }

                            let proxy2 = tree
                                .dynamic_tree
                                .proxies
                                .get(proxy_index2 as usize)
                                .unwrap();

                            // Check if the AABBs intersect.
                            if !proxy1.aabb.intersect_aabb(&proxy2.aabb) {
                                continue;
                            }

                            let entity1 = proxy1.entity;
                            let entity2 = proxy2.entity;

                            // A proxy cannot collide with itself.
                            if entity1 == entity2 {
                                continue;
                            }

                            // Check if the layers interact.
                            if !proxy1.layers.interacts_with(proxy2.layers) {
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
                    });

                // Find potential collisions against static or sleeping proxies.
                tree.static_tree
                    .bvh
                    .aabb_traverse(proxy1.aabb, |bvh, node_index| {
                        let node = bvh.nodes[node_index as usize];
                        let start = node.first_index as usize;
                        let end = start + node.prim_count as usize;

                        for node_primitive_index in start..end {
                            let proxy_index2 =
                                tree.static_tree.bvh.primitive_indices[node_primitive_index];

                            let Some(proxy2) = tree.static_tree.proxies.get(proxy_index2 as usize)
                            else {
                                error!(
                                    "Proxy index {} exceeds the number of proxies {}",
                                    proxy_index2,
                                    tree.static_tree.proxies.len()
                                );
                                continue;
                            };

                            // Check if the AABBs intersect.
                            if !proxy1.aabb.intersect_aabb(&proxy2.aabb) {
                                continue;
                            }

                            let entity1 = proxy1.entity;
                            let entity2 = proxy2.entity;

                            // Note: Unlike with the dynamic tree, we don't need to check for self-collision or duplicate pairs.

                            // Check if the layers interact.
                            if !proxy1.layers.interacts_with(proxy2.layers) {
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
            }

            pairs
        },
    );

    // Drain the pairs into a single vector.
    for mut chunk in pairs {
        broad_collision_pairs.append(&mut chunk);
    }

    for (entity1, entity2) in broad_collision_pairs {
        let mut contact_edge = ContactEdge::new(entity1, entity2);

        if let (Ok(collider_of1), Ok(collider_of2)) = (
            collider_of_query.get(entity1),
            collider_of_query.get(entity2),
        ) {
            contact_edge.body1 = Some(collider_of1.body);
            contact_edge.body2 = Some(collider_of2.body);
        }

        collisions.add_edge_with(contact_edge, |contact_pair| {
            if let (Ok(collider_of1), Ok(collider_of2)) = (
                collider_of_query.get(entity1),
                collider_of_query.get(entity2),
            ) {
                contact_pair.body1 = Some(collider_of1.body);
                contact_pair.body2 = Some(collider_of2.body);
            }
        });
    }

    diagnostics.find_pairs += start.elapsed();
}
