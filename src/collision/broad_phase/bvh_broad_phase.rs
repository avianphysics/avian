use core::marker::PhantomData;

use crate::{
    collision::broad_phase::BroadPhaseDiagnostics, data_structures::pair_key::PairKey, prelude::*,
};
use bevy::{
    ecs::system::{StaticSystemParam, SystemParamItem},
    prelude::*,
    tasks::{ComputeTaskPool, ParallelSlice},
};

/// A [broad phase](crate::collision::broad_phase) plugin that uses a [Bounding Volume Hierarchy (BVH)][BVH]
/// to efficiently find pairs of colliders with overlapping AABBs.
///
/// The BVH structures are provided by [`ColliderTrees`].
///
/// [`CollisionHooks`] can be provided with generics to apply custom filtering for collision pairs.
///
/// See the [`broad_phase`](crate::collision::broad_phase) module for more information
/// and an example of creating a custom broad phase plugin.
///
/// [BVH]: https://en.wikipedia.org/wiki/Bounding_volume_hierarchy
pub struct BvhBroadPhasePlugin<H: CollisionHooks = ()>(PhantomData<H>);

impl<H: CollisionHooks> Default for BvhBroadPhasePlugin<H> {
    fn default() -> Self {
        Self(PhantomData)
    }
}

impl<H: CollisionHooks + 'static> Plugin for BvhBroadPhasePlugin<H>
where
    for<'w, 's> SystemParamItem<'w, 's, H>: CollisionHooks,
{
    fn build(&self, app: &mut App) {
        app.add_systems(
            PhysicsSchedule,
            collect_collision_pairs::<H>.in_set(BroadPhaseSystems::CollectCollisions),
        );
    }
}

fn collect_collision_pairs<H: CollisionHooks>(
    tree: ResMut<ColliderTrees>,
    moved_proxies: Res<MovedProxies>,
    collider_of_query: Query<&ColliderOf>,
    hooks: StaticSystemParam<H>,
    par_commands: ParallelCommands,
    mut collisions: ResMut<ContactGraph>,
    mut diagnostics: ResMut<BroadPhaseDiagnostics>,
) where
    for<'w, 's> SystemParamItem<'w, 's, H>: CollisionHooks,
{
    let start = crate::utils::Instant::now();

    let hooks = hooks.into_inner();
    let mut broad_collision_pairs = Vec::new();

    // Perform tree queries for all moving proxies.
    let pairs = moved_proxies.proxies().par_splat_map(
        ComputeTaskPool::get(),
        None,
        |_chunk_index, proxies| {
            let mut pairs = Vec::new();

            par_commands.command_scope(|mut commands| {
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
                                if proxy_index2 < proxy_index1
                                    && moved_proxies.contains(proxy_index2)
                                {
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

                                // No collisions between colliders on the same body.
                                if proxy1.body == proxy2.body {
                                    continue;
                                }

                                // Avoid duplicate pairs.
                                let pair_key = PairKey::new(entity1.index(), entity2.index());
                                if collisions.contains_key(&pair_key) {
                                    continue;
                                }

                                pairs.push((entity1, entity2));

                                // Apply user-defined filter.
                                if proxy1
                                    .flags
                                    .union(proxy2.flags)
                                    .contains(ColliderTreeProxyFlags::CUSTOM_FILTER)
                                {
                                    let should_collide = hooks.filter_pairs(
                                        proxy1.entity,
                                        proxy2.entity,
                                        &mut commands,
                                    );
                                    if !should_collide {
                                        continue;
                                    }
                                }

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

                                let Some(proxy2) =
                                    tree.static_tree.proxies.get(proxy_index2 as usize)
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
            });

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
