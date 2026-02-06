use core::marker::PhantomData;

use crate::{
    collider_tree::{
        ColliderTree, ColliderTreeProxy, ColliderTreeProxyFlags, ColliderTreeProxyKey,
        ColliderTreeType, ColliderTrees, MovedProxies, ProxyId,
    },
    collision::{CollisionDiagnostics, contact_types::ContactEdgeFlags},
    data_structures::pair_key::PairKey,
    dynamics::solver::joint_graph::JointGraph,
    prelude::*,
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
    trees: ResMut<ColliderTrees>,
    moved_proxies: Res<MovedProxies>,
    hooks: StaticSystemParam<H>,
    par_commands: ParallelCommands,
    mut contact_graph: ResMut<ContactGraph>,
    joint_graph: Res<JointGraph>,
    mut diagnostics: ResMut<CollisionDiagnostics>,
) where
    for<'w, 's> SystemParamItem<'w, 's, H>: CollisionHooks,
{
    let start = crate::utils::Instant::now();

    let hooks = hooks.into_inner();
    let mut broad_collision_pairs = Vec::<(ColliderTreeProxyKey, ColliderTreeProxyKey)>::new();

    // Perform tree queries for all moving proxies.
    // TODO. We could iterate moved proxies of each tree separately
    //       to get rid of tree lookups and body type checks.
    //       May not be worth it though?
    let pairs = moved_proxies.proxies().par_splat_map(
        ComputeTaskPool::get(),
        None,
        |_chunk_index, proxies| {
            let mut pairs = Vec::new();

            par_commands.command_scope(|mut commands| {
                for proxy_key1 in proxies {
                    let proxy_id1 = proxy_key1.id();
                    let proxy_type1 = proxy_key1.tree_type();

                    // Get the proxy from its appropriate tree.
                    let tree = trees.tree_for_type(proxy_type1);
                    let proxy1 = tree.get_proxy(proxy_key1.id()).unwrap();

                    // Query dynamic tree.
                    query_tree(
                        &trees.dynamic_tree,
                        ColliderTreeType::Dynamic,
                        *proxy_key1,
                        proxy_id1,
                        proxy_type1,
                        proxy1,
                        &moved_proxies,
                        &hooks,
                        &mut commands,
                        &contact_graph,
                        &joint_graph,
                        &mut pairs,
                    );

                    // Query kinematic tree.
                    query_tree(
                        &trees.kinematic_tree,
                        ColliderTreeType::Kinematic,
                        *proxy_key1,
                        proxy_id1,
                        proxy_type1,
                        proxy1,
                        &moved_proxies,
                        &hooks,
                        &mut commands,
                        &contact_graph,
                        &joint_graph,
                        &mut pairs,
                    );

                    // Skip static-static body collisions unless sensors or standalone colliders are involved.
                    if proxy_type1 != ColliderTreeType::Static || proxy1.is_sensor() {
                        // Query static tree.
                        query_tree(
                            &trees.static_tree,
                            ColliderTreeType::Static,
                            *proxy_key1,
                            proxy_id1,
                            proxy_type1,
                            proxy1,
                            &moved_proxies,
                            &hooks,
                            &mut commands,
                            &contact_graph,
                            &joint_graph,
                            &mut pairs,
                        );
                    }

                    // Query standalone tree (colliders with no body).
                    query_tree(
                        &trees.standalone_tree,
                        ColliderTreeType::Standalone,
                        *proxy_key1,
                        proxy_id1,
                        proxy_type1,
                        proxy1,
                        &moved_proxies,
                        &hooks,
                        &mut commands,
                        &contact_graph,
                        &joint_graph,
                        &mut pairs,
                    );
                }
            });

            pairs
        },
    );

    // Drain the pairs into a single vector.
    for mut chunk in pairs {
        broad_collision_pairs.append(&mut chunk);
    }

    // Add the found collision pairs to the contact graph.
    for (proxy_key1, proxy_key2) in broad_collision_pairs {
        let proxy1 = trees.get_proxy(proxy_key1).unwrap();
        let proxy2 = trees.get_proxy(proxy_key2).unwrap();

        let mut contact_edge = ContactEdge::new(proxy1.collider, proxy2.collider);
        contact_edge.body1 = proxy1.body;
        contact_edge.body2 = proxy2.body;

        let flags_union = proxy1.flags.union(proxy2.flags);

        // Contact event flags
        contact_edge.flags.set(
            ContactEdgeFlags::CONTACT_EVENTS,
            flags_union.contains(ColliderTreeProxyFlags::CONTACT_EVENTS),
        );

        contact_graph.add_edge_with(contact_edge, |contact_pair| {
            contact_pair.body1 = proxy1.body;
            contact_pair.body2 = proxy2.body;

            contact_pair.flags.set(
                ContactPairFlags::MODIFY_CONTACTS,
                flags_union.contains(ColliderTreeProxyFlags::MODIFY_CONTACTS),
            );

            contact_pair.flags.set(
                ContactPairFlags::GENERATE_CONSTRAINTS,
                !flags_union.contains(ColliderTreeProxyFlags::BODY_DISABLED)
                    && !flags_union.contains(ColliderTreeProxyFlags::SENSOR),
            );
        });
    }

    diagnostics.broad_phase += start.elapsed();
}

#[inline]
fn query_tree(
    tree: &ColliderTree,
    tree_type: ColliderTreeType,
    proxy_key1: ColliderTreeProxyKey,
    proxy_id1: ProxyId,
    proxy_type1: ColliderTreeType,
    proxy1: &ColliderTreeProxy,
    moved_proxies: &MovedProxies,
    hooks: &impl CollisionHooks,
    commands: &mut Commands,
    contact_graph: &ContactGraph,
    joint_graph: &JointGraph,
    pairs: &mut Vec<(ColliderTreeProxyKey, ColliderTreeProxyKey)>,
) {
    tree.bvh.aabb_traverse(proxy1.aabb, |bvh, node_index| {
        let node = &bvh.nodes[node_index as usize];
        let start = node.first_index as usize;
        let end = start + node.prim_count as usize;

        for node_primitive_index in start..end {
            let proxy_id2 = ProxyId::new(tree.bvh.primitive_indices[node_primitive_index]);
            let proxy_key2 = ColliderTreeProxyKey::new(proxy_id2, tree_type);

            // Skip self-collision.
            if proxy_key1 == proxy_key2 {
                continue;
            }

            let proxy2 = tree.get_proxy(proxy_id2).unwrap();

            // Avoid duplicate pairs for moving proxies.
            //
            // Most of the time, only dynamic and kinematic bodies will be moving, but static bodies
            // can also be in the move set (ex: when spawned or teleported).
            //
            // If sensors are involved, we handle the pair here regardless of movement,
            // just to be safe. Otherwise a static sensor colliding with a static body might be missed.
            // TODO: There's probably a better way to handle sensors.
            let proxy1_greater = ((tree_type as u8) < (proxy_type1 as u8))
                || (tree_type == proxy_type1 && proxy_id2.id() < proxy_id1.id());
            if proxy1_greater && moved_proxies.contains(proxy_key2) && !proxy1.is_sensor() {
                // Both proxies are moving, so the other query will handle this pair.
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

            let entity1 = proxy1.collider;
            let entity2 = proxy2.collider;

            // Avoid duplicate pairs.
            let pair_key = PairKey::new(entity1.index_u32(), entity2.index_u32());
            if contact_graph.contains_key(&pair_key) {
                continue;
            }

            // Check if a joint disables contacts between the two bodies.
            if let Some(body1) = proxy1.body
                && let Some(body2) = proxy2.body
                && joint_graph
                    .joints_between(body1, body2)
                    .any(|edge| edge.collision_disabled)
            {
                continue;
            }

            // Apply user-defined filter.
            if proxy1
                .flags
                .union(proxy2.flags)
                .contains(ColliderTreeProxyFlags::CUSTOM_FILTER)
            {
                let should_collide = hooks.filter_pairs(entity1, entity2, commands);
                if !should_collide {
                    continue;
                }
            }

            pairs.push((proxy_key1, proxy_key2));
        }

        true
    });
}
