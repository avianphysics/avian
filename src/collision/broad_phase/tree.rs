use core::marker::PhantomData;
#[cfg(feature = "parallel")]
use std::cell::RefCell;

use crate::{
    collision::collider::EnlargedAabb, data_structures::bit_vec::BitVec,
    dynamics::solver::solver_body::SolverBody, prelude::*,
};
use bevy::{
    ecs::{system::StaticSystemParam, world::CommandQueue},
    prelude::*,
    tasks::{AsyncComputeTaskPool, Task, block_on, poll_once},
};
use obvhs::{
    aabb::Aabb,
    bvh2::{
        Bvh2, insertion_removal::SiblingInsertionCandidate, node::Bvh2Node,
        reinsertion::ReinsertionOptimizer,
    },
    faststack::HeapStack,
    ploc::{PlocBuilder, PlocSearchDistance, SortPrecision},
};
use slab::Slab;
#[cfg(feature = "parallel")]
use thread_local::ThreadLocal;

use super::BroadPhaseDiagnostics;

pub struct ColliderTreePlugin<C: AnyCollider>(PhantomData<C>);

impl<C: AnyCollider> Default for ColliderTreePlugin<C> {
    fn default() -> Self {
        Self(PhantomData)
    }
}

impl<C: AnyCollider> Plugin for ColliderTreePlugin<C> {
    fn build(&self, app: &mut App) {
        app.init_resource::<ColliderTrees>()
            .init_resource::<ColliderTreeRebuild>()
            .init_resource::<MovedProxies>()
            .init_resource::<ThreadLocalMovedProxies>();

        // Allowing ambiguities is required so that it's possible
        // to have multiple collision backends at the same time.
        app.add_systems(
            PhysicsSchedule,
            (update_dynamic_aabbs::<C>, update_static_aabbs::<C>)
                .chain()
                .in_set(PhysicsStepSystems::BroadPhase)
                .after(BroadPhaseSystems::First)
                .before(BroadPhaseSystems::CollectCollisions)
                .ambiguous_with_all(),
        );

        app.add_systems(
            PhysicsSchedule,
            full_rebuild.in_set(BroadPhaseSystems::BeginOptimize),
        );

        app.add_systems(
            PhysicsSchedule,
            block_on_tree_rebuild.in_set(PhysicsStepSet::Finalize),
        );

        // Initialize `ColliderAabb` for colliders.
        app.add_observer(
            |trigger: On<Add, C>,
             mut query: Query<(
                &C,
                &Position,
                &Rotation,
                Option<&CollisionMargin>,
                &mut ColliderAabb,
            )>,
             narrow_phase_config: Res<NarrowPhaseConfig>,
             length_unit: Res<PhysicsLengthUnit>,
             collider_context: StaticSystemParam<C::Context>| {
                let contact_tolerance = length_unit.0 * narrow_phase_config.contact_tolerance;
                let aabb_context = AabbContext::new(trigger.entity, &*collider_context);

                if let Ok((collider, pos, rot, collision_margin, mut aabb)) =
                    query.get_mut(trigger.entity)
                {
                    let collision_margin = collision_margin.map_or(0.0, |m| m.0);
                    *aabb = collider
                        .aabb_with_context(pos.0, *rot, aabb_context)
                        .grow(Vector::splat(contact_tolerance + collision_margin));
                }
            },
        );

        app.add_observer(
            |trigger: On<Add, EnlargedAabb>,
             mut collider_query: Query<(
                &RigidBody,
                &ColliderAabb,
                &EnlargedAabb,
                &mut BroadPhaseProxyIndex,
                Option<&CollisionLayers>,
            )>,
             mut trees: ResMut<ColliderTrees>| {
                let entity = trigger.entity;

                let Ok((rb, collider_aabb, enlarged_aabb, mut proxy_index, layers)) =
                    collider_query.get_mut(entity)
                else {
                    return;
                };

                let proxy = BvhProxy {
                    entity,
                    layers: layers.copied().unwrap_or_default(),
                    aabb: *collider_aabb,
                    flags: 0,
                };

                match *rb {
                    RigidBody::Dynamic | RigidBody::Kinematic => {
                        proxy_index.0 = trees
                            .dynamic_tree
                            .add_proxy(Aabb::from(enlarged_aabb.get()), proxy);
                    }
                    RigidBody::Static => {
                        proxy_index.0 = trees
                            .static_tree
                            .add_proxy(Aabb::from(enlarged_aabb.get()), proxy);
                    }
                }
            },
        );

        app.add_observer(
            |trigger: On<Add, Sleeping>,
             body_query: Query<&RigidBodyColliders>,
             mut collider_query: Query<&mut BroadPhaseProxyIndex>,
             mut trees: ResMut<ColliderTrees>| {
                let entity = trigger.entity;

                let Ok(body_colliders) = body_query.get(entity) else {
                    return;
                };

                let mut iter = collider_query.iter_many_mut(body_colliders.iter());
                for mut proxy_index in iter.fetch_next() {
                    let proxy_index_value = proxy_index.0;

                    // Remove from dynamic tree and insert into static tree.
                    if let Some(proxy) = trees
                        .dynamic_tree
                        .proxies
                        .try_remove(proxy_index_value as usize)
                    {
                        trees.dynamic_tree.bvh.remove_primitive(proxy_index_value);

                        let new_index = trees.static_tree.add_proxy(Aabb::from(proxy.aabb), proxy);

                        proxy_index.0 = new_index;
                    }
                }
            },
        );

        app.add_observer(
            |trigger: On<Remove, Sleeping>,
             body_query: Query<&RigidBodyColliders>,
             mut collider_query: Query<&mut BroadPhaseProxyIndex>,
             mut trees: ResMut<ColliderTrees>| {
                let entity = trigger.entity;

                let Ok(body_colliders) = body_query.get(entity) else {
                    return;
                };

                let mut iter = collider_query.iter_many_mut(body_colliders.iter());
                for mut proxy_index in iter.fetch_next() {
                    let proxy_index_value = proxy_index.0;

                    // Remove from static tree and insert into dynamic tree.
                    if let Some(proxy) = trees
                        .static_tree
                        .proxies
                        .try_remove(proxy_index_value as usize)
                    {
                        trees.static_tree.bvh.remove_primitive(proxy_index_value);

                        let new_index = trees.dynamic_tree.add_proxy(Aabb::from(proxy.aabb), proxy);

                        proxy_index.0 = new_index;
                    }
                }
            },
        );

        // TODO: Remove proxies when colliders are removed or disabled.
        app.add_observer(
            |trigger: On<Remove, EnlargedAabb>,
             collider_query: Query<(&BroadPhaseProxyIndex, &ColliderOf)>,
             body_query: Query<&RigidBody>,
             mut trees: ResMut<ColliderTrees>| {
                let entity = trigger.entity;

                let Ok((proxy_index, &ColliderOf { body })) = collider_query.get(entity) else {
                    return;
                };

                let Ok(rb) = body_query.get(body) else {
                    return;
                };

                match *rb {
                    RigidBody::Dynamic | RigidBody::Kinematic => {
                        trees.dynamic_tree.remove_proxy(proxy_index.0);
                    }
                    RigidBody::Static => {
                        trees.static_tree.remove_proxy(proxy_index.0);
                    }
                }
            },
        );
    }
}

/// A resource representing an ongoing async rebuild of a collider tree.
#[derive(Resource)]
struct ColliderTreeRebuild {
    /// The collider tree being rebuilt.
    ///
    /// This is moved into the async task when the rebuild starts.
    tree: ColliderTree,
    /// The async task performing the rebuild.
    task: Option<Task<CommandQueue>>,
}

impl Default for ColliderTreeRebuild {
    fn default() -> Self {
        Self {
            tree: ColliderTree {
                workspace: ColliderTreeWorkspace {
                    ploc_builder: PlocBuilder::default(),
                    reinsertion_optimizer: ReinsertionOptimizer::default(),
                    // Tree rebuilds will swap in the correct workspace,
                    // so this stack can be empty, as it won't be actually used.
                    // The stack requires a capacity of at least 1 to avoid panics however.
                    insertion_stack: HeapStack::new_with_capacity(1),
                },
                ..ColliderTree::default()
            },
            task: None,
        }
    }
}

/// A workspace for performing operations on a [`ColliderTree`].
///
/// This stores temporary data structures and working memory used when modifying the tree.
/// It is recommended to reuse a single instance of this struct for all operations on a tree
/// to avoid unnecessary allocations.
#[derive(Resource)]
pub struct ColliderTreeWorkspace {
    /// Builds the tree using PLOC (*Parallel, Locally Ordered Clustering*).
    pub ploc_builder: PlocBuilder,
    /// Restructures the BVH, optimizing node locations within the BVH hierarchy per SAH cost.
    pub reinsertion_optimizer: ReinsertionOptimizer,
    /// A stack for tracking insertion candidates during proxy insertions.
    pub insertion_stack: HeapStack<SiblingInsertionCandidate>,
}

impl Clone for ColliderTreeWorkspace {
    fn clone(&self) -> Self {
        Self {
            ploc_builder: self.ploc_builder.clone(),
            reinsertion_optimizer: ReinsertionOptimizer::default(),
            insertion_stack: self.insertion_stack.clone(),
        }
    }
}

impl Default for ColliderTreeWorkspace {
    fn default() -> Self {
        Self {
            ploc_builder: PlocBuilder::default(),
            reinsertion_optimizer: ReinsertionOptimizer::default(),
            insertion_stack: HeapStack::new_with_capacity(2000),
        }
    }
}

/// The index of a proxy in the broad phase.
#[derive(Component, Clone, Copy, Debug, Default, Reflect)]
pub struct BroadPhaseProxyIndex(pub u32);

/// Trees for accelerating queries on a set of colliders.
#[derive(Resource)]
pub struct ColliderTrees {
    /// A tree for the colliders of dynamic and kinematic bodies.
    pub dynamic_tree: ColliderTree,
    /// A tree for the colliders of static and sleeping bodies.
    pub static_tree: ColliderTree,
}

impl Default for ColliderTrees {
    fn default() -> Self {
        let mut proxies = Slab::with_capacity(2000);
        proxies.insert(BvhProxy {
            entity: Entity::PLACEHOLDER,
            layers: CollisionLayers::ALL,
            aabb: ColliderAabb::default(),
            flags: 0,
        });
        Self {
            dynamic_tree: ColliderTree {
                bvh: Bvh2 {
                    nodes: vec![Bvh2Node::new(default(), 1, 0)],
                    primitive_indices: vec![0],
                    ..default()
                },
                proxies: proxies.clone(),
                workspace: ColliderTreeWorkspace::default(),
            },
            static_tree: ColliderTree {
                bvh: Bvh2 {
                    nodes: vec![Bvh2Node::new(default(), 1, 0)],
                    primitive_indices: vec![0],
                    ..default()
                },
                proxies,
                workspace: ColliderTreeWorkspace::default(),
            },
        }
    }
}

/// A Bounding Volume Hierarchy for accelerating queries on a set of colliders.
///
/// Add the [`BroadPhasePairs`] component to use this tree for broad phase collision detection.
#[derive(Default)]
pub struct ColliderTree {
    /// The underlying BVH structure.
    pub bvh: Bvh2,
    /// The proxies stored in the tree.
    pub proxies: Slab<BvhProxy>,
    /// A workspace for reusing allocations across tree operations.
    pub workspace: ColliderTreeWorkspace,
}

/// A component that stores pairs of entities with overlapping AABBs for a [`ColliderTree`].
#[derive(Component, Clone, Debug, Default)]
pub struct BroadPhasePairs(pub Vec<(Entity, Entity)>);

impl ColliderTree {
    /// Adds a proxy to the tree, returning its index.
    #[inline]
    pub fn add_proxy(&mut self, aabb: Aabb, proxy: BvhProxy) -> u32 {
        let id = self.proxies.insert(proxy) as u32;
        self.bvh
            .insert_primitive(aabb, id, &mut self.workspace.insertion_stack);
        id
    }

    /// Removes a proxy from the tree.
    #[inline]
    pub fn remove_proxy(&mut self, proxy_index: u32) {
        if self.proxies.try_remove(proxy_index as usize).is_none() {
            return;
        }
        self.bvh.remove_primitive(proxy_index);
    }

    /// Moves a proxy to a new position in the tree.
    ///
    /// Returns the new index of the proxy.
    #[inline]
    pub fn move_proxy(&mut self, proxy_index: u32, aabb: Aabb) -> u32 {
        self.bvh.remove_primitive(proxy_index);
        self.bvh
            .insert_primitive(aabb, proxy_index, &mut self.workspace.insertion_stack) as u32
    }

    /// Fully rebuilds the tree from the given list of AABBs.
    #[inline]
    pub fn rebuild_full(&mut self) {
        let mut aabbs: Vec<Aabb> = Vec::with_capacity(self.proxies.len());
        let mut indices: Vec<u32> = Vec::with_capacity(self.proxies.len());

        aabbs.push(Aabb::default());
        indices.push(0);

        for (i, proxy) in self.proxies.iter() {
            aabbs.push(Aabb::from(proxy.aabb));
            indices.push(i as u32);
        }

        self.workspace.ploc_builder.build_with_bvh(
            &mut self.bvh,
            PlocSearchDistance::Minimum,
            &aabbs,
            indices,
            SortPrecision::U64,
            0,
        );
    }

    /// Restructures the tree using parallel reinsertion, optimizing node locations based on SAH cost.
    ///
    /// This can be used to improve query performance after the tree quality has degraded,
    /// for example after many proxy insertions and removals.
    #[inline]
    pub fn optimize(&mut self, batch_size_ratio: f32) {
        self.workspace
            .reinsertion_optimizer
            .run(&mut self.bvh, batch_size_ratio, None);
    }

    /// Updates the AABB of a proxy and reinserts it at an optimal place in the tree.
    #[inline]
    pub fn reinsert_proxy(&mut self, proxy_index: u32, aabb: ColliderAabb) {
        // Update the static tree with the new AABB.
        self.proxies[proxy_index as usize].aabb = aabb;

        // Reinsert the node into the BVH.
        let node_id = self.bvh.primitives_to_nodes[proxy_index as usize];
        self.bvh.resize_node(node_id as usize, Aabb::from(aabb));
        self.bvh.reinsert_node(node_id as usize);
    }

    /// Updates the AABB of a proxy in the tree and refits the BVH, resizing parent nodes as necessary.
    ///
    /// Unlike [`reinsert_proxy`](Self::reinsert_proxy), this does not change the position of the proxy in the tree.
    /// Enlarging proxies withoujt optimizing the tree can degrade query performance over time,
    /// so it is recommended to periodically call [`rebuild_full`](Self::rebuild_full) or [`optimize`](Self::optimize).
    #[inline]
    pub fn enlarge_proxy(&mut self, proxy_index: u32, aabb: Aabb) {
        // Get the node index for the proxy.
        let node_index = self.bvh.primitives_to_nodes[proxy_index as usize] as usize;

        // Update the proxy's AABB in the BVH.
        self.bvh.nodes[node_index].aabb = aabb;

        // Compute the parent indices if they haven't been initialized yet.
        self.bvh.init_parents_if_uninit();

        let mut index = self.bvh.parents[node_index] as usize;

        // Refit the BVH working up the tree.
        // Stop when a parent node's AABB is not changed.
        loop {
            let parent_node = &mut self.bvh.nodes[index];
            let new_aabb = parent_node.aabb.union(&aabb);

            if parent_node.aabb == new_aabb {
                break;
            }

            parent_node.aabb = new_aabb;

            if index == 0 {
                break;
            }

            index = self.bvh.parents[index] as usize;
        }
    }
}

/// A proxy representing a collider in the [`ColliderTree`].
#[derive(Clone, Debug)]
pub struct BvhProxy {
    /// The entity this proxy represents.
    pub entity: Entity,
    /// The collision layers of the collider.
    pub layers: CollisionLayers,
    /// The tight AABB of the collider.
    pub aabb: ColliderAabb,
    /// Flags for the proxy.
    pub flags: u32,
}

impl From<ColliderAabb> for Aabb {
    fn from(value: ColliderAabb) -> Self {
        Self {
            #[cfg(feature = "3d")]
            min: value.min.into(),
            #[cfg(feature = "3d")]
            max: value.max.into(),
            #[cfg(feature = "2d")]
            min: value.min.extend(0.0).into(),
            #[cfg(feature = "2d")]
            max: value.max.extend(0.0).into(),
        }
    }
}

fn update_static_aabbs<C: AnyCollider>(
    bodies: Query<&RigidBodyColliders, Without<SolverBody>>,
    mut colliders: Query<
        (
            Entity,
            &Position,
            &Rotation,
            &mut ColliderAabb,
            &C,
            Option<&CollisionMargin>,
            &BroadPhaseProxyIndex,
        ),
        Or<(Changed<Position>, Changed<Rotation>, Changed<C>)>,
    >,
    narrow_phase_config: Res<NarrowPhaseConfig>,
    length_unit: Res<PhysicsLengthUnit>,
    mut collider_trees: ResMut<ColliderTrees>,
    mut diagnostics: ResMut<BroadPhaseDiagnostics>,
    collider_context: StaticSystemParam<C::Context>,
) {
    let start = crate::utils::Instant::now();

    let contact_tolerance = length_unit.0 * narrow_phase_config.contact_tolerance;

    collider_trees
        .static_tree
        .bvh
        .init_primitives_to_nodes_if_uninit();

    for body_colliders in &bodies {
        let mut iter = colliders.iter_many_mut(body_colliders.iter());
        while let Some((
            entity,
            collider_pos,
            collider_rot,
            mut aabb,
            collider,
            margin,
            proxy_index,
        )) = iter.fetch_next()
        {
            let margin = margin.map_or(0.0, |margin| margin.0);

            let context = AabbContext::new(entity, &*collider_context);

            // Compute the AABB of the collider.
            *aabb = collider
                .aabb_with_context(collider_pos.0, *collider_rot, context)
                .grow(Vector::splat(contact_tolerance + margin));

            // Reinsert the proxy into the BVH.
            collider_trees
                .static_tree
                .reinsert_proxy(proxy_index.0, *aabb);
        }
    }

    diagnostics.update += start.elapsed();
}

/// A bit vector for tracking moved proxies in the broad phase.
///
/// Set bits correspond to colliders whose [`ColliderAabb`] has moved
/// outside of the previous [`EnlargedAabb`].
#[derive(Resource, Default, Deref, DerefMut)]
struct MovedProxies(BitVec);

/// A resource storing thread-local bit vectors for tracking moved proxies in the broad phase.
#[derive(Resource, Default, Deref, DerefMut)]
struct ThreadLocalMovedProxies(ThreadLocal<RefCell<BitVec>>);

fn update_dynamic_aabbs<C: AnyCollider>(
    mut commands: Commands,
    mut colliders: ParamSet<(
        Query<(
            Entity,
            &C,
            &mut ColliderAabb,
            &mut EnlargedAabb,
            &BroadPhaseProxyIndex,
            &Position,
            &Rotation,
            Option<&CollisionMargin>,
            Option<&SpeculativeMargin>,
        )>,
        Query<(&ColliderAabb, &EnlargedAabb)>,
    )>,
    rb_query: Query<
        (
            &Position,
            &ComputedCenterOfMass,
            &LinearVelocity,
            &AngularVelocity,
            &RigidBodyColliders,
            Has<SweptCcd>,
        ),
        With<SolverBody>,
    >,
    narrow_phase_config: Res<NarrowPhaseConfig>,
    length_unit: Res<PhysicsLengthUnit>,
    mut collider_trees: ResMut<ColliderTrees>,
    mut moved_proxies: ResMut<MovedProxies>,
    mut thread_local_moved_proxies: ResMut<ThreadLocalMovedProxies>,
    time: Res<Time>,
    collider_context: StaticSystemParam<C::Context>,
    mut diagnostics: ResMut<BroadPhaseDiagnostics>,
) {
    let start = crate::utils::Instant::now();

    // An upper bound on the number of dynamic proxies, for sizing the bit vectors.
    // TODO: Use a better way to track the number of proxies.
    let max_num_dynamic_proxies = collider_trees.dynamic_tree.proxies.capacity();

    // Clear the bit vector used to track status changes for each contact pair.
    moved_proxies.set_bit_count_and_clear(max_num_dynamic_proxies);

    thread_local_moved_proxies.iter_mut().for_each(|context| {
        let bit_vec_mut = &mut context.borrow_mut();
        bit_vec_mut.set_bit_count_and_clear(max_num_dynamic_proxies);
    });

    let delta_secs = time.delta_seconds_adjusted();
    let default_speculative_margin = length_unit.0 * narrow_phase_config.default_speculative_margin;
    let contact_tolerance = length_unit.0 * narrow_phase_config.contact_tolerance;
    let margin = length_unit.0 * 0.05;

    let mut insertion_buffer = Vec::new();

    collider_trees
        .dynamic_tree
        .bvh
        .init_primitives_to_nodes_if_uninit();

    let collider_query = colliders.p0();

    rb_query.par_iter().for_each(
        |(rb_pos, center_of_mass, lin_vel, ang_vel, body_colliders, has_swept_ccd)| {
            for collider_entity in body_colliders.iter() {
                let Ok((
                    entity,
                    collider,
                    mut aabb,
                    mut enlarged_aabb,
                    proxy_index,
                    pos,
                    rot,
                    collision_margin,
                    speculative_margin,
                )) = (unsafe { collider_query.get_unchecked(collider_entity) })
                else {
                    continue;
                };
                let collision_margin = collision_margin.map_or(0.0, |margin| margin.0);
                let speculative_margin = if has_swept_ccd {
                    Scalar::MAX
                } else {
                    speculative_margin.map_or(default_speculative_margin, |margin| margin.0)
                };

                let context = AabbContext::new(entity, &*collider_context);

                if speculative_margin <= 0.0 {
                    *aabb = collider
                        .aabb_with_context(pos.0, *rot, context)
                        .grow(Vector::splat(contact_tolerance + collision_margin));
                } else {
                    // If the rigid body is rotating, off-center colliders will orbit around it,
                    // which affects their linear velocities. We need to compute the linear velocity
                    // at the offset position.
                    // TODO: This assumes that the colliders would continue moving in the same direction,
                    //       but because they are orbiting, the direction will change. We should take
                    //       into account the uniform circular motion.
                    let offset = pos.0 - rb_pos.0 - center_of_mass.0;
                    #[cfg(feature = "2d")]
                    let vel = lin_vel.0 + Vector::new(-ang_vel.0 * offset.y, ang_vel.0 * offset.x);
                    #[cfg(feature = "3d")]
                    let vel = lin_vel.0 + ang_vel.cross(offset);
                    let movement = (vel * delta_secs)
                        .clamp_length_max(speculative_margin.max(contact_tolerance));

                    // Current position and predicted position for next feame
                    #[cfg(feature = "2d")]
                    let (end_pos, end_rot) = (
                        pos.0 + movement,
                        *rot * Rotation::radians(ang_vel.0 * delta_secs),
                    );

                    #[cfg(feature = "3d")]
                    let (end_pos, end_rot) = (
                        pos.0 + movement,
                        Rotation(Quaternion::from_scaled_axis(ang_vel.0 * delta_secs) * rot.0)
                            .fast_renormalize(),
                    );

                    // Compute swept AABB, the space that the body would occupy if it was integrated for one frame
                    // TODO: Should we expand the AABB in all directions for speculative contacts?
                    *aabb = collider
                        .swept_aabb_with_context(pos.0, *rot, end_pos, end_rot, context)
                        .grow(Vector::splat(collision_margin));
                }

                let moved = enlarged_aabb.update(&aabb, margin);

                if moved {
                    let mut thread_local_bit_vec = thread_local_moved_proxies
                        .get_or(|| {
                            let mut bit_vec = BitVec::default();
                            bit_vec.set_bit_count_and_clear(max_num_dynamic_proxies);
                            RefCell::new(bit_vec)
                        })
                        .borrow_mut();
                    thread_local_bit_vec.set(proxy_index.0 as usize);
                }
            }
        },
    );

    // Combine thread-local moved proxy bit vectors into the main one.
    thread_local_moved_proxies.iter_mut().for_each(|context| {
        let thread_local_bit_vec = context.borrow();
        moved_proxies.or(&thread_local_bit_vec);
    });

    // Serially enlarge moved proxies in the dynamic tree.
    let enlarged_colliders = colliders.p1();
    for (i, mut bits) in moved_proxies.blocks().enumerate() {
        while bits != 0 {
            let trailing_zeros = bits.trailing_zeros();
            let proxy_index = i as u32 * 64 + trailing_zeros;
            let proxy = &mut collider_trees.dynamic_tree.proxies[proxy_index as usize];
            let entity = proxy.entity;
            let (aabb, enlarged_aabb) = enlarged_colliders.get(entity).unwrap_or_else(|_| {
                panic!(
                    "EnlargedAabb missing for moved collider entity {:?}",
                    entity
                )
            });

            proxy.aabb = *aabb;
            collider_trees
                .dynamic_tree
                .enlarge_proxy(proxy_index, Aabb::from(enlarged_aabb.get()));

            insertion_buffer.push(entity);

            // Clear the least significant set bit
            bits &= bits - 1;
        }
    }

    // Insert `MovedProxy` components for moved colliders.
    commands.insert_batch(
        insertion_buffer
            .into_iter()
            .map(|entity| (entity, MovedProxy)),
    );

    diagnostics.update += start.elapsed();
}

/// A marker component for a collider whose [`ColliderAabb`] has moved
/// outside of the previous [`EnlargedAabb`] and needs to be updated in the [`ColliderTree`].
#[derive(Component, Default)]
pub struct MovedProxy;

/// Begins an async full rebuild of the dynamic [`ColliderTree`] to optimize its structure.
///
/// This spawns an async task that performs the rebuild concurrently with the simulation step.
fn full_rebuild(
    mut collider_trees: ResMut<ColliderTrees>,
    mut rebuild_task: ResMut<ColliderTreeRebuild>,
    mut diagnostics: ResMut<BroadPhaseDiagnostics>,
) {
    let start = crate::utils::Instant::now();

    let task_pool = AsyncComputeTaskPool::get();

    // Use the workspace from the current dynamic tree for the rebuild.
    // It will be swapped back when the rebuild is done.
    core::mem::swap(
        &mut collider_trees.dynamic_tree.workspace,
        &mut rebuild_task.tree.workspace,
    );

    let mut tree = core::mem::take(&mut rebuild_task.tree);

    // Update the proxies in the tree to match the current dynamic tree.
    // TODO: Try to avoid this clone.
    tree.proxies
        .clone_from(&collider_trees.dynamic_tree.proxies);

    // Spawn the async task for rebuilding the tree.
    let task = task_pool.spawn(async move {
        let mut tree = tree;
        tree.rebuild_full();

        let mut command_queue = CommandQueue::default();
        command_queue.push(move |world: &mut World| {
            let mut collider_trees = world
                .get_resource_mut::<ColliderTrees>()
                .expect("ColliderTrees resource missing");
            collider_trees.dynamic_tree = tree;
        });
        command_queue
    });

    rebuild_task.task = Some(task);

    diagnostics.optimize += start.elapsed();
}

/// Completes the async rebuild of the collider tree started in [`full_rebuild`].
fn block_on_tree_rebuild(
    mut commands: Commands,
    mut collider_trees: ResMut<ColliderTrees>,
    mut tree_rebuild: ResMut<ColliderTreeRebuild>,
    mut diagnostics: ResMut<BroadPhaseDiagnostics>,
) {
    if let Some(task) = &mut tree_rebuild.task
        && let Some(mut command_queue) = block_on(poll_once(task))
    {
        commands.append(&mut command_queue);
    }

    tree_rebuild.task = None;

    let start = crate::utils::Instant::now();
    // Swap back the workspace to the dynamic tree.
    core::mem::swap(
        &mut collider_trees.dynamic_tree.workspace,
        &mut tree_rebuild.tree.workspace,
    );

    diagnostics.optimize += start.elapsed();
}
