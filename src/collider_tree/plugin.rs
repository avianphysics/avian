#[cfg(feature = "parallel")]
use core::cell::RefCell;
use core::marker::PhantomData;

use crate::{
    collision::{broad_phase::BroadPhaseDiagnostics, collider::EnlargedAabb},
    data_structures::{bit_vec::BitVec, stable_vec::StableVec},
    dynamics::solver::solver_body::SolverBody,
    prelude::*,
};
use bevy::{
    ecs::{system::StaticSystemParam, world::CommandQueue},
    platform::collections::HashSet,
    prelude::*,
    tasks::{AsyncComputeTaskPool, Task, block_on},
};
use obvhs::{
    aabb::Aabb,
    bvh2::{Bvh2, reinsertion::ReinsertionOptimizer},
    faststack::HeapStack,
    ploc::PlocBuilder,
};
#[cfg(feature = "parallel")]
use thread_local::ThreadLocal;

/// A plugin that manages [`ColliderTrees`] for a collider type `C`.
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
            .init_resource::<MovedProxies>();

        app.configure_sets(
            PhysicsSchedule,
            (ColliderTreeSystems::UpdateAabbs
                .in_set(PhysicsStepSystems::BroadPhase)
                .after(BroadPhaseSystems::First)
                .before(BroadPhaseSystems::CollectCollisions),),
        );

        app.configure_sets(
            PhysicsSchedule,
            ColliderTreeSystems::BeginOptimize.in_set(NarrowPhaseSystems::Update),
        );

        app.configure_sets(
            PhysicsSchedule,
            ColliderTreeSystems::EndOptimize.in_set(PhysicsStepSystems::Finalize),
        );

        // Allowing ambiguities is required so that it's possible
        // to have multiple collision backends at the same time.
        app.add_systems(
            PhysicsSchedule,
            (update_dynamic_aabbs::<C>, update_static_aabbs::<C>)
                .chain()
                .in_set(ColliderTreeSystems::UpdateAabbs)
                .ambiguous_with_all(),
        );

        app.add_systems(
            PhysicsSchedule,
            full_rebuild.in_set(ColliderTreeSystems::BeginOptimize),
        );

        app.add_systems(
            PhysicsSchedule,
            block_on_tree_rebuild.in_set(ColliderTreeSystems::EndOptimize),
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
                &mut ColliderTreeProxyIndex,
                Option<&CollisionLayers>,
            )>,
             mut trees: ResMut<ColliderTrees>| {
                let entity = trigger.entity;

                let Ok((rb, collider_aabb, enlarged_aabb, mut proxy_index, layers)) =
                    collider_query.get_mut(entity)
                else {
                    return;
                };

                let aabb = Aabb::from(*collider_aabb);
                let enlarged_aabb = Aabb::from(enlarged_aabb.get());

                let proxy = ColliderTreeProxy {
                    entity,
                    layers: layers.copied().unwrap_or_default(),
                    aabb,
                    flags: 0,
                };

                match *rb {
                    RigidBody::Dynamic | RigidBody::Kinematic => {
                        proxy_index.0 = trees.dynamic_tree.add_proxy(enlarged_aabb, proxy);
                    }
                    RigidBody::Static => {
                        proxy_index.0 = trees.static_tree.add_proxy(enlarged_aabb, proxy);
                    }
                }
            },
        );

        // TODO: Remove proxies when colliders are removed or disabled.
        app.add_observer(
            |trigger: On<Remove, EnlargedAabb>,
             collider_query: Query<(&ColliderTreeProxyIndex, &ColliderOf)>,
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

/// System sets for managing [`ColliderTrees`].
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ColliderTreeSystems {
    /// Updates the AABBs of colliders.
    UpdateAabbs,
    /// Begins optimizing acceleration structures to keep their query performance good.
    ///
    /// This runs concurrently with the simulation step as an async task.
    BeginOptimize,
    /// Completes the optimization of acceleration structures started in [`ColliderTreeSystems::BeginOptimize`].
    ///
    /// This runs at the end of the simulation step.
    EndOptimize,
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
        let tree = ColliderTree {
            bvh: Bvh2::default(),
            proxies: StableVec::with_capacity(2000),
            workspace: ColliderTreeWorkspace::default(),
        };

        Self {
            dynamic_tree: tree.clone(),
            static_tree: tree,
        }
    }
}

/// The index of a [`ColliderTreeProxy`] in a [`ColliderTree`].
#[derive(Component, Clone, Copy, Debug, Default, Reflect)]
pub struct ColliderTreeProxyIndex(pub u32);

fn update_static_aabbs<C: AnyCollider>(
    static_bodies: Query<&RigidBodyColliders, (Without<SolverBody>, Without<Sleeping>)>,
    mut colliders: Query<
        (
            Entity,
            &Position,
            &Rotation,
            &mut ColliderAabb,
            &C,
            Option<&CollisionMargin>,
            &ColliderTreeProxyIndex,
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

    for body_colliders in &static_bodies {
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
                .reinsert_proxy(proxy_index.0, Aabb::from(*aabb));
        }
    }

    diagnostics.update += start.elapsed();
}

/// A resource for tracking proxies whose [`ColliderAabb`] has moved
/// outside of the previous [`EnlargedAabb`].
#[derive(Resource, Default)]
pub struct MovedProxies {
    /// A bit vector tracking moved proxies.
    ///
    /// Set bits indicate moved proxy indices.
    bit_vec: BitVec,
    /// Thread-local bit vectors for tracking moved proxies in parallel.
    /// These are combined into [`bit_vec`](Self::bit_vec) after parallel processing.
    thread_local_bit_vec: ThreadLocal<RefCell<BitVec>>,
    /// A vector of moved proxy indices.
    proxies: Vec<u32>,
    /// A set of moved proxy indices for quick lookup.
    set: HashSet<u32>,
}

impl MovedProxies {
    /// Returns the moved proxy indices.
    #[inline]
    pub fn proxies(&self) -> &[u32] {
        &self.proxies
    }

    /// Returns `true` if the proxy with the given index has moved.
    #[inline]
    pub fn contains(&self, proxy_index: u32) -> bool {
        self.set.contains(&proxy_index)
    }

    /// Clears the moved proxies and sets the capacity of the internal structures.
    #[inline]
    pub fn clear_and_set_capacity(&mut self, capacity: usize) {
        self.bit_vec.set_bit_count_and_clear(capacity);

        self.thread_local_bit_vec.iter_mut().for_each(|context| {
            let bit_vec_mut = &mut context.borrow_mut();
            bit_vec_mut.set_bit_count_and_clear(capacity);
        });

        self.proxies.clear();
        self.set.clear();

        if self.proxies.capacity() < capacity {
            self.proxies.reserve(capacity - self.proxies.capacity());
        }
        if self.set.capacity() < capacity {
            self.set.reserve(capacity - self.set.capacity());
        }
    }

    /// Combines the thread-local moved proxy bit vectors into the main one.
    #[inline]
    pub fn combine_thread_local(&mut self) {
        let bit_vec = &mut self.bit_vec;
        self.thread_local_bit_vec.iter_mut().for_each(|context| {
            let thread_local_bit_vec = context.borrow();
            bit_vec.or(&thread_local_bit_vec);
        });
    }
}

fn update_dynamic_aabbs<C: AnyCollider>(
    mut colliders: ParamSet<(
        Query<(
            Entity,
            &C,
            &mut ColliderAabb,
            &mut EnlargedAabb,
            &ColliderTreeProxyIndex,
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
    time: Res<Time>,
    collider_context: StaticSystemParam<C::Context>,
    mut diagnostics: ResMut<BroadPhaseDiagnostics>,
) {
    let start = crate::utils::Instant::now();

    // An upper bound on the number of dynamic proxies, for sizing the bit vectors.
    // TODO: Use a better way to track the number of proxies.
    let max_num_dynamic_proxies = collider_trees.dynamic_tree.proxies.capacity();

    // Clear and resize the moved proxy structures.
    moved_proxies.clear_and_set_capacity(max_num_dynamic_proxies);

    let delta_secs = time.delta_seconds_adjusted();
    let default_speculative_margin = length_unit.0 * narrow_phase_config.default_speculative_margin;
    let contact_tolerance = length_unit.0 * narrow_phase_config.contact_tolerance;
    let margin = length_unit.0 * 0.05;

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
                    let mut thread_local_bit_vec = moved_proxies
                        .thread_local_bit_vec
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
    moved_proxies.combine_thread_local();

    // Serially enlarge moved proxies in the dynamic tree.
    let tree = &mut collider_trees.dynamic_tree;
    let aabbs = colliders.p1();

    let MovedProxies {
        bit_vec,
        proxies: moved_proxies,
        set: moved_set,
        ..
    } = &mut *moved_proxies;

    for (i, mut bits) in bit_vec.blocks().enumerate() {
        while bits != 0 {
            let trailing_zeros = bits.trailing_zeros();
            let proxy_index = i as u32 * 64 + trailing_zeros;
            let proxy = &mut tree.proxies[proxy_index as usize];
            let entity = proxy.entity;
            let (aabb, enlarged_aabb) = aabbs.get(entity).unwrap_or_else(|_| {
                panic!(
                    "EnlargedAabb missing for moved collider entity {:?}",
                    entity
                )
            });

            let aabb = Aabb::from(*aabb);
            let enlarged_aabb = Aabb::from(enlarged_aabb.get());

            // Update the proxy's AABB and enlarge it in the tree.
            proxy.aabb = aabb;
            tree.enlarge_proxy(proxy_index, enlarged_aabb);

            // Record the moved proxy.
            moved_proxies.push(proxy_index);
            moved_set.insert(proxy_index);

            // Clear the least significant set bit
            bits &= bits - 1;
        }
    }

    diagnostics.update += start.elapsed();
}

/// Begins an async full rebuild of the dynamic [`ColliderTree`] to optimize its structure.
///
/// This spawns an async task that performs the rebuild concurrently with the simulation step.
fn full_rebuild(
    mut collider_trees: ResMut<ColliderTrees>,
    mut tree_rebuild: ResMut<ColliderTreeRebuild>,
    mut diagnostics: ResMut<BroadPhaseDiagnostics>,
) {
    let start = crate::utils::Instant::now();

    let task_pool = AsyncComputeTaskPool::get();

    // Use the workspace from the current dynamic tree for the rebuild.
    // It will be swapped back when the rebuild is done.
    core::mem::swap(
        &mut collider_trees.dynamic_tree.workspace,
        &mut tree_rebuild.tree.workspace,
    );

    let mut tree = core::mem::take(&mut tree_rebuild.tree);

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

    tree_rebuild.task = Some(task);

    diagnostics.optimize += start.elapsed();
}

/// Completes the async rebuild of the collider tree started in [`full_rebuild`].
fn block_on_tree_rebuild(
    mut commands: Commands,
    mut collider_trees: ResMut<ColliderTrees>,
    mut tree_rebuild: ResMut<ColliderTreeRebuild>,
    mut diagnostics: ResMut<BroadPhaseDiagnostics>,
) {
    let start = crate::utils::Instant::now();

    if let Some(task) = &mut tree_rebuild.task {
        let mut command_queue = block_on(task);
        commands.append(&mut command_queue);
    }

    tree_rebuild.task = None;

    // Swap back the workspace to the dynamic tree.
    core::mem::swap(
        &mut collider_trees.dynamic_tree.workspace,
        &mut tree_rebuild.tree.workspace,
    );

    diagnostics.optimize += start.elapsed();
}
