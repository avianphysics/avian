use crate::{
    collider_tree::{
        ColliderTree, ColliderTreeDiagnostics, ColliderTreeSystems, ColliderTreeType, ColliderTrees,
    },
    data_structures::stable_vec::StableVec,
    prelude::*,
};
use bevy::{
    ecs::world::CommandQueue,
    prelude::*,
    tasks::{AsyncComputeTaskPool, Task, block_on},
};

/// A plugin that optimizes the dynamic [`ColliderTree`] to maintain good query performance.
pub(super) struct ColliderTreeOptimizationPlugin;

impl Plugin for ColliderTreeOptimizationPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ColliderTreeOptimization>()
            .init_resource::<OptimizationTasks>();

        app.add_systems(
            PhysicsSchedule,
            (
                optimize_trees.in_set(ColliderTreeSystems::BeginOptimize),
                #[cfg(all(not(target_arch = "wasm32"), not(target_os = "unknown")))]
                block_on_optimize_trees.in_set(ColliderTreeSystems::EndOptimize),
            ),
        );
    }
}

/// Settings for optimizing each [`ColliderTree`].
// TODO: Per-tree settings could be useful.
#[derive(Resource, Debug, PartialEq, Reflect)]
pub struct ColliderTreeOptimization {
    /// The optimization mode for the collider tree.
    ///
    /// **Default**: [`TreeOptimizationMode::Adaptive`]
    pub optimization_mode: TreeOptimizationMode,

    /// If `true`, tree optimization will be performed in-place with minimal allocations.
    /// This has the downside that the tree will be unavailable for [spatial queries]
    /// during the simulation step while the optimization is ongoing (ex: in [collision hooks]).
    ///
    /// Otherwise, parts of the the tree will be cloned for the optimization,
    /// allowing spatial queries to use the old tree during the simulation step,
    /// but incurring additional memory allocation overhead.
    ///
    /// For optimal performance, set this to `true` if your application
    /// does not perform spatial queries during the simulation step.
    ///
    /// **Default**: `false`
    ///
    /// [spatial queries]: crate::spatial_query
    /// [collision hooks]: crate::collision::hooks
    pub optimize_in_place: bool,

    /// If `true`, tree optimization will be performed in parallel
    /// with the narrow phase and solver using async tasks.
    ///
    /// **Default**: `true` (on supported platforms)
    pub use_async_tasks: bool,
}

impl Default for ColliderTreeOptimization {
    fn default() -> Self {
        Self {
            optimization_mode: TreeOptimizationMode::default(),
            optimize_in_place: false,
            #[cfg(any(target_arch = "wasm32", target_os = "unknown"))]
            use_async_tasks: false,
            #[cfg(all(not(target_arch = "wasm32"), not(target_os = "unknown")))]
            use_async_tasks: true,
        }
    }
}

/// The optimization mode for a [`ColliderTree`].
#[derive(Clone, Copy, Debug, PartialEq, Reflect)]
pub enum TreeOptimizationMode {
    /// The tree is optimized by reinserting proxies whose AABB in the tree has changed.
    ///
    /// This is the fastest method when only a small portion of proxies have moved,
    /// but is less effective for large numbers of moved proxies.
    Reinsert,

    /// The tree is optimized by performing a partial rebuild that only rebuilds
    /// parts of the tree affected by proxies that have moved.
    ///
    /// This method is more effective than reinsertion when a moderate number of proxies
    /// have moved. However, if a large portion of proxies have moved, a full rebuild
    /// can be more effective and have less overhead.
    PartialRebuild,

    /// The tree is optimized by performing a full rebuild.
    ///
    /// This method can produce the highest quality tree, and can have less overhead
    /// than other methods when a large portion of proxies have moved.
    /// This makes it suitable for highly dynamic scenes.
    FullRebuild,

    /// The tree is optimized adaptively based on how many proxies have moved.
    ///
    /// - If the ratio of moved proxies to total proxies is below
    ///   `reinsert_threshold`, [`Reinsert`](TreeOptimizationMode::Reinsert) is used.
    /// - If the ratio is between `reinsert_threshold` and `partial_rebuild_threshold`,
    ///   [`PartialRebuild`](TreeOptimizationMode::PartialRebuild) is used.
    /// - Otherwise, [`FullRebuild`](TreeOptimizationMode::FullRebuild) is used.
    ///
    /// This is the default mode.
    Adaptive {
        /// The threshold ratio of moved proxies to total proxies
        /// below which reinsertion is performed.
        ///
        /// **Default**: `0.15`
        reinsert_threshold: f32,

        /// The threshold ratio of moved proxies to total proxies
        /// below which a partial rebuild is performed.
        ///
        /// **Default**: `0.45`
        partial_rebuild_threshold: f32,
    },
}

impl Default for TreeOptimizationMode {
    fn default() -> Self {
        TreeOptimizationMode::Adaptive {
            reinsert_threshold: 0.15,
            partial_rebuild_threshold: 0.45,
        }
    }
}

impl TreeOptimizationMode {
    /// Resolves the optimization mode based on the ratio of moved proxies.
    ///
    /// `moved_ratio` is the ratio of moved proxies to total proxies in the tree.
    #[inline]
    pub fn resolve(&self, moved_ratio: f32) -> TreeOptimizationMode {
        match self {
            TreeOptimizationMode::Adaptive {
                reinsert_threshold,
                partial_rebuild_threshold,
            } => {
                if moved_ratio < *reinsert_threshold {
                    TreeOptimizationMode::Reinsert
                } else if moved_ratio < *partial_rebuild_threshold {
                    TreeOptimizationMode::PartialRebuild
                } else {
                    TreeOptimizationMode::FullRebuild
                }
            }
            other => *other,
        }
    }
}

/// A resource tracking ongoing optimization tasks for [`ColliderTree`]s.
#[derive(Resource, Default, Deref, DerefMut)]
struct OptimizationTasks(Vec<Task<CommandQueue>>);

/// Begins optimizing the dynamic and kinematic [`ColliderTree`]s to maintain good query performance.
///
/// If [`ColliderTreeOptimization::use_async_tasks`] is enabled, this spawns an async task
/// that runs concurrently with the simulation step. Otherwise, the optimization is performed
/// in-place on the main thread.
fn optimize_trees(
    mut collider_trees: ResMut<ColliderTrees>,
    mut optimization_tasks: ResMut<OptimizationTasks>,
    optimization_settings: Res<ColliderTreeOptimization>,
    mut diagnostics: ResMut<ColliderTreeDiagnostics>,
) {
    let start = crate::utils::Instant::now();

    let task_pool = AsyncComputeTaskPool::get();

    // We cannot block on wasm.
    #[cfg(any(target_arch = "wasm32", target_os = "unknown"))]
    let use_async_tasks = false;
    #[cfg(all(not(target_arch = "wasm32"), not(target_os = "unknown")))]
    let use_async_tasks = optimization_settings.use_async_tasks;

    // Spawn optimization tasks for each tree.
    for tree_type in ColliderTreeType::ALL {
        let tree = collider_trees.tree_for_type_mut(tree_type);

        let moved_ratio = tree.moved_proxies.len() as f32 / tree.proxies.len() as f32;
        let optimization_strategy = optimization_settings.optimization_mode.resolve(moved_ratio);

        if moved_ratio == 0.0 && optimization_strategy != TreeOptimizationMode::FullRebuild {
            // No moved proxies, no need to optimize.
            continue;
        }

        #[cfg(all(not(target_arch = "wasm32"), not(target_os = "unknown")))]
        if use_async_tasks {
            // Take or clone the BVH for the optimization task.
            // TODO: For small changes to large trees, the cost of cloning can exceed the cost of the async task.
            //       We could have a threshold for cloning vs in-place optimization based on tree size and moved ratio.
            let bvh = if optimization_settings.optimize_in_place {
                core::mem::take(&mut tree.bvh)
            } else {
                // TODO: Can we avoid cloning the entire BVH?
                tree.bvh.clone()
            };

            // Create a new tree for the optimization task.
            let new_tree = ColliderTree {
                bvh,
                proxies: StableVec::new(),
                // These are not needed during the simulation step.
                moved_proxies: core::mem::take(&mut tree.moved_proxies),
                workspace: core::mem::take(&mut tree.workspace),
            };

            let task = spawn_optimization_task(task_pool, new_tree, tree_type, move |tree| {
                optimize_tree_in_place(tree, optimization_strategy);
            });

            optimization_tasks.push(task);
        }

        if !use_async_tasks {
            // Optimize in place on the main thread.
            optimize_tree_in_place(tree, optimization_strategy);
        }
    }

    diagnostics.optimize += start.elapsed();
}

fn optimize_tree_in_place(tree: &mut ColliderTree, optimization_strategy: TreeOptimizationMode) {
    match optimization_strategy {
        TreeOptimizationMode::Reinsert => {
            let moved_leaves = tree
                .moved_proxies
                .iter()
                .map(|key| tree.bvh.primitives_to_nodes[key.index()])
                .collect::<Vec<u32>>();

            tree.optimize_candidates(&moved_leaves, 1);
        }
        TreeOptimizationMode::PartialRebuild => {
            let moved_leaves = tree
                .moved_proxies
                .iter()
                .map(|key| tree.bvh.primitives_to_nodes[key.index()])
                .collect::<Vec<u32>>();

            tree.rebuild_partial(&moved_leaves);
        }
        TreeOptimizationMode::FullRebuild => {
            tree.rebuild_full();
        }

        TreeOptimizationMode::Adaptive { .. } => unreachable!(),
    }
}

/// Spawns and returns an async task to optimize the given collider tree
/// using the provided optimization function.
#[cfg(all(not(target_arch = "wasm32"), not(target_os = "unknown")))]
fn spawn_optimization_task(
    task_pool: &AsyncComputeTaskPool,
    mut tree: ColliderTree,
    tree_type: ColliderTreeType,
    optimize: impl FnOnce(&mut ColliderTree) + Send + 'static,
) -> Task<CommandQueue> {
    task_pool.spawn(async move {
        optimize(&mut tree);

        let mut command_queue = CommandQueue::default();
        command_queue.push(move |world: &mut World| {
            let mut collider_trees = world
                .get_resource_mut::<ColliderTrees>()
                .expect("ColliderTrees resource missing");
            let collider_tree = collider_trees.tree_for_type_mut(tree_type);
            collider_tree.bvh = tree.bvh;
            collider_tree.workspace = tree.workspace;
        });
        command_queue
    })
}

/// Completes the [`ColliderTree`] optimization tasks started in [`optimize_trees`].
#[cfg(all(not(target_arch = "wasm32"), not(target_os = "unknown")))]
fn block_on_optimize_trees(
    mut commands: Commands,
    mut optimization: ResMut<OptimizationTasks>,
    mut diagnostics: ResMut<ColliderTreeDiagnostics>,
) {
    let start = crate::utils::Instant::now();

    // Complete all ongoing optimization tasks.
    optimization.drain(..).for_each(|task| {
        let mut command_queue = block_on(task);
        commands.append(&mut command_queue);
    });

    diagnostics.optimize += start.elapsed();
}
