//! Miscallaneous utility functions.

use bevy::ecs::{
    query::{IterQueryData, QueryFilter, QueryItem, ROQueryItem, ReadOnlyQueryData},
    system::Query,
};

pub(crate) use bevy::platform::time::Instant;

/// A conservative default minimum number of matched entities required before query iteration
/// switches from serial to parallel. This is used by the [`ParallelQueryForEach`] extension trait
/// and its [`par_for_each`] and [`par_for_each_mut`] methods.
///
/// This threshold was chosen by measuring the rough entity count where parallel iteration
/// outperforms serial iteration for cheap per-entity workloads such as position integration.
///
/// [`par_for_each`]: ParallelQueryForEach::par_for_each
/// [`par_for_each_mut`]: ParallelQueryForEach::par_for_each_mut
pub(crate) const MIN_PAR_ITER_ENTITIES: usize = 4096;

/// An extension trait for [`Query`] that chooses between serial and parallel
/// iteration based on the number of matched entities.
///
/// # Note
///
/// The entity count is determined with [`Query::count`], which is cheap for
/// archetypal queries but iterates the query for non-archetypal ones.
pub trait ParallelQueryForEach<'w, 's, D: IterQueryData, F: QueryFilter> {
    /// Iterates over the read-only query items, calling `f` for each one either
    /// serially or in parallel depending on `min_parallel_len`.
    ///
    /// See the [trait-level documentation](ParallelQueryForEach) for details.
    fn par_for_each<FN>(&self, min_parallel_len: usize, f: FN)
    where
        D: ReadOnlyQueryData,
        FN: for<'a> Fn(ROQueryItem<'a, 's, D>) + Send + Sync + Clone;

    /// Iterates over the mutable query items, calling `f` for each one either
    /// serially or in parallel depending on `min_parallel_len`.
    ///
    /// See the [trait-level documentation](ParallelQueryForEach) for details.
    fn par_for_each_mut<FN>(&mut self, min_parallel_len: usize, f: FN)
    where
        FN: for<'a> Fn(QueryItem<'a, 's, D>) + Send + Sync + Clone;
}

impl<'w, 's, D: IterQueryData, F: QueryFilter> ParallelQueryForEach<'w, 's, D, F>
    for Query<'w, 's, D, F>
{
    #[inline(always)]
    #[allow(unused_variables)]
    fn par_for_each<FN>(&self, min_parallel_len: usize, f: FN)
    where
        D: ReadOnlyQueryData,
        FN: for<'a> Fn(ROQueryItem<'a, 's, D>) + Send + Sync + Clone,
    {
        #[cfg(feature = "parallel")]
        {
            let task_pool = bevy::tasks::ComputeTaskPool::get();
            if task_pool.thread_num() > 1 && self.count() >= min_parallel_len {
                self.par_iter().for_each(f);
                return;
            }
        }
        self.iter().for_each(f);
    }

    #[inline(always)]
    #[allow(unused_variables)]
    fn par_for_each_mut<FN>(&mut self, min_parallel_len: usize, f: FN)
    where
        FN: for<'a> Fn(QueryItem<'a, 's, D>) + Send + Sync + Clone,
    {
        #[cfg(feature = "parallel")]
        {
            let task_pool = bevy::tasks::ComputeTaskPool::get();
            if task_pool.thread_num() > 1 && self.count() >= min_parallel_len {
                self.par_iter_mut().for_each(f);
                return;
            }
        }
        self.iter_mut().for_each(f);
    }
}

// TODO: The single-threaded and multi-threaded versions are duplicated here because
//       of the different trait bounds on `F`. Unify them somehow?

/// A helper function for iterating over a slice in parallel or serially
/// based on the `parallel` feature.
///
/// If `slice.len() < min_len`, serial iteration will be used.
///
/// The `ComputeTaskPool` is used if parallelism is enabled.
///
/// # Example
///
/// ```ignore
/// let mut slice = vec![1, 2, 3, 4];
///
/// par_for_each(&mut slice, |index, item| {
///     *item += index;
/// });
///
/// assert_eq!(slice, vec![1, 3, 5, 7]);
/// ```
#[inline(always)]
#[allow(unused_variables, unused_mut)]
#[cfg(not(feature = "parallel"))]
pub fn par_for_each<T, F>(mut slice: &mut [T], min_len: usize, mut f: F)
where
    T: Send + Sync,
    F: FnMut(usize, &mut T) + Send + Sync,
{
    slice.iter_mut().enumerate().for_each(|(index, item)| {
        f(index, item);
    });
}

/// A helper function for iterating over a slice in parallel or serially
/// based on the `parallel` feature.
///
/// If `slice.len() < min_len`, serial iteration will be used.
///
/// The `ComputeTaskPool` is used if parallelism is enabled.
///
/// # Example
///
/// ```ignore
/// let mut slice = vec![1, 2, 3, 4];
///
/// par_for_each(&mut slice, |index, item| {
///     *item += index;
/// });
///
/// assert_eq!(slice, vec![1, 3, 5, 7]);
/// ```
#[inline(always)]
#[allow(unused_variables, unused_mut)]
#[cfg(feature = "parallel")]
pub fn par_for_each<T, F>(mut slice: &mut [T], min_len: usize, mut f: F)
where
    T: Send + Sync,
    F: Fn(usize, &mut T) + Send + Sync,
{
    let task_pool_ = bevy::tasks::ComputeTaskPool::get();

    if task_pool_.thread_num() == 1 || slice.len() < min_len {
        slice.iter_mut().enumerate().for_each(|(index, item)| {
            f(index, item);
        });
    } else {
        // TODO: Is there a better approach than `par_chunk_map_mut`?
        let chunk_size_ = (slice.len() / task_pool_.thread_num()).max(1);
        bevy::tasks::ParallelSliceMut::par_chunk_map_mut(
            &mut slice,
            task_pool_,
            chunk_size_,
            |chunk_index_, chunk_| {
                let index_offset_ = chunk_index_ * chunk_size_;
                chunk_.iter_mut().enumerate().for_each(|(i, item)| {
                    let index = index_offset_ + i;
                    f(index, item);
                });
            },
        );
    }
}
