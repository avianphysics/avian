//! Miscallaneous utility functions.

pub(crate) use bevy::platform::time::Instant;

/// A helper macro for iterating over a slice in parallel or serially
/// based on the `parallel` feature.
///
/// The macro takes an expression that evaluates to a mutable slice,
/// and a closure that takes two arguments: the index and the item at the index.
///
/// The `ComputeTaskPool` is used if parallelism is enabled.
///
/// # Example
///
/// ```ignore
/// let mut slice = vec![1, 2, 3, 4];
///
/// par_for_each!(slice, |index, item| {
///     *item += index;
/// });
///
/// assert_eq!(slice, vec![1, 3, 5, 7]);
/// ```
macro_rules! par_for_each {
    ($expression:expr, |$index:ident, $item:ident $(,)?| $body:block) => {
        #[cfg(not(feature = "parallel"))]
        $expression
            .iter_mut()
            .enumerate()
            .for_each(|($index, $item)| $body);

        #[cfg(feature = "parallel")]
        {
            let task_pool_ = bevy::tasks::ComputeTaskPool::get();

            if task_pool_.thread_num() == 1 {
                $expression
                    .iter_mut()
                    .enumerate()
                    .for_each(|($index, $item)| $body);
            } else {
                // TODO: Is there a better approach than `par_chunk_map_mut`?
                let chunk_size_ = ($expression.len() / task_pool_.thread_num()).max(1);
                bevy::tasks::ParallelSliceMut::par_chunk_map_mut(
                    &mut $expression,
                    task_pool_,
                    chunk_size_,
                    |chunk_index_, chunk_| {
                        let index_offset_ = chunk_index_ * chunk_size_;
                        chunk_.iter_mut().enumerate().for_each(|(index_, $item)| {
                            let $index = index_offset_ + index_;
                            $body
                        });
                    },
                );
            }
        }
    };
}

pub(crate) use par_for_each;
