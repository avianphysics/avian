//! A vector data structure that maintains stable indices for its elements.
//!
//! # Why Not `slab`?
//!
//! The `slab` crate provides a similar `Slab` data structure.
//! However, its insertion order is not necessarily preserved when elements are removed.
//! For example, after clearing the slab, the next element might not be inserted at index 0,
//! but rather it reuses some other slot. This can lead to seemingly non-deterministic behavior
//! when clearing and repopulating scenes, for example.
//!
//! This `StableVec` implementation instead always pushes elements to the first available slot
//! with the lowest index, ensuring that insertion order is preserved across removals and clear operations.
//!
//! # Why Not `stable_vec`?
//!
//! The `stable_vec` crate provides a similar `StableVec` data structure.
//! However, it doesn't actually reuse slots of removed elements, leading to unbounded memory growth
//! if elements are frequently added and removed. This makes it unsuitable for general-purpose use
//! for gameplay scenarios.
//!
//! This `StableVec` implementation reuses slots of removed elements, using the [`IdPool`] type
//! to track free indices.

use crate::data_structures::id_pool::IdPool;

/// A [`Vec<T>`]-like collection that maintains stable indices for its elements.
///
/// Unlike with a standard [`Vec<T>`], removing elements from a [`StableVec<T>`] is O(1),
/// and it does not move other elements or invalidate their indices.
/// This is achieved by internally storing each element as an [`Option<T>`],
/// and reusing freed slots for new elements.
///
/// # Overiew of Important Methods
///
/// - [`push`](Self::push) adds a new element and returns its stable index (O(1)).
/// - [`remove`](Self::remove) removes an element at a given index and returns it (O(1)).
/// - [`try_remove`](Self::try_remove) attempts to remove an element at a given index, returning `None` if it doesn't exist (O(1)).
/// - [`get`](Self::get) and [`get_mut`](Self::get_mut) provide access to elements by index (O(1)).
/// - [`clear`](Self::clear) removes all elements without deallocating memory (O(1)).
#[derive(Clone, Debug)]
pub struct StableVec<T> {
    data: Vec<Option<T>>,
    indices: IdPool,
}

impl<T> StableVec<T> {
    /// Creates a new empty [`StableVec`].
    #[inline(always)]
    pub const fn new() -> Self {
        Self {
            data: Vec::new(),
            indices: IdPool::new(),
        }
    }

    /// Creates a new [`StableVec`] with the given initial capacity.
    ///
    /// This is useful for preallocating space to avoid reallocations.
    #[inline(always)]
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            data: Vec::with_capacity(capacity),
            indices: IdPool::with_capacity(capacity),
        }
    }

    /// Pushes a new element to the first available slot, returning its index.
    ///
    /// This may reuse a previously freed slot.
    ///
    /// # Time Complexity
    ///
    /// O(1)
    #[inline(always)]
    pub fn push(&mut self, element: T) -> usize {
        let index = self.indices.alloc() as usize;

        if index >= self.data.len() {
            self.data.push(Some(element));
        } else {
            self.data[index] = Some(element);
        }

        index
    }

    /// Returns the next index that will be used for a push.
    ///
    /// This index may reuse a previously freed slot.
    ///
    /// # Time Complexity
    ///
    /// O(1)
    #[inline(always)]
    pub fn next_push_index(&self) -> usize {
        self.indices.next_id() as usize
    }

    /// Removes the element at the given index, returning it.
    ///
    /// # Time Complexity
    ///
    /// O(1)
    ///
    /// # Panics
    ///
    /// Panics if there is no element at the given index.
    #[inline(always)]
    pub fn remove(&mut self, index: usize) -> T {
        self.try_remove(index).unwrap_or_else(|| {
            panic!("no element at index {} in StableVec::remove", index);
        })
    }

    /// Tries to remove the element at the given index, returning it if it existed.
    ///
    /// # Time Complexity
    ///
    /// O(1)
    #[inline(always)]
    pub fn try_remove(&mut self, index: usize) -> Option<T> {
        if index >= self.data.len() {
            return None;
        }
        let element = self.data[index].take();
        if element.is_some() {
            self.indices.free(index as u32);
        }
        element
    }

    /// Removes all elements from the [`StableVec`].
    ///
    /// No memory is deallocated, so the capacity remains the same.
    ///
    /// # Time Complexity
    ///
    /// O(1)
    #[inline(always)]
    pub fn clear(&mut self) {
        self.data.clear();
        self.indices.clear();
    }

    /// Returns a reference to the element at the given index, if it exists.
    ///
    /// # Time Complexity
    ///
    /// O(1)
    #[inline(always)]
    pub fn get(&self, index: usize) -> Option<&T> {
        self.data.get(index)?.as_ref()
    }

    /// Returns a mutable reference to the element at the given index, if it exists.
    ///
    /// # Time Complexity
    ///
    /// O(1)
    #[inline(always)]
    pub fn get_mut(&mut self, index: usize) -> Option<&mut T> {
        self.data.get_mut(index)?.as_mut()
    }

    /// Returns a reference to the element at the given index without bounds checking.
    ///
    /// # Time Complexity
    ///
    /// O(1)
    ///
    /// # Safety
    ///
    /// The caller must ensure that the index is in bounds
    /// and that there is an element at that index.
    #[inline(always)]
    pub unsafe fn get_unchecked(&self, index: usize) -> &T {
        unsafe { self.data.get_unchecked(index).as_ref().unwrap_unchecked() }
    }

    /// Returns a mutable reference to the element at the given index without bounds checking.
    ///
    /// # Time Complexity
    ///
    /// O(1)
    ///
    /// # Safety
    ///
    /// The caller must ensure that the index is in bounds
    /// and that there is an element at that index.
    #[inline(always)]
    pub unsafe fn get_unchecked_mut(&mut self, index: usize) -> &mut T {
        unsafe {
            self.data
                .get_unchecked_mut(index)
                .as_mut()
                .unwrap_unchecked()
        }
    }

    /// Returns mutable references to two disjoint elements at the given indices, if they exist.
    ///
    /// If the indices are the same, or if either index is out of bounds or has no element,
    /// `None` is returned.
    ///
    /// # Time Complexity
    ///
    /// O(1)
    #[inline(always)]
    pub fn get_disjoint_mut2(&mut self, index1: usize, index2: usize) -> Option<[&mut T; 2]> {
        // TODO: Return a `Result`.
        let [first, second] = self.data.get_disjoint_mut([index1, index2]).ok()?;
        Some([first.as_mut()?, second.as_mut()?])
    }

    /// Returns mutable references to two disjoint elements at the given indices without bounds checking.
    ///
    /// If the indices are the same, or if either index has no element,
    /// the behavior is undefined.
    ///
    /// # Time Complexity
    ///
    /// O(1)
    ///
    /// # Safety
    ///
    /// The caller must ensure that the indices are disjoint
    /// and that there are elements at both indices.
    #[inline(always)]
    pub unsafe fn get_disjoint_mut_unchecked(
        &mut self,
        index1: usize,
        index2: usize,
    ) -> [&mut T; 2] {
        // TODO: Return a `Result`.
        unsafe {
            let [first, second] = self.data.get_disjoint_unchecked_mut([index1, index2]);
            [
                first.as_mut().unwrap_unchecked(),
                second.as_mut().unwrap_unchecked(),
            ]
        }
    }

    /// Returns the number of elements in the [`StableVec`].
    ///
    /// # Time Complexity
    ///
    /// O(1)
    #[inline(always)]
    pub fn len(&self) -> usize {
        self.indices.len()
    }

    /// Returns `true` if the [`StableVec`] is empty.
    ///
    /// # Time Complexity
    ///
    /// O(1)
    #[inline(always)]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Returns the capacity of the [`StableVec`].
    ///
    /// # Time Complexity
    ///
    /// O(1)
    #[inline(always)]
    pub fn capacity(&self) -> usize {
        self.data.capacity()
    }

    /// Returns an iterator over the elements and indices of the [`StableVec`].
    #[inline(always)]
    pub fn iter(&self) -> impl Iterator<Item = (usize, &T)> {
        self.data
            .iter()
            .enumerate()
            .filter_map(|(i, opt)| opt.as_ref().map(|v| (i, v)))
    }

    /// Returns a mutable iterator over the elements and indices of the [`StableVec`].
    #[inline(always)]
    pub fn iter_mut(&mut self) -> impl Iterator<Item = (usize, &mut T)> {
        self.data
            .iter_mut()
            .enumerate()
            .filter_map(|(i, opt)| opt.as_mut().map(|v| (i, v)))
    }
}

impl<T> Default for StableVec<T> {
    /// Creates a new empty [`StableVec`].
    #[inline(always)]
    fn default() -> Self {
        Self::new()
    }
}

impl<T> core::ops::Index<usize> for StableVec<T> {
    type Output = T;

    /// Returns a reference to the element at the given index.
    ///
    /// # Time Complexity
    ///
    /// O(1)
    ///
    /// # Panics
    ///
    /// Panics if there is no element at the given index.
    #[inline(always)]
    fn index(&self, index: usize) -> &Self::Output {
        self.get(index).unwrap_or_else(|| {
            panic!("no element at index {} in StableVec::index", index);
        })
    }
}

impl<T> core::ops::IndexMut<usize> for StableVec<T> {
    /// Returns a mutable reference to the element at the given index.
    ///
    /// # Time Complexity
    ///
    /// O(1)
    ///
    /// # Panics
    ///
    /// Panics if there is no element at the given index.
    #[inline(always)]
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        self.get_mut(index).unwrap_or_else(|| {
            panic!("no element at index {} in StableVec::index_mut", index);
        })
    }
}

impl<T> IntoIterator for StableVec<T> {
    type Item = T;
    type IntoIter = StableVecIntoIterator<T>;

    #[inline(always)]
    fn into_iter(self) -> Self::IntoIter {
        StableVecIntoIterator {
            inner: self.data.into_iter(),
        }
    }
}

/// An iterator over the elements of a [`StableVec`].
pub struct StableVecIntoIterator<T> {
    inner: alloc::vec::IntoIter<Option<T>>,
}

impl<T> Iterator for StableVecIntoIterator<T> {
    type Item = T;

    #[inline(always)]
    fn next(&mut self) -> Option<Self::Item> {
        self.inner.by_ref().flatten().next()
    }
}

impl<T> DoubleEndedIterator for StableVecIntoIterator<T> {
    #[inline(always)]
    fn next_back(&mut self) -> Option<Self::Item> {
        self.inner.by_ref().flatten().next_back()
    }
}

impl<T> core::iter::FusedIterator for StableVecIntoIterator<T> {}

#[cfg(test)]
mod tests {
    use super::StableVec;

    #[test]
    fn test_stable_vec_push_remove() {
        let mut sv = StableVec::new();
        let idx1 = sv.push(10);
        let idx2 = sv.push(20);

        assert_eq!(sv.len(), 2);
        assert_eq!(sv[idx1], 10);
        assert_eq!(sv[idx2], 20);

        let val = sv.remove(idx1);
        assert_eq!(val, 10);
        assert_eq!(sv.len(), 1);
        assert!(sv.get(idx1).is_none());

        let idx3 = sv.push(30);
        assert_eq!(idx3, idx1); // Reused index
        assert_eq!(sv[idx3], 30);
    }

    #[test]
    fn test_stable_vec_clear() {
        let mut sv = StableVec::new();
        sv.push(1);
        sv.push(2);
        sv.clear();
        assert_eq!(sv.len(), 0);
    }

    #[test]
    fn test_stable_vec_iter() {
        let mut sv = StableVec::new();
        sv.push(1);
        sv.push(2);
        sv.push(3);

        let collected: Vec<_> = sv.into_iter().collect();
        assert_eq!(collected, vec![1, 2, 3]);
    }
}
