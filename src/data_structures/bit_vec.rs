//! A minimalistic dynamically sized compact bit vector with a fixed block size of 64 bits.
//!
//! Only a very limited set of operations are supported.

use core::ops::BitOrAssign;
use core::slice;

use bevy::reflect::Reflect;
#[cfg(feature = "serialize")]
use bevy::reflect::{ReflectDeserialize, ReflectSerialize};

/// A dynamically sized compact bit vector with a fixed block size of 64 bits.
#[derive(Clone, Debug, Default, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug)]
pub struct BitVec {
    blocks: Vec<u64>,
    block_capacity: usize,
    block_count: usize,
}

#[inline]
fn bits_to_blocks(bits: usize) -> usize {
    bits.div_ceil(u64::BITS as usize)
}

impl BitVec {
    /// Creates a new [`BitVec`] with the specified bit capacity.
    #[inline]
    pub fn new(bit_capacity: usize) -> Self {
        let block_capacity = bits_to_blocks(bit_capacity);

        Self {
            blocks: vec![0; block_capacity * size_of::<u64>()],
            block_capacity,
            block_count: 0,
        }
    }

    /// Sets the bit count of the [`BitVec`] and clears all bits.
    ///
    /// If the new bit count exceeds the current block capacity, the block capacity is increased.
    #[inline]
    pub fn set_bit_count_and_clear(&mut self, bit_count: usize) {
        let block_count = bits_to_blocks(bit_count);

        if self.block_capacity < block_count {
            let new_bit_capacity = bit_count + (bit_count >> 1);
            *self = Self::new(new_bit_capacity);
        }

        self.block_count = block_count;
        self.blocks.iter_mut().for_each(|b| *b = 0);
    }

    /// Sets the bit at the specified index.
    ///
    /// # Panics
    ///
    /// Panics if the new bit count exceeds the current block capacity with `debug_assertions` enabled.
    #[inline]
    pub fn set(&mut self, index: usize) {
        let block_index = index / 64;
        debug_assert!(
            block_index < self.block_count,
            "block index out of bounds for `BitVec::set` ({} >= {})",
            block_index,
            self.block_count
        );
        let bit_index = index % 64;
        let mask = 1 << bit_index;
        self.blocks[block_index] |= mask;
    }

    /// Sets the specified bit and grows the [`BitVec`] if necessary.
    #[inline]
    pub fn set_and_grow(&mut self, index: usize) {
        let block_index = index / 64;

        if block_index >= self.block_count {
            self.grow(block_index + 1);
        }

        let bit_index = index % 64;
        let mask = 1 << bit_index;
        self.blocks[block_index] |= mask;
    }

    /// Resizes the [`BitVec`] to the specified number of blocks.
    ///
    /// If the new block count exceeds the current capacity, the capacity is increased by half.
    /// The blocks are not cleared.
    #[inline]
    pub fn grow(&mut self, new_block_count: usize) {
        if new_block_count > self.block_capacity {
            // Increase the block capacity by half if the new block count
            // exceeds the current capacity.
            let new_block_capacity = new_block_count + new_block_count / 2;
            self.blocks.resize(new_block_capacity, 0);
            self.block_capacity = new_block_capacity;
        }

        self.block_count = new_block_count;
    }

    /// Unsets the bit at the specified index.
    #[inline]
    pub fn unset(&mut self, index: usize) {
        let block_index = index / 64;

        if block_index >= self.block_count {
            return;
        }

        let bit_index = index % 64;
        let mask = 1 << bit_index;
        self.blocks[block_index] &= !mask;
    }

    /// Gets the bit at the specified index.
    ///
    /// Returns `false` if the index is out of bounds or the bit is unset.
    #[inline]
    pub fn get(&self, index: usize) -> bool {
        let block_index = index / 64;

        if block_index >= self.block_count {
            return false;
        }

        let bit_index = index % 64;
        let mask = 1 << bit_index;
        (self.blocks[block_index] & mask) != 0
    }

    /// Returns the block capacity of the [`BitVec`].
    #[inline]
    pub fn block_capacity(&self) -> usize {
        self.block_capacity
    }

    /// Returns the block count of the [`BitVec`].
    #[inline]
    pub fn block_count(&self) -> usize {
        self.block_count
    }

    /// Returns the number of set bits in the [`BitVec`].
    #[inline]
    pub fn count_ones(&self) -> usize {
        self.blocks.iter().map(|&b| b.count_ones() as usize).sum()
    }

    /// Returns the number of unset bits in the [`BitVec`].
    #[inline]
    pub fn count_zeros(&self) -> usize {
        self.block_count * 64 - self.count_ones()
    }

    /// Clears all bits in the [`BitVec`].
    #[inline]
    pub fn clear(&mut self) {
        self.blocks.iter_mut().for_each(|b| *b = 0);
    }

    /// Returns an iterator over the blocks of the [`BitVec`].
    #[inline]
    pub fn blocks(&self) -> Blocks<'_> {
        Blocks {
            iter: self.blocks.iter(),
        }
    }

    /// Performs an in-place bitwise OR operation with another [`BitVec`].
    #[inline]
    pub fn or(&mut self, other: &Self) {
        debug_assert!(
            self.block_count == other.block_count,
            "block counts do not match for `BitVec::or` ({} != {})",
            self.block_count,
            other.block_count
        );

        for i in 0..self.block_count {
            self.blocks[i] |= other.blocks[i];
        }
    }
}

impl BitOrAssign<&BitVec> for BitVec {
    #[inline]
    fn bitor_assign(&mut self, rhs: &BitVec) {
        self.or(rhs);
    }
}

/// An iterator over the blocks of a [`BitVec`].
#[derive(Clone)]
pub struct Blocks<'a> {
    iter: slice::Iter<'a, u64>,
}

impl Iterator for Blocks<'_> {
    type Item = u64;

    #[inline]
    fn next(&mut self) -> Option<u64> {
        self.iter.next().cloned()
    }

    #[inline]
    fn size_hint(&self) -> (usize, Option<usize>) {
        self.iter.size_hint()
    }
}

impl DoubleEndedIterator for Blocks<'_> {
    #[inline]
    fn next_back(&mut self) -> Option<u64> {
        self.iter.next_back().cloned()
    }
}

impl ExactSizeIterator for Blocks<'_> {}
