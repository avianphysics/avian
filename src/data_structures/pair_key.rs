//! A unique key for a pair of identifiers.

use bevy::prelude::*;

/// A unique key for a pair of identifiers.
///
/// This can be used for efficient storage and lookup of pairs of entities or other objects.
#[derive(Clone, Copy, Debug, Deref, DerefMut, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct PairKey(pub u64);

impl PairKey {
    /// Creates a new pair key from two IDs.
    #[inline]
    pub const fn new(id1: u32, id2: u32) -> Self {
        if id1 < id2 {
            Self(((id1 as u64) << 32) | id2 as u64)
        } else {
            Self(((id2 as u64) << 32) | id1 as u64)
        }
    }

    /// Gets the two IDs stored in the pair key in ascending order.
    #[inline]
    pub fn get(&self) -> (u32, u32) {
        (
            ((self.0 >> 32) & 0xFFFF_FFFF) as u32,
            (self.0 & 0xFFFF_FFFF) as u32,
        )
    }
}
