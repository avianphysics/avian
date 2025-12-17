//! A map for associating data with previously stored entities in a generational arena.
//!
//! This is an adaptation of [`slotmap::SparseSecondaryMap`], tailored for Avian.
//! Some modifications include:
//!
//! - The key is always an [`Entity`] instead of a generic key type.
//! - Much more minimalistic. No entry API or iterators.
//! - `no_std` compatible.
//!
//! [`slotmap::SparseSecondaryMap`]: https://docs.rs/slotmap/1.0.7/slotmap/struct.SparseSecondaryMap.html

use alloc::collections::TryReserveError;
use bevy::platform::hash::RandomState;
use core::mem::MaybeUninit;
use std::collections::hash_map::{self, HashMap};
use std::hash;

use bevy::ecs::entity::{Entity, EntityGeneration};

#[derive(Debug, Clone)]
struct Slot<T> {
    generation: EntityGeneration,
    value: T,
}

/// Sparse secondary map for associating data with previously stored entities
/// in a generational arena.
#[derive(Debug, Clone)]
pub struct SparseSecondaryEntityMap<V, S: hash::BuildHasher = RandomState> {
    slots: HashMap<u32, Slot<V>, S>,
}

impl<V> SparseSecondaryEntityMap<V, hash_map::RandomState> {
    /// Constructs a new, empty [`SparseSecondaryEntityMap`].
    #[inline]
    pub fn new() -> Self {
        Self::with_capacity(0)
    }

    /// Creates an empty [`SparseSecondaryEntityMap`] with the given capacity of slots.
    ///
    /// The secondary map will not reallocate until it holds at least `capacity`
    /// slots.
    #[inline]
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            slots: HashMap::with_capacity(capacity),
        }
    }
}

/// Returns if a is an older generation than b, taking into account wrapping of
/// generations.
fn is_older_generation(a: u32, b: u32) -> bool {
    let diff = a.wrapping_sub(b);
    diff >= (1 << 31)
}

impl<V, S: hash::BuildHasher> SparseSecondaryEntityMap<V, S> {
    /// Creates an empty [`SparseSecondaryEntityMap`] which will use the given hash
    /// builder to hash keys.
    ///
    /// The secondary map will not reallocate until it holds at least `capacity`
    /// slots.
    #[inline]
    pub fn with_hasher(hash_builder: S) -> Self {
        Self {
            slots: HashMap::with_hasher(hash_builder),
        }
    }

    /// Creates an empty [`SparseSecondaryEntityMap`] with the given capacity of slots,
    /// using `hash_builder` to hash the keys.
    ///
    /// The secondary map will not reallocate until it holds at least `capacity`
    /// slots.
    #[inline]
    pub fn with_capacity_and_hasher(capacity: usize, hash_builder: S) -> Self {
        Self {
            slots: HashMap::with_capacity_and_hasher(capacity, hash_builder),
        }
    }

    /// Returns the number of elements in the secondary map.
    #[inline]
    pub fn len(&self) -> usize {
        self.slots.len()
    }

    /// Returns if the secondary map is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.slots.is_empty()
    }

    /// Returns the number of elements the [`SparseSecondaryEntityMap`] can hold without
    /// reallocating.
    #[inline]
    pub fn capacity(&self) -> usize {
        self.slots.capacity()
    }

    /// Reserves capacity for at least `additional` more slots in the
    /// [`SparseSecondaryEntityMap`]. The collection may reserve more space to avoid
    /// frequent reallocations.
    ///
    /// # Panics
    ///
    /// Panics if the new allocation size overflows [`usize`].
    #[inline]
    pub fn reserve(&mut self, additional: usize) {
        self.slots.reserve(additional);
    }

    /// Tries to reserve capacity for at least `additional` more slots in the
    /// [`SparseSecondaryEntityMap`].  The collection may reserve more space to avoid
    /// frequent reallocations.
    #[inline]
    pub fn try_reserve(&mut self, additional: usize) -> Result<(), TryReserveError> {
        self.slots.try_reserve(additional)
    }

    /// Returns `true` if the secondary map contains the given `entity`.
    #[inline]
    pub fn contains(&self, entity: Entity) -> bool {
        self.slots
            .get(&entity.index_u32())
            .is_some_and(|slot| slot.generation == entity.generation())
    }

    /// Inserts a value into the secondary map at the given `entity`.
    ///
    /// Returns [`None`] if this entity was not present in the map,
    /// and the old value otherwise.
    #[inline]
    pub fn insert(&mut self, entity: Entity, value: V) -> Option<V> {
        if entity == Entity::PLACEHOLDER {
            return None;
        }

        let (index, generation) = (entity.index_u32(), entity.generation());

        if let Some(slot) = self.slots.get_mut(&index) {
            if slot.generation == generation {
                return Some(core::mem::replace(&mut slot.value, value));
            }

            // Don't replace existing newer values.
            if unsafe {
                is_older_generation(
                    core::mem::transmute::<EntityGeneration, u32>(generation),
                    core::mem::transmute::<EntityGeneration, u32>(slot.generation),
                )
            } {
                return None;
            }

            *slot = Slot { generation, value };

            return None;
        }

        self.slots.insert(index, Slot { generation, value });

        None
    }

    /// Removes a entity from the secondary map, returning the value at the entity if
    /// the entity was not previously removed.
    #[inline]
    pub fn remove(&mut self, entity: Entity) -> Option<V> {
        if let hash_map::Entry::Occupied(entry) = self.slots.entry(entity.index_u32())
            && entry.get().generation == entity.generation()
        {
            return Some(entry.remove_entry().1.value);
        }

        None
    }

    /// Clears the secondary map. Keeps the allocated memory for reuse.
    #[inline]
    pub fn clear(&mut self) {
        self.slots.clear();
    }

    /// Returns a reference to the value corresponding to the entity.
    #[inline]
    pub fn get(&self, entity: Entity) -> Option<&V> {
        self.slots
            .get(&entity.index_u32())
            .filter(|slot| slot.generation == entity.generation())
            .map(|slot| &slot.value)
    }

    /// Returns a reference to the value corresponding to the entity without
    /// version or bounds checking.
    ///
    /// # Safety
    ///
    /// This should only be used if `contains(entity)` is true. Otherwise it is
    /// potentially unsafe.
    #[inline]
    pub unsafe fn get_unchecked(&self, entity: Entity) -> &V {
        debug_assert!(self.contains(entity));
        unsafe { self.get(entity).unwrap_unchecked() }
    }

    /// Returns a mutable reference to the value corresponding to the entity.
    #[inline]
    pub fn get_mut(&mut self, entity: Entity) -> Option<&mut V> {
        self.slots
            .get_mut(&entity.index_u32())
            .filter(|slot| slot.generation == entity.generation())
            .map(|slot| &mut slot.value)
    }

    /// Returns a mutable reference to the value corresponding to the entity
    /// without version or bounds checking.
    ///
    /// # Safety
    ///
    /// This should only be used if `contains(entity)` is true. Otherwise it is
    /// potentially unsafe.
    #[inline]
    pub unsafe fn get_unchecked_mut(&mut self, entity: Entity) -> &mut V {
        debug_assert!(self.contains(entity));
        unsafe { self.get_mut(entity).unwrap_unchecked() }
    }

    /// Returns the value corresponding to the entity if it exists, otherwise inserts
    /// the value returned by `f` and returns it.
    #[inline]
    pub fn get_or_insert_with<F>(&mut self, entity: Entity, f: F) -> V
    where
        F: FnOnce() -> V,
        V: Clone + Copy,
    {
        if let Some(slot) = self
            .slots
            .get(&entity.index_u32())
            .filter(|s| s.generation == entity.generation())
        {
            slot.value
        } else {
            let value = f();
            self.insert(entity, value);
            value
        }
    }

    /// Returns mutable references to the values corresponding to the given
    /// keys. All keys must be valid and disjoint, otherwise `None` is returned.
    #[inline]
    pub fn get_disjoint_mut<const N: usize>(
        &mut self,
        entities: [Entity; N],
    ) -> Option<[&mut V; N]> {
        // Create an uninitialized array of `MaybeUninit`. The `assume_init` is
        // safe because the type we are claiming to have initialized here is a
        // bunch of `MaybeUninit`s, which do not require initialization.
        let mut ptrs: [MaybeUninit<*mut V>; N] = unsafe { MaybeUninit::uninit().assume_init() };

        let mut i = 0;
        while i < N {
            let entity = entities[i];

            match self.slots.get_mut(&entity.index_u32()) {
                Some(Slot { generation, value }) if *generation == entity.generation() => {
                    // This entity is valid, and the slot is occupied. Temporarily
                    // make the generation even so duplicate keys would show up as
                    // invalid, since keys always have an odd generation. This
                    // gives us a linear time disjointness check.
                    ptrs[i] = MaybeUninit::new(&mut *value);
                    unsafe {
                        *core::mem::transmute::<&mut EntityGeneration, &mut u32>(generation) ^= 1;
                    }
                }

                _ => break,
            }

            i += 1;
        }

        // Undo temporary even versions.
        for entity in &entities[0..i] {
            match self.slots.get_mut(&entity.index_u32()) {
                Some(Slot { generation, .. }) => unsafe {
                    *core::mem::transmute::<&mut EntityGeneration, &mut u32>(generation) ^= 1;
                },
                _ => unsafe { core::hint::unreachable_unchecked() },
            }
        }

        if i == N {
            // All were valid and disjoint.
            Some(unsafe { core::mem::transmute_copy::<_, [&mut V; N]>(&ptrs) })
        } else {
            None
        }
    }

    /// Returns mutable references to the values corresponding to the given
    /// keys. All keys must be valid and disjoint.
    ///
    /// # Safety
    ///
    /// This should only be used if `contains(entity)` is true for every given
    /// entity and no two keys are equal. Otherwise it is potentially unsafe.
    #[inline]
    pub unsafe fn get_disjoint_unchecked_mut<const N: usize>(
        &mut self,
        entities: [Entity; N],
    ) -> [&mut V; N] {
        // Safe, see get_disjoint_mut.
        unsafe {
            let mut ptrs: [MaybeUninit<*mut V>; N] = MaybeUninit::uninit().assume_init();
            for i in 0..N {
                ptrs[i] = MaybeUninit::new(self.get_unchecked_mut(entities[i]));
            }
            core::mem::transmute_copy::<_, [&mut V; N]>(&ptrs)
        }
    }
}

impl<V, S> Default for SparseSecondaryEntityMap<V, S>
where
    S: hash::BuildHasher + Default,
{
    fn default() -> Self {
        Self::with_hasher(Default::default())
    }
}
