//! Finds pairs of entities with overlapping [`ColliderAabb`]s to reduce
//! the number of potential contacts for the [narrow phase](super::narrow_phase).
//!
//! See [`BroadPhasePlugin`].

use core::marker::PhantomData;

use crate::{data_structures::pair_key::PairKey, prelude::*};
use bevy::{
    ecs::{
        entity::{EntityHashSet, EntityMapper, MapEntities},
        entity_disabling::Disabled,
        system::{lifetimeless::Read, StaticSystemParam, SystemParamItem},
    },
    prelude::*,
};

use super::CollisionDiagnostics;

/// Finds pairs of entities with overlapping [`ColliderAabb`]s to reduce
/// the number of potential contacts for the [narrow phase](super::narrow_phase).
///
/// A contact pair is created in the [`ContactGraph`] resource for each pair found.
/// Removing and updating these pairs is left to the [narrow phase](super::narrow_phase).
///
/// Currently, the broad phase uses the [sweep and prune](https://en.wikipedia.org/wiki/Sweep_and_prune) algorithm.
///
/// The broad phase systems run in [`PhysicsStepSet::BroadPhase`].
///
/// [`CollisionHooks`] can be provided with generics to apply custom filtering for collision pairs.
pub struct BroadPhasePlugin<H: CollisionHooks = ()>(PhantomData<H>);

impl<H: CollisionHooks> Default for BroadPhasePlugin<H> {
    fn default() -> Self {
        Self(PhantomData)
    }
}

impl<H: CollisionHooks + 'static> Plugin for BroadPhasePlugin<H>
where
    for<'w, 's> SystemParamItem<'w, 's, H>: CollisionHooks,
{
    fn build(&self, app: &mut App) {
        app.init_resource::<AabbIntervals>();

        app.configure_sets(
            PhysicsSchedule,
            (
                BroadPhaseSet::First,
                BroadPhaseSet::UpdateStructures,
                BroadPhaseSet::CollectCollisions,
                BroadPhaseSet::Last,
            )
                .chain()
                .in_set(PhysicsStepSet::BroadPhase),
        );

        let physics_schedule = app
            .get_schedule_mut(PhysicsSchedule)
            .expect("add PhysicsSchedule first");

        physics_schedule.add_systems(
            (update_aabb_intervals, add_new_aabb_intervals)
                .chain()
                .in_set(BroadPhaseSet::UpdateStructures),
        );

        physics_schedule
            .add_systems(collect_collision_pairs::<H>.in_set(BroadPhaseSet::CollectCollisions));
    }

    fn finish(&self, app: &mut App) {
        // Register timer and counter diagnostics for collision detection.
        app.register_physics_diagnostics::<CollisionDiagnostics>();
    }
}

/// System sets for systems running in [`PhysicsStepSet::BroadPhase`].
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum BroadPhaseSet {
    /// Runs at the start of the broad phase. Empty by default.
    First,
    /// Updates acceleration structures and other data needed for broad phase collision detection.
    UpdateStructures,
    /// Finds pairs of entities with overlapping [`ColliderAabb`]s
    /// and creates contact pairs for them in the [`ContactGraph`].
    CollectCollisions,
    /// Runs at the end of the broad phase. Empty by default.
    Last,
}

/// Entities with [`ColliderAabb`]s sorted along an axis by their extents.
#[derive(Resource, Default)]
struct AabbIntervals(
    Vec<(
        Entity,
        ColliderOf,
        ColliderAabb,
        CollisionLayers,
        AabbIntervalFlags,
    )>,
);

bitflags::bitflags! {
    /// Flags for AABB intervals in the broad phase.
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
    pub struct AabbIntervalFlags: u8 {
        /// Set if the body is sleeping or static.
        const IS_INACTIVE = 1 << 0;
        /// Set if the collider is a sensor.
        const IS_SENSOR = 1 << 1;
        /// Set if collision events are enabled for this entity.
        const CONTACT_EVENTS = 1 << 2;
        /// Set if [`CollisionHooks::filter_pairs`] should be called for this entity.
        const CUSTOM_FILTER = 1 << 3;
        /// Set if [`CollisionHooks::modify_contacts`] should be called for this entity.
        const MODIFY_CONTACTS = 1 << 4;
    }
}

impl MapEntities for AabbIntervals {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        for interval in self.0.iter_mut() {
            interval.0 = entity_mapper.get_mapped(interval.0);
        }
    }
}

/// Updates [`AabbIntervals`] to keep them in sync with the [`ColliderAabb`]s.
#[allow(clippy::type_complexity)]
fn update_aabb_intervals(
    aabbs: Query<
        (
            &ColliderAabb,
            Option<&ColliderOf>,
            &CollisionLayers,
            Has<Sensor>,
            Has<CollisionEventsEnabled>,
            Option<&ActiveCollisionHooks>,
            Has<Sleeping>,
        ),
        Without<ColliderDisabled>,
    >,
    rbs: Query<&RigidBody>,
    mut intervals: ResMut<AabbIntervals>,
) {
    intervals
        .0
        .retain_mut(|(collider_entity, collider_of, aabb, layers, flags)| {
            if let Ok((
                new_aabb,
                new_collider_of,
                new_layers,
                is_sensor,
                events_enabled,
                hooks,
                is_sleeping,
            )) = aabbs.get(*collider_entity)
            {
                if !new_aabb.min.is_finite() || !new_aabb.max.is_finite() {
                    return false;
                }

                *aabb = *new_aabb;
                *collider_of = new_collider_of.map_or(
                    ColliderOf {
                        body: *collider_entity,
                    },
                    |p| *p,
                );
                *layers = *new_layers;

                let is_static = new_collider_of.is_some_and(|collider_of| {
                    rbs.get(collider_of.body).is_ok_and(RigidBody::is_static)
                });

                flags.set(AabbIntervalFlags::IS_INACTIVE, is_static || is_sleeping);
                flags.set(AabbIntervalFlags::IS_SENSOR, is_sensor);
                flags.set(AabbIntervalFlags::CONTACT_EVENTS, events_enabled);
                flags.set(
                    AabbIntervalFlags::CUSTOM_FILTER,
                    hooks.is_some_and(|h| h.contains(ActiveCollisionHooks::FILTER_PAIRS)),
                );
                flags.set(
                    AabbIntervalFlags::MODIFY_CONTACTS,
                    hooks.is_some_and(|h| h.contains(ActiveCollisionHooks::MODIFY_CONTACTS)),
                );

                true
            } else {
                false
            }
        });
}

type AabbIntervalQueryData = (
    Entity,
    Option<Read<ColliderOf>>,
    Read<ColliderAabb>,
    Option<Read<RigidBody>>,
    Read<CollisionLayers>,
    Has<Sensor>,
    Has<CollisionEventsEnabled>,
    Option<Read<ActiveCollisionHooks>>,
);

// TODO: This is pretty gross and inefficient. This should be done with observers or hooks
//       once we rework the broad phase.
/// Adds new [`ColliderAabb`]s to [`AabbIntervals`].
#[allow(clippy::type_complexity)]
fn add_new_aabb_intervals(
    added_aabbs: Query<AabbIntervalQueryData, (Added<ColliderAabb>, Without<ColliderDisabled>)>,
    aabbs: Query<AabbIntervalQueryData, Without<ColliderDisabled>>,
    mut intervals: ResMut<AabbIntervals>,
    mut re_enabled_colliders: RemovedComponents<ColliderDisabled>,
    mut re_enabled_entities: RemovedComponents<Disabled>,
) {
    // Collect re-enabled entities without duplicates.
    let re_enabled = re_enabled_colliders
        .read()
        .chain(re_enabled_entities.read())
        .collect::<EntityHashSet>();
    let re_enabled_aabbs = aabbs.iter_many(re_enabled);

    let aabbs = added_aabbs.iter().chain(re_enabled_aabbs).map(
        |(entity, collider_of, aabb, rb, layers, is_sensor, events_enabled, hooks)| {
            let mut flags = AabbIntervalFlags::empty();
            flags.set(
                AabbIntervalFlags::IS_INACTIVE,
                rb.is_some_and(|rb| rb.is_static()),
            );
            flags.set(AabbIntervalFlags::IS_SENSOR, is_sensor);
            flags.set(AabbIntervalFlags::CONTACT_EVENTS, events_enabled);
            flags.set(
                AabbIntervalFlags::CUSTOM_FILTER,
                hooks.is_some_and(|h| h.contains(ActiveCollisionHooks::FILTER_PAIRS)),
            );
            flags.set(
                AabbIntervalFlags::MODIFY_CONTACTS,
                hooks.is_some_and(|h| h.contains(ActiveCollisionHooks::MODIFY_CONTACTS)),
            );
            (
                entity,
                collider_of.map_or(ColliderOf { body: entity }, |p| *p),
                *aabb,
                *layers,
                flags,
            )
        },
    );
    intervals.0.extend(aabbs);
}

/// Finds pairs of entities with overlapping [`ColliderAabb`]s
/// and creates contact pairs for them in the [`ContactGraph`].
fn collect_collision_pairs<H: CollisionHooks>(
    intervals: ResMut<AabbIntervals>,
    mut contact_graph: ResMut<ContactGraph>,
    hooks: StaticSystemParam<H>,
    mut commands: Commands,
    mut diagnostics: ResMut<CollisionDiagnostics>,
) where
    for<'w, 's> SystemParamItem<'w, 's, H>: CollisionHooks,
{
    let start = crate::utils::Instant::now();

    sweep_and_prune::<H>(
        intervals,
        &mut contact_graph,
        &mut hooks.into_inner(),
        &mut commands,
    );

    diagnostics.broad_phase = start.elapsed();
}

/// Sorts the entities by their minimum extents along an axis and collects the entity pairs that have intersecting AABBs.
///
/// Sweep and prune exploits temporal coherence, as bodies are unlikely to move significantly between two simulation steps. Insertion sort is used, as it is good at sorting nearly sorted lists efficiently.
fn sweep_and_prune<H: CollisionHooks>(
    mut intervals: ResMut<AabbIntervals>,
    contact_graph: &mut ContactGraph,
    hooks: &mut H::Item<'_, '_>,
    commands: &mut Commands,
) where
    for<'w, 's> SystemParamItem<'w, 's, H>: CollisionHooks,
{
    // Sort bodies along the x-axis using insertion sort, a sorting algorithm great for sorting nearly sorted lists.
    insertion_sort(&mut intervals.0, |a, b| a.2.min.x > b.2.min.x);

    // Find potential collisions by checking for AABB intersections along all axes.
    // TODO: Find pairs in parallel, but create contact pairs serially for determinism.
    for (i, (entity1, collider_of1, aabb1, layers1, flags1)) in intervals.0.iter().enumerate() {
        for (entity2, collider_of2, aabb2, layers2, flags2) in intervals.0.iter().skip(i + 1) {
            // x doesn't intersect; check this first so we can discard as soon as possible.
            if aabb2.min.x > aabb1.max.x {
                break;
            }

            // y doesn't intersect.
            if aabb1.min.y > aabb2.max.y || aabb1.max.y < aabb2.min.y {
                continue;
            }

            #[cfg(feature = "3d")]
            // z doesn't intersect.
            if aabb1.min.z > aabb2.max.z || aabb1.max.z < aabb2.min.z {
                continue;
            }

            // No collisions between bodies that haven't moved or colliders with incompatible layers
            // or colliders attached to the same rigid body.
            if flags1
                .intersection(*flags2)
                .contains(AabbIntervalFlags::IS_INACTIVE)
                || !layers1.interacts_with(*layers2)
                || collider_of1 == collider_of2
            {
                continue;
            }

            // Avoid duplicate pairs.
            let pair_key = PairKey::new(entity1.index(), entity2.index());
            if contact_graph.contains_key(&pair_key) {
                continue;
            }

            // Apply user-defined filter.
            if flags1
                .union(*flags2)
                .contains(AabbIntervalFlags::CUSTOM_FILTER)
            {
                let should_collide = hooks.filter_pairs(*entity1, *entity2, commands);
                if !should_collide {
                    continue;
                }
            }

            // Create a new contact pair as non-touching.
            // The narrow phase will determine if the entities are touching and compute contact data.
            let mut contacts = ContactPair::new(*entity1, *entity2);

            // Initialize flags and other data for the contact pair.
            contacts.body1 = Some(collider_of1.body);
            contacts.body2 = Some(collider_of2.body);
            contacts.flags.set(
                ContactPairFlags::SENSOR,
                flags1.union(*flags2).contains(AabbIntervalFlags::IS_SENSOR),
            );
            contacts.flags.set(
                ContactPairFlags::CONTACT_EVENTS,
                flags1
                    .union(*flags2)
                    .contains(AabbIntervalFlags::CONTACT_EVENTS),
            );
            contacts.flags.set(
                ContactPairFlags::MODIFY_CONTACTS,
                flags1
                    .union(*flags2)
                    .contains(AabbIntervalFlags::MODIFY_CONTACTS),
            );

            // Add the contact pair to the contact graph.
            contact_graph.add_pair_with_key(contacts, pair_key);
        }
    }
}

/// Sorts a list iteratively using comparisons. In an ascending sort order, when a smaller value is encountered, it is moved lower in the list until it is larger than the item before it.
///
/// This is relatively slow for large lists, but very efficient in cases where the list is already mostly sorted.
fn insertion_sort<T>(items: &mut [T], comparison: fn(&T, &T) -> bool) {
    for i in 1..items.len() {
        let mut j = i;
        while j > 0 && comparison(&items[j - 1], &items[j]) {
            items.swap(j - 1, j);
            j -= 1;
        }
    }
}
