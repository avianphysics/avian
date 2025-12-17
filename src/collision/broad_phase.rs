//! Finds pairs of entities with overlapping [`ColliderAabb`]s to reduce
//! the number of potential contacts for the [narrow phase](super::narrow_phase).
//!
//! See [`BroadPhasePlugin`].

use core::marker::PhantomData;

use crate::{
    data_structures::pair_key::PairKey, dynamics::solver::joint_graph::JointGraph, prelude::*,
};
use bevy::{
    ecs::{
        entity::{EntityMapper, MapEntities},
        entity_disabling::Disabled,
        system::{StaticSystemParam, SystemParamItem, lifetimeless::Read},
    },
    prelude::*,
};

use super::{
    CollisionDiagnostics,
    contact_types::{ContactEdge, ContactEdgeFlags},
};

/// Finds pairs of entities with overlapping [`ColliderAabb`]s to reduce
/// the number of potential contacts for the [narrow phase](super::narrow_phase).
///
/// A contact pair is created in the [`ContactGraph`] resource for each pair found.
/// Removing and updating these pairs is left to the [narrow phase](super::narrow_phase).
///
/// Currently, the broad phase uses the [sweep and prune](https://en.wikipedia.org/wiki/Sweep_and_prune) algorithm.
///
/// The broad phase systems run in [`PhysicsStepSystems::BroadPhase`].
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
                BroadPhaseSystems::First,
                BroadPhaseSystems::UpdateStructures,
                BroadPhaseSystems::CollectCollisions,
                BroadPhaseSystems::Last,
            )
                .chain()
                .in_set(PhysicsStepSystems::BroadPhase),
        );

        let physics_schedule = app
            .get_schedule_mut(PhysicsSchedule)
            .expect("add PhysicsSchedule first");

        physics_schedule.add_systems(
            (update_aabb_intervals, add_new_aabb_intervals)
                .chain()
                .in_set(BroadPhaseSystems::UpdateStructures),
        );

        physics_schedule
            .add_systems(collect_collision_pairs::<H>.in_set(BroadPhaseSystems::CollectCollisions));

        // TODO: Deduplicate these observers.
        // Add colliders back to the broad phase when `Disabled` is removed.
        app.add_observer(
            |trigger: On<Remove, Disabled>,
             // TODO: Use `Allows<T>` in Bevy 0.17.
             query: Query<
                AabbIntervalQueryData,
                (
                    Without<ColliderDisabled>,
                    Or<(With<Disabled>, Without<Disabled>)>,
                ),
            >,
             rbs: Query<(&RigidBody, Has<RigidBodyDisabled>)>,
             mut intervals: ResMut<AabbIntervals>| {
                let entity = trigger.entity;

                // Re-enable the collider.
                if let Ok((entity, collider_of, aabb, layers, is_sensor, events_enabled, hooks)) =
                    query.get(entity)
                {
                    let flags = init_aabb_interval_flags(
                        collider_of,
                        &rbs,
                        is_sensor,
                        events_enabled,
                        hooks,
                    );
                    let interval = (
                        entity,
                        collider_of.map_or(ColliderOf { body: entity }, |p| *p),
                        *aabb,
                        *layers,
                        flags,
                    );

                    // Add the re-enabled collider to the intervals.
                    intervals.0.push(interval);
                }
            },
        );

        // Add colliders back to the broad phase when `ColliderDisabled` is removed.
        app.add_observer(
            |trigger: On<Remove, ColliderDisabled>,
             query: Query<AabbIntervalQueryData>,
             rbs: Query<(&RigidBody, Has<RigidBodyDisabled>)>,
             mut intervals: ResMut<AabbIntervals>| {
                let entity = trigger.entity;

                // Re-enable the collider.
                if let Ok((entity, collider_of, aabb, layers, is_sensor, events_enabled, hooks)) =
                    query.get(entity)
                {
                    let flags = init_aabb_interval_flags(
                        collider_of,
                        &rbs,
                        is_sensor,
                        events_enabled,
                        hooks,
                    );
                    let interval = (
                        entity,
                        collider_of.map_or(ColliderOf { body: entity }, |p| *p),
                        *aabb,
                        *layers,
                        flags,
                    );

                    // Add the re-enabled collider to the intervals.
                    intervals.0.push(interval);
                }
            },
        );
    }

    fn finish(&self, app: &mut App) {
        // Register timer and counter diagnostics for collision detection.
        app.register_physics_diagnostics::<CollisionDiagnostics>();
    }
}

/// System sets for systems running in [`PhysicsStepSystems::BroadPhase`].
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum BroadPhaseSystems {
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

/// A deprecated alias for [`BroadPhaseSystems`].
#[deprecated(since = "0.4.0", note = "Renamed to `BroadPhaseSystems`")]
pub type BroadPhaseSet = BroadPhaseSystems;

/// Entities with [`ColliderAabb`]s sorted along an axis by their extents.
#[derive(Resource, Default)]
struct AabbIntervals(Vec<AabbInterval>);

type AabbInterval = (
    Entity,
    ColliderOf,
    ColliderAabb,
    CollisionLayers,
    AabbIntervalFlags,
);

bitflags::bitflags! {
    /// Flags for AABB intervals in the broad phase.
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
    pub struct AabbIntervalFlags: u8 {
        /// Set if the body is sleeping or static.
        const IS_INACTIVE = 1 << 0;
        /// Set if collision events are enabled for this entity.
        const CONTACT_EVENTS = 1 << 1;
        /// Set if the collider should generate contact constraints.
        const GENERATE_CONSTRAINTS = 1 << 2;
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
    rbs: Query<(&RigidBody, Has<RigidBodyDisabled>)>,
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

                let rb = new_collider_of.and_then(|collider_of| rbs.get(collider_of.body).ok());
                let is_static = rb.is_some_and(|(body, _)| body.is_static());
                let is_disabled = rb.is_some_and(|(_, is_disabled)| is_disabled);

                flags.set(AabbIntervalFlags::IS_INACTIVE, is_static || is_sleeping);
                flags.set(AabbIntervalFlags::CONTACT_EVENTS, events_enabled);
                flags.set(
                    AabbIntervalFlags::GENERATE_CONSTRAINTS,
                    !is_sensor && !is_disabled,
                );
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
    rbs: Query<(&RigidBody, Has<RigidBodyDisabled>)>,
    mut intervals: ResMut<AabbIntervals>,
) {
    let aabbs = added_aabbs.iter().map(
        |(entity, collider_of, aabb, layers, is_sensor, events_enabled, hooks)| {
            let flags =
                init_aabb_interval_flags(collider_of, &rbs, is_sensor, events_enabled, hooks);
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

fn init_aabb_interval_flags(
    collider_of: Option<&ColliderOf>,
    rbs: &Query<(&RigidBody, Has<RigidBodyDisabled>)>,
    is_sensor: bool,
    events_enabled: bool,
    hooks: Option<&ActiveCollisionHooks>,
) -> AabbIntervalFlags {
    let mut flags = AabbIntervalFlags::empty();
    let rb = collider_of.and_then(|collider_of| rbs.get(collider_of.body).ok());
    let is_static = rb.is_some_and(|(body, _)| body.is_static());
    let is_body_disabled = rb.is_some_and(|(_, is_disabled)| is_disabled);
    flags.set(AabbIntervalFlags::IS_INACTIVE, is_static);
    flags.set(AabbIntervalFlags::CONTACT_EVENTS, events_enabled);
    flags.set(
        AabbIntervalFlags::GENERATE_CONSTRAINTS,
        !is_sensor && !is_body_disabled,
    );
    flags.set(
        AabbIntervalFlags::CUSTOM_FILTER,
        hooks.is_some_and(|h| h.contains(ActiveCollisionHooks::FILTER_PAIRS)),
    );
    flags.set(
        AabbIntervalFlags::MODIFY_CONTACTS,
        hooks.is_some_and(|h| h.contains(ActiveCollisionHooks::MODIFY_CONTACTS)),
    );
    flags
}

/// Finds pairs of entities with overlapping [`ColliderAabb`]s
/// and creates contact pairs for them in the [`ContactGraph`].
fn collect_collision_pairs<H: CollisionHooks>(
    intervals: ResMut<AabbIntervals>,
    mut contact_graph: ResMut<ContactGraph>,
    joint_graph: Res<JointGraph>,
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
        &joint_graph,
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
    joint_graph: &JointGraph,
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
            let pair_key = PairKey::new(entity1.index_u32(), entity2.index_u32());
            if contact_graph.contains_key(&pair_key) {
                continue;
            }

            // Check if a joint disables contacts between the two bodies.
            if joint_graph
                .joints_between(collider_of1.body, collider_of2.body)
                .any(|edge| edge.collision_disabled)
            {
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
            let mut contact_edge = ContactEdge::new(*entity1, *entity2);
            contact_edge.body1 = Some(collider_of1.body);
            contact_edge.body2 = Some(collider_of2.body);
            contact_edge.flags.set(
                ContactEdgeFlags::CONTACT_EVENTS,
                flags1
                    .union(*flags2)
                    .contains(AabbIntervalFlags::CONTACT_EVENTS),
            );
            contact_graph
                .add_edge_and_key_with(contact_edge, pair_key, |contact_pair| {
                    contact_pair.body1 = Some(collider_of1.body);
                    contact_pair.body2 = Some(collider_of2.body);
                    contact_pair.flags.set(
                        ContactPairFlags::MODIFY_CONTACTS,
                        flags1
                            .union(*flags2)
                            .contains(AabbIntervalFlags::MODIFY_CONTACTS),
                    );
                    contact_pair.flags.set(
                        ContactPairFlags::GENERATE_CONSTRAINTS,
                        flags1
                            .union(*flags2)
                            .contains(AabbIntervalFlags::GENERATE_CONSTRAINTS),
                    );
                })
                .unwrap_or_else(|| {
                    panic!("Pair key already exists in contact graph: {pair_key:?}")
                });
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
