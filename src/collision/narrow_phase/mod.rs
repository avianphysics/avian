//! Manages contacts and generates contact constraints.
//!
//! See [`NarrowPhasePlugin`].

mod system_param;
use system_param::ContactStatusBits;
pub use system_param::NarrowPhase;
#[cfg(feature = "parallel")]
use system_param::NarrowPhaseThreadLocals;

use core::marker::PhantomData;

use crate::{dynamics::solver::ContactConstraints, prelude::*};
use bevy::{
    ecs::{
        entity_disabling::Disabled,
        intern::Interned,
        schedule::ScheduleLabel,
        system::{StaticSystemParam, SystemParam, SystemParamItem, SystemState},
    },
    prelude::*,
};
use dynamics::solver::SolverDiagnostics;

use super::CollisionDiagnostics;

/// Manages contacts and generates contact constraints.
///
/// # Overview
///
/// Before the narrow phase, the [broad phase](super::broad_phase) creates a contact pair
/// in the [`ContactGraph`] resource for each pair of intersecting [`ColliderAabb`]s.
///
/// The narrow phase then determines which contact pairs found in the [`ContactGraph`] are touching,
/// and computes updated contact points and normals in a parallel loop.
///
/// Afterwards, the narrow phase removes contact pairs whose AABBs no longer overlap,
/// and sends collision events for colliders that started or stopped touching.
/// This is done in a fast serial loop to preserve determinism.
///
/// Finally, a [`ContactConstraint`] is generated for each contact pair that is touching
/// or expected to touch during the time step. These constraints are added to the [`ContactConstraints`]
/// resource, and are later used by the [`SolverPlugin`] to solve contacts.
///
/// [`ContactConstraint`]: dynamics::solver::contact::ContactConstraint
///
/// # Collider Types
///
/// The plugin takes a collider type. This should be [`Collider`] for
/// the vast majority of applications, but for custom collision backends
/// you may use any collider that implements the [`AnyCollider`] trait.
pub struct NarrowPhasePlugin<C: AnyCollider, H: CollisionHooks = ()> {
    schedule: Interned<dyn ScheduleLabel>,
    /// If `true`, the narrow phase will generate [`ContactConstraint`]s
    /// and add them to the [`ContactConstraints`] resource.
    ///
    /// Contact constraints are used by the [`SolverPlugin`] for solving contacts.
    ///
    /// [`ContactConstraint`]: dynamics::solver::contact::ContactConstraint
    generate_constraints: bool,
    _phantom: PhantomData<(C, H)>,
}

impl<C: AnyCollider, H: CollisionHooks> NarrowPhasePlugin<C, H> {
    /// Creates a [`NarrowPhasePlugin`] with the schedule used for running its systems
    /// and whether it should generate [`ContactConstraint`]s for the [`ContactConstraints`] resource.
    ///
    /// Contact constraints are used by the [`SolverPlugin`] for solving contacts.
    ///
    /// The default schedule is [`PhysicsSchedule`].
    ///
    /// [`ContactConstraint`]: dynamics::solver::contact::ContactConstraint
    pub fn new(schedule: impl ScheduleLabel, generate_constraints: bool) -> Self {
        Self {
            schedule: schedule.intern(),
            generate_constraints,
            _phantom: PhantomData,
        }
    }
}

impl<C: AnyCollider, H: CollisionHooks> Default for NarrowPhasePlugin<C, H> {
    fn default() -> Self {
        Self::new(PhysicsSchedule, true)
    }
}

/// A resource that indicates that the narrow phase has been initialized.
///
/// This is used to ensure that some systems are only added once
/// even with multiple collider types.
#[derive(Resource, Default)]
struct NarrowPhaseInitialized;

impl<C: AnyCollider, H: CollisionHooks + 'static> Plugin for NarrowPhasePlugin<C, H>
where
    for<'w, 's> SystemParamItem<'w, 's, H>: CollisionHooks,
{
    fn build(&self, app: &mut App) {
        let already_initialized = app.world().is_resource_added::<NarrowPhaseInitialized>();

        app.init_resource::<NarrowPhaseConfig>()
            .init_resource::<ContactGraph>()
            .init_resource::<ContactStatusBits>()
            .init_resource::<DefaultFriction>()
            .init_resource::<DefaultRestitution>();

        #[cfg(feature = "parallel")]
        app.init_resource::<NarrowPhaseThreadLocals>();

        app.register_type::<(NarrowPhaseConfig, DefaultFriction, DefaultRestitution)>();

        app.add_event::<CollisionStarted>()
            .add_event::<CollisionEnded>();

        if self.generate_constraints {
            app.init_resource::<ContactConstraints>();
        }

        // Set up system set scheduling.
        app.configure_sets(
            self.schedule,
            (
                NarrowPhaseSet::First,
                NarrowPhaseSet::Update,
                NarrowPhaseSet::Last,
            )
                .chain()
                .in_set(PhysicsStepSet::NarrowPhase),
        );
        app.configure_sets(
            self.schedule,
            CollisionEventSystems.in_set(PhysicsStepSet::Finalize),
        );

        // Perform narrow phase collision detection.
        app.add_systems(
            self.schedule,
            update_narrow_phase::<C, H>
                .in_set(NarrowPhaseSet::Update)
                // Allowing ambiguities is required so that it's possible
                // to have multiple collision backends at the same time.
                .ambiguous_with_all(),
        );

        if !already_initialized {
            // Remove collision pairs when colliders are disabled or removed.
            app.add_observer(remove_collider_on::<OnAdd, (Disabled, ColliderDisabled)>);
            app.add_observer(remove_collider_on::<OnRemove, ColliderMarker>);

            // Trigger collision events for colliders that started or stopped touching.
            app.add_systems(
                PhysicsSchedule,
                trigger_collision_events.in_set(CollisionEventSystems),
            );
        }

        app.init_resource::<NarrowPhaseInitialized>();
    }

    fn finish(&self, app: &mut App) {
        // Register timer and counter diagnostics for collision detection.
        app.register_physics_diagnostics::<CollisionDiagnostics>();
    }
}

/// A system set for triggering the [`OnCollisionStart`] and [`OnCollisionEnd`] events.
///
/// Runs in [`PhysicsStepSet::Finalize`], after the solver has run and contact impulses
/// have been computed and applied.
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct CollisionEventSystems;

/// A resource for configuring the [narrow phase](NarrowPhasePlugin).
#[derive(Resource, Reflect, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Resource, PartialEq)]
pub struct NarrowPhaseConfig {
    /// The default maximum [speculative margin](SpeculativeMargin) used for
    /// [speculative collisions](dynamics::ccd#speculative-collision). This can be overridden
    /// for individual entities with the [`SpeculativeMargin`] component.
    ///
    /// By default, the maximum speculative margin is unbounded, so contacts can be predicted
    /// from any distance, provided that the bodies are moving fast enough. As the prediction distance
    /// grows, the contact data becomes more and more approximate, and in rare cases, it can even cause
    /// [issues](dynamics::ccd#caveats-of-speculative-collision) such as ghost collisions.
    ///
    /// By limiting the maximum speculative margin, these issues can be mitigated, at the cost
    /// of an increased risk of tunneling. Setting it to `0.0` disables speculative collision
    /// altogether for entities without [`SpeculativeMargin`].
    ///
    /// This is implicitly scaled by the [`PhysicsLengthUnit`].
    ///
    /// Default: `MAX` (unbounded)
    pub default_speculative_margin: Scalar,

    /// A contact tolerance that acts as a minimum bound for the [speculative margin](dynamics::ccd#speculative-collision).
    ///
    /// A small, positive contact tolerance helps ensure that contacts are not missed
    /// due to numerical issues or solver jitter for objects that are in continuous
    /// contact, such as pushing against each other.
    ///
    /// Making the contact tolerance too large will have a negative impact on performance,
    /// as contacts will be computed even for objects that are not in close proximity.
    ///
    /// This is implicitly scaled by the [`PhysicsLengthUnit`].
    ///
    /// Default: `0.005`
    pub contact_tolerance: Scalar,

    /// If `true`, the current contacts will be matched with the previous contacts
    /// based on feature IDs or contact positions, and the contact impulses from
    /// the previous frame will be copied over for the new contacts.
    ///
    /// Using these impulses as the initial guess is referred to as *warm starting*,
    /// and it can help the contact solver resolve overlap and stabilize much faster.
    ///
    /// Default: `true`
    pub match_contacts: bool,
}

impl Default for NarrowPhaseConfig {
    fn default() -> Self {
        Self {
            default_speculative_margin: Scalar::MAX,
            contact_tolerance: 0.005,
            match_contacts: true,
        }
    }
}

/// System sets for systems running in [`PhysicsStepSet::NarrowPhase`].
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum NarrowPhaseSet {
    /// Runs at the start of the narrow phase. Empty by default.
    First,
    /// Updates contacts in the [`ContactGraph`] and processes contact state changes.
    Update,
    /// Runs at the end of the narrow phase. Empty by default.
    Last,
}

fn update_narrow_phase<C: AnyCollider, H: CollisionHooks + 'static>(
    mut narrow_phase: NarrowPhase<C>,
    mut collision_started_event_writer: EventWriter<CollisionStarted>,
    mut collision_ended_event_writer: EventWriter<CollisionEnded>,
    time: Res<Time>,
    hooks: StaticSystemParam<H>,
    context: StaticSystemParam<C::Context>,
    mut commands: ParallelCommands,
    mut diagnostics: ResMut<CollisionDiagnostics>,
    solver_diagnostics: Option<ResMut<SolverDiagnostics>>,
) where
    for<'w, 's> SystemParamItem<'w, 's, H>: CollisionHooks,
{
    let start = crate::utils::Instant::now();

    narrow_phase.update::<H>(
        &mut collision_started_event_writer,
        &mut collision_ended_event_writer,
        time.delta_seconds_adjusted(),
        &hooks,
        &context,
        &mut commands,
    );

    diagnostics.narrow_phase = start.elapsed();
    diagnostics.contact_count = narrow_phase.contact_graph.internal.edge_count() as u32;

    if let Some(mut solver_diagnostics) = solver_diagnostics {
        solver_diagnostics.contact_constraint_count = narrow_phase.contact_constraints.len() as u32;
    }
}

#[derive(SystemParam)]
struct TriggerCollisionEventsContext<'w, 's> {
    query: Query<'w, 's, Option<&'static ColliderOf>, With<CollisionEventsEnabled>>,
    started: EventReader<'w, 's, CollisionStarted>,
    ended: EventReader<'w, 's, CollisionEnded>,
}

/// Triggers [`OnCollisionStart`] and [`OnCollisionEnd`] events for colliders
/// that started or stopped touching and have the [`CollisionEventsEnabled`] component.
fn trigger_collision_events(
    // We use exclusive access here to avoid queuing a new command for each event.
    world: &mut World,
    state: &mut SystemState<TriggerCollisionEventsContext>,
    // Cache pairs in buffers to avoid reallocating every time.
    mut started_pairs: Local<Vec<(Entity, OnCollisionStart)>>,
    mut ended_pairs: Local<Vec<(Entity, OnCollisionEnd)>>,
) {
    let mut state = state.get_mut(world);

    // Collect `OnCollisionStart` and `OnCollisionEnd` events
    // for entities that have events enabled.
    for event in state.started.read() {
        if let Ok(collider_of) = state.query.get(event.0) {
            let collider = event.1;
            let body = collider_of.map(|c| c.body);
            started_pairs.push((event.0, OnCollisionStart { collider, body }));
        }
        if let Ok(collider_of) = state.query.get(event.1) {
            let collider = event.0;
            let body = collider_of.map(|c| c.body);
            started_pairs.push((event.1, OnCollisionStart { collider, body }));
        }
    }
    for event in state.ended.read() {
        if let Ok(collider_of) = state.query.get(event.0) {
            let collider = event.1;
            let body = collider_of.map(|c| c.body);
            ended_pairs.push((event.0, OnCollisionEnd { collider, body }));
        }
        if let Ok(collider_of) = state.query.get(event.1) {
            let collider = event.0;
            let body = collider_of.map(|c| c.body);
            ended_pairs.push((event.1, OnCollisionEnd { collider, body }));
        }
    }

    // Trigger the events, draining the buffers in the process.
    started_pairs.drain(..).for_each(|(entity, event)| {
        world.trigger_targets(event, entity);
    });
    ended_pairs.drain(..).for_each(|(entity, event)| {
        world.trigger_targets(event, entity);
    });
}

/// Removes colliders from the [`ContactGraph`] when the given trigger is activated.
///
/// Also removes the collider from the [`CollidingEntities`] of the other entity,
/// wakes up the other body, and sends a [`CollisionEnded`] event.
fn remove_collider_on<E: Event, B: Bundle>(
    trigger: Trigger<E, B>,
    mut contact_graph: ResMut<ContactGraph>,
    mut query: Query<&mut CollidingEntities>,
    mut event_writer: EventWriter<CollisionEnded>,
    mut commands: Commands,
) {
    let entity = trigger.target();

    // Remove the collider from the contact graph.
    contact_graph.remove_collider_with(entity, |contact_pair| {
        // If the contact pair was not touching, we don't need to do anything.
        if !contact_pair.flags.contains(ContactPairFlags::TOUCHING) {
            return;
        }

        // Send a collision ended event.
        if contact_pair
            .flags
            .contains(ContactPairFlags::CONTACT_EVENTS)
        {
            event_writer.write(CollisionEnded(
                contact_pair.collider1,
                contact_pair.collider2,
            ));
        }

        // Remove the entity from the `CollidingEntities` of the other entity.
        let other_entity = if contact_pair.collider1 == entity {
            contact_pair.collider2
        } else {
            contact_pair.collider1
        };
        if let Ok(mut colliding_entities) = query.get_mut(other_entity) {
            colliding_entities.remove(&entity);
        }

        // Wake up the other body.
        commands.queue(WakeUpBody(other_entity));
    });
}
