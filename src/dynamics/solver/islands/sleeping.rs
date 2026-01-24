//! Sleeping and waking for [`PhysicsIsland`](super::PhysicsIsland)s.
//!
//! See [`IslandSleepingPlugin`].

use bevy::{
    app::{App, Plugin},
    ecs::{
        entity::Entity,
        entity_disabling::Disabled,
        error::Result,
        lifecycle::{HookContext, Insert, Replace},
        observer::On,
        query::{Changed, Has, Or, With, Without},
        resource::Resource,
        schedule::{
            IntoScheduleConfigs,
            common_conditions::{resource_changed, resource_exists},
        },
        system::{
            Command, Commands, Local, ParamSet, Query, Res, ResMut, SystemChangeTick, SystemState,
            lifetimeless::{SQuery, SResMut},
        },
        world::{DeferredWorld, Mut, Ref, World},
    },
    prelude::{Deref, DerefMut},
    time::Time,
};

use crate::{
    data_structures::bit_vec::BitVec,
    dynamics::solver::{
        constraint_graph::ConstraintGraph,
        islands::{BodyIslandNode, IslandId, PhysicsIslands},
        joint_graph::JointGraph,
        solver_body::SolverBody,
    },
    prelude::*,
    schedule::{LastPhysicsTick, is_changed_after_tick},
};

/// A plugin for managing sleeping and waking of [`PhysicsIsland`](super::PhysicsIsland)s.
pub struct IslandSleepingPlugin;

impl Plugin for IslandSleepingPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<AwakeIslandBitVec>();
        app.init_resource::<TimeToSleep>();

        // Insert `SleepThreshold` and `SleepTimer` for each `SolverBody`.
        app.register_required_components::<SolverBody, SleepThreshold>();
        app.register_required_components::<SolverBody, SleepTimer>();

        // Set up cached system states for sleeping and waking bodies or islands.
        let cached_system_state1 = CachedBodySleepingSystemState(SystemState::new(app.world_mut()));
        let cached_system_state2 =
            CachedIslandSleepingSystemState(SystemState::new(app.world_mut()));
        let cached_system_state3 = CachedIslandWakingSystemState(SystemState::new(app.world_mut()));
        app.insert_resource(cached_system_state1);
        app.insert_resource(cached_system_state2);
        app.insert_resource(cached_system_state3);

        // Set up hooks to automatically sleep/wake islands when `Sleeping` is added/removed.
        app.world_mut()
            .register_component_hooks::<Sleeping>()
            .on_add(sleep_on_add_sleeping)
            .on_remove(wake_on_remove_sleeping);

        app.add_observer(wake_on_replace_rigid_body);
        app.add_observer(wake_on_enable_rigid_body);

        app.add_systems(
            PhysicsSchedule,
            (
                update_sleeping_states,
                wake_islands_with_sleeping_disabled,
                wake_on_changed,
                wake_all_islands.run_if(resource_changed::<Gravity>),
                sleep_islands,
            )
                .chain()
                .run_if(resource_exists::<PhysicsIslands>)
                .in_set(PhysicsStepSystems::Sleeping),
        );
    }
}

fn sleep_on_add_sleeping(mut world: DeferredWorld, ctx: HookContext) {
    let Some(body_island) = world.get::<BodyIslandNode>(ctx.entity) else {
        return;
    };

    let island_id = body_island.island_id;

    // Check if the island is already sleeping.
    if let Some(island) = world
        .get_resource::<PhysicsIslands>()
        .and_then(|islands| islands.get(island_id))
        && island.is_sleeping
    {
        return;
    }

    world.commands().queue_silenced(SleepBody(ctx.entity));
}

fn wake_on_remove_sleeping(mut world: DeferredWorld, ctx: HookContext) {
    let Some(body_island) = world.get::<BodyIslandNode>(ctx.entity) else {
        return;
    };

    let island_id = body_island.island_id;

    // Check if the island is already awake.
    if let Some(island) = world
        .get_resource::<PhysicsIslands>()
        .and_then(|islands| islands.get(island_id))
        && !island.is_sleeping
    {
        return;
    }

    world.commands().queue_silenced(WakeBody(ctx.entity));
}

fn wake_on_replace_rigid_body(
    trigger: On<Replace, RigidBody>,
    mut commands: Commands,
    query: Query<&BodyIslandNode>,
) {
    let Ok(body_island) = query.get(trigger.entity) else {
        return;
    };

    commands.queue(WakeIslands(vec![body_island.island_id]));
}

fn wake_on_enable_rigid_body(
    trigger: On<Insert, BodyIslandNode>,
    mut commands: Commands,
    mut query: Query<
        (&BodyIslandNode, &mut SleepTimer, Has<Sleeping>),
        Or<(With<Disabled>, Without<Disabled>)>,
    >,
) {
    let Ok((body_island, mut sleep_timer, is_sleeping)) = query.get_mut(trigger.entity) else {
        return;
    };

    if is_sleeping {
        commands.entity(trigger.entity).try_remove::<Sleeping>();
    }

    // Reset the sleep timer and wake up the island.
    if sleep_timer.0 > 0.0 {
        sleep_timer.0 = 0.0;
        commands.queue(WakeIslands(vec![body_island.island_id]));
    }
}

/// A bit vector that stores which islands are kept awake and which are allowed to sleep.
#[derive(Resource, Default, Deref, DerefMut)]
pub(crate) struct AwakeIslandBitVec(pub(crate) BitVec);

fn wake_islands_with_sleeping_disabled(
    mut awake_island_bit_vec: ResMut<AwakeIslandBitVec>,
    mut query: Query<
        (&BodyIslandNode, &mut SleepTimer),
        Or<(
            With<SleepingDisabled>,
            With<Disabled>,
            With<RigidBodyDisabled>,
        )>,
    >,
) {
    // Wake up all islands that have a body with `SleepingDisabled`.
    for (body_island, mut sleep_timer) in &mut query {
        awake_island_bit_vec.set_and_grow(body_island.island_id.0 as usize);

        // Reset the sleep timer.
        sleep_timer.0 = 0.0;
    }
}

fn update_sleeping_states(
    mut awake_island_bit_vec: ResMut<AwakeIslandBitVec>,
    mut islands: ResMut<PhysicsIslands>,
    mut query: Query<
        (
            &mut SleepTimer,
            &SleepThreshold,
            &SolverBody,
            &BodyIslandNode,
        ),
        (Without<Sleeping>, Without<SleepingDisabled>),
    >,
    length_unit: Res<PhysicsLengthUnit>,
    time_to_sleep: Res<TimeToSleep>,
    time: Res<Time>,
) {
    let length_unit_squared = length_unit.0 * length_unit.0;
    let delta_secs = time.delta_secs();

    islands.split_candidate_sleep_timer = 0.0;

    // TODO: This would be nice to do in parallel.
    for (mut sleep_timer, sleep_threshold, solver_body, island_data) in query.iter_mut() {
        let lin_vel_squared = solver_body.linear_velocity.length_squared();
        #[cfg(feature = "2d")]
        let ang_vel_squared = solver_body.angular_velocity * solver_body.angular_velocity;
        #[cfg(feature = "3d")]
        let ang_vel_squared = solver_body.angular_velocity.length_squared();

        // Keep signs.
        let lin_threshold_squared = sleep_threshold.linear * sleep_threshold.linear.abs();
        let ang_threshold_squared = sleep_threshold.angular * sleep_threshold.angular.abs();

        if lin_vel_squared < length_unit_squared * lin_threshold_squared as Scalar
            && ang_vel_squared < ang_threshold_squared as Scalar
        {
            // Increment the sleep timer.
            sleep_timer.0 += delta_secs;
        } else {
            // Reset the sleep timer if the body is moving.
            sleep_timer.0 = 0.0;
        }

        if sleep_timer.0 < time_to_sleep.0 {
            // Keep the island awake.
            awake_island_bit_vec.set_and_grow(island_data.island_id.0 as usize);
        } else if let Some(island) = islands.get(island_data.island_id)
            && island.constraints_removed > 0
        {
            // The body wants to sleep, but its island needs splitting first.
            if sleep_timer.0 > islands.split_candidate_sleep_timer {
                // This island is now the sleepiest candidate for splitting.
                islands.split_candidate = Some(island_data.island_id);
                islands.split_candidate_sleep_timer = sleep_timer.0;
            }
        }
    }
}

fn sleep_islands(
    mut awake_island_bit_vec: ResMut<AwakeIslandBitVec>,
    mut islands: ResMut<PhysicsIslands>,
    mut commands: Commands,
    mut sleep_buffer: Local<Vec<IslandId>>,
    mut wake_buffer: Local<Vec<IslandId>>,
) {
    // Clear the buffers.
    sleep_buffer.clear();
    wake_buffer.clear();

    // Sleep islands that are not in the awake bit vector.
    for island in islands.iter_mut() {
        if awake_island_bit_vec.get(island.id.0 as usize) {
            if island.is_sleeping {
                wake_buffer.push(island.id);
            }
        } else if !island.is_sleeping && island.constraints_removed == 0 {
            // The island does not have a pending split, so it can go to sleep.
            sleep_buffer.push(island.id);
        }
    }

    // Sleep islands.
    let sleep_buffer = sleep_buffer.clone();
    commands.queue(|world: &mut World| {
        SleepIslands(sleep_buffer).apply(world);
    });

    // Wake islands.
    let wake_buffer = wake_buffer.clone();
    commands.queue(|world: &mut World| {
        WakeIslands(wake_buffer).apply(world);
    });

    // Reset the awake island bit vector.
    awake_island_bit_vec.set_bit_count_and_clear(islands.len());
}

#[derive(Resource)]
struct CachedBodySleepingSystemState(
    SystemState<(
        SQuery<&'static mut BodyIslandNode, Or<(With<Disabled>, Without<Disabled>)>>,
        SQuery<&'static RigidBodyColliders>,
        SResMut<PhysicsIslands>,
        SResMut<ContactGraph>,
        SResMut<JointGraph>,
    )>,
);

/// A [`Command`] that forces a [`RigidBody`] and its [`PhysicsIsland`][super::PhysicsIsland] to be [`Sleeping`].
pub struct SleepBody(pub Entity);

impl Command<Result> for SleepBody {
    fn apply(self, world: &mut World) -> Result {
        if let Ok(entity) = world.get_entity(self.0) {
            if let Some(island_id) = entity.get::<BodyIslandNode>().map(|node| node.island_id) {
                world.try_resource_scope(|world, mut state: Mut<CachedBodySleepingSystemState>| {
                    let (
                        mut body_islands,
                        body_colliders,
                        mut islands,
                        mut contact_graph,
                        mut joint_graph,
                    ) = state.0.get_mut(world);

                    let Some(island) = islands.get_mut(island_id) else {
                        return;
                    };

                    // The island must be split before it can be woken up.
                    // Note that this is expensive.
                    if island.constraints_removed > 0 {
                        islands.split_island(
                            island_id,
                            &mut body_islands,
                            &body_colliders,
                            &mut contact_graph,
                            &mut joint_graph,
                        );
                    }

                    // The ID of the body's island might have changed due to the split,
                    // so we need to retrieve it again.
                    let island_id = body_islands.get(self.0).map(|node| node.island_id).unwrap();

                    // Sleep the island.
                    SleepIslands(vec![island_id]).apply(world);
                });
                Ok(())
            } else {
                Err(format!(
                    "Tried to sleep entity {:?} that is not a body or does not belong to an island",
                    self.0
                )
                .into())
            }
        } else {
            Err(format!("Tried to sleep entity {:?} that does not exist", self.0).into())
        }
    }
}

#[derive(Resource)]
struct CachedIslandSleepingSystemState(
    SystemState<(
        SQuery<(
            &'static BodyIslandNode,
            &'static mut SleepTimer,
            Option<&'static RigidBodyColliders>,
        )>,
        SResMut<PhysicsIslands>,
        SResMut<ContactGraph>,
        SResMut<ConstraintGraph>,
    )>,
);

/// A [`Command`] that makes the [`PhysicsIsland`](super::PhysicsIsland)s with the given IDs sleep if they are not already sleeping.
pub struct SleepIslands(pub Vec<IslandId>);

impl Command for SleepIslands {
    fn apply(self, world: &mut World) {
        world.try_resource_scope(|world, mut state: Mut<CachedIslandSleepingSystemState>| {
            let (bodies, mut islands, mut contact_graph, mut constraint_graph) =
                state.0.get_mut(world);

            let mut bodies_to_sleep = Vec::<(Entity, Sleeping)>::new();

            for island_id in self.0 {
                if let Some(island) = islands.get_mut(island_id) {
                    if island.is_sleeping {
                        // The island is already sleeping, no need to sleep it again.
                        return;
                    }

                    island.is_sleeping = true;

                    let mut body = island.head_body;

                    while let Some(entity) = body {
                        let Ok((body_island, _, colliders)) = bodies.get(entity) else {
                            body = None;
                            continue;
                        };

                        // Transfer the contact pairs to the sleeping set, and remove the body from the constraint graph.
                        if let Some(colliders) = colliders {
                            for collider in colliders {
                                contact_graph.sleep_entity_with(collider, |graph, contact_pair| {
                                    // Remove touching contacts from the constraint graph.
                                    if !contact_pair.is_touching()
                                        || !contact_pair.generates_constraints()
                                    {
                                        return;
                                    }
                                    let contact_edge = graph
                                    .get_edge_mut_by_id(contact_pair.contact_id)
                                    .unwrap_or_else(|| {
                                        panic!(
                                            "Contact edge with id {:?} not found in contact graph.",
                                            contact_pair.contact_id
                                        )
                                    });
                                    if let (Some(body1), Some(body2)) =
                                        (contact_pair.body1, contact_pair.body2)
                                    {
                                        for _ in 0..contact_edge.constraint_handles.len() {
                                            constraint_graph.pop_manifold(
                                                &mut graph.edges,
                                                contact_pair.contact_id,
                                                body1,
                                                body2,
                                            );
                                        }
                                    }
                                });
                            }
                        }

                        bodies_to_sleep.push((entity, Sleeping));
                        body = body_island.next;
                    }
                }
            }

            // Batch insert `Sleeping` to the bodies.
            world.insert_batch(bodies_to_sleep);
        });
    }
}

#[derive(Resource)]
struct CachedIslandWakingSystemState(
    SystemState<(
        SQuery<(
            &'static BodyIslandNode,
            &'static mut SleepTimer,
            Option<&'static RigidBodyColliders>,
        )>,
        SResMut<PhysicsIslands>,
        SResMut<ContactGraph>,
        SResMut<ConstraintGraph>,
    )>,
);

/// A [`Command`] that wakes up a [`RigidBody`] and its [`PhysicsIsland`](super::PhysicsIsland) if it is [`Sleeping`].
pub struct WakeBody(pub Entity);

impl Command<Result> for WakeBody {
    fn apply(self, world: &mut World) -> Result {
        if let Ok(entity) = world.get_entity(self.0) {
            if let Some(body_island) = entity.get::<BodyIslandNode>() {
                WakeIslands(vec![body_island.island_id]).apply(world);
                Ok(())
            } else {
                Err(format!(
                    "Tried to wake entity {:?} that is not a body or does not belong to an island",
                    self.0
                )
                .into())
            }
        } else {
            Err(format!("Tried to wake entity {:?} that does not exist", self.0).into())
        }
    }
}

/// A [`Command`] that wakes up the [`PhysicsIsland`](super::PhysicsIsland)s with the given IDs if they are sleeping.
pub struct WakeIslands(pub Vec<IslandId>);

impl Command for WakeIslands {
    fn apply(self, world: &mut World) {
        world.try_resource_scope(|world, mut state: Mut<CachedIslandWakingSystemState>| {
            let (mut bodies, mut islands, mut contact_graph, mut constraint_graph) =
                state.0.get_mut(world);

            let mut bodies_to_wake = Vec::<Entity>::new();

            for island_id in self.0 {
                if let Some(island) = islands.get_mut(island_id) {
                    if !island.is_sleeping {
                        // The island is not sleeping, no need to wake it up.
                        continue;
                    }

                    island.is_sleeping = false;

                    let mut body = island.head_body;

                    while let Some(entity) = body {
                        let Ok((body_island, mut sleep_timer, colliders)) = bodies.get_mut(entity)
                        else {
                            body = None;
                            continue;
                        };

                        // Transfer the contact pairs to the awake set, and add touching contacts to the constraint graph.
                        if let Some(colliders) = colliders {
                            for collider in colliders {
                                contact_graph.wake_entity_with(collider, |graph, contact_pair| {
                                    // Add touching contacts to the constraint graph.
                                    if !contact_pair.is_touching()
                                        || !contact_pair.generates_constraints()
                                    {
                                        return;
                                    }
                                    let contact_edge = graph
                                    .get_edge_mut_by_id(contact_pair.contact_id)
                                    .unwrap_or_else(|| {
                                        panic!(
                                            "Contact edge with id {:?} not found in contact graph.",
                                            contact_pair.contact_id
                                        )
                                    });
                                    for _ in contact_pair.manifolds.iter() {
                                        constraint_graph.push_manifold(contact_edge, contact_pair);
                                    }
                                });
                            }
                        }

                        bodies_to_wake.push(entity);
                        body = body_island.next;
                        sleep_timer.0 = 0.0;
                    }
                }
            }

            // Remove `Sleeping` from the bodies.
            bodies_to_wake.into_iter().for_each(|entity| {
                world.entity_mut(entity).remove::<Sleeping>();
            });
        });
    }
}

#[cfg(feature = "2d")]
type ConstantForceChanges = Or<(
    Changed<ConstantForce>,
    Changed<ConstantTorque>,
    Changed<ConstantLinearAcceleration>,
    Changed<ConstantAngularAcceleration>,
    Changed<ConstantLocalForce>,
    Changed<ConstantLocalLinearAcceleration>,
)>;
#[cfg(feature = "3d")]
type ConstantForceChanges = Or<(
    Changed<ConstantForce>,
    Changed<ConstantTorque>,
    Changed<ConstantLinearAcceleration>,
    Changed<ConstantAngularAcceleration>,
    Changed<ConstantLocalForce>,
    Changed<ConstantLocalTorque>,
    Changed<ConstantLocalLinearAcceleration>,
    Changed<ConstantLocalAngularAcceleration>,
)>;

/// Removes the [`Sleeping`] component from sleeping bodies when properties like
/// position, rotation, velocity and external forces are changed by the user.
fn wake_on_changed(
    mut query: ParamSet<(
        // These could've been changed by physics too.
        // We need to ignore non-user changes.
        Query<
            (
                Ref<Position>,
                Ref<Rotation>,
                Ref<LinearVelocity>,
                Ref<AngularVelocity>,
                Ref<SleepTimer>,
                &BodyIslandNode,
            ),
            (
                With<Sleeping>,
                Or<(
                    Changed<Position>,
                    Changed<Rotation>,
                    Changed<LinearVelocity>,
                    Changed<AngularVelocity>,
                    Changed<SleepTimer>,
                )>,
            ),
        >,
        // These are not modified by the physics engine
        // and don't need special handling.
        Query<&BodyIslandNode, Or<(ConstantForceChanges, Changed<GravityScale>)>>,
    )>,
    mut awake_island_bit_vec: ResMut<AwakeIslandBitVec>,
    last_physics_tick: Res<LastPhysicsTick>,
    system_tick: SystemChangeTick,
) {
    let this_run = system_tick.this_run();

    for (pos, rot, lin_vel, ang_vel, sleep_timer, body_island) in &query.p0() {
        if is_changed_after_tick(pos, last_physics_tick.0, this_run)
            || is_changed_after_tick(rot, last_physics_tick.0, this_run)
            || is_changed_after_tick(lin_vel, last_physics_tick.0, this_run)
            || is_changed_after_tick(ang_vel, last_physics_tick.0, this_run)
            || is_changed_after_tick(sleep_timer, last_physics_tick.0, this_run)
        {
            awake_island_bit_vec.set_and_grow(body_island.island_id.0 as usize);
        }
    }

    for body_island in &query.p1() {
        awake_island_bit_vec.set_and_grow(body_island.island_id.0 as usize);
    }
}

/// Wakes up all sleeping [`PhysicsIsland`](super::PhysicsIsland)s. Triggered automatically when [`Gravity`] is changed.
fn wake_all_islands(mut commands: Commands, islands: Res<PhysicsIslands>) {
    let sleeping_islands: Vec<IslandId> = islands
        .iter()
        .filter_map(|island| island.is_sleeping.then_some(island.id))
        .collect();

    if !sleeping_islands.is_empty() {
        commands.queue(WakeIslands(sleeping_islands));
    }
}
