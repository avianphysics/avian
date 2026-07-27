use bevy::{
    ecs::{entity_disabling::Disabled, query::QueryFilter},
    prelude::*,
};

use super::{SolverBodies, SolverBody, SolverBodyIndex, SolverBodyInertia};
use crate::{
    AngularVelocity, LinearVelocity, PhysicsSchedule, PhysicsTransform, RigidBody,
    RigidBodyActiveFilter, RigidBodyDisabled, Rot, Sleeping, SolverSystems, Vector,
    dynamics::{
        integrator::CustomPositionIntegration,
        solver::{SolverDiagnostics, solver_body::SolverBodyFlags},
    },
    math::ToRealPrecision,
    prelude::{
        AppDiagnosticsExt, ComputedAngularInertia, ComputedCenterOfMass, ComputedMass, Dominance,
        LockedAxes,
    },
    utils::{MIN_PAR_ITER_ENTITIES, ParallelQueryForEach},
};
#[cfg(feature = "3d")]
use crate::{
    MatExt, QuatExt,
    dynamics::integrator::{IntegrationSystems, integrate_positions},
    prelude::SubstepSchedule,
};

/// A plugin for managing solver bodies stored in the [`SolverBodies`] resource.
///
/// A [`SolverBody`] is created for each dynamic and kinematic rigid body when:
///
/// 1. The rigid body is created.
/// 2. The rigid body is enabled by removing [`Disabled`]/[`RigidBodyDisabled`].
/// 3. The rigid body is woken up.
/// 4. The rigid body is set to be dynamic or kinematic.
///
/// A [`SolverBody`] is removed when:
///
/// 1. The rigid body is removed.
/// 2. The rigid body is disabled by adding [`Disabled`]/[`RigidBodyDisabled`].
/// 3. The rigid body is put to sleep.
/// 4. The rigid body is set to be static.
///
/// Each body's position in the [`SolverBodies`] resource is tracked by a [`SolverBodyIndex`]
/// component, which is only present while the body has an associated solver body.
pub struct SolverBodyPlugin;

impl Plugin for SolverBodyPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<SolverBodies>();

        // Add or remove solver bodies when the `RigidBody` component is added or replaced.
        app.add_observer(on_insert_rigid_body);

        // Add a solver body for each dynamic and kinematic rigid body
        // when the associated rigid body is enabled or woken up.
        app.add_observer(
            |trigger: On<Remove, RigidBodyDisabled>,
             rb_query: Query<(&RigidBody, Has<SolverBodyIndex>), Without<Sleeping>>,
             solver_bodies: ResMut<SolverBodies>,
             commands: Commands| {
                add_solver_body::<Without<Sleeping>>(
                    In(trigger.entity),
                    rb_query,
                    solver_bodies,
                    commands,
                );
            },
        );
        app.add_observer(
            |trigger: On<Remove, Disabled>,
             rb_query: Query<
                (&RigidBody, Has<SolverBodyIndex>),
                (
                    // The body still has `Disabled` at this point,
                    // and we need to include it in the query to match against the entity.
                    With<Disabled>,
                    Without<RigidBodyDisabled>,
                    Without<Sleeping>,
                ),
            >,
             solver_bodies: ResMut<SolverBodies>,
             commands: Commands| {
                add_solver_body::<(
                    With<Disabled>,
                    Without<RigidBodyDisabled>,
                    Without<Sleeping>,
                )>(In(trigger.entity), rb_query, solver_bodies, commands);
            },
        );
        app.add_observer(
            |trigger: On<Remove, Sleeping>,
             rb_query: Query<(&RigidBody, Has<SolverBodyIndex>), Without<RigidBodyDisabled>>,
             solver_bodies: ResMut<SolverBodies>,
             commands: Commands| {
                add_solver_body::<Without<RigidBodyDisabled>>(
                    In(trigger.entity),
                    rb_query,
                    solver_bodies,
                    commands,
                );
            },
        );

        // Remove solver bodies when their associated rigid body is removed.
        app.add_observer(
            |trigger: On<Remove, RigidBody>,
             index_query: Query<&mut SolverBodyIndex>,
             solver_bodies: ResMut<SolverBodies>,
             commands: Commands| {
                remove_solver_body(In(trigger.entity), index_query, solver_bodies, commands);
            },
        );

        // Remove solver bodies when their associated rigid body is disabled or put to sleep.
        app.add_observer(
            |trigger: On<Add, (Disabled, RigidBodyDisabled, Sleeping)>,
             index_query: Query<&mut SolverBodyIndex>,
             solver_bodies: ResMut<SolverBodies>,
             commands: Commands| {
                remove_solver_body(In(trigger.entity), index_query, solver_bodies, commands);
            },
        );

        // Prepare solver bodies before the substepping loop.
        app.add_systems(
            PhysicsSchedule,
            prepare_solver_bodies
                .chain()
                .in_set(SolverSystems::PrepareSolverBodies),
        );

        // Write back solver body data to rigid bodies after the substepping loop.
        app.add_systems(
            PhysicsSchedule,
            writeback_solver_bodies.in_set(SolverSystems::Finalize),
        );

        // Update the world-space angular inertia of solver bodies right after position integration
        // in the substepping loop.
        #[cfg(feature = "3d")]
        app.add_systems(
            SubstepSchedule,
            update_solver_body_angular_inertia
                .in_set(IntegrationSystems::Position)
                .after(integrate_positions),
        );
    }

    fn finish(&self, app: &mut App) {
        // Register timer and counter diagnostics for the solver.
        app.register_physics_diagnostics::<SolverDiagnostics>();
    }
}

fn on_insert_rigid_body(
    trigger: On<Insert, RigidBody>,
    bodies: Query<(&RigidBody, Has<SolverBodyIndex>), RigidBodyActiveFilter>,
    index_query: Query<&mut SolverBodyIndex>,
    mut solver_bodies: ResMut<SolverBodies>,
    mut commands: Commands,
) {
    let entity = trigger.entity;
    let Ok((rb, has_solver_body)) = bodies.get(entity) else {
        return;
    };

    if rb.is_static() && has_solver_body {
        // Remove the solver body if the rigid body is now static.
        remove_solver_body_inner(entity, index_query, &mut solver_bodies, &mut commands);
    } else if !rb.is_static() && !has_solver_body {
        // Create a new solver body if the rigid body is dynamic or kinematic.
        add_solver_body_inner(entity, &mut solver_bodies, &mut commands);
    }
}

fn add_solver_body<F: QueryFilter>(
    In(entity): In<Entity>,
    rb_query: Query<(&RigidBody, Has<SolverBodyIndex>), F>,
    mut solver_bodies: ResMut<SolverBodies>,
    mut commands: Commands,
) {
    if let Ok((rb, has_solver_body)) = rb_query.get(entity) {
        if rb.is_static() || has_solver_body {
            return;
        }

        add_solver_body_inner(entity, &mut solver_bodies, &mut commands);
    }
}

/// Pushes a new solver body for `entity` and inserts its [`SolverBodyIndex`] component.
fn add_solver_body_inner(
    entity: Entity,
    solver_bodies: &mut SolverBodies,
    commands: &mut Commands,
) {
    let index = solver_bodies.push(entity, SolverBody::default(), SolverBodyInertia::default());
    commands.entity(entity).try_insert(index);
}

fn remove_solver_body(
    In(entity): In<Entity>,
    index_query: Query<&mut SolverBodyIndex>,
    mut solver_bodies: ResMut<SolverBodies>,
    mut commands: Commands,
) {
    remove_solver_body_inner(entity, index_query, &mut solver_bodies, &mut commands);
}

/// Swap-removes the solver body of `entity` from the [`SolverBodies`] resource, fixes up the
/// [`SolverBodyIndex`] of the body swapped into its slot, and removes the index component.
fn remove_solver_body_inner(
    entity: Entity,
    mut index_query: Query<&mut SolverBodyIndex>,
    solver_bodies: &mut SolverBodies,
    commands: &mut Commands,
) {
    let Ok(index) = index_query.get(entity).copied() else {
        return;
    };
    if !index.is_valid() {
        return;
    }

    // Swap-remove the body. If another body was swapped into this slot, update its index.
    if let Some(swapped_entity) = solver_bodies.swap_remove(index)
        && let Ok(mut swapped_index) = index_query.get_mut(swapped_entity)
    {
        *swapped_index = index;
    }

    commands.entity(entity).try_remove::<SolverBodyIndex>();
}

fn prepare_solver_bodies(
    mut solver_bodies: ResMut<SolverBodies>,
    query: Query<(
        &RigidBody,
        &SolverBodyIndex,
        &LinearVelocity,
        &AngularVelocity,
        &PhysicsTransform,
        &ComputedMass,
        &ComputedAngularInertia,
        Option<&LockedAxes>,
        Option<&Dominance>,
        Has<CustomPositionIntegration>,
    )>,
) {
    let access = solver_bodies.access();

    #[allow(unused_variables)]
    query.par_for_each(
        MIN_PAR_ITER_ENTITIES,
        |(
            rb,
            index,
            linear_velocity,
            angular_velocity,
            physics_transform,
            mass,
            angular_inertia,
            locked_axes,
            dominance,
            custom_position_integration,
        )| {
            // SAFETY: Each entity has a unique, valid solver body index, so the writes below
            //         target disjoint bodies and inertias.
            let solver_body = unsafe { access.body_unchecked_mut(*index) };
            let inertial_properties = unsafe { access.inertia_unchecked_mut(*index) };

            solver_body.linear_velocity = linear_velocity.0;
            solver_body.angular_velocity = angular_velocity.0;
            solver_body.delta_position = Vector::ZERO;
            solver_body.delta_rotation = Rot::IDENTITY;

            let locked_axes = locked_axes.copied().unwrap_or_default();
            *inertial_properties = SolverBodyInertia::new(
                mass.inverse(),
                #[cfg(feature = "2d")]
                angular_inertia.inverse(),
                #[cfg(feature = "3d")]
                angular_inertia
                    .rotated(physics_transform.rotation)
                    .inverse(),
                locked_axes,
                dominance.map_or(0, |dominance| dominance.0),
                rb.is_dynamic(),
            );
            solver_body.flags = SolverBodyFlags(locked_axes.to_bits() as u32);
            solver_body
                .flags
                .set(SolverBodyFlags::IS_KINEMATIC, rb.is_kinematic());
            solver_body.flags.set(
                SolverBodyFlags::CUSTOM_POSITION_INTEGRATION,
                custom_position_integration,
            );

            #[cfg(feature = "3d")]
            {
                // Only compute gyroscopic motion if the following conditions are met:
                //
                // 1. Rotation is unlocked on at least one axis.
                // 2. The inertia tensor is not isotropic.
                //
                // If the inertia tensor is isotropic, it is invariant under all rotations,
                // and the same around any axis passing through the object's center of mass.
                // This is the case for shapes like spheres, as well as cubes and other regular solids.
                //
                // From Moments of inertia of Archimedean solids by Frédéric Perrier:
                // "The condition of two axes of threefold or higher rotational symmetry
                // crossing at the center of gravity is sufficient to produce an isotropic tensor of inertia.
                // The MI around any axis passing by the center of gravity are then identical."

                // TODO: Kinematic bodies should not have gyroscopic motion.
                // TODO: Should we scale the epsilon based on the `PhysicsLengthUnit`?
                // TODO: We should only do this when the body is added or the local inertia tensor is changed.
                let epsilon = 1e-6;
                let is_gyroscopic = !locked_axes.is_rotation_locked()
                    && !angular_inertia.inverse().is_isotropic(epsilon);

                solver_body
                    .flags
                    .set(SolverBodyFlags::GYROSCOPIC_MOTION, is_gyroscopic);
            }
        },
    );
}

/// Writes back solver body data to rigid bodies.
#[allow(clippy::type_complexity)]
fn writeback_solver_bodies(
    solver_bodies: Res<SolverBodies>,
    mut query: Query<(
        &SolverBodyIndex,
        &mut PhysicsTransform,
        &ComputedCenterOfMass,
        &mut LinearVelocity,
        &mut AngularVelocity,
    )>,
    mut diagnostics: ResMut<SolverDiagnostics>,
) {
    let start = bevy::platform::time::Instant::now();

    query.par_for_each_mut(
        MIN_PAR_ITER_ENTITIES,
        |(index, mut physics_transform, com, mut lin_vel, mut ang_vel)| {
            let Some(solver_body) = solver_bodies.get(*index) else {
                return;
            };

            // Write back the position and rotation deltas,
            // rotating the body around its center of mass.
            let old_world_com = physics_transform.rotation * com.0;
            physics_transform.rotation =
                (solver_body.delta_rotation * physics_transform.rotation).fast_renormalize();
            let new_world_com = physics_transform.rotation * com.0;
            physics_transform.translation +=
                (solver_body.delta_position + (old_world_com - new_world_com)).real();

            // Write back velocities.
            lin_vel.0 = solver_body.linear_velocity;
            ang_vel.0 = solver_body.angular_velocity;
        },
    );

    diagnostics.finalize += start.elapsed();
}

#[cfg(feature = "3d")]
pub(crate) fn update_solver_body_angular_inertia(
    mut solver_bodies: ResMut<SolverBodies>,
    mut query: Query<(&SolverBodyIndex, &ComputedAngularInertia, &PhysicsTransform)>,
) {
    let access = solver_bodies.access();

    query.par_for_each_mut(
        MIN_PAR_ITER_ENTITIES,
        |(index, angular_inertia, physics_transform)| {
            // SAFETY: Each entity has a unique, valid solver body index, so the writes below
            //         target disjoint inertias.
            let inertia = unsafe { access.inertia_unchecked_mut(*index) };
            inertia
                .update_effective_inv_angular_inertia(angular_inertia, physics_transform.rotation);
        },
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{PhysicsSchedulePlugin, SolverSchedulePlugin};

    fn create_app() -> App {
        let mut app = App::new();

        app.add_plugins((
            PhysicsSchedulePlugin::default(),
            SolverSchedulePlugin,
            SolverBodyPlugin,
        ));

        app
    }

    fn has_solver_body(app: &App, entity: Entity) -> bool {
        app.world()
            .resource::<SolverBodies>()
            .contains_entity(entity)
            && app.world().get::<SolverBodyIndex>(entity).is_some()
    }

    #[test]
    fn add_remove_solver_bodies() {
        let mut app = create_app();

        // Create a dynamic, kinematic, and static rigid body.
        let entity1 = app.world_mut().spawn(RigidBody::Dynamic).id();
        let entity2 = app.world_mut().spawn(RigidBody::Kinematic).id();
        let entity3 = app.world_mut().spawn(RigidBody::Static).id();

        app.update();

        // The dynamic and kinematic rigid bodies should have solver bodies.
        assert!(has_solver_body(&app, entity1));
        assert!(has_solver_body(&app, entity2));
        assert!(!has_solver_body(&app, entity3));

        // Disable the dynamic rigid body.
        app.world_mut()
            .entity_mut(entity1)
            .insert(RigidBodyDisabled);

        app.update();

        // The entity should no longer have a solver body.
        assert!(!has_solver_body(&app, entity1));

        // Enable the dynamic rigid body.
        app.world_mut()
            .entity_mut(entity1)
            .remove::<RigidBodyDisabled>();

        app.update();

        // The dynamic rigid body should have a solver body again.
        assert!(has_solver_body(&app, entity1));
    }
}
