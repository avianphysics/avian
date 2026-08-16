//! Regression tests for island contact accounting.
//!
//! These hit the real physics schedule: narrow phase records contact status
//! changes, and the solver applies them to [`PhysicsIslands`].

use core::time::Duration;

use bevy::{prelude::*, time::TimeUpdateStrategy};

use crate::{
    collision::contact_types::{ContactGraph, ContactId},
    dynamics::solver::islands::PhysicsIslands,
    prelude::*,
};

#[cfg(all(
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
fn create_app() -> App {
    let mut app = App::new();
    app.add_plugins((
        MinimalPlugins,
        TransformPlugin,
        PhysicsPlugins::default(),
        bevy::asset::AssetPlugin::default(),
        #[cfg(feature = "bevy_scene")]
        bevy::scene::ScenePlugin,
        #[cfg(all(feature = "collider-from-mesh", feature = "default-collider"))]
        bevy::mesh::MeshPlugin,
    ))
    .insert_resource(Gravity::ZERO)
    .insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f32(
        1.0 / 60.0,
    )));
    app.finish();
    app
}

#[cfg(all(
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
fn tick(app: &mut App) {
    app.update();
}

#[cfg(all(
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
fn spawn_box(app: &mut App, body: RigidBody, pos: Vector) -> Entity {
    #[cfg(feature = "2d")]
    let collider = Collider::rectangle(1.0, 1.0);
    #[cfg(feature = "3d")]
    let collider = Collider::cuboid(1.0, 1.0, 1.0);

    app.world_mut()
        .spawn((Transform::default(), body, Position(pos), collider))
        .id()
}

#[cfg(all(
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
fn generating_touching_count(app: &App) -> usize {
    app.world()
        .resource::<ContactGraph>()
        .iter_active()
        .filter(|pair| pair.is_touching() && pair.generates_constraints())
        .count()
}

#[cfg(all(
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
fn island_contact_count(app: &App) -> u32 {
    app.world()
        .resource::<PhysicsIslands>()
        .iter()
        .map(|island| island.contact_count())
        .sum()
}

/// Island contact lists must match touching constraint-generating pairs.
#[cfg(all(
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
fn assert_island_contacts_match_graph(app: &App) {
    assert_eq!(
        island_contact_count(app) as usize,
        generating_touching_count(app),
        "island contact_count must match touching constraint-generating pairs"
    );

    let islands = app.world().resource::<PhysicsIslands>();
    for island in islands.iter() {
        island.validate_contacts(islands.contact_nodes_for_test());
    }
}

#[cfg(all(
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
fn first_generating_contact_id(app: &App) -> ContactId {
    app.world()
        .resource::<ContactGraph>()
        .iter_active()
        .find(|pair| pair.is_touching() && pair.generates_constraints())
        .map(|pair| pair.contact_id)
        .expect("expected a touching constraint-generating contact")
}

/// Resting contacts stay linked through a normal separate-and-recontact cycle.
#[test]
#[cfg(all(
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
fn resting_contact_survives_separate_and_recontact() {
    let mut app = create_app();

    spawn_box(&mut app, RigidBody::Static, Vector::ZERO);
    let dynamic = spawn_box(&mut app, RigidBody::Dynamic, Vector::Y * 0.9);

    for _ in 0..10 {
        tick(&mut app);
    }
    assert!(generating_touching_count(&app) >= 1);
    assert_island_contacts_match_graph(&app);

    app.world_mut()
        .entity_mut(dynamic)
        .insert(Position(Vector::Y * 20.0));
    for _ in 0..10 {
        tick(&mut app);
    }
    assert_eq!(generating_touching_count(&app), 0);
    assert_island_contacts_match_graph(&app);

    app.world_mut()
        .entity_mut(dynamic)
        .insert(Position(Vector::Y * 0.9));
    for _ in 0..10 {
        tick(&mut app);
    }
    assert!(generating_touching_count(&app) >= 1);
    assert_island_contacts_match_graph(&app);
}

/// A duplicate Started for a live contact must not leave a stale head after Stopped.
///
/// This is the #1025 lifecycle: add_contact used to overwrite the island node for a
/// ContactId that was already linked, so remove_contact left `head_contact` pointing
/// at a taken slot. The next add then panicked with "Head contact has no island".
#[test]
#[cfg(all(
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
fn duplicate_started_then_stopped_does_not_leave_stale_head() {
    let mut app = create_app();

    spawn_box(&mut app, RigidBody::Static, Vector::ZERO);
    let dynamic = spawn_box(&mut app, RigidBody::Dynamic, Vector::Y * 0.9);

    for _ in 0..10 {
        tick(&mut app);
    }
    assert_island_contacts_match_graph(&app);

    let contact_id = first_generating_contact_id(&app);
    app.world_mut()
        .resource_mut::<ContactStatusChangeQueue>()
        .push(ContactStatusChange::StartedGeneratingConstraints(
            contact_id,
        ));
    tick(&mut app);
    assert_island_contacts_match_graph(&app);

    app.world_mut()
        .entity_mut(dynamic)
        .insert(Position(Vector::Y * 20.0));
    for _ in 0..10 {
        tick(&mut app);
    }
    assert_eq!(generating_touching_count(&app), 0);
    assert_island_contacts_match_graph(&app);

    app.world_mut()
        .entity_mut(dynamic)
        .insert(Position(Vector::Y * 0.9));
    for _ in 0..10 {
        tick(&mut app);
    }
    assert!(generating_touching_count(&app) >= 1);
    assert_island_contacts_match_graph(&app);
}

/// Clearing GENERATE_CONSTRAINTS and then destroying the pair must unlink the island
/// contact. Otherwise a later pair can reuse the ContactId while `head_contact` is stale.
#[test]
#[cfg(all(
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
fn disabled_pair_destroy_unlinks_island_contact() {
    let mut app = create_app();

    spawn_box(&mut app, RigidBody::Static, Vector::ZERO);
    let dynamic = spawn_box(&mut app, RigidBody::Dynamic, Vector::Y * 0.9);

    for _ in 0..10 {
        tick(&mut app);
    }
    assert!(generating_touching_count(&app) >= 1);
    assert_island_contacts_match_graph(&app);

    {
        let mut graph = app.world_mut().resource_mut::<ContactGraph>();
        for pair in graph.iter_active_mut() {
            pair.flags.remove(ContactPairFlags::GENERATE_CONSTRAINTS);
        }
    }

    app.world_mut()
        .entity_mut(dynamic)
        .insert(Position(Vector::Y * 20.0));
    for _ in 0..10 {
        tick(&mut app);
    }
    assert_eq!(generating_touching_count(&app), 0);
    assert_island_contacts_match_graph(&app);

    spawn_box(&mut app, RigidBody::Dynamic, Vector::Y * 0.9);
    for _ in 0..10 {
        tick(&mut app);
    }
    assert!(generating_touching_count(&app) >= 1);
    assert_island_contacts_match_graph(&app);
}

/// A leftover generation-stop and a new start in the same still-touching visit
/// must both be applied. Otherwise the start is dropped and the pair stays
/// constraintless after re-enable.
#[test]
#[cfg(all(
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
fn leftover_stop_and_start_in_same_visit_relink() {
    let mut app = create_app();
    app.insert_resource(NarrowPhaseConfig {
        recycle_distance: 0.0,
        ..default()
    });

    spawn_box(&mut app, RigidBody::Static, Vector::ZERO);
    spawn_box(&mut app, RigidBody::Dynamic, Vector::Y * 0.9);

    for _ in 0..10 {
        tick(&mut app);
    }
    assert!(generating_touching_count(&app) >= 1);
    assert_island_contacts_match_graph(&app);

    {
        let mut graph = app.world_mut().resource_mut::<ContactGraph>();
        for pair in graph.iter_active_mut() {
            pair.flags.remove(ContactPairFlags::GENERATE_CONSTRAINTS);
            pair.flags
                .insert(ContactPairFlags::STOPPED_GENERATING_CONSTRAINTS);
        }
    }
    tick(&mut app);
    assert!(generating_touching_count(&app) >= 1);
    assert_island_contacts_match_graph(&app);
}
