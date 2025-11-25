//! Run with:
//! `cargo run --example debugdump_3d > dump.dot  && dot -Tsvg dump.dot > dump.svg`

use avian3d::prelude::*;
use bevy::prelude::*;

fn main() {
    let mut app = App::new();

    app.add_plugins((PhysicsPlugins::default(), PhysicsDebugPlugin));

    // Schedules of interest:
    // - PhysicsSchedule
    // - SubstepSchedule
    // - FixedPostUpdate
    // - Update
    bevy_mod_debugdump::print_schedule_graph(&mut app, PhysicsSchedule);
}
