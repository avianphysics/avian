use core::time::Duration;

use avian3d::prelude::SubstepCount;
use bevy::{
    MinimalPlugins,
    app::{App, Plugin, PluginGroup, PluginGroupBuilder},
    time::{Time, TimeUpdateStrategy},
    transform::TransformPlugin,
};

use crate::Benchmark;

mod large_pyramid;
mod many_pyramids;

/// All benchmarks for `avian3d`.
pub const BENCHMARKS: &[Benchmark] = &[
    Benchmark::new("Large Pyramid 3D", "pyramid", || {
        large_pyramid::create_bench(100)
    }),
    Benchmark::new("Many Pyramids 3D", "many_pyramids", || {
        many_pyramids::create_bench(10, 10, 10)
    }),
];

/// A plugin group that includes the minimal set of plugins used for benchmarking `avian3d`.
pub struct Benchmark3dPlugins;

impl PluginGroup for Benchmark3dPlugins {
    fn build(self) -> bevy::app::PluginGroupBuilder {
        PluginGroupBuilder::start::<Benchmark3dPlugins>()
            .add_group(MinimalPlugins)
            .add(Benchmark3dCorePlugin)
            .add(TransformPlugin)
    }
}

/// A plugin that sets up the core resources and configuration for benchmarking.
struct Benchmark3dCorePlugin;

impl Plugin for Benchmark3dCorePlugin {
    fn build(&self, app: &mut App) {
        // All benchmarks use a fixed time step of 60 FPS with 4 substeps.
        app.insert_resource(Time::from_hz(60.0));
        app.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f64(
            1.0 / 60.0,
        )));
        app.insert_resource(SubstepCount(4));
    }
}
