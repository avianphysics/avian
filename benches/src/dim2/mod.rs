use core::time::Duration;

use avian2d::prelude::SubstepCount;
use bevy::{
    MinimalPlugins,
    app::{App, Plugin, PluginGroup, PluginGroupBuilder},
    time::{Time, TimeUpdateStrategy},
    transform::TransformPlugin,
};

use crate::Benchmark;

mod large_pyramid;
mod many_pyramids;

/// All benchmarks for `avian2d`.
pub const BENCHMARKS: &[Benchmark] = &[
    Benchmark::new("Large Pyramid 2D", "pyramid", || {
        large_pyramid::create_bench(100)
    }),
    Benchmark::new("Many Pyramids 2D", "many_pyramids", || {
        many_pyramids::create_bench(10, 10, 10)
    }),
];

/// A plugin group that includes the minimal set of plugins used for benchmarking `avian2d`.
pub struct Benchmark2dPlugins;

impl PluginGroup for Benchmark2dPlugins {
    fn build(self) -> bevy::app::PluginGroupBuilder {
        PluginGroupBuilder::start::<Benchmark2dPlugins>()
            .add_group(MinimalPlugins)
            .add(Benchmark2dCorePlugin)
            .add(TransformPlugin)
    }
}

/// A plugin that sets up the core resources and configuration for benchmarking.
struct Benchmark2dCorePlugin;

impl Plugin for Benchmark2dCorePlugin {
    fn build(&self, app: &mut App) {
        // All benchmarks use a fixed time step of 60 FPS with 4 substeps.
        app.insert_resource(Time::from_hz(60.0));
        app.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f64(
            1.0 / 60.0,
        )));
        app.insert_resource(SubstepCount(4));
    }
}
