use bevy::app::App;

mod cli;
#[cfg(feature = "2d")]
mod dim2;
#[cfg(feature = "3d")]
mod dim3;

/// A benchmark that can be run with the CLI.
#[derive(Clone, Copy, Debug)]
pub struct Benchmark {
    /// The name of the benchmark.
    pub name: &'static str,
    /// The name of the module where the benchmark is defined.
    pub module: &'static str,
    /// A function that constructs the benchmark application.
    pub constructor: fn() -> App,
}

impl Benchmark {
    /// Creates a new benchmark with the given name, module, and constructor function.
    pub const fn new(name: &'static str, module: &'static str, constructor: fn() -> App) -> Self {
        Self {
            name,
            module,
            constructor,
        }
    }
}

fn main() {
    let benchmarks = [
        #[cfg(feature = "2d")]
        dim2::BENCHMARKS,
        #[cfg(feature = "3d")]
        dim3::BENCHMARKS,
    ]
    .concat();

    if benchmarks.is_empty() {
        eprintln!("No benchmarks available. Please enable either the `2d` or `3d` feature.");
        return;
    }

    let args = cli::Args::parse();

    if args.list {
        cli::list(&benchmarks);
        return;
    }

    if let Some(name) = &args.name {
        // Run the given benchmark with the specified number of threads.
        if let Some(benchmark) = benchmarks.iter().find(|b| b.name == name) {
            cli::run(benchmark, &args);
        } else if let Ok(Some(benchmark)) = name.parse::<usize>().map(|i| benchmarks.get(i - 1)) {
            cli::run(benchmark, &args);
        } else {
            eprintln!("Benchmark '{name}' not found. Use --list to see available benchmarks.",);
        }
    } else {
        // If no specific benchmark is requested, run all benchmarks.
        cli::run_all(&benchmarks, &args);
    }
}
