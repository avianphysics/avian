//! A command line interface for running physics benchmarks.

use core::{ops::Range, time::Duration};

use bevy::{
    app::{App, PluginsState},
    tasks::{ComputeTaskPool, TaskPoolBuilder},
};
use clap::Parser;

use crate::Benchmark;

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
pub struct Args {
    /// The name or number of the benchmark to run. Leave empty to run all benchmarks.
    #[arg(short, long)]
    pub name: Option<String>,

    /// A range for which thread counts to run the benchmarks with.
    /// Can be specified as `start..end` (exclusive), `start..=end` (inclusive), or `start`.
    #[arg(short, long, value_parser = parse_range, default_value = "1..13")]
    pub threads: Range<u32>,

    /// The number of steps to run for each benchmark.
    #[arg(short, long, default_value_t = 500)]
    pub steps: u32,

    /// The number of times to repeat each benchmark.
    /// The results will be averaged over these repetitions.
    #[arg(short, long, default_value_t = 5)]
    pub repeat: u32,

    /// List all available benchmarks in a numbered list.
    #[arg(short, long)]
    pub list: bool,

    /// The output directory where results are written in CSV format.
    /// Leave empty to disable output.
    #[arg(short, long)]
    pub output: Option<String>,
}

impl Args {
    /// Parses the command line arguments and returns an `Args` instance.
    pub fn parse() -> Self {
        Parser::parse()
    }
}

fn parse_range(s: &str) -> Result<Range<u32>, String> {
    let parts: Vec<&str> = s.split("..").collect();
    if parts.len() == 1 {
        // Single value, for example "4"
        let start = parts[0].parse::<u32>().map_err(|e| e.to_string())?;
        Ok(start..start + 1)
    } else if parts.len() == 2 {
        // Range for example "4..10" or "4..=10"
        let start = parts[0].parse::<u32>().map_err(|e| e.to_string())?;
        let end = if parts[1].starts_with('=') {
            // Inclusive range
            parts[1][1..].parse::<u32>().map_err(|e| e.to_string())? + 1
        } else {
            // Exclusive range
            parts[1].parse::<u32>().map_err(|e| e.to_string())?
        };
        Ok(start..end)
    } else {
        Err("Invalid range format".to_string())
    }
}

#[derive(Clone, Copy, Debug)]
pub struct BenchmarkOptions {
    /// The number of threads to use for the benchmark.
    pub threads: u32,
    /// The number of steps to run for each benchmark.
    pub steps: u32,
    /// The number of times to repeat the benchmark.
    ///
    /// This is used to average the results over multiple runs.
    pub repeat: u32,
}

pub struct BenchmarkResult {
    /// The average time taken for a single step in the benchmark.
    pub average_time: Duration,
    /// The minimum time taken for a single step in the benchmark.
    pub min_time: Duration,
}

/// Lists all available benchmarks in the console.
pub fn list(benchmarks: &[Benchmark]) {
    println!("Available benchmarks:");
    for (i, benchmark) in benchmarks.iter().enumerate() {
        println!("{:>2}. {}", i + 1, benchmark.name);
    }
}

/// Runs the given benchmark and prints the results to the console.
///
/// If `--output` is specified, the results will also be written to a CSV file
/// in the specified directory.
pub fn run(benchmark: &Benchmark, args: &Args) {
    if args.threads.len() == 1 {
        println!(
            "Running benchmark '{}' with {:?} threads:",
            benchmark.name, args.threads.start
        );
        let result = run_benchmark(
            benchmark.constructor,
            &BenchmarkOptions {
                threads: args.threads.start,
                steps: args.steps,
                repeat: args.repeat,
            },
        );
        println!("Avg step time (s): {}", result.average_time.as_secs_f64());
        println!("Min step time (s): {}", result.min_time.as_secs_f64());
        if let Some(output_dir) = &args.output {
            // Create output directory if it doesn't exist
            std::fs::create_dir_all(output_dir).unwrap();
            // Write the CSV file
            let path = format!(
                "{output_dir}/{}.csv",
                benchmark.name.to_lowercase().replace(" ", "_")
            );
            std::fs::write(
                &path,
                format!(
                    "benchmark,avg_step_ms,min_step_ms\n{},{},{}\n",
                    benchmark.name,
                    result.average_time.as_secs_f64(),
                    result.min_time.as_secs_f64()
                ),
            )
            .unwrap();
            println!("Results written to {path}");
        }
    } else {
        println!(
            "Running benchmark '{}' with threads ranging from {} to {}:",
            benchmark.name,
            args.threads.start,
            args.threads.end - 1
        );
        run_benchmark_with_thread_range(benchmark, args);
    }
}

/// Runs all benchmarks and prints the results to the console.
///
/// If `--output` is specified, the results will also be written to a CSV file
/// in the specified directory.
pub fn run_all(benchmarks: &[Benchmark], args: &Args) {
    if args.threads.len() == 1 {
        println!(
            "Running all benchmarks with {} threads:",
            args.threads.start
        );
        let max_benchmark_name_len = benchmarks.iter().map(|b| b.name.len()).max().unwrap_or(0);
        println!(
            "| benchmark{} | avg time / step | min time / step |",
            " ".repeat(max_benchmark_name_len.max(9) - 9)
        );
        println!(
            "| {} | --------------- | --------------- |",
            "-".repeat(max_benchmark_name_len.max(9))
        );

        let mut results = Vec::new();
        for benchmark in benchmarks {
            let result = run_benchmark(
                benchmark.constructor,
                &BenchmarkOptions {
                    threads: args.threads.start,
                    steps: args.steps,
                    repeat: args.repeat,
                },
            );
            println!(
                "| {:<9} | {:>10.10} ms | {:>10.10} ms |",
                benchmark.name,
                result.average_time.as_secs_f64() * 1000.0,
                result.min_time.as_secs_f64() * 1000.0
            );
            results.push((benchmark.name, result));
        }
        if let Some(output_dir) = &args.output {
            // Create output directory if it doesn't exist
            std::fs::create_dir_all(output_dir).unwrap();
            // Write the CSV file
            let mut csv = String::new();
            csv.push_str("benchmark,avg_step_ms,min_step_ms\n");
            for (name, result) in results {
                csv.push_str(&format!(
                    "{},{:.10},{:.10}\n",
                    name,
                    result.average_time.as_secs_f64() * 1000.0,
                    result.min_time.as_secs_f64() * 1000.0
                ));
            }
            let path = format!("{output_dir}/benchmarks_{}_threads.csv", args.threads.start);
            std::fs::write(&path, csv).unwrap();
            println!("Results written to {path}");
        }
    } else {
        println!(
            "Running all benchmarks with threads ranging from {} to {}:",
            args.threads.start,
            args.threads.end - 1
        );
        for benchmark in benchmarks {
            println!("'{}'", benchmark.name);
            run_benchmark_with_thread_range(benchmark, args);
        }
    }
}

/// Runs a benchmark with a range of thread counts and prints the results to the console.
///
/// If `--output` is specified, the results will also be written to a CSV file
/// in the specified directory.
fn run_benchmark_with_thread_range(benchmark: &Benchmark, args: &Args) {
    println!("| threads | avg time / step | min time / step |");
    println!("| ------- | --------------- | --------------- |");

    let mut step_times = Vec::new();
    for num_threads in args.threads.clone() {
        let result = run_benchmark_with_child_process(
            benchmark.name,
            &BenchmarkOptions {
                threads: num_threads,
                steps: args.steps,
                repeat: args.repeat,
            },
        );
        match result {
            Ok(result) => {
                println!(
                    "|      {num_threads:>2} | {:>12.5} ms | {:>12.5} ms |",
                    result.average_time.as_secs_f64() * 1000.0,
                    result.min_time.as_secs_f64() * 1000.0
                );
                step_times.push(result.average_time);
            }
            Err(e) => {
                eprintln!(
                    "Failed to run benchmark '{}' with {num_threads} threads: {e}",
                    benchmark.name
                );
                continue;
            }
        }
    }
    if step_times.is_empty() {
        eprintln!("No step times recorded for benchmark '{}'", benchmark.name);
    }
    let mut csv = String::new();
    csv.push_str("threads,step_ms\n");
    for (i, time) in step_times.iter().enumerate() {
        csv.push_str(&format!("{},{}\n", i + 1, time.as_secs_f64() * 1000.0));
    }
    if let Some(output_dir) = &args.output {
        // Create output directory if it doesn't exist
        std::fs::create_dir_all(output_dir).unwrap();
        // Write the CSV file
        let path = format!(
            "{output_dir}/{}.csv",
            benchmark.name.to_lowercase().replace(" ", "_")
        );
        std::fs::write(&path, csv).unwrap();
        println!("Results written to {path}");
    }
}

/// Runs a benchmark in a child process and returns the result.
///
/// This is a hacky way to reset the thread pool for each benchmark run.
fn run_benchmark_with_child_process(
    name: &str,
    options: &BenchmarkOptions,
) -> Result<BenchmarkResult, String> {
    // Spawn a child process to run the benchmark with the specified options.
    let exe = std::env::current_exe()
        .map_err(|e| format!("Failed to get current executable path: {e}"))?;
    let output = std::process::Command::new(exe)
        .arg("--name")
        .arg(name)
        .arg("--threads")
        .arg(options.threads.to_string())
        .arg("--steps")
        .arg(options.steps.to_string())
        .arg("--repeat")
        .arg(options.repeat.to_string())
        .stdout(std::process::Stdio::piped())
        // .stderr(std::process::Stdio::null())
        .env("RUSTFLAGS", "-A warnings")
        .spawn()
        .unwrap()
        .wait_with_output()
        .unwrap();

    if !output.status.success() {
        return Err(format!("Benchmark failed with status: {}", output.status));
    }

    // Now, parse the output to extract the average and minimum step times.
    let output_str = String::from_utf8_lossy(&output.stdout);
    let lines: Vec<&str> = output_str.lines().collect();
    if lines.len() < 2 {
        return Err("Unexpected output format: not enough lines".to_string());
    }
    let avg_time = lines[lines.len() - 2]
        .split_whitespace()
        .nth(4)
        .ok_or("Failed to parse average time")?
        .parse::<f64>()
        .map_err(|e| format!("Failed to parse average time: {e}"))?;
    let min_time = lines[lines.len() - 1]
        .split_whitespace()
        .nth(4)
        .ok_or("Failed to parse minimum time")?
        .parse::<f64>()
        .map_err(|e| format!("Failed to parse minimum time: {e}"))?;

    Ok(BenchmarkResult {
        average_time: Duration::from_secs_f64(avg_time),
        min_time: Duration::from_secs_f64(min_time),
    })
}

fn run_benchmark(builder: impl Fn() -> App, options: &BenchmarkOptions) -> BenchmarkResult {
    // Initialize task pools with the specified number of threads.
    // We also limit rayon's global thread pool here in case it's used.
    ComputeTaskPool::get_or_init(|| {
        TaskPoolBuilder::new()
            .num_threads(options.threads as usize)
            .build()
    });
    rayon::ThreadPoolBuilder::new()
        .num_threads(options.threads as usize)
        .build_global()
        .expect("Failed to build global thread pool");

    assert!(
        options.steps > 0,
        "The number of steps must be greater than 0"
    );
    assert!(
        options.repeat > 0,
        "The number of repetitions must be greater than 0"
    );

    let mut average_time = Duration::ZERO;
    let mut average_min_time = Duration::ZERO;

    for _ in 0..options.repeat {
        let mut app = builder();

        while app.plugins_state() != PluginsState::Ready {
            bevy::tasks::tick_global_task_pools_on_main_thread();
        }

        app.finish();
        app.cleanup();

        // Run the initial setup step before starting measurements to avoid skewing the results.
        app.update();

        let mut average_step_time = Duration::ZERO;
        let mut min_step_time = Duration::MAX;

        // Run 100 steps.
        for _ in 0..options.steps {
            // TODO: We should probably use the physics engine's own diagnostic system here?
            //       For this, the callee would need to pass some function that takes `&World`
            //       and fetches the time.
            let start = std::time::Instant::now();
            app.update();
            let step_time = start.elapsed();

            // Update the average time and minimum time.
            average_step_time += step_time;
            if step_time < min_step_time {
                min_step_time = step_time;
            }
        }

        average_step_time /= options.steps;

        // Update the average time for this repetition.
        average_time += average_step_time;
        average_min_time += min_step_time;
    }

    // Divide the total time by the number of runs to get the average.
    average_time /= options.repeat;
    average_min_time /= options.repeat;

    BenchmarkResult {
        average_time,
        min_time: average_min_time,
    }
}
