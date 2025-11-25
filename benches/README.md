# Avian Benchmarks

Benchmarks for `avian2d` and `avian3d` for measuring performance and scaling in various test scenes.

## Running the Benchmarks

The `benches` CLI is provided for running the benchmarks with various configurations.

```text
Options:
  -n, --name <NAME>                The name or number of the benchmark to run. Leave empty to run all benchmarks
  -t, --threads <THREADS>          A range for which thread counts to run the benchmarks with. Can be specified as `start..end` (exclusive), `start..=end` (inclusive), or `start` [default: 1..13]
  -s, --steps <STEPS>              The number of steps to run for each benchmark [default: 300]
  -r, --repeat <REPEAT>            The number of times to repeat each benchmark. The results will be averaged over these repetitions [default: 5]
  -l, --list                       List all available benchmarks in a numbered list
  -o, --output <OUTPUT>            The output directory where results are written in CSV format. Leave empty to disable output
  -h, --help                       Print help
  -V, --version                    Print version
```

An example of running the "Large Pyramid 2D" benchmark with 1-6 threads and 500 steps, repeated 5 times:

```shell
# Note: Make sure `benches` is the active working directory
cargo run -- --name "Large Pyramid 2D" --threads 1..=6 --steps 500 --repeat 5
```

The output might look something like this:

```text
Running benchmark 'Large Pyramid 2D' with threads ranging from 1 to 6:
| threads | avg time / step | min time / step |
| ------- | --------------- | --------------- |
|       1 |     12.29045 ms |     11.22260 ms |
|       2 |     10.40321 ms |      9.27592 ms |
|       3 |      9.65242 ms |      8.53754 ms |
|       4 |      9.19108 ms |      8.15204 ms |
|       5 |      9.03052 ms |      8.08754 ms |
|       6 |      8.91510 ms |      7.87406 ms |
```

To only run benchmarks for a specific dimension, disable default features and enable the `2d` or `3d` feature:

```shell
# List all 2D benchmarks
cargo run --no-default-features --features 2d -- --list

# Run all 3D benchmarks with default options
cargo run --no-default-features --features 3d
```

Note that the `dev` profile has been configured to be equivalent to the `release` profile
in the `Cargo.toml`, so the `--release` flag is optional.
