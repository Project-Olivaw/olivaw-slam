# olivaw-slam

2D lidar SLAM in pure Rust. No ROS2, no C++ dependencies, no distro lock-in.

Part of [Project Olivaw](https://github.com/Project-Olivaw) — tools and examples
for robotics in Rust. Architecturally equivalent to what `slam_toolbox` does
inside ROS2, but as a plain Rust library: it runs on macOS, Linux, and anything
else Rust targets, and cross-compiles to a Raspberry Pi or Jetson with
`cargo build --target aarch64-unknown-linux-gnu`.

## What it does

Feed it lidar scans (from [`olivaw-lidar`](../olivaw-lidar) or any other
source); it produces a consistent occupancy-grid map and a pose estimate.

```rust,no_run
use olivaw_slam::{Slam, SlamConfig, ScanCloud};

let mut slam = Slam::new(SlamConfig::default())?;
for cloud in scans {                       // ScanCloud: metres, x-fwd/y-left
    let pose = slam.process_scan(&cloud)?;
}
slam.grid().export_map("house".as_ref())?; // house.pgm + house.yaml (map_server format)
slam.save("house.olivaw".as_ref())?;       // full state, reloadable (feature "serialize")
```

The pipeline: **preprocess** (range gate, outlier rejection, voxel subsampling)
→ **correlative scan matching** against the accumulated grid (Olson 2009 CSM —
no odometry required, bounded runtime) → **keyframes + pose graph** (backed by
[factrs](https://docs.rs/factrs)) → **loop closure** (candidate search →
wide-window CSM verification → χ²-gated speculative optimization). Every layer
is independently usable: `matcher::`, `grid::`, `graph::`, `loop_closure::`.

## Status

All phases of the 0.1.0 plan are implemented and tested:

- CSM recovers synthetic transforms to < 1 cm / 0.5° (tested).
- Pose graph reproduces the published `M3500.g2o` optimum (χ² ≈ 138, tested).
- On a synthetic noisy circuit with zero odometry: ~0.75 m of accumulated
  drift is corrected to **2.3 cm** final error by loop closure (tested).
- Maps save and reload with bit-identical geometry; localization mode tracks
  in a frozen map without modifying it (tested).
- `#![forbid(unsafe_code)]`, no `unwrap`/`panic!` in library code, clippy
  pedantic clean, benchmarked with criterion.

## Examples (rerun visualization)

```sh
# Occupancy grid from a raw olivaw-lidar recording (Phase 1 milestone):
cargo run --release --example grid_from_recording --features viz -- recording.bin

# Watch a correlative scan match + its score surface:
cargo run --release --example scan_matching_viz --features viz

# Full SLAM on a recording — map, trajectory, loop closures:
cargo run --release --example slam_from_recording --features "viz serialize" -- recording.bin
```

Recordings are raw serial captures made with `olivaw-lidar`'s `record`
example; each example defaults to the committed fixture so a stranger can run
it with no setup. Install the rerun viewer with `uv tool install rerun-sdk`
(or pass `--save out.rrd` to run headless).

## Features

| feature     | default | what it adds |
|-------------|---------|--------------|
| `std`       | ✓       | (marker for future no_std work) |
| `parallel`  |         | rayon-parallel CSM search |
| `serialize` |         | `Slam::save`/`Slam::load`, serde on configs |
| `viz`       |         | `OccupancyGrid::log_to_rerun` + the examples |

The deployable configuration (`parallel serialize`) is pure Rust all the way
down and cross-compiles to `aarch64-unknown-linux-gnu` out of the box; `viz`
(rerun) is development tooling.

## Units

**Metres and radians, everywhere, no exceptions.** Sensor drivers that report
millimetres/degrees (like `olivaw-lidar`) are converted once at the boundary —
see the examples. Angles are `(-π, π]`, CCW-positive, x-forward y-left frame.

See [CLAUDE.md](CLAUDE.md) for the full architecture and engineering rules.
