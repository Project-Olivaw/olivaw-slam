# 12 — Roadmap & beating slam_toolbox

What is missing, what should improve, and where this project can genuinely
exceed the incumbent. Ordered by leverage.

## Near-term gaps (highest value first)

1. **Motion deskew** (`preprocess`). The C1 takes ~120 ms per revolution;
   points within one scan are captured at different poses. At walking pace
   this smears a few centimetres; on a fast-turning robot it dominates error.
   Design: interpolate per-point poses across the revolution from consecutive
   pose estimates (or an IMU/odometry input when available) and correct each
   point before matching. This is the single biggest map-quality win
   available.
2. **The slam_toolbox oracle diff.** CLAUDE.md names slam_toolbox as the
   oracle: run the same recording through ROS2 slam_toolbox once (in Docker,
   development-time only), save its map and trajectory, and diff ours in CI
   against the saved output. Needs a real circuit recording of a house.
   The export formats (PGM+YAML, poses) already exist on our side. Also the
   README side-by-side image comes from this.
3. **Odometry input.** `process_scan` currently starts every match from the
   previous pose. An optional `process_scan_with_odometry(cloud, delta)` would
   shrink CSM windows (faster), survive featureless stretches, and make the
   localization smoother meaningful. `PoseGraph::add_prior` and the edge API
   are already shaped for it.
4. **Point-to-line ICP.** Removes the ~1.5 cm point-to-point sampling bias;
   useful as a cheap high-precision refiner after CSM. The `ScanMatcher`
   trait, tests, and benches make this a well-scoped addition.
5. **Range-scaled outlier rejection** (pitfall P8): scale the neighbour
   radius with range so distant wall points survive on sparse sensors.
6. **CI**: GitHub Actions running test + clippy + fmt + `cargo deny`
   (licence/advisory audit) + the aarch64 cross-check of
   `--features "parallel serialize"` + criterion regression gating.
   Everything is already green locally; the definition of done requires it
   enforced.

## Structural improvements (when scale demands)

- **Submaps** (the planned growth mechanism): bounded local grids stitched by
  the pose graph, replacing the fixed 30 m grid; enables building-scale maps
  and cheaper loop-closure rebuilds (re-render affected submaps only).
- **Keyframe culling / map maintenance** for lifelong mapping: without it the
  keyframe list and rebuild cost grow forever.
- **k-d tree candidate search** in loop closure once keyframe counts leave
  house scale (the linear scan is fine below ~10k).
- **Incremental grid updates after loop closure**: rebuild-from-scratch is
  correct and simple; at building scale, only re-integrate keyframes whose
  pose moved more than a threshold.
- **`no_std` core**: the `std = []` marker exists; the geometry/matching core
  has no inherent std dependency. Would open microcontroller-class targets.

## Where we can beat slam_toolbox (the honest comparison)

Already ahead, structurally:

| axis | why we win |
|---|---|
| portability | pure-Rust deployable config; cross-compiles with one flag; no Ubuntu/ROS-version coupling; WASM is reachable |
| embeddability | a library you call, not a node graph you join; no threads, no middleware, no lifecycle manager |
| runtime predictability | CSM cost is bounded by window configuration, not data; slam_toolbox's Ceres-based matcher is data-dependent |
| memory safety | `forbid(unsafe_code)` end to end; a panic-free library contract |
| reproducibility | deterministic preprocessing and replay; same recording in, same map out, byte-for-byte serialization round-trips |
| tooling | rerun-first debugging (score surfaces, live maps) beats RViz for algorithm work |

Where slam_toolbox is still ahead, and what closing each gap takes:

| axis | their edge | closing it |
|---|---|---|
| maturity | years of real-world deployments and tuning | miles on real robots; the oracle diff harness is how we measure convergence |
| motion handling | scan deskew via odometry/TF | roadmap item 1 |
| map scale | Karto's submap-like spatial structure | submaps |
| interactive tools | pose-graph manipulation GUI, localization init tools | a rerun-based inspector; blueprint-driven UI |
| ecosystem | drop-in for the whole ROS2 nav stack | the bridge crate (below) |

Realistic verdict: at house scale, with deskew added and the oracle diff
proving parity, this is a credible slam_toolbox replacement for Rust-native
robots — and a strictly better foundation for embedded and cross-platform
work. Warehouse scale needs the structural items.

## The bridge strategy (unchanged from CLAUDE.md)

ROS2 interop is a real commercial requirement and stays a **separate crate**:
`olivaw-ros2-bridge` converting `ScanCloud`/`Pose2`/`OccupancyGrid` to
`sensor_msgs/LaserScan`, `geometry_msgs/PoseStamped`, `nav_msgs/OccupancyGrid`.
The types here are message-shaped on purpose. Same pattern later for a
`Nav2`-equivalent (`olivaw-nav`): planning and control consume `OccupancyGrid`
and `Pose2` — nothing in this crate needs to change.

## Deferred decisions worth re-examining someday

- Localization-mode graph smoothing (deliberately skipped without odometry;
  see doc 09).
- Cartographer-style gentler sensor model (0.55/0.49) once real-world map
  quality can be compared via the oracle harness.
- factrs alternatives if graphs outgrow rebuild-per-optimize; the wrapper
  boundary in `graph/mod.rs` is the only file that would change.
- A structured recording format (timestamped scans, serde) if replay ever
  needs more than raw serial byte streams.
