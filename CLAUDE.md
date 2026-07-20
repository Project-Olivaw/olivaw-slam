# CLAUDE.md — olivaw-slam

> 2D lidar SLAM in pure Rust. No ROS2, no C++ dependencies, no distro lock-in.
> Part of [Project Olivaw](https://github.com/Project-Olivaw) — tools and examples for robotics in Rust.

---

## Project intent

`olivaw-slam` is a 2D graph-based SLAM library. It takes lidar scans (from `olivaw-lidar` or any
other source) and produces a consistent map plus a pose estimate. It is architecturally equivalent
to what `slam_toolbox` does inside ROS2, but as a **plain Rust library with no middleware
dependency**.

**The thesis, stated plainly:** SLAM and ROS2 are separate concerns that got welded together by
history. The algorithms — scan matching, pose graphs, loop closure — have nothing to do with
message passing, TF trees, or Ubuntu versions. Separating them yields a library that:

- runs on macOS, Linux, and anything else Rust targets
- cross-compiles to a Raspberry Pi or Jetson with `cargo build --target`
- has no `apt` dependencies and no distro version constraints
- can be embedded in a binary, a WASM module, or a dora-rs node
- can be *bridged* to ROS2 later for interop, without ROS2 ever being a build dependency

**Deliberately non-goal:** matching Cartographer's accuracy on a warehouse-scale dataset.
The target is "produces a metrically usable map of a house or small building, reliably."
That is a solvable problem and it is the one that matters for the products we are building toward.

---

## Ground truth discipline — read this before writing any algorithm

**You cannot debug SLAM by looking at the code.** The bugs are in the interaction between the
algorithm and physical sensor noise. Therefore:

1. **Every algorithm is developed offline against recorded data first.** Never against a live device.
2. **`slam_toolbox` is the oracle.** Run the same recorded dataset through ROS2 `slam_toolbox`
   once, save its output map and trajectory, and diff our output against it. When ours disagrees,
   ours is wrong until proven otherwise.
3. **Public benchmark datasets are the second oracle.** The Intel Research Lab, MIT CSAIL, and
   ACES datasets (available in Carmen log format from the standard SLAM benchmark collections)
   have known-good reference solutions. `M3500.g2o` is the canonical pose-graph optimization
   benchmark and `factrs` ships it as an example.
4. **Every stage gets a rerun visualization.** If you cannot see it, you cannot debug it. This is
   not optional polish — it is the primary debugging instrument.

The workflow for every new algorithm: implement → run on recorded data → visualize in rerun →
diff against oracle → tune → only then try live.

---

## Architecture

```
                    ┌──────────────────────────────────────────┐
   Scan input  ───► │  preprocess/  filtering, subsampling      │
   (olivaw-lidar,   │               range gating, deskew        │
    replay file)    └──────────────────────────────────────────┘
                                      │
                    ┌──────────────────────────────────────────┐
                    │  matcher/     scan → scan and scan → map  │
                    │               ICP, correlative (CSM)      │
                    │               returns Pose2 + covariance  │
                    └──────────────────────────────────────────┘
                                      │
                    ┌──────────────────────────────────────────┐
                    │  grid/        occupancy grid, log-odds    │
                    │               ray casting, submaps        │
                    └──────────────────────────────────────────┘
                                      │
                    ┌──────────────────────────────────────────┐
                    │  graph/       pose graph — nodes, edges   │
                    │               backed by factrs            │
                    └──────────────────────────────────────────┘
                                      │
                    ┌──────────────────────────────────────────┐
                    │  loop/        place recognition,          │
                    │               candidate search + gating   │
                    └──────────────────────────────────────────┘
                                      │
                    ┌──────────────────────────────────────────┐
                    │  slam.rs      orchestration, keyframes,   │
                    │               the public Slam type        │
                    └──────────────────────────────────────────┘
```

**Every layer is independently testable and independently useful.** Someone who only wants a
scan matcher should be able to depend on this crate and use `matcher::` without touching the rest.
Feature-gate aggressively so they don't pay for what they don't use.

---

## Dependencies

```toml
[dependencies]
nalgebra = "0.34"          # linear algebra, SE(2)/SO(2), matrices
factrs = "0.2"             # factor graph / pose graph optimization backend
thiserror = "2"
kiddo = "5"                # k-d tree for nearest neighbour (ICP correspondence)
rayon = { version = "1", optional = true }   # data parallelism, feature-gated

[dev-dependencies]
rerun = "0.34.1"
olivaw-lidar = { path = "../olivaw-lidar" }
criterion = "0.5"          # benchmarks — regressions are bugs
approx = "0.5"

[features]
default = ["std"]
std = []
parallel = ["rayon"]
```

**Do not add:** any C/C++ linking crate, tokio (SLAM is CPU-bound, not IO-bound), OpenCV bindings,
a custom linear algebra implementation (nalgebra is good and battle-tested).

**On `factrs`:** it is the pose-graph backend, API-inspired by GTSAM, benchmarked as the fastest
Rust option and competitive with the C++ libraries. Known gap: GTSAM and Ceres exploit pose-graph
sparsity better via the Bayes tree (iSAM2) on very large graphs. Irrelevant at house scale.
Do not reimplement pose-graph optimization — this is a solved problem and factrs solved it.

**Reference implementations to read (do not vendor, do not copy):**
- `slam_toolbox` (C++/ROS2) — the architecture we are mirroring, especially the Karto-derived front end
- `rust_robotics` by rsasaki0109 — already has a brute-force correlative scan matcher and 2D pose
  graph optimization in Rust. Read it for approach; write our own with better structure and tests.
- Olson 2009, "Real-Time Correlative Scan Matching" — the CSM paper. This is the core algorithm.
- Grisetti et al., "A Tutorial on Graph-Based SLAM" — the backend theory.

---

## Core types

Define these first and get them right; everything else depends on them.

```rust
/// 2D rigid body pose. Internally SE(2).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose2 {
    pub x: f64,
    pub y: f64,
    pub theta: f64,   // radians, normalized to (-π, π]
}

impl Pose2 {
    pub fn identity() -> Self;
    pub fn compose(&self, other: &Pose2) -> Pose2;      // self ⊕ other
    pub fn inverse(&self) -> Pose2;
    pub fn between(&self, other: &Pose2) -> Pose2;      // relative transform
    pub fn transform_point(&self, p: Point2) -> Point2;
    pub fn to_isometry(&self) -> nalgebra::Isometry2<f64>;
}

/// A point in 2D, metres.
pub type Point2 = nalgebra::Point2<f64>;

/// A scan in the sensor frame, already converted to Cartesian.
#[derive(Debug, Clone)]
pub struct ScanCloud {
    pub points: Vec<Point2>,
    pub timestamp_ns: u64,
}

/// Result of a scan match.
#[derive(Debug, Clone)]
pub struct MatchResult {
    pub pose: Pose2,
    pub covariance: nalgebra::Matrix3<f64>,
    pub score: f64,          // normalized 0..1, higher is better
    pub iterations: usize,
    pub converged: bool,
}
```

**Angle normalization is a recurring bug source.** Write `normalize_angle` once, use it everywhere,
test it at the boundaries (±π, ±2π, ±3π). Getting this wrong produces maps that look almost right,
which is the worst kind of wrong.

**Units are metres and radians throughout the library.** `olivaw-lidar` gives millimetres and
degrees — convert once at the boundary and never again. Document this loudly.

---

## Implementation order

Each step ships independently, has its own tests, and has a rerun example. Do not start the next
step until the current one is verified against recorded data.

### Phase 1 — Foundations

**Step 1: Core types + `preprocess`**
`Pose2`, `Point2`, `ScanCloud`, angle normalization. Then preprocessing: range gating (drop
returns below ~0.15m and above the sensor max), voxel/grid subsampling to a fixed resolution,
and outlier rejection (isolated points with no neighbours within a radius).

Test: property tests on `Pose2` — compose/inverse round-trips, identity behaviour, angle wrapping.

**Step 2: `grid` — occupancy grid**
Log-odds occupancy grid with configurable resolution (default 0.05 m/cell). Bresenham ray casting
from sensor origin to each hit: cells along the ray get a miss update, the endpoint gets a hit
update. Clamp log-odds to bounds so cells stay responsive to change.

```rust
pub struct OccupancyGrid {
    resolution: f64,        // metres per cell
    origin: Point2,
    width: usize,
    height: usize,
    log_odds: Vec<f32>,     // flat, row-major
}

impl OccupancyGrid {
    pub fn integrate_scan(&mut self, pose: &Pose2, cloud: &ScanCloud);
    pub fn probability_at(&self, p: Point2) -> Option<f32>;
    pub fn to_pgm(&self, path: &Path) -> io::Result<()>;   // standard map format
    pub fn log_to_rerun(&self, rec: &rerun::RecordingStream, entity: &str);
}
```

Example: `examples/grid_from_recording.rs` — replay a recording, integrate every scan at identity
pose, and watch the grid build in rerun. It will smear (no pose estimation yet) — that is expected
and it proves ray casting works.

### Phase 2 — Scan matching (the front end)

**Step 3: ICP scan matcher**
Point-to-point ICP first (simpler, easier to verify), then point-to-line as an upgrade.
Correspondence via k-d tree (`kiddo`). Rejection of correspondences beyond a distance threshold.
Huber or Tukey robust weighting to survive outliers.

```rust
pub trait ScanMatcher {
    fn match_scans(
        &self,
        reference: &ScanCloud,
        query: &ScanCloud,
        initial_guess: &Pose2,
    ) -> Result<MatchResult, SlamError>;
}
```

Test: synthetic data first — take a scan, apply a known transform, verify the matcher recovers it.
Then recorded data. Then add synthetic noise and verify graceful degradation.

**Step 4: Correlative scan matcher (CSM)**
This is the important one and it is what makes the system robust without odometry.

CSM computes a rigid-body transform aligning two scans by evaluating a cost function over a
3D search space — translation in x, y, and rotation θ — and taking the numerical maximum.
Unlike ICP it does not need a good initial guess, which is exactly our situation with encoderless
motors.

Implementation: multi-resolution search. Build a coarse lookup table from the reference scan
(a blurred occupancy likelihood field), search the full window coarsely, then refine at full
resolution around the best coarse candidate. This is the standard optimization and it turns an
intractable brute force into something real-time.

Key parameters to expose: search window (linear ±, angular ±), resolution at each level, and the
number of pyramid levels.

**Important**: CSM's real advantage over ICP is *runtime predictability* — ICP's runtime varies
significantly with the data, while CSM's search is bounded. For a real-time loop this matters more
than raw speed.

Example: `examples/scan_matching_viz.rs` — two scans, the search space heatmap, and the recovered
transform, all in rerun. This visualization is how you will debug every future matching bug.

**Step 5: Scan-to-map matching**
Match the incoming scan against the accumulated occupancy grid rather than against the previous
scan. This is what kills incremental drift. Same CSM machinery, but the likelihood field comes
from the grid.

**Milestone:** at the end of Phase 2 you can replay a recording and produce a map that is
recognizably your house, with drift but no catastrophic failure. This is the demo.

### Phase 3 — The back end

**Step 6: Pose graph on factrs**
Nodes are keyframe poses; edges are relative-pose constraints with covariance from the matcher.

Keyframe policy: insert a new keyframe when the robot has moved more than a distance threshold
(default 0.3 m) or rotated more than an angular threshold (default 0.3 rad) since the last one.
This bounds graph growth.

```rust
pub struct PoseGraph {
    // wraps factrs
}

impl PoseGraph {
    pub fn add_node(&mut self, pose: Pose2) -> NodeId;
    pub fn add_edge(&mut self, from: NodeId, to: NodeId, measurement: Pose2, info: Matrix3<f64>);
    pub fn optimize(&mut self, max_iterations: usize) -> Result<(), SlamError>;
    pub fn node_pose(&self, id: NodeId) -> Option<Pose2>;
}
```

Verify against `M3500.g2o` — load the standard benchmark, optimize, compare the objective value
to published results. If it matches, the backend is correct.

**Step 7: Loop closure**
This is the hardest part and the one that produces the most spectacular failures. Structure it in
three separate, individually testable stages:

1. **Candidate search** — find keyframes whose estimated pose is within a radius of the current
   pose but which are temporally distant (more than N keyframes ago). k-d tree over keyframe
   positions.
2. **Verification** — run CSM between the current scan and the candidate keyframe's scan with a
   wide search window. This is a "loop-closing" problem in CSM terms and needs a larger window
   than incremental matching.
3. **Gating** — accept only if the match score exceeds a threshold *and* the residual after
   optimization is consistent. **A single false positive folds the map in half.** Be conservative:
   a missed loop closure costs accuracy, a false one destroys the map.

Use a robust kernel (Huber, provided by factrs) on loop-closure edges specifically, so a bad
constraint degrades gracefully rather than catastrophically.

Test: a recording with a deliberate loop (walk a circuit through the house and return to the
start). Before loop closure the start and end diverge; after, they coincide.

**Step 8: Map serialization**
Save and load the full SLAM state — pose graph, keyframes with their scans, and the grid.
This is what enables `slam_toolbox`-style lifelong mapping: load an existing map, localize in it,
extend it.

Format: `serde` + a binary codec. Also export the grid as PGM+YAML (the standard map format)
for interoperability.

### Phase 4 — Localization mode

**Step 9: Localization against a saved map**
Load a serialized map, maintain a rolling buffer of recent scans in the graph, and expire old ones
without modifying the underlying map. This mirrors the localization mode design in `slam_toolbox`
and is what a production robot actually runs day to day.

---

## Performance practices

Correctness first, then measure, then optimize. But structure the code so optimization is possible.

**Measure before optimizing.** `criterion` benchmarks for the matcher and grid integration from
day one. A performance regression is a bug and CI should catch it.

**Where the time actually goes in 2D SLAM:**
1. Scan matching search — dominant cost, especially CSM's 3D search space
2. Grid ray casting — many cells per scan
3. Graph optimization — only on loop closure, amortized

**Concrete practices:**
- **Preallocate and reuse.** The scan pipeline runs at sensor rate; allocating per scan is waste.
  Keep scratch buffers in the matcher struct and clear rather than reallocate.
- **`&[T]` over `&Vec<T>`** in function signatures, always.
- **Avoid `collect()` into intermediate `Vec`s** in hot paths — chain iterators.
- **Use `f32` for the grid, `f64` for poses.** Grid cells are memory-bound and don't need the
  precision; pose composition accumulates error and does.
- **Bounds-check elision:** iterate with iterators rather than indexing where possible; when
  indexing is necessary in a hot loop, hoist the bound check with a slice reborrow.
- **`#[inline]` on small geometric helpers** (`transform_point`, `normalize_angle`) — they are
  called millions of times.
- **`rayon` behind the `parallel` feature** for the CSM coarse search — it is embarrassingly
  parallel over the search grid. Feature-gate it so embedded targets can opt out.
- **Profile with real data.** `cargo flamegraph` on a recording replay. Intuitions about where
  the time goes are usually wrong.

**Do not:** use `unsafe` for performance. If a hot loop seems to need it, the answer is a better
algorithm or a better data layout. This crate has a hard no-`unsafe` rule and that is a feature —
it is the entire safety argument for building this in Rust.

---

## Safety and code quality

Non-negotiables, enforced in CI:

```toml
# lib.rs
#![forbid(unsafe_code)]
#![warn(missing_docs)]
#![warn(clippy::pedantic)]
#![warn(clippy::float_cmp)]
#![warn(clippy::indexing_slicing)]   # in library code; allow in tests
```

- **No `unwrap()` / `expect()` / `panic!` in library code.** Ever. Return `Result`. A SLAM library
  that panics takes the robot down with it. Tests and examples may unwrap freely.
- **No `as` casts between numeric types where precision could be lost.** Use `TryFrom` or an
  explicit, commented conversion.
- **Never compare floats with `==`.** Use `approx` in tests, epsilon comparisons in code.
- **Every numeric parameter is a named, documented config field with a sane default.** Magic
  numbers in SLAM code are how projects become unmaintainable — every threshold in this crate
  will need tuning by someone eventually.
- **Denial-of-service through input:** validate scan sizes and grid dimensions. A malformed input
  should return an error, not allocate 40GB.
- **`cargo deny` in CI** for licence and advisory checking of the dependency tree.
- **Document panics, errors, and units** on every public function. Units especially — half of all
  robotics bugs are unit confusion.

---

## Testing strategy

**Unit tests** — every geometric primitive, every parser, property tests on `Pose2` algebra
via `proptest`.

**Synthetic tests** — generate a scan from a known 2D environment, apply a known transform,
verify the matcher recovers it. Add controlled Gaussian noise, verify graceful degradation.
These are fast, deterministic, and catch most regressions.

**Recorded-data tests** — replay a real recording, assert on aggregate properties: no panics,
match score above threshold for N% of scans, final pose within tolerance of the known endpoint.
Commit a small (few MB) recording to the repo as a fixture; keep larger ones in a separate
data repo or via Git LFS.

**Benchmark datasets** — Intel Research Lab and MIT CSAIL datasets for map quality comparison,
`M3500.g2o` for backend correctness.

**Oracle diff** — a script that runs `slam_toolbox` on a dataset (in Docker, once, offline) and
saves the reference trajectory. Our test compares against that saved output. ROS2 is a
*development-time* tool here, never a runtime dependency.

**No test may require hardware.** CI runs everything.

---

## Public API sketch

The high-level API should be trivial for the common case:

```rust
use olivaw_slam::{Slam, SlamConfig};
use olivaw_lidar::Lidar;

let mut slam = Slam::new(SlamConfig::default());
let mut lidar = Lidar::open("/dev/ttyUSB0")?;
lidar.start_scan()?;

for scan in lidar.scans() {
    let cloud = scan?.to_cloud();          // olivaw-lidar → olivaw-slam boundary
    let pose = slam.process_scan(&cloud)?;
    println!("pose: {:?}", pose);
}

slam.save("house.olivaw")?;
slam.grid().to_pgm("house.pgm")?;
```

And configurable for the serious case — every threshold, window size, and resolution exposed
in `SlamConfig` with documented defaults.

---

## Modularity for future ROS2 interop

ROS2 is **never** a dependency of this crate. But interop is a real commercial requirement, so
design for a bridge to exist later:

- Keep message-shaped structs (`ScanCloud`, `Pose2`, `OccupancyGrid`) free of internal state so
  they can be trivially converted to/from ROS2 message types by a separate `olivaw-ros2-bridge`
  crate.
- Provide standard-format export: PGM+YAML for maps, TUM/g2o for trajectories.
- Expose the SLAM loop as discrete callable steps, not a hidden internal thread, so an external
  orchestrator (dora-rs node, ROS2 node, custom binary) can drive it.

If someone wants `olivaw-slam` inside ROS2, they write a thin node that calls our API. That is
their dependency, not ours.

---

## What this crate is NOT

- ❌ Path planning, obstacle avoidance, navigation → future `olivaw-nav`
- ❌ Sensor drivers → `olivaw-lidar`
- ❌ Message passing → dora-rs
- ❌ Visualization → rerun
- ❌ 3D SLAM, visual SLAM → later, different crate
- ❌ A ROS2 node

---

## Definition of done for 0.1.0

- [ ] Replays a recorded session and produces a recognizable map of a real room
- [ ] CSM recovers known synthetic transforms to within 1cm / 0.5°
- [ ] Pose graph matches published `M3500.g2o` objective value
- [ ] Loop closure demonstrably corrects drift on a circuit recording
- [ ] Map saves and reloads with identical geometry
- [ ] Zero `unsafe`, zero clippy pedantic warnings, no `unwrap` in lib code
- [ ] Every phase has a rerun example that a stranger can run on our committed fixture
- [ ] Benchmarks in CI with regression detection
- [ ] Cross-compiles to `aarch64-unknown-linux-gnu`
- [ ] README shows a side-by-side of our map vs `slam_toolbox` on the same data
