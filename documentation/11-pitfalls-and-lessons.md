# 11 — Pitfalls & lessons learned

Every trap actually hit during development, recorded as symptom → cause → fix
→ lesson. Read this file before debugging anything; most future bugs will
rhyme with one of these.

## P1 — The lidar's angles are clockwise

- **Symptom (potential)**: maps that are mirror images of reality; matches
  that converge to plausible-looking but wrong poses.
- **Cause**: SLAMTEC lidars report angles increasing clockwise; standard
  robotics frames are counter-clockwise positive.
- **Fix**: the Cartesian conversion negates y
  (`olivaw_lidar::Point::to_cartesian`), done once at the boundary.
- **Lesson**: frame conventions are checked at the boundary or nowhere.
  Document the frame on every type that carries geometry.

## P2 — kiddo panics on axis-aligned walls

- **Symptom**: `panicked at ... 'Too many items with the same position on one
  axis. Bucket size must be increased'` — first in ICP tests, then again in an
  example that quietly re-introduced the default tree type.
- **Cause**: kiddo's default bucket size is 32; a scan of a long axis-aligned
  wall puts dozens of points at the *exact same coordinate* on one axis.
  Synthetic data makes this certain; real data can too.
- **Fix**: `KdTree<f64, u64, 2, 512, u32>` — bucket size 512 — everywhere.
- **Lesson**: a dependency's default configuration is part of your input
  domain. This panic lives inside library code paths, so the type alias in
  `icp.rs` carries a comment explaining why it must not be "simplified" back.

## P3 — factrs speaks a different dialect at every interface

Three sub-traps, all silent if missed:

1. `SE2::new(theta, x, y)` — **angle first**, opposite of our `(x, y, theta)`.
2. factrs tangent/covariance ordering is rotation-first `[theta, x, y]`;
   information matrices must be **permuted**, not copied.
3. factrs uses nalgebra **0.33** internally while we use 0.34 — same type
   names, incompatible types; matrices must cross the boundary element-wise.

Plus two compile-time quirks: `assign_symbols!` must be imported (it calls
itself unqualified), and `GaussianNoise::from_matrix_inf` panics on
non-positive-definite input, so we pre-check and regularize.

- **Lesson**: quarantine a dependency's conventions in one file
  (`graph/mod.rs`) and convert at the boundary — the same discipline as
  sensor units. Also: when adopting a new dependency, read its vendored
  source for conventions *before* writing code against it; the exploration
  report saved hours here.

## P4 — CSM accuracy is quantized without sub-step refinement

- **Symptom**: the "recover within 1 cm" test failed with an error of exactly
  1 cm — one search step.
- **Cause**: a grid search cannot resolve below its step size; the score
  surface maximum almost never sits exactly on a grid point.
- **Fix**: parabolic vertex fit through best +- one step along each axis,
  using bilinear field lookups (which is why the fine stage needs bilinear
  interpolation at all).
- **Lesson**: when a test fails by *exactly* the resolution of your method,
  the method — not the test — is telling you something structural.

## P5 — Instant 90-degree turns broke tracking (and were unreal)

- **Symptom**: the circuit integration test diverged 17 m; only half the
  expected keyframes existed; zero loop closures.
- **Cause**: the synthetic trajectory jumped heading by 90 degrees between
  consecutive scans at corners — far outside the +-0.3 rad matcher window.
  The matcher correctly refused (boundary flag), the pose froze, and
  subsequent matching was garbage-in.
- **Fix**: turn in place in 0.2 rad steps at corners, like a real robot.
- **Lessons**: (a) the not-converged-on-boundary protection works — the system
  refused to corrupt the map with impossible matches; (b) simulators must
  respect the physical assumptions the algorithms encode; (c) "half the
  keyframes are missing" is a tracking-loss signature worth remembering.

## P6 — Localization looked broken; it was being graded against the wrong frame

- **Symptom**: localization "drifted" 0.55 m against ground truth.
- **Cause**: the map itself (built from a partial run, no loop closure) had
  ~0.5 m of drift baked in. The localizer was consistent *with its map* to
  6 cm — which is its actual job.
- **Fix**: assert localization against the mapper's own estimates at the same
  locations, not against world ground truth.
- **Lesson**: define which frame a claim lives in before measuring it.
  Localization consistency and map accuracy are different quantities.

## P7 — Sparse-looking PGM exports are thresholds, not bugs

- **Symptom**: a map with clearly free corridors renders mostly grey/unknown;
  walls look thin or patchy on short recordings.
- **Cause**: one miss moves a cell to p = 0.4, which is still above the 0.196
  free threshold; one hit gives p = 0.7, barely above 0.65. The `map_server`
  export convention is deliberately conservative.
- **Lesson**: check `probability_at`/log-odds before concluding the mapper is
  wrong; density comes with revisits. Real captures (many scans per cell)
  render far denser than short synthetic runs.

## P8 — The outlier filter eats distant walls

- **Symptom**: real C1 scans lose most points beyond ~6 m (93 of 366 survived
  preprocessing on the first fixture).
- **Cause**: at ~1 degree angular spacing, adjacent wall points are farther
  apart than the 0.10 m outlier radius beyond ~5.8 m — real points get culled
  as isolated.
- **Status**: acceptable for house-scale rooms; range-scaled radius is the
  planned fix (roadmap).
- **Lesson**: fixed spatial thresholds interact with polar sensors; densities
  fall off with range.

## P9 — rerun's dependency tree breaks cross-compilation

- **Symptom**: `cargo check --target aarch64-unknown-linux-gnu --all-features`
  fails in `ring` (C code needing a cross C toolchain).
- **Cause**: rerun (the `viz` feature) transitively depends on C crates.
- **Fix**: none needed — `viz` is development tooling. The deployable feature
  set (`parallel serialize`) is pure Rust and cross-compiles clean.
- **Lesson**: keep visualization strictly behind a feature; "pure Rust" is a
  property of the deployable configuration, and CI should check that exact
  configuration.

## P10 — Assorted smaller traps

- `olivaw-lidar` `Scan.timestamp` is a `std::time::Instant` — monotonic, no
  epoch, not serializable. Timestamps become nanoseconds-since-first-scan at
  the boundary.
- The record example writes `c1_scan_1000_nodes.bin` **regardless of the
  actual `--nodes` value** — do not let the filename fool you about capture
  length.
- Recordings are raw serial byte streams replayed through the driver — there
  is no structured scan file format anywhere; if one is ever needed,
  olivaw-slam must define it.
- `criterion_group!` expands to an undocumented function; benches need
  `#![allow(missing_docs)]` with a comment, not a lint-config change.
- Point-to-point ICP has an inherent ~1.5 cm bias from surface sampling
  mismatch. That is a property of the algorithm class, not a bug — hence
  point-to-line as the planned upgrade and CSM as the workhorse.
- Grid-bounded accuracy: scan-to-map matching cannot beat the grid resolution
  (~0.05 m); tests encode 4 cm tolerances on purpose.
