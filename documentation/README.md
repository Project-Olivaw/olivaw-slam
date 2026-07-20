# olivaw-slam documentation

This folder is the project's long-term memory: what was built, how each part
works, why decisions went the way they did, what went wrong along the way, and
what should happen next. It is written so that someone — including a future
you — can arrive cold, read in order, and end up able to modify the system
confidently.

## Reading order

| doc | read it when you want to know… |
|---|---|
| [01 — Overview & architecture](01-overview.md) | what this project is, the thesis behind it, and how the modules fit together |
| [02 — Development history](02-development-history.md) | the phase-by-phase story: what was built in what order and why that order |
| [03 — Core types & conventions](03-core-types.md) | `Pose2`, `ScanCloud`, units, frames, angle normalization — the contracts everything rests on |
| [04 — Preprocessing](04-preprocessing.md) | the scan cleanup pipeline: range gate, outlier rejection, voxel subsampling |
| [05 — Occupancy grid](05-occupancy-grid.md) | log-odds mapping, ray casting, map export |
| [06 — Scan matching](06-scan-matching.md) | ICP, correlative matching (CSM), scan-to-map — the front end |
| [07 — Pose graph](07-pose-graph.md) | the factrs backend, its conventions, and the M3500 verification |
| [08 — Loop closure](08-loop-closure.md) | place recognition, verification, and the χ² gate |
| [09 — The Slam orchestrator](09-slam-orchestrator.md) | how it all composes: keyframes, map rebuilds, save/load, localization |
| [10 — Testing & benchmarks](10-testing-and-benchmarks.md) | the test strategy, the fixtures, and how to run everything |
| [11 — Pitfalls & lessons learned](11-pitfalls-and-lessons.md) | **the highest-value file** — every trap we hit, with symptom → cause → fix |
| [12 — Roadmap & beating slam_toolbox](12-roadmap.md) | known gaps, future work, and where this can genuinely exceed the incumbent |

## Quick orientation

- **New developer, first hour**: read 01, skim 02, then run the examples in
  the top-level [README](../README.md).
- **Coming back after months**: read 02 (what exists), 11 (what bites), 12
  (what's next).
- **Debugging a bad map**: 06 (matching) and 11 (pitfalls), plus the
  `scan_matching_viz` example — visualization is the primary debugging
  instrument in this project, not an afterthought.
- **Changing any public type**: 03 first. Units and frame conventions are the
  root of most robotics bugs.

## Ground rules that shaped everything

These come from [CLAUDE.md](../CLAUDE.md) (the authoritative engineering spec)
and were enforced throughout:

1. **Algorithms are developed offline against recorded data**, never against a
   live device. Live runs are demos, not development.
2. **Every stage has a rerun visualization** — if you cannot see it, you
   cannot debug it.
3. **Metres and radians inside the crate, no exceptions**; sensor units are
   converted once at the boundary.
4. **No `unsafe`, no `unwrap`/`panic!` in library code, clippy pedantic
   clean** — a SLAM library that panics takes the robot down with it.
5. **Every tunable is a documented config field with a default** — magic
   numbers are how SLAM projects become unmaintainable.
