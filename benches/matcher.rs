//! Criterion benchmarks for scan matching — the dominant cost in 2D SLAM.
//!
//! CSM's runtime is bounded by its search window (its core selling point);
//! a regression here directly cuts into the real-time budget.

// The criterion_group! macro expands to an undocumented function; benches are
// not library code, so the missing-docs rule does not apply here.
#![allow(missing_docs)]

#[path = "../tests/common/mod.rs"]
mod common;

use criterion::{Criterion, black_box, criterion_group, criterion_main};
use olivaw_slam::matcher::{CorrelativeMatcher, IcpMatcher, ScanToMapMatcher};
use olivaw_slam::{GridConfig, OccupancyGrid, Pose2, ScanMatcher};

fn bench_matchers(c: &mut Criterion) {
    let walls = common::corridor_walls();
    let pose_a = Pose2::new(4.5, 0.0, std::f64::consts::FRAC_PI_2);
    let pose_b = Pose2::new(4.6, 0.2, std::f64::consts::FRAC_PI_2 + 0.05);
    let reference = common::simulate_scan(&walls, pose_a, 0.01, 1);
    let query = common::simulate_scan(&walls, pose_b, 0.01, 2);
    let guess = Pose2::identity();

    let csm = CorrelativeMatcher::default();
    c.bench_function("csm_scan_to_scan_360pts", |b| {
        b.iter(|| csm.match_scans(black_box(&reference), black_box(&query), &guess).unwrap());
    });

    let icp = IcpMatcher::default();
    c.bench_function("icp_scan_to_scan_360pts", |b| {
        b.iter(|| icp.match_scans(black_box(&reference), black_box(&query), &guess).unwrap());
    });

    let mut grid = OccupancyGrid::new(GridConfig::default()).unwrap();
    for _ in 0..3 {
        grid.integrate_scan(&pose_a, &reference);
    }
    let s2m = ScanToMapMatcher::default();
    c.bench_function("csm_scan_to_map_600x600", |b| {
        b.iter(|| s2m.match_scan(black_box(&grid), black_box(&query), &pose_b).unwrap());
    });
}

criterion_group!(benches, bench_matchers);
criterion_main!(benches);
