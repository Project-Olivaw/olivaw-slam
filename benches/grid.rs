//! Criterion benchmark for occupancy-grid scan integration.
//!
//! Grid ray casting is the second-largest cost in 2D SLAM after scan
//! matching; a regression here is a bug.

// The criterion_group! macro expands to an undocumented function; benches are
// not library code, so the missing-docs rule does not apply here.
#![allow(missing_docs)]

use criterion::{Criterion, black_box, criterion_group, criterion_main};
use olivaw_slam::{GridConfig, OccupancyGrid, Point2, Pose2, ScanCloud};

/// Synthetic 360-beam scan: a ring at ~5 m with deterministic pseudo-noise.
fn synthetic_scan(beams: usize) -> ScanCloud {
    let cloud: Vec<Point2> = (0..beams)
        .map(|i| {
            let angle = f64::from(u32::try_from(i).unwrap()) / f64::from(u32::try_from(beams).unwrap())
                * std::f64::consts::TAU;
            let r = 5.0 + 0.03 * (angle * 41.0).sin();
            Point2::new(r * angle.cos(), r * angle.sin())
        })
        .collect();
    ScanCloud::new(cloud, 0)
}

fn bench_integrate(c: &mut Criterion) {
    let cloud = synthetic_scan(360);
    let mut grid = OccupancyGrid::new(GridConfig::default()).unwrap();
    let pose = Pose2::identity();

    c.bench_function("grid_integrate_360_beams_600x600", |b| {
        b.iter(|| {
            grid.integrate_scan(black_box(&pose), black_box(&cloud));
        });
    });
}

criterion_group!(benches, bench_integrate);
criterion_main!(benches);
