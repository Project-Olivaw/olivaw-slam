//! Synthetic lidar scenes for matcher tests: a rectangular room with an
//! asymmetric inner obstacle, sampled by exact ray casting. Fast,
//! deterministic, and free of real-sensor noise unless requested.

#![allow(clippy::cast_possible_truncation, clippy::cast_precision_loss)]

use crate::pose::{Point2, Pose2};
use crate::scan::ScanCloud;

/// Wall segments of the test environment (world frame, metres): an outer room
/// with an inner box placed off-centre so the scene has no rotational symmetry.
fn walls() -> Vec<(Point2, Point2)> {
    let mut segs = Vec::new();
    let mut rect = |x0: f64, y0: f64, x1: f64, y1: f64| {
        segs.push((Point2::new(x0, y0), Point2::new(x1, y0)));
        segs.push((Point2::new(x1, y0), Point2::new(x1, y1)));
        segs.push((Point2::new(x1, y1), Point2::new(x0, y1)));
        segs.push((Point2::new(x0, y1), Point2::new(x0, y0)));
    };
    rect(-3.0, -2.0, 4.0, 3.0); // outer room
    rect(0.5, 0.5, 1.5, 1.2); // inner box
    segs
}

/// Distance along ray `origin + t·dir` to segment `(a, b)`, if it hits.
fn ray_segment(origin: Point2, dir: (f64, f64), a: Point2, b: Point2) -> Option<f64> {
    let (ex, ey) = (b.x - a.x, b.y - a.y);
    let denom = dir.0 * ey - dir.1 * ex;
    if denom.abs() < 1e-12 {
        return None; // parallel
    }
    let (ox, oy) = (a.x - origin.x, a.y - origin.y);
    let t = (ox * ey - oy * ex) / denom;
    let u = (ox * dir.1 - oy * dir.0) / -denom;
    (t > 1e-9 && (0.0..=1.0).contains(&u)).then_some(t)
}

/// Simulate a 360-beam scan from `pose` (sensor frame output, metres).
pub(crate) fn room_scan(pose: Pose2) -> ScanCloud {
    noisy_room_scan(pose, 0.0, 0)
}

/// Simulated scan with deterministic pseudo-Gaussian range noise of standard
/// deviation `sigma_m` (LCG-seeded — reproducible across runs).
pub(crate) fn noisy_room_scan(pose: Pose2, sigma_m: f64, seed: u64) -> ScanCloud {
    let segs = walls();
    let origin = Point2::new(pose.x, pose.y);
    let mut rng = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15).wrapping_add(1);
    let mut noise = || {
        // Sum of 4 uniforms ≈ Gaussian (Irwin–Hall), zero-mean, unit-ish var.
        let mut acc = 0.0;
        for _ in 0..4 {
            rng = rng.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            acc += ((rng >> 11) as f64) / ((1_u64 << 53) as f64) - 0.5;
        }
        acc * 1.732 // scale to ~unit std-dev
    };
    let mut points = Vec::with_capacity(360);
    for i in 0..360 {
        let bearing = f64::from(i) * std::f64::consts::TAU / 360.0;
        let world_angle = pose.theta + bearing;
        let dir = (world_angle.cos(), world_angle.sin());
        let range = segs
            .iter()
            .filter_map(|&(a, b)| ray_segment(origin, dir, a, b))
            .fold(f64::INFINITY, f64::min);
        if range.is_finite() && range < 12.0 {
            let r = range + noise() * sigma_m;
            points.push(Point2::new(r * bearing.cos(), r * bearing.sin()));
        }
    }
    ScanCloud::new(points, 0)
}
