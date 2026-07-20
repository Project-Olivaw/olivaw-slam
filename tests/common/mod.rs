//! Shared synthetic world for integration tests: a square ring corridor
//! (outer room with an inner block) decorated with asymmetric pillars, and a
//! deterministic lidar simulator. No hardware, no fixtures, CI-safe.

// Test scaffolding, not library code: display-precision casts and indexing
// are fine here, and not every consumer uses every helper.
#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    clippy::indexing_slicing,
    dead_code
)]

use olivaw_slam::{Point2, Pose2, ScanCloud};

/// Wall segments of the corridor world (world frame, metres).
#[must_use]
pub fn corridor_walls() -> Vec<(Point2, Point2)> {
    let mut segs = Vec::new();
    let mut rect = |x0: f64, y0: f64, x1: f64, y1: f64| {
        segs.push((Point2::new(x0, y0), Point2::new(x1, y0)));
        segs.push((Point2::new(x1, y0), Point2::new(x1, y1)));
        segs.push((Point2::new(x1, y1), Point2::new(x0, y1)));
        segs.push((Point2::new(x0, y1), Point2::new(x0, y0)));
    };
    rect(-6.0, -6.0, 6.0, 6.0); // outer walls
    rect(-3.0, -3.0, 3.0, 3.0); // inner block → 3 m wide ring corridor
    // Asymmetric pillars: break the ring's rotational symmetry and give the
    // matcher features along otherwise-degenerate straight corridor sections.
    for &(px, py, s) in &[
        (4.6_f64, 1.4_f64, 0.35_f64),
        (1.1, 4.5, 0.3),
        (-2.1, 4.7, 0.4),
        (-4.5, 0.6, 0.3),
        (-1.2, -4.6, 0.35),
        (3.9, -4.4, 0.25),
    ] {
        rect(px, py, px + s, py + s);
    }
    segs
}

fn ray_segment(origin: Point2, dir: (f64, f64), a: Point2, b: Point2) -> Option<f64> {
    let (ex, ey) = (b.x - a.x, b.y - a.y);
    let denom = dir.0 * ey - dir.1 * ex;
    if denom.abs() < 1e-12 {
        return None;
    }
    let (ox, oy) = (a.x - origin.x, a.y - origin.y);
    let t = (ox * ey - oy * ex) / denom;
    let u = (ox * dir.1 - oy * dir.0) / -denom;
    (t > 1e-9 && (0.0..=1.0).contains(&u)).then_some(t)
}

/// Simulate a 360-beam scan from `pose` with Gaussian-ish range noise of
/// standard deviation `sigma_m` (deterministic, seeded).
#[must_use]
pub fn simulate_scan(walls: &[(Point2, Point2)], pose: Pose2, sigma_m: f64, seed: u64) -> ScanCloud {
    let origin = Point2::new(pose.x, pose.y);
    let mut rng = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15).wrapping_add(0xB5);
    let mut noise = || {
        let mut acc = 0.0;
        for _ in 0..4 {
            rng = rng.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            acc += ((rng >> 11) as f64) / ((1_u64 << 53) as f64) - 0.5;
        }
        acc * 1.732
    };
    let mut points = Vec::with_capacity(360);
    for i in 0..360 {
        let bearing = f64::from(i) * std::f64::consts::TAU / 360.0;
        let world_angle = pose.theta + bearing;
        let dir = (world_angle.cos(), world_angle.sin());
        let range = walls
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

/// Ground-truth poses walking one full circuit of the ring corridor
/// (counter-clockwise, heading along the direction of travel), starting and
/// ending near `(4.5, -3.5)`. `step_m` is the spacing between poses.
#[must_use]
pub fn circuit_trajectory(step_m: f64) -> Vec<Pose2> {
    use std::f64::consts::{FRAC_PI_2, PI};
    let corners: [(f64, f64, f64); 6] = [
        (4.5, -3.5, FRAC_PI_2), // east corridor, heading north
        (4.5, 4.5, PI),         // north corridor, heading west
        (-4.5, 4.5, -FRAC_PI_2), // west corridor, heading south
        (-4.5, -4.5, 0.0),      // south corridor, heading east
        (4.5, -4.5, FRAC_PI_2), // back on the east side
        (4.5, -3.4, FRAC_PI_2), // overlap the start to enable loop closure
    ];
    let mut poses: Vec<Pose2> = Vec::new();
    for w in corners.windows(2) {
        let [(x0, y0, heading), (x1, y1, next_heading)] = [w[0], w[1]];
        // Turn in place at the corner: no real robot rotates 90° between two
        // consecutive scans, and the matcher's angular window is finite.
        if let Some(prev) = poses.last().copied() {
            let mut h = prev.theta;
            loop {
                let remaining = olivaw_slam::normalize_angle(heading - h);
                if remaining.abs() < 0.2 {
                    break;
                }
                h = olivaw_slam::normalize_angle(h + remaining.signum() * 0.2);
                poses.push(Pose2::new(x0, y0, h));
            }
        }
        let (dx, dy) = (x1 - x0, y1 - y0);
        let len = (dx * dx + dy * dy).sqrt();
        let n = (len / step_m).floor().max(1.0);
        let steps = n as usize;
        for k in 0..steps {
            let t = k as f64 / n;
            poses.push(Pose2::new(x0 + t * dx, y0 + t * dy, heading));
        }
        let _ = next_heading;
    }
    poses
}
