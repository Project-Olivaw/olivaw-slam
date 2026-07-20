//! End-to-end SLAM on a synthetic circuit: walk a ring corridor with noisy
//! scans and no odometry, return to the start, and verify that a loop
//! closure fires and the trajectory stays consistent.

#![allow(clippy::indexing_slicing)] // tests may index freely

mod common;

use olivaw_slam::matcher::CsmConfig;
use olivaw_slam::{Slam, SlamConfig};

#[test]
fn circuit_closes_loop_and_stays_consistent() {
    let walls = common::corridor_walls();
    let truth = common::circuit_trajectory(0.4);

    let mut config = SlamConfig::default();
    // The simulated robot moves 0.4 m between scans (no odometry), so widen
    // the incremental search window; coarser steps keep the test fast.
    config.matcher.csm = CsmConfig {
        search_x_m: 0.7,
        search_y_m: 0.7,
        search_theta_rad: 0.3,
        linear_step_m: 0.02,
        angular_step_rad: 0.01,
        ..CsmConfig::default()
    };
    let mut slam = Slam::new(config).unwrap();

    // The world origin of the SLAM map is the first scan's pose, so express
    // ground truth relative to the starting pose.
    let start = truth[0];
    let mut worst_error = 0.0_f64;
    for (i, pose) in truth.iter().enumerate() {
        let scan = common::simulate_scan(&walls, *pose, 0.01, 1000 + i as u64);
        let estimate = slam.process_scan(&scan).unwrap();
        let truth_rel = start.between(pose);
        let err =
            ((estimate.x - truth_rel.x).powi(2) + (estimate.y - truth_rel.y).powi(2)).sqrt();
        worst_error = worst_error.max(err);
    }

    let final_est = slam.pose();
    let final_truth = start.between(truth.last().unwrap());
    let final_err = ((final_est.x - final_truth.x).powi(2)
        + (final_est.y - final_truth.y).powi(2))
    .sqrt();

    println!(
        "keyframes: {}, loops closed: {}, worst error {worst_error:.3} m, final error {final_err:.3} m",
        slam.keyframes().len(),
        slam.loops_closed()
    );
    assert!(slam.keyframes().len() > 40, "expected a keyframe-rich circuit");
    assert!(
        slam.loops_closed() >= 1,
        "returning to the start must close at least one loop"
    );
    assert!(
        final_err < 0.3,
        "final pose error {final_err:.3} m too large after loop closure"
    );
    assert!(worst_error < 1.0, "trajectory diverged mid-run: {worst_error:.3} m");

    // The map must show real structure: walls occupied, corridor carved free.
    let occupied = slam.grid().occupied_cell_centres(0.65);
    assert!(occupied.len() > 200, "map has only {} occupied cells", occupied.len());

    // Debug hook: OLIVAW_EXPORT_MAP=<stem> exports the final map for visual
    // inspection (not part of the assertion set).
    if let Ok(stem) = std::env::var("OLIVAW_EXPORT_MAP") {
        slam.grid().export_map(std::path::Path::new(&stem)).unwrap();
    }
}
