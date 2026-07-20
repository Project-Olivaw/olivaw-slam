//! Save/load round-trip and localization-mode tests (`serialize` feature).
//!
//! The 0.1.0 contract: a map saves and reloads with *identical* geometry,
//! and a loaded map supports localization without being modified.

#![cfg(feature = "serialize")]
#![allow(clippy::indexing_slicing)] // tests may index freely

mod common;

use olivaw_slam::matcher::CsmConfig;
use olivaw_slam::{Pose2, Slam, SlamConfig};

fn quick_config() -> SlamConfig {
    let mut config = SlamConfig::default();
    config.matcher.csm = CsmConfig {
        search_x_m: 0.7,
        search_y_m: 0.7,
        search_theta_rad: 0.3,
        linear_step_m: 0.02,
        angular_step_rad: 0.01,
        ..CsmConfig::default()
    };
    config
}

/// Map a partial circuit; returns the system, the ground-truth poses fed,
/// and the mapper's own per-scan estimates (map frame).
fn map_partial_circuit(n_scans: usize) -> (Slam, Vec<Pose2>, Vec<Pose2>) {
    let walls = common::corridor_walls();
    let truth: Vec<Pose2> = common::circuit_trajectory(0.4).into_iter().take(n_scans).collect();
    let mut slam = Slam::new(quick_config()).unwrap();
    let mut estimates = Vec::with_capacity(truth.len());
    for (i, pose) in truth.iter().enumerate() {
        let scan = common::simulate_scan(&walls, *pose, 0.01, 500 + i as u64);
        estimates.push(slam.process_scan(&scan).unwrap());
    }
    (slam, truth, estimates)
}

#[test]
fn save_load_round_trips_with_identical_geometry() {
    let (slam, _, _) = map_partial_circuit(25);
    let path = std::env::temp_dir().join("olivaw_slam_roundtrip.olivaw");
    slam.save(&path).unwrap();
    let loaded = Slam::load(&path).unwrap();
    std::fs::remove_file(&path).ok();

    assert_eq!(loaded.keyframes().len(), slam.keyframes().len());
    for (a, b) in loaded.keyframes().iter().zip(slam.keyframes()) {
        assert!((a.pose.x - b.pose.x).abs() < 1e-12);
        assert!((a.pose.y - b.pose.y).abs() < 1e-12);
        assert!((a.pose.theta - b.pose.theta).abs() < 1e-12);
        assert_eq!(a.cloud.len(), b.cloud.len());
    }
    // The rebuilt grid must be bit-for-bit the same map.
    assert_eq!(loaded.grid().cells().len(), slam.grid().cells().len());
    let mismatches = loaded
        .grid()
        .cells()
        .iter()
        .zip(slam.grid().cells())
        .filter(|(a, b)| (**a - **b).abs() > f32::EPSILON)
        .count();
    assert_eq!(mismatches, 0, "reloaded grid differs in {mismatches} cells");
    assert_eq!(loaded.graph().edges().len(), slam.graph().edges().len());
}

#[test]
fn corrupt_file_is_rejected_not_panicked() {
    let path = std::env::temp_dir().join("olivaw_slam_corrupt.olivaw");
    std::fs::write(&path, b"definitely not a slam state").unwrap();
    let result = Slam::load(&path);
    std::fs::remove_file(&path).ok();
    assert!(result.is_err(), "corrupt input must error, not panic");
}

#[test]
fn localization_tracks_in_a_frozen_map() {
    let walls = common::corridor_walls();
    let (slam, truth, mapped_estimates) = map_partial_circuit(30);
    let path = std::env::temp_dir().join("olivaw_slam_localize.olivaw");
    slam.save(&path).unwrap();

    let mut localizer = Slam::load(&path).unwrap();
    std::fs::remove_file(&path).ok();
    localizer.set_localization_mode(true);
    assert!(localizer.is_localization_mode());
    let cells_before: Vec<f32> = localizer.grid().cells().to_vec();
    let keyframes_before = localizer.keyframes().len();

    // Retrace the mapped corridor backwards from where the map ended, with
    // fresh (different-noise) scans: every pose stays inside mapped territory
    // and within the matcher window of the previous estimate. The reference
    // is the *mapper's* estimate at the same location — localization must be
    // consistent with the map it localizes in, regardless of any global
    // drift the (loop-closure-free, partial) map itself carries.
    let mut worst = 0.0_f64;
    for (i, pose) in truth.iter().enumerate().rev().take(14) {
        let scan = common::simulate_scan(&walls, *pose, 0.01, 9000 + i as u64);
        let est = localizer.process_scan(&scan).unwrap();
        let reference = mapped_estimates[i];
        let err = ((est.x - reference.x).powi(2) + (est.y - reference.y).powi(2)).sqrt();
        worst = worst.max(err);
    }
    println!("localization worst error vs map frame: {worst:.3} m");
    assert!(worst < 0.15, "localization inconsistent with its map: {worst:.3} m");

    // The map must be untouched: same keyframes, bit-identical grid.
    assert_eq!(localizer.keyframes().len(), keyframes_before);
    let changed = localizer
        .grid()
        .cells()
        .iter()
        .zip(&cells_before)
        .filter(|(a, b)| (**a - **b).abs() > f32::EPSILON)
        .count();
    assert_eq!(changed, 0, "localization modified {changed} grid cells");
}
