//! Recorded-data smoke test: replay the olivaw-lidar fixture end to end
//! through preprocessing and grid integration. No hardware; CI-safe.

use std::time::Instant;

use olivaw_lidar::{Lidar, Scan, transport::ReplayTransport};
use olivaw_slam::preprocess::{PreprocessConfig, Preprocessor};
use olivaw_slam::{GridConfig, OccupancyGrid, Point2, Pose2, ScanCloud};

const FIXTURE: &str =
    concat!(env!("CARGO_MANIFEST_DIR"), "/../olivaw-lidar/tests/fixtures/c1_scan_1000_nodes.bin");

/// Boundary conversion (mirrors the example): sensor units → metres/radians,
/// timestamps → ns since the first scan.
fn scan_to_cloud(scan: &Scan, session_start: Instant) -> ScanCloud {
    let timestamp_ns = u64::try_from(
        scan.timestamp.saturating_duration_since(session_start).as_nanos(),
    )
    .unwrap_or(u64::MAX);
    let points = scan
        .to_cartesian()
        .into_iter()
        .map(|(x, y)| Point2::new(f64::from(x), f64::from(y)))
        .collect();
    ScanCloud::new(points, timestamp_ns)
}

#[test]
fn fixture_replays_into_a_sane_grid() {
    let replay = ReplayTransport::from_file(FIXTURE).expect("fixture missing — is olivaw-lidar checked out next to olivaw-slam?");
    let mut lidar = Lidar::with_transport(replay);
    lidar.start_scan().unwrap();

    let mut preprocessor = Preprocessor::new(PreprocessConfig::default()).unwrap();
    let mut grid = OccupancyGrid::new(GridConfig::default()).unwrap();
    let mut filtered = ScanCloud::default();
    let mut session_start: Option<Instant> = None;
    let mut scans = 0_u64;
    let mut kept_points = 0_usize;

    for scan in lidar.scans() {
        let scan = scan.expect("replay stream must decode cleanly");
        let start = *session_start.get_or_insert(scan.timestamp);
        let cloud = scan_to_cloud(&scan, start);
        preprocessor.process(&cloud, &mut filtered).unwrap();
        grid.integrate_scan(&Pose2::identity(), &filtered);
        scans += 1;
        kept_points += filtered.len();
    }

    assert!(scans >= 1, "fixture should contain at least one full rotation");
    assert!(kept_points > 0, "preprocessing should keep some points");

    // The grid must have picked up real structure: some occupied cells, and
    // every occupied cell within sensor range of the (identity) pose.
    let width = grid.width();
    let res = grid.resolution_m();
    let origin = grid.origin();
    let occupied: Vec<(usize, usize)> = grid
        .cells()
        .iter()
        .enumerate()
        .filter(|&(_, &l)| l > 0.6)
        .map(|(i, _)| (i % width, i / width))
        .collect();
    assert!(
        occupied.len() >= 5,
        "expected at least 5 occupied cells, got {}",
        occupied.len()
    );
    for &(cx, cy) in &occupied {
        // Cell centre in world coordinates.
        #[allow(clippy::cast_precision_loss)]
        let world = Point2::new(
            origin.x + (cx as f64 + 0.5) * res,
            origin.y + (cy as f64 + 0.5) * res,
        );
        let range = (world.x * world.x + world.y * world.y).sqrt();
        assert!(
            range <= 12.0 + res,
            "occupied cell at ({cx}, {cy}) is {range:.2} m out — beyond sensor range"
        );
    }
    // Not everything is unknown: free space was carved out too.
    assert!(grid.cells().iter().any(|&l| l < 0.0), "expected free-space evidence");
}
