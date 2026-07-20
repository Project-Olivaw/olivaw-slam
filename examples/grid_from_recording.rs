//! Replay a recorded lidar session and build an occupancy grid in rerun.
//!
//! Every scan is integrated at the **identity pose** — there is no pose
//! estimation yet (that is Phase 2). With a moving sensor the map will smear;
//! that is expected and correct. What this example proves is that ray
//! casting, the log-odds update, and the coordinate conventions are right.
//!
//! ```text
//! cargo run --example grid_from_recording --features viz -- \
//!     [recording.bin] [out.pgm] [--save out.rrd]
//! ```
//!
//! - `recording.bin` — any raw capture made with olivaw-lidar's `record`
//!   example, from anywhere on disk. Defaults to olivaw-lidar's committed
//!   fixture (~2–3 rotations, so expect a sparse partial ring rather than a
//!   full room; record a longer session for the real smear test).
//! - `out.pgm` — occupancy map written on exit (default `grid_from_recording.pgm`).
//! - `--save out.rrd` — record rerun data to a file instead of spawning the
//!   viewer (useful headless; open it later with `rerun out.rrd`).

// Examples convert f64 world coordinates to f32 for display; that truncation
// is fine here and not worth ceremony outside library code.
#![allow(clippy::cast_possible_truncation)]

use std::path::PathBuf;
use std::time::Instant;

use olivaw_lidar::{Lidar, Scan, transport::ReplayTransport};
use olivaw_slam::preprocess::{PreprocessConfig, Preprocessor};
use olivaw_slam::{GridConfig, OccupancyGrid, Point2, Pose2, ScanCloud};

const DEFAULT_RECORDING: &str =
    concat!(env!("CARGO_MANIFEST_DIR"), "/../olivaw-lidar/tests/fixtures/c1_scan_1000_nodes.bin");

/// olivaw-lidar → olivaw-slam boundary: sensor units (clockwise degrees,
/// millimetres) become metres/radians here, via `Scan::to_cartesian` (which
/// already yields metres in the x-forward, y-left, CCW-positive frame), and
/// never get converted again. Timestamps become nanoseconds since the first
/// scan of the session (`Instant` has no epoch).
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

fn as_f32_pairs(points: &[Point2]) -> Vec<(f32, f32)> {
    points.iter().map(|p| (p.x as f32, p.y as f32)).collect()
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Minimal std-only argument parsing: positionals + an optional --save.
    let mut recording: Option<PathBuf> = None;
    let mut pgm_out: Option<PathBuf> = None;
    let mut save_rrd: Option<PathBuf> = None;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        if arg == "--save" {
            save_rrd = Some(PathBuf::from(args.next().ok_or("--save needs a path")?));
        } else if recording.is_none() {
            recording = Some(PathBuf::from(arg));
        } else if pgm_out.is_none() {
            pgm_out = Some(PathBuf::from(arg));
        } else {
            return Err(format!("unexpected argument: {arg}").into());
        }
    }
    let recording = recording.unwrap_or_else(|| PathBuf::from(DEFAULT_RECORDING));
    let pgm_out = pgm_out.unwrap_or_else(|| PathBuf::from("grid_from_recording.pgm"));

    let builder = rerun::RecordingStreamBuilder::new("olivaw_slam_grid");
    let rec = match &save_rrd {
        Some(path) => builder.save(path)?,
        None => builder.spawn().map_err(|e| {
            format!(
                "failed to spawn the rerun viewer ({e}).\n\
                 Install it with `uv tool install rerun-sdk` (or `pip3 install rerun-sdk`),\n\
                 or skip the viewer with `--save out.rrd`."
            )
        })?,
    };

    println!("replaying {}", recording.display());
    let replay = ReplayTransport::from_file(&recording)?;
    let mut lidar = Lidar::with_transport(replay);
    lidar.start_scan()?;

    let mut preprocessor = Preprocessor::new(PreprocessConfig::default())?;
    let mut grid = OccupancyGrid::new(GridConfig::default())?;
    let mut filtered = ScanCloud::default();
    let mut session_start: Option<Instant> = None;
    let (mut scans, mut raw_points, mut kept_points) = (0_u64, 0_usize, 0_usize);

    for (i, scan) in lidar.scans().enumerate() {
        let scan = scan?;
        let start = *session_start.get_or_insert(scan.timestamp);
        let cloud = scan_to_cloud(&scan, start);

        rec.set_time_sequence("scan", i64::try_from(i).unwrap_or(i64::MAX));
        rec.log(
            "scan/raw",
            &rerun::Points2D::new(as_f32_pairs(&cloud.points))
                .with_colors([rerun::Color::from_rgb(120, 120, 120)])
                .with_radii([0.015]),
        )?;

        preprocessor.process(&cloud, &mut filtered)?;
        rec.log(
            "scan/filtered",
            &rerun::Points2D::new(as_f32_pairs(&filtered.points))
                .with_colors([rerun::Color::from_rgb(60, 200, 60)])
                .with_radii([0.02]),
        )?;

        // No pose estimation yet: identity pose, smearing expected.
        grid.integrate_scan(&Pose2::identity(), &filtered);
        grid.log_to_rerun(&rec, "map/grid")?;

        scans += 1;
        raw_points += cloud.len();
        kept_points += filtered.len();
    }

    grid.to_pgm(&pgm_out)?;
    println!(
        "{scans} scan(s), {raw_points} raw points, {kept_points} after preprocessing"
    );
    println!("occupancy map written to {}", pgm_out.display());
    if let Some(path) = &save_rrd {
        println!("rerun recording written to {} (open with `rerun {}`)", path.display(), path.display());
    }
    Ok(())
}
