//! Full SLAM on a recorded lidar session: preprocessing, scan-to-map
//! matching, keyframes, pose graph, loop closure — with the live map,
//! trajectory, and scans in rerun.
//!
//! ```text
//! cargo run --release --example slam_from_recording --features viz -- \
//!     [recording.bin] [out-map-stem] [--save out.rrd]
//! ```
//!
//! - `recording.bin` — any raw capture from olivaw-lidar's `record` example.
//!   Defaults to the committed fixture (a single rotation — enough to see the
//!   pipeline run; walk a real circuit for the full effect).
//! - `out-map-stem` — the map is exported as `<stem>.pgm` + `<stem>.yaml`
//!   (`map_server` format) on exit. Default `slam_map`.
//! - With the `serialize` feature enabled, the full SLAM state is also saved
//!   to `<stem>.olivaw` for later reload/localization.

#![allow(clippy::cast_possible_truncation)]

use std::path::PathBuf;
use std::time::Instant;

use olivaw_lidar::{Lidar, Scan, transport::ReplayTransport};
use olivaw_slam::{Point2, ScanCloud, Slam, SlamConfig};

const DEFAULT_RECORDING: &str =
    concat!(env!("CARGO_MANIFEST_DIR"), "/../olivaw-lidar/tests/fixtures/c1_scan_1000_nodes.bin");

/// olivaw-lidar → olivaw-slam boundary: sensor units become metres/radians
/// here and never get converted again.
fn scan_to_cloud(scan: &Scan, session_start: Instant) -> ScanCloud {
    let timestamp_ns =
        u64::try_from(scan.timestamp.saturating_duration_since(session_start).as_nanos())
            .unwrap_or(u64::MAX);
    let points = scan
        .to_cartesian()
        .into_iter()
        .map(|(x, y)| Point2::new(f64::from(x), f64::from(y)))
        .collect();
    ScanCloud::new(points, timestamp_ns)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut recording: Option<PathBuf> = None;
    let mut stem: Option<PathBuf> = None;
    let mut save_rrd: Option<PathBuf> = None;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        if arg == "--save" {
            save_rrd = Some(PathBuf::from(args.next().ok_or("--save needs a path")?));
        } else if recording.is_none() {
            recording = Some(PathBuf::from(arg));
        } else if stem.is_none() {
            stem = Some(PathBuf::from(arg));
        } else {
            return Err(format!("unexpected argument: {arg}").into());
        }
    }
    let recording = recording.unwrap_or_else(|| PathBuf::from(DEFAULT_RECORDING));
    let stem = stem.unwrap_or_else(|| PathBuf::from("slam_map"));

    let builder = rerun::RecordingStreamBuilder::new("olivaw_slam");
    let rec = match &save_rrd {
        Some(path) => builder.save(path)?,
        None => builder.spawn()?,
    };

    println!("replaying {}", recording.display());
    let replay = ReplayTransport::from_file(&recording)?;
    let mut lidar = Lidar::with_transport(replay);
    lidar.start_scan()?;

    let mut slam = Slam::new(SlamConfig::default())?;
    let mut session_start: Option<Instant> = None;
    let mut trajectory: Vec<(f32, f32)> = Vec::new();

    for (i, scan) in lidar.scans().enumerate() {
        let scan = scan?;
        let start = *session_start.get_or_insert(scan.timestamp);
        let cloud = scan_to_cloud(&scan, start);

        let pose = slam.process_scan(&cloud)?;
        trajectory.push((pose.x as f32, pose.y as f32));

        rec.set_time_sequence("scan", i64::try_from(i).unwrap_or(i64::MAX));
        rec.log(
            "slam/scan",
            &rerun::Points2D::new(
                cloud.points.iter().map(|p| {
                    let w = pose.transform_point(*p);
                    (w.x as f32, w.y as f32)
                }),
            )
            .with_colors([rerun::Color::from_rgb(120, 200, 255)])
            .with_radii([0.015]),
        )?;
        rec.log(
            "slam/trajectory",
            &rerun::LineStrips2D::new([trajectory.clone()])
                .with_colors([rerun::Color::from_rgb(255, 170, 40)]),
        )?;
        slam.grid().log_to_rerun(&rec, "slam/map")?;
    }

    println!(
        "{} keyframes, {} loop closure(s), final pose {:?}",
        slam.keyframes().len(),
        slam.loops_closed(),
        slam.pose()
    );
    slam.grid().export_map(&stem)?;
    println!("map exported to {}.pgm / {}.yaml", stem.display(), stem.display());

    #[cfg(feature = "serialize")]
    {
        let state = stem.with_extension("olivaw");
        slam.save(&state)?;
        println!("SLAM state saved to {} (reload with Slam::load)", state.display());
    }
    if let Some(path) = &save_rrd {
        println!("rerun recording written to {}", path.display());
    }
    Ok(())
}
