//! Live, real-time SLAM from a SLAMTEC C1 plugged into this machine: scans
//! stream straight from the device through the full pipeline — preprocessing,
//! scan-to-map matching, keyframes, pose graph, loop closure — with the map,
//! trajectory, and live scan in the rerun viewer.
//!
//! ```text
//! cargo run --release --example slam_live --features "viz serialize" -- \
//!     [--port /dev/cu.usbserial-XXXX] [--seconds 120] [--stem live_map] [--save out.rrd]
//! ```
//!
//! - `--port` — serial port; auto-detected if omitted.
//! - `--seconds` — how long to run (default 120). The map and SLAM state are
//!   exported when the time is up.
//! - `--stem` — output name: `<stem>.pgm` + `<stem>.yaml` (map_server format)
//!   and, with the `serialize` feature, `<stem>.olivaw` (reloadable state).
//! - `--save` — record rerun data to a file instead of spawning the viewer.
//!
//! Carry the lidar around (steadily, ~walking pace) to build a map; keyframes
//! are created every 0.3 m / 0.3 rad of motion, so a stationary sensor shows
//! a live pose over a single-keyframe map.

#![allow(clippy::cast_possible_truncation)]

use std::path::PathBuf;
use std::time::Instant;

use olivaw_lidar::transport::{auto_detect_port, prefer_callout_device};
use olivaw_lidar::{Lidar, Scan};
use olivaw_slam::{Point2, ScanCloud, Slam, SlamConfig};

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
    let mut port: Option<String> = None;
    let mut seconds: u64 = 120;
    let mut stem = PathBuf::from("live_map");
    let mut save_rrd: Option<PathBuf> = None;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--port" => port = Some(args.next().ok_or("--port needs a path")?),
            "--seconds" => seconds = args.next().ok_or("--seconds needs a number")?.parse()?,
            "--stem" => stem = PathBuf::from(args.next().ok_or("--stem needs a name")?),
            "--save" => save_rrd = Some(PathBuf::from(args.next().ok_or("--save needs a path")?)),
            other => return Err(format!("unexpected argument: {other}").into()),
        }
    }
    let port = match port {
        Some(p) => prefer_callout_device(&p),
        None => auto_detect_port().ok_or(
            "no serial port found that looks like a lidar; pass one with --port <PATH>",
        )?,
    };

    let builder = rerun::RecordingStreamBuilder::new("olivaw_slam_live");
    let rec = match &save_rrd {
        Some(path) => builder.save(path)?,
        None => builder.spawn().map_err(|e| {
            format!(
                "failed to spawn the rerun viewer ({e}).\n\
                 Install it with `uv tool install rerun-sdk`, or run headless with `--save out.rrd`."
            )
        })?,
    };

    println!("opening lidar on {port}");
    let mut lidar = Lidar::open(&port)?;
    let health = lidar.health()?;
    println!("device health: {health:?}");
    lidar.start_scan()?;
    println!("running live SLAM for {seconds} s — move the lidar to build a map");

    let mut slam = Slam::new(SlamConfig::default())?;
    let started = Instant::now();
    let mut session_start: Option<Instant> = None;
    let mut trajectory: Vec<(f32, f32)> = Vec::new();
    let mut processed: u64 = 0;

    for (i, scan) in lidar.scans().enumerate() {
        let scan = scan?;
        let start = *session_start.get_or_insert(scan.timestamp);
        let cloud = scan_to_cloud(&scan, start);

        let t0 = Instant::now();
        let pose = slam.process_scan(&cloud)?;
        let match_ms = t0.elapsed().as_secs_f64() * 1e3;
        processed += 1;
        trajectory.push((pose.x as f32, pose.y as f32));

        rec.set_time_sequence("scan", i64::try_from(i).unwrap_or(i64::MAX));
        rec.log(
            "live/scan",
            &rerun::Points2D::new(cloud.points.iter().map(|p| {
                let w = pose.transform_point(*p);
                (w.x as f32, w.y as f32)
            }))
            .with_colors([rerun::Color::from_rgb(120, 200, 255)])
            .with_radii([0.015]),
        )?;
        rec.log(
            "live/trajectory",
            &rerun::LineStrips2D::new([trajectory.clone()])
                .with_colors([rerun::Color::from_rgb(255, 170, 40)]),
        )?;
        // The map image is heavier than the points; refresh it at ~1 Hz.
        if i % 10 == 0 {
            slam.grid().log_to_rerun(&rec, "live/map")?;
        }
        print!(
            "\r  scan {i}: pose ({:+.2}, {:+.2}, {:+.2} rad), {} keyframes, {} loop(s), {match_ms:.0} ms  ",
            pose.x,
            pose.y,
            pose.theta,
            slam.keyframes().len(),
            slam.loops_closed(),
        );
        use std::io::Write as _;
        std::io::stdout().flush().ok();

        if started.elapsed().as_secs() >= seconds {
            break;
        }
    }
    println!();
    lidar.stop()?;

    println!(
        "{processed} scans, {} keyframes, {} loop closure(s), final pose {:?}",
        slam.keyframes().len(),
        slam.loops_closed(),
        slam.pose()
    );
    slam.grid().log_to_rerun(&rec, "live/map")?;
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
