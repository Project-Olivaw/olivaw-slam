//! Visualize a correlative scan match in rerun: the reference scan, the query
//! at its (wrong) initial guess, the recovered alignment, and the score
//! surface of the translation search — the primary instrument for debugging
//! matching problems.
//!
//! ```text
//! cargo run --example scan_matching_viz --features viz [-- --save out.rrd]
//! ```

#![allow(clippy::cast_possible_truncation, clippy::cast_precision_loss, clippy::cast_sign_loss)]

use olivaw_slam::matcher::CorrelativeMatcher;
use olivaw_slam::{Point2, Pose2, ScanCloud, ScanMatcher};

/// Simple synthetic room (an outer rectangle plus an off-centre box) sampled
/// by exact ray casting — the same construction the library's tests use.
fn room_scan(pose: Pose2) -> ScanCloud {
    let mut segs: Vec<(Point2, Point2)> = Vec::new();
    let mut rect = |x0: f64, y0: f64, x1: f64, y1: f64| {
        segs.push((Point2::new(x0, y0), Point2::new(x1, y0)));
        segs.push((Point2::new(x1, y0), Point2::new(x1, y1)));
        segs.push((Point2::new(x1, y1), Point2::new(x0, y1)));
        segs.push((Point2::new(x0, y1), Point2::new(x0, y0)));
    };
    rect(-3.0, -2.0, 4.0, 3.0);
    rect(0.5, 0.5, 1.5, 1.2);
    let origin = Point2::new(pose.x, pose.y);
    let mut points = Vec::new();
    for i in 0..360 {
        let bearing = f64::from(i).to_radians();
        let ang = pose.theta + bearing;
        let dir = (ang.cos(), ang.sin());
        let mut best = f64::INFINITY;
        for &(a, b) in &segs {
            let (ex, ey) = (b.x - a.x, b.y - a.y);
            let denom = dir.0 * ey - dir.1 * ex;
            if denom.abs() < 1e-12 {
                continue;
            }
            let (ox, oy) = (a.x - origin.x, a.y - origin.y);
            let t = (ox * ey - oy * ex) / denom;
            let u = (ox * dir.1 - oy * dir.0) / -denom;
            if t > 1e-9 && (0.0..=1.0).contains(&u) {
                best = best.min(t);
            }
        }
        if best.is_finite() {
            points.push(Point2::new(best * bearing.cos(), best * bearing.sin()));
        }
    }
    ScanCloud::new(points, 0)
}

fn to_pairs(points: &[Point2], pose: &Pose2) -> Vec<(f32, f32)> {
    points
        .iter()
        .map(|p| {
            let w = pose.transform_point(*p);
            (w.x as f32, w.y as f32)
        })
        .collect()
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let save = std::env::args().nth(1).filter(|a| a == "--save").and(std::env::args().nth(2));
    let builder = rerun::RecordingStreamBuilder::new("olivaw_slam_matching");
    let rec = match &save {
        Some(path) => builder.save(path)?,
        None => builder.spawn()?,
    };

    // The query scan is taken from a displaced pose; the matcher gets an
    // identity guess and must recover the displacement on its own.
    let truth = Pose2::new(0.35, -0.25, 0.20);
    let reference = room_scan(Pose2::identity());
    let query = room_scan(truth);

    let matcher = CorrelativeMatcher::default();
    let result = matcher.match_scans(&reference, &query, &Pose2::identity())?;
    println!(
        "truth:     x={:+.3} y={:+.3} θ={:+.3}\nrecovered: x={:+.3} y={:+.3} θ={:+.3}\nscore {:.3}, {} evaluations, converged: {}",
        truth.x, truth.y, truth.theta,
        result.pose.x, result.pose.y, result.pose.theta,
        result.score, result.iterations, result.converged,
    );

    rec.log(
        "match/reference",
        &rerun::Points2D::new(to_pairs(&reference.points, &Pose2::identity()))
            .with_colors([rerun::Color::from_rgb(90, 140, 255)])
            .with_radii([0.02]),
    )?;
    rec.log(
        "match/query_at_guess",
        &rerun::Points2D::new(to_pairs(&query.points, &Pose2::identity()))
            .with_colors([rerun::Color::from_rgb(230, 90, 90)])
            .with_radii([0.02]),
    )?;
    rec.log(
        "match/query_recovered",
        &rerun::Points2D::new(to_pairs(&query.points, &result.pose))
            .with_colors([rerun::Color::from_rgb(70, 210, 90)])
            .with_radii([0.02]),
    )?;

    // Score surface over (dx, dy) at the recovered rotation: brute-force
    // nearest-reference-point likelihood, the same quantity CSM maximizes.
    let entries: Vec<[f64; 2]> = reference.points.iter().map(|p| [p.x, p.y]).collect();
    // Large bucket size: axis-aligned walls put many points at the exact same
    // coordinate on one axis, which overflows kiddo's default bucket of 32.
    let tree: kiddo::float::kdtree::KdTree<f64, u64, 2, 512, u32> = (&entries).into();
    let (half, step, sigma) = (0.5_f64, 0.02_f64, 0.08_f64);
    let n = (2.0 * half / step) as usize + 1;
    let mut img = Vec::with_capacity(n * n);
    for row in 0..n {
        let dy = half - row as f64 * step; // top row = +dy
        for col in 0..n {
            let dx = col as f64 * step - half;
            let mut sum = 0.0;
            for p in &query.points {
                let w = Pose2::new(result.pose.x + dx, result.pose.y + dy, result.pose.theta)
                    .transform_point(*p);
                let nn = tree.nearest_one::<kiddo::SquaredEuclidean>(&[w.x, w.y]);
                sum += (-nn.distance / (2.0 * sigma * sigma)).exp();
            }
            let score = sum / query.points.len() as f64;
            img.push((score * 255.0).clamp(0.0, 255.0) as u8);
        }
    }
    rec.log(
        "search/score_surface",
        &rerun::Image::from_color_model_and_bytes(
            img,
            [n as u32, n as u32],
            rerun::ColorModel::L,
            rerun::ChannelDatatype::U8,
        ),
    )?;
    println!("score surface spans ±{half} m around the recovered pose (bright = better)");
    if let Some(path) = &save {
        println!("rerun recording written to {path}");
    }
    Ok(())
}
