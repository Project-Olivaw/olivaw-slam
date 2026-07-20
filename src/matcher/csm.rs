//! Correlative scan matching (CSM) after Olson 2009, "Real-Time Correlative
//! Scan Matching".
//!
//! CSM aligns a query scan to a likelihood field by evaluating a cost function
//! over a bounded 3D search window — translation in x and y, rotation in θ —
//! and taking the numerical maximum. Unlike ICP it needs no good initial
//! guess, and its runtime is bounded by the window, not by the data.
//!
//! Two-level search: a coarse pass over a max-pooled field (optimistic, so
//! candidates are never pruned by pooling), then fine refinement with bilinear
//! interpolation around the best coarse candidates.

use nalgebra::{Matrix3, Vector3};
#[cfg(feature = "parallel")]
use rayon::prelude::*;

use crate::convert::floor_to_i64;
use crate::error::SlamError;
use crate::matcher::likelihood::LikelihoodField;
use crate::matcher::{MatchResult, ScanMatcher};
use crate::pose::{Point2, Pose2, normalize_angle};
use crate::scan::ScanCloud;

/// Tunables for correlative scan matching. Distances in metres, angles in
/// radians.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct CsmConfig {
    /// Half-width of the translation search window in x, metres. Default `0.5`.
    pub search_x_m: f64,
    /// Half-width of the translation search window in y, metres. Default `0.5`.
    pub search_y_m: f64,
    /// Half-width of the rotation search window, radians. Default `0.35` (~20°).
    pub search_theta_rad: f64,
    /// Rotation step of the fine search, radians. Default `0.005` (~0.29°).
    pub angular_step_rad: f64,
    /// Coarse rotation step = `angular_step_rad × this`. Default `5`.
    pub angular_coarse_factor: usize,
    /// Translation step of the fine search, metres. Default `0.01`.
    pub linear_step_m: f64,
    /// Likelihood field raster resolution, metres. Default `0.03`.
    pub field_resolution_m: f64,
    /// Gaussian smoothing σ of the likelihood field, metres. Default `0.08`.
    pub sigma_m: f64,
    /// Coarse translation cells = `field_resolution_m × this`. Default `8`.
    pub coarse_factor: usize,
    /// Number of coarse candidates refined at fine resolution. Default `5`.
    pub refine_top_k: usize,
    /// Score below which the result is flagged not-converged. Default `0.25`.
    pub min_score: f64,
    /// Maximum likelihood-field cells — denial-of-service guard.
    /// Default `4_000_000`.
    pub max_field_cells: usize,
    /// Maximum accepted query points — denial-of-service guard. Default `4096`.
    pub max_points: usize,
}

impl Default for CsmConfig {
    fn default() -> Self {
        Self {
            search_x_m: 0.5,
            search_y_m: 0.5,
            search_theta_rad: 0.35,
            angular_step_rad: 0.005,
            angular_coarse_factor: 5,
            linear_step_m: 0.01,
            field_resolution_m: 0.03,
            sigma_m: 0.08,
            coarse_factor: 8,
            refine_top_k: 5,
            min_score: 0.25,
            max_field_cells: 4_000_000,
            max_points: 4096,
        }
    }
}

impl CsmConfig {
    /// A wide-window variant for loop-closure verification, where the pose
    /// prior is weak: ±`radius_m` translation, ±π rotation.
    #[must_use]
    pub fn wide(radius_m: f64) -> Self {
        Self {
            search_x_m: radius_m,
            search_y_m: radius_m,
            search_theta_rad: std::f64::consts::PI,
            angular_step_rad: 0.01,
            ..Self::default()
        }
    }
}

/// Correlative scan-to-scan matcher. See the module docs.
#[derive(Debug, Clone, Default)]
pub struct CorrelativeMatcher {
    config: CsmConfig,
}

impl CorrelativeMatcher {
    /// Create a matcher with the given configuration.
    #[must_use]
    pub fn new(config: CsmConfig) -> Self {
        Self { config }
    }

    /// The active configuration.
    #[must_use]
    pub fn config(&self) -> &CsmConfig {
        &self.config
    }
}

impl ScanMatcher for CorrelativeMatcher {
    fn match_scans(
        &self,
        reference: &ScanCloud,
        query: &ScanCloud,
        initial_guess: &Pose2,
    ) -> Result<MatchResult, SlamError> {
        let cfg = &self.config;
        let margin = cfg.search_x_m.max(cfg.search_y_m) + 4.0 * cfg.sigma_m + 0.2;
        let field = LikelihoodField::from_points(
            &reference.points,
            cfg.field_resolution_m,
            cfg.sigma_m,
            margin,
            cfg.max_field_cells,
        )?;
        search(&field, &query.points, initial_guess, cfg)
    }
}

/// One evaluated candidate: pose offsets relative to the guess plus score.
#[derive(Debug, Clone, Copy)]
struct Candidate {
    dx: f64,
    dy: f64,
    dtheta: f64,
    score: f64,
}

/// The shared CSM search over a prebuilt likelihood field. Used by both the
/// scan-to-scan and the scan-to-map matcher.
pub(crate) fn search(
    field: &LikelihoodField,
    query: &[Point2],
    guess: &Pose2,
    cfg: &CsmConfig,
) -> Result<MatchResult, SlamError> {
    if query.is_empty() {
        return Err(SlamError::MatchFailed { reason: "empty query scan".to_owned() });
    }
    if query.len() > cfg.max_points {
        return Err(SlamError::ScanTooLarge { actual: query.len(), limit: cfg.max_points });
    }
    let inv_n = 1.0 / f64::from(u32::try_from(query.len()).unwrap_or(u32::MAX));

    // ---- Coarse stage: every coarse θ × coarse translation grid. ----
    let coarse_field = field.max_pool(cfg.coarse_factor);
    let step_c = cfg.field_resolution_m * as_f64(cfg.coarse_factor);
    let coarse_theta_step = cfg.angular_step_rad * as_f64(cfg.angular_coarse_factor);
    let thetas = symmetric_steps(cfg.search_theta_rad, coarse_theta_step);
    let xs = symmetric_steps(cfg.search_x_m, step_c);
    let ys = symmetric_steps(cfg.search_y_m, step_c);

    let eval_theta = |dtheta: &f64| -> Candidate {
        let rotated = rotate_into_world(query, guess, *dtheta);
        let mut best = Candidate { dx: 0.0, dy: 0.0, dtheta: *dtheta, score: -1.0 };
        for &dx in &xs {
            for &dy in &ys {
                let mut sum = 0.0_f64;
                for p in &rotated {
                    sum += f64::from(coarse_field.lookup(Point2::new(p.x + dx, p.y + dy)));
                }
                let score = sum * inv_n;
                if score > best.score {
                    best = Candidate { dx, dy, dtheta: *dtheta, score };
                }
            }
        }
        best
    };

    #[cfg(feature = "parallel")]
    let mut coarse: Vec<Candidate> = thetas.par_iter().map(eval_theta).collect();
    #[cfg(not(feature = "parallel"))]
    let mut coarse: Vec<Candidate> = thetas.iter().map(eval_theta).collect();
    let coarse_evals = thetas.len() * xs.len() * ys.len();

    coarse.sort_by(|a, b| b.score.partial_cmp(&a.score).unwrap_or(std::cmp::Ordering::Equal));
    coarse.truncate(cfg.refine_top_k.max(1));

    // ---- Fine stage: bilinear scoring around each surviving candidate. ----
    let refine_span = step_c / 2.0 + cfg.linear_step_m;
    let theta_span = coarse_theta_step / 2.0 + cfg.angular_step_rad;
    let dxs = symmetric_steps(refine_span, cfg.linear_step_m);
    let dts = symmetric_steps(theta_span, cfg.angular_step_rad);

    let refine_one = |cand: &Candidate| -> (Candidate, Vec<Candidate>) {
        let mut samples = Vec::with_capacity(dts.len() * dxs.len() * dxs.len());
        let mut best = Candidate { score: -1.0, ..*cand };
        for &ddt in &dts {
            let dtheta = cand.dtheta + ddt;
            let rotated = rotate_into_world(query, guess, dtheta);
            for &ddx in &dxs {
                for &ddy in &dxs {
                    let (dx, dy) = (cand.dx + ddx, cand.dy + ddy);
                    let mut sum = 0.0_f64;
                    for p in &rotated {
                        sum += f64::from(field.lookup_bilinear(Point2::new(p.x + dx, p.y + dy)));
                    }
                    let score = sum * inv_n;
                    let sample = Candidate { dx, dy, dtheta, score };
                    samples.push(sample);
                    if score > best.score {
                        best = sample;
                    }
                }
            }
        }
        (best, samples)
    };

    #[cfg(feature = "parallel")]
    let refined: Vec<(Candidate, Vec<Candidate>)> = coarse.par_iter().map(refine_one).collect();
    #[cfg(not(feature = "parallel"))]
    let refined: Vec<(Candidate, Vec<Candidate>)> = coarse.iter().map(refine_one).collect();

    let fine_evals: usize = refined.iter().map(|(_, s)| s.len()).sum();
    let Some(mut best) = refined
        .iter()
        .map(|(b, _)| *b)
        .max_by(|a, b| a.score.partial_cmp(&b.score).unwrap_or(std::cmp::Ordering::Equal))
    else {
        return Err(SlamError::MatchFailed { reason: "empty search window".to_owned() });
    };
    if best.score <= 0.0 {
        return Err(SlamError::MatchFailed {
            reason: "no overlap between query and reference anywhere in the window".to_owned(),
        });
    }

    best = parabolic_refine(field, query, guess, inv_n, cfg, best);

    // Covariance from weighted second moments of the fine samples around the
    // maximum (Olson 2009 §IV) — sharp peaks give tight covariance, ridges
    // (corridors) stretch it along the ambiguous direction.
    let all_samples = refined.iter().flat_map(|(_, s)| s.iter());
    let covariance = sample_covariance(all_samples, &best, cfg);

    let on_boundary = best.dx.abs() >= cfg.search_x_m - cfg.linear_step_m
        || best.dy.abs() >= cfg.search_y_m - cfg.linear_step_m
        || best.dtheta.abs() >= cfg.search_theta_rad - cfg.angular_step_rad;

    Ok(MatchResult {
        pose: Pose2::new(guess.x + best.dx, guess.y + best.dy, guess.theta + best.dtheta),
        covariance,
        score: best.score.clamp(0.0, 1.0),
        iterations: coarse_evals + fine_evals,
        converged: !on_boundary && best.score >= cfg.min_score,
    })
}

/// Sub-step parabolic refinement: fit a 1D parabola through the score at
/// best ± one step along each axis and move to its vertex. Pushes accuracy
/// well below the search step on smooth score surfaces.
fn parabolic_refine(
    field: &LikelihoodField,
    query: &[Point2],
    guess: &Pose2,
    inv_n: f64,
    cfg: &CsmConfig,
    mut best: Candidate,
) -> Candidate {
    let score_at = |dx: f64, dy: f64, dtheta: f64| -> f64 {
        let rotated = rotate_into_world(query, guess, dtheta);
        let mut sum = 0.0_f64;
        for p in &rotated {
            sum += f64::from(field.lookup_bilinear(Point2::new(p.x + dx, p.y + dy)));
        }
        sum * inv_n
    };
    let vertex = |s_minus: f64, s0: f64, s_plus: f64, step: f64| -> f64 {
        let denom = s_plus + s_minus - 2.0 * s0;
        if denom < -1e-12 {
            (step * (s_minus - s_plus) / (2.0 * denom)).clamp(-step, step)
        } else {
            0.0 // not a local peak along this axis — leave it
        }
    };
    let lin = cfg.linear_step_m;
    let ang = cfg.angular_step_rad;
    best.dx += vertex(
        score_at(best.dx - lin, best.dy, best.dtheta),
        best.score,
        score_at(best.dx + lin, best.dy, best.dtheta),
        lin,
    );
    best.dy += vertex(
        score_at(best.dx, best.dy - lin, best.dtheta),
        best.score,
        score_at(best.dx, best.dy + lin, best.dtheta),
        lin,
    );
    best.dtheta += vertex(
        score_at(best.dx, best.dy, best.dtheta - ang),
        best.score,
        score_at(best.dx, best.dy, best.dtheta + ang),
        ang,
    );
    best.score = best.score.max(score_at(best.dx, best.dy, best.dtheta));
    best
}

/// Transform the query points into the world frame at the guess pose rotated
/// by an extra `dtheta` (translation applied from the guess; the search adds
/// its own offsets on top).
fn rotate_into_world(query: &[Point2], guess: &Pose2, dtheta: f64) -> Vec<Point2> {
    let pose = Pose2::new(guess.x, guess.y, normalize_angle(guess.theta + dtheta));
    query.iter().map(|p| pose.transform_point(*p)).collect()
}

/// Symmetric sample offsets `-n·step ..= n·step` covering ±`half_width`.
fn symmetric_steps(half_width: f64, step: f64) -> Vec<f64> {
    let n = floor_to_i64((half_width / step).floor())
        .and_then(|v| usize::try_from(v).ok())
        .unwrap_or(0);
    let total = 2 * n + 1;
    let mut out = Vec::with_capacity(total);
    for i in 0..total {
        out.push(step * (as_f64(i) - as_f64(n)));
    }
    out
}

/// Weighted covariance of the candidate samples about the best candidate.
fn sample_covariance<'a>(
    samples: impl Iterator<Item = &'a Candidate>,
    best: &Candidate,
    cfg: &CsmConfig,
) -> Matrix3<f64> {
    let mut w_sum = 0.0_f64;
    let mut mean = Vector3::<f64>::zeros();
    let mut second = Matrix3::<f64>::zeros();
    for s in samples {
        // Sharpen the score surface so the covariance reflects the peak, not
        // the whole window.
        let w = (s.score / best.score).clamp(0.0, 1.0).powi(8);
        if w < 1e-6 {
            continue;
        }
        let v = Vector3::new(s.dx, s.dy, s.dtheta);
        w_sum += w;
        mean += w * v;
        second += w * v * v.transpose();
    }
    if w_sum <= 0.0 {
        return Matrix3::from_diagonal_element(1e3);
    }
    mean /= w_sum;
    let mut cov = second / w_sum - mean * mean.transpose();
    // Floor at the search quantization variance so the matrix stays PD.
    let lin_q = cfg.linear_step_m * cfg.linear_step_m / 12.0;
    let ang_q = cfg.angular_step_rad * cfg.angular_step_rad / 12.0;
    cov.m11 = cov.m11.max(lin_q);
    cov.m22 = cov.m22.max(lin_q);
    cov.m33 = cov.m33.max(ang_q);
    cov
}

/// Lossless usize→f64 for small values (step counts, pool factors).
fn as_f64(v: usize) -> f64 {
    u32::try_from(v).map_or(f64::MAX, f64::from)
}

#[cfg(test)]
mod tests {
    use approx::assert_abs_diff_eq;

    use super::*;
    use crate::matcher::test_scenes::{noisy_room_scan, room_scan};

    #[test]
    fn recovers_transform_within_1cm_half_degree_without_guess() {
        let reference = room_scan(Pose2::identity());
        let truth = Pose2::new(0.31, -0.22, 0.17); // ~10° — far beyond ICP's basin
        let query = room_scan(truth);
        let matcher = CorrelativeMatcher::default();
        let result = matcher.match_scans(&reference, &query, &Pose2::identity()).unwrap();
        assert!(result.converged, "score {}", result.score);
        // 0.1.0 definition of done: within 1 cm / 0.5°.
        assert_abs_diff_eq!(result.pose.x, truth.x, epsilon = 0.01);
        assert_abs_diff_eq!(result.pose.y, truth.y, epsilon = 0.01);
        assert_abs_diff_eq!(result.pose.theta, truth.theta, epsilon = 0.5_f64.to_radians());
    }

    #[test]
    fn survives_sensor_noise() {
        let reference = noisy_room_scan(Pose2::identity(), 0.01, 7);
        let truth = Pose2::new(-0.18, 0.25, -0.12);
        let query = noisy_room_scan(truth, 0.01, 8);
        let matcher = CorrelativeMatcher::default();
        let result = matcher.match_scans(&reference, &query, &Pose2::identity()).unwrap();
        assert!(result.converged, "score {}", result.score);
        assert_abs_diff_eq!(result.pose.x, truth.x, epsilon = 0.03);
        assert_abs_diff_eq!(result.pose.y, truth.y, epsilon = 0.03);
        assert_abs_diff_eq!(result.pose.theta, truth.theta, epsilon = 0.02);
    }

    #[test]
    fn covariance_is_positive_definite_and_ordered() {
        let reference = room_scan(Pose2::identity());
        let query = room_scan(Pose2::new(0.1, 0.1, 0.05));
        let matcher = CorrelativeMatcher::default();
        let result = matcher.match_scans(&reference, &query, &Pose2::identity()).unwrap();
        assert!(result.covariance.cholesky().is_some(), "covariance must be PD");
        assert!(result.covariance.m11 < 0.1, "confident match should be tight");
    }

    #[test]
    fn flags_boundary_maxima_as_not_converged() {
        let reference = room_scan(Pose2::identity());
        // True offset outside the ±0.5 m window: the best in-window score
        // lands on the boundary and must not be trusted.
        let query = room_scan(Pose2::new(1.2, 0.0, 0.0));
        let matcher = CorrelativeMatcher::default();
        if let Ok(result) = matcher.match_scans(&reference, &query, &Pose2::identity()) {
            assert!(
                !result.converged || result.score < 0.5,
                "an out-of-window transform must not produce a confident match"
            );
        }
    }
}
