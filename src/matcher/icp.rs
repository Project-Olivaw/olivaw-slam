//! Point-to-point ICP with k-d tree correspondences and Huber weighting.

use kiddo::SquaredEuclidean;
use nalgebra::{Matrix3, Vector2, Vector3};

/// k-d tree with a large bucket size: kiddo panics if more points than the
/// bucket size share the exact same coordinate on one axis, which happens with
/// synthetic (and occasionally real) scans of long axis-aligned walls.
type RefTree = kiddo::float::kdtree::KdTree<f64, u64, 2, 512, u32>;

use crate::error::SlamError;
use crate::matcher::{MatchResult, ScanMatcher};
use crate::pose::{Pose2, normalize_angle};
use crate::scan::ScanCloud;

/// Tunables for [`IcpMatcher`]. Distances in metres, angles in radians.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct IcpConfig {
    /// Maximum Gauss-Newton iterations. Default `50`.
    pub max_iterations: usize,
    /// Correspondences farther apart than this are rejected, metres.
    /// Default `1.0`.
    pub max_correspondence_dist_m: f64,
    /// Huber kernel width, metres: residuals beyond this are down-weighted
    /// linearly instead of quadratically. Default `0.1`.
    pub huber_delta_m: f64,
    /// Convergence threshold on the translation update, metres. Default `1e-5`.
    pub translation_epsilon_m: f64,
    /// Convergence threshold on the rotation update, radians. Default `1e-5`.
    pub rotation_epsilon_rad: f64,
    /// Minimum points required in each scan. Default `10`.
    pub min_points: usize,
    /// Minimum inlier fraction for the result to count as converged.
    /// Default `0.5`.
    pub min_inlier_ratio: f64,
    /// RMSE scale for the score: `score = inlier_ratio × exp(-rmse / this)`.
    /// Default `0.1`.
    pub score_sigma_m: f64,
}

impl Default for IcpConfig {
    fn default() -> Self {
        Self {
            max_iterations: 50,
            max_correspondence_dist_m: 1.0,
            huber_delta_m: 0.1,
            translation_epsilon_m: 1e-5,
            rotation_epsilon_rad: 1e-5,
            min_points: 10,
            min_inlier_ratio: 0.5,
            score_sigma_m: 0.1,
        }
    }
}

/// Point-to-point ICP scan matcher.
///
/// Fast and precise when the initial guess is within roughly half the scan's
/// feature spacing; diverges silently otherwise (use
/// [`CorrelativeMatcher`](crate::matcher::CorrelativeMatcher) when no good
/// guess exists). Runtime varies with the data — for bounded runtime, CSM.
#[derive(Debug, Clone, Default)]
pub struct IcpMatcher {
    config: IcpConfig,
}

impl IcpMatcher {
    /// Create a matcher with the given configuration.
    #[must_use]
    pub fn new(config: IcpConfig) -> Self {
        Self { config }
    }

    /// The active configuration.
    #[must_use]
    pub fn config(&self) -> &IcpConfig {
        &self.config
    }
}

/// Accumulated weighted Gauss-Newton normal equations for one iteration.
#[derive(Debug, Default)]
struct NormalEquations {
    hessian: Matrix3<f64>,
    rhs: Vector3<f64>,
    inliers: usize,
    sse: f64,
}

/// One correspondence + accumulation pass at the current pose estimate.
fn accumulate(
    tree: &RefTree,
    entries: &[[f64; 2]],
    query: &ScanCloud,
    pose: &Pose2,
    cfg: &IcpConfig,
) -> NormalEquations {
    let max_dist2 = cfg.max_correspondence_dist_m * cfg.max_correspondence_dist_m;
    let (sin, cos) = pose.theta.sin_cos();
    let mut n = NormalEquations::default();
    for q in &query.points {
        let world = pose.transform_point(*q);
        let nn = tree.nearest_one::<SquaredEuclidean>(&[world.x, world.y]);
        if nn.distance > max_dist2 {
            continue;
        }
        let Some(target) = usize::try_from(nn.item).ok().and_then(|i| entries.get(i)) else {
            continue;
        };
        let residual = Vector2::new(world.x - target[0], world.y - target[1]);
        let err = residual.norm();
        // Huber: quadratic inside delta, linear outside.
        let weight =
            if err <= cfg.huber_delta_m || err == 0.0 { 1.0 } else { cfg.huber_delta_m / err };
        // d(R·q)/dθ.
        let dtheta = Vector2::new(-sin * q.x - cos * q.y, cos * q.x - sin * q.y);
        // J = [ 1 0 dθx ; 0 1 dθy ], products accumulated directly.
        n.hessian.m11 += weight;
        n.hessian.m22 += weight;
        n.hessian.m13 += weight * dtheta.x;
        n.hessian.m23 += weight * dtheta.y;
        n.hessian.m33 += weight * (dtheta.x * dtheta.x + dtheta.y * dtheta.y);
        n.rhs +=
            weight * Vector3::new(residual.x, residual.y, dtheta.dot(&residual));
        n.inliers += 1;
        n.sse += weight * err * err;
    }
    // Symmetrize the accumulated upper triangle.
    n.hessian.m31 = n.hessian.m13;
    n.hessian.m32 = n.hessian.m23;
    n
}

impl ScanMatcher for IcpMatcher {
    fn match_scans(
        &self,
        reference: &ScanCloud,
        query: &ScanCloud,
        initial_guess: &Pose2,
    ) -> Result<MatchResult, SlamError> {
        let cfg = &self.config;
        if reference.len() < cfg.min_points || query.len() < cfg.min_points {
            return Err(SlamError::MatchFailed {
                reason: format!(
                    "too few points (reference {}, query {}, need {})",
                    reference.len(),
                    query.len(),
                    cfg.min_points
                ),
            });
        }

        // k-d tree over the (finite) reference points.
        let entries: Vec<[f64; 2]> = reference
            .points
            .iter()
            .filter(|p| p.x.is_finite() && p.y.is_finite())
            .map(|p| [p.x, p.y])
            .collect();
        if entries.len() < cfg.min_points {
            return Err(SlamError::MatchFailed {
                reason: "too few finite reference points".to_owned(),
            });
        }
        let tree: RefTree = (&entries).into();

        let mut pose = *initial_guess;
        let mut iterations = 0;
        let mut converged = false;
        let mut normal = NormalEquations::default();

        for iter in 0..cfg.max_iterations {
            iterations = iter + 1;
            normal = accumulate(&tree, &entries, query, &pose, cfg);
            if normal.inliers < 2 {
                return Err(SlamError::MatchFailed {
                    reason: format!("only {} correspondences within range", normal.inliers),
                });
            }
            let Some(chol) = normal.hessian.cholesky() else {
                return Err(SlamError::MatchFailed {
                    reason: "singular normal equations (degenerate geometry)".to_owned(),
                });
            };
            let delta = chol.solve(&(-normal.rhs));
            pose = Pose2 {
                x: pose.x + delta.x,
                y: pose.y + delta.y,
                theta: normalize_angle(pose.theta + delta.z),
            };
            if delta.xy().norm() < cfg.translation_epsilon_m
                && delta.z.abs() < cfg.rotation_epsilon_rad
            {
                converged = true;
                break;
            }
        }

        let n_inliers_f = f64::from(u32::try_from(normal.inliers).unwrap_or(u32::MAX));
        let n_query_f = f64::from(u32::try_from(query.len()).unwrap_or(u32::MAX));
        let inlier_ratio = n_inliers_f / n_query_f;
        let rmse = (normal.sse / n_inliers_f.max(1.0)).sqrt();
        let score = (inlier_ratio * (-rmse / cfg.score_sigma_m).exp()).clamp(0.0, 1.0);

        // Covariance ≈ σ² H⁻¹ with σ² from the weighted residuals
        // (2 scalar residuals per correspondence, 3 parameters).
        let dof = (2.0 * n_inliers_f - 3.0).max(1.0);
        let sigma2 = (normal.sse / dof).max(1e-12);
        let covariance = normal
            .hessian
            .try_inverse()
            .map_or_else(|| Matrix3::from_diagonal_element(1e3), |inv| inv * sigma2);

        Ok(MatchResult {
            pose,
            covariance,
            score,
            iterations,
            converged: converged && inlier_ratio >= cfg.min_inlier_ratio,
        })
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_abs_diff_eq;

    use super::*;
    use crate::matcher::test_scenes::{noisy_room_scan, room_scan};

    fn recover(true_pose: Pose2, reference: &ScanCloud, query: &ScanCloud, guess: Pose2) -> Pose2 {
        let matcher = IcpMatcher::default();
        let result = matcher.match_scans(reference, query, &guess).unwrap();
        assert!(result.converged, "ICP did not converge (score {})", result.score);
        assert!(result.score > 0.5, "weak score {}", result.score);
        let _ = true_pose;
        result.pose
    }

    #[test]
    fn recovers_known_transform_from_clean_data() {
        let reference = room_scan(Pose2::identity());
        let truth = Pose2::new(0.12, -0.08, 0.07);
        let query = room_scan(truth);
        let est = recover(truth, &reference, &query, Pose2::identity());
        // Point-to-point ICP carries an inherent bias when the two scans
        // sample different physical points along the same surface (the reason
        // point-to-line exists as the planned upgrade). ~1.5 cm on ray-cast
        // walls is expected; the 1 cm accuracy requirement is met by CSM.
        assert_abs_diff_eq!(est.x, truth.x, epsilon = 2e-2);
        assert_abs_diff_eq!(est.y, truth.y, epsilon = 2e-2);
        assert_abs_diff_eq!(est.theta, truth.theta, epsilon = 1e-2);
    }

    #[test]
    fn degrades_gracefully_with_noise() {
        let reference = noisy_room_scan(Pose2::identity(), 0.005, 1);
        let truth = Pose2::new(0.10, 0.05, -0.05);
        let query = noisy_room_scan(truth, 0.005, 2);
        let est = recover(truth, &reference, &query, Pose2::identity());
        // 5 mm noise: expect centimetre-level recovery, not failure.
        assert_abs_diff_eq!(est.x, truth.x, epsilon = 2e-2);
        assert_abs_diff_eq!(est.y, truth.y, epsilon = 2e-2);
        assert_abs_diff_eq!(est.theta, truth.theta, epsilon = 2e-2);
    }

    #[test]
    fn rejects_degenerate_input() {
        let matcher = IcpMatcher::default();
        let empty = ScanCloud::default();
        let scan = room_scan(Pose2::identity());
        assert!(matches!(
            matcher.match_scans(&empty, &scan, &Pose2::identity()),
            Err(SlamError::MatchFailed { .. })
        ));
    }
}
