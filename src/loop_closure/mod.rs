//! Loop closure: recognizing previously visited places and producing
//! verified graph constraints.
//!
//! Structured as three separate, individually testable stages:
//!
//! 1. **Candidate search** — keyframes whose estimated pose is within a
//!    radius of the current pose but temporally distant.
//! 2. **Verification** — wide-window CSM between the current scan and the
//!    candidate's scan.
//! 3. **Gating** — performed by the caller ([`crate::slam::Slam`]): the edge
//!    is added speculatively, the graph optimized on a copy, and the
//!    constraint accepted only if its residual stays consistent. **A single
//!    false positive folds the map in half** — every threshold here errs on
//!    the conservative side, because a missed closure costs accuracy while a
//!    false one destroys the map.

use nalgebra::Matrix3;

use crate::error::SlamError;
use crate::matcher::{CorrelativeMatcher, CsmConfig, ScanMatcher};
use crate::pose::Pose2;
use crate::scan::ScanCloud;

/// Tunables for loop-closure detection. Distances in metres, angles in
/// radians.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct LoopClosureConfig {
    /// Candidate radius: keyframes estimated to be within this distance of
    /// the current pose are considered. Default `3.0`.
    pub search_radius_m: f64,
    /// Minimum keyframe-index separation for a candidate — recent keyframes
    /// are not loops. Default `20`.
    pub min_node_separation: usize,
    /// CSM translation half-window for verification (the pose estimate may
    /// have drifted this far). Default `2.0`.
    pub verification_window_m: f64,
    /// CSM rotation half-window for verification. Default `π` (all headings).
    pub verification_theta_rad: f64,
    /// Minimum verification match score to accept a candidate. Conservative
    /// on purpose. Default `0.6`.
    pub min_score: f64,
    /// Maximum candidates verified per keyframe (verification is the
    /// expensive step). Default `3`.
    pub max_candidates: usize,
    /// Post-optimization χ² gate on the loop edge residual (3 degrees of
    /// freedom; 9.0 ≈ the 97th percentile). Applied by the caller during
    /// gating. Default `9.0`.
    pub residual_gate: f64,
}

impl Default for LoopClosureConfig {
    fn default() -> Self {
        Self {
            search_radius_m: 3.0,
            min_node_separation: 20,
            verification_window_m: 2.0,
            verification_theta_rad: std::f64::consts::PI,
            min_score: 0.6,
            max_candidates: 3,
            residual_gate: 9.0,
        }
    }
}

/// A verified loop-closure constraint, ready for speculative insertion.
#[derive(Debug, Clone)]
pub struct LoopConstraint {
    /// Index of the matched (older) keyframe.
    pub node: usize,
    /// Measured relative pose: `keyframe[node].between(current)`.
    pub relative: Pose2,
    /// Information matrix of the measurement over `[x, y, θ]`.
    pub information: Matrix3<f64>,
    /// Verification match score (`0..=1`).
    pub score: f64,
}

/// Anything that pairs a pose estimate with a scan — implemented by
/// [`crate::Keyframe`] and by plain `(Pose2, ScanCloud)` tuples.
pub trait KeyframeLike {
    /// Current pose estimate of this keyframe (world frame).
    fn pose(&self) -> Pose2;
    /// The keyframe's scan in its own sensor frame.
    fn scan(&self) -> &ScanCloud;
}

impl KeyframeLike for (Pose2, ScanCloud) {
    fn pose(&self) -> Pose2 {
        self.0
    }
    fn scan(&self) -> &ScanCloud {
        &self.1
    }
}

/// Loop-closure detector (stages 1 and 2; gating is the caller's).
#[derive(Debug, Clone, Default)]
pub struct LoopDetector {
    config: LoopClosureConfig,
}

impl LoopDetector {
    /// Create a detector with the given configuration.
    #[must_use]
    pub fn new(config: LoopClosureConfig) -> Self {
        Self { config }
    }

    /// The active configuration.
    #[must_use]
    pub fn config(&self) -> &LoopClosureConfig {
        &self.config
    }

    /// Stage 1: indices of keyframes near `poses[current]` but at least
    /// `min_node_separation` keyframes older, nearest first, capped at
    /// `max_candidates`.
    #[must_use]
    pub fn candidates(&self, poses: &[Pose2], current: usize) -> Vec<usize> {
        let Some(cur) = poses.get(current) else { return Vec::new() };
        let r2 = self.config.search_radius_m * self.config.search_radius_m;
        let mut found: Vec<(f64, usize)> = poses
            .iter()
            .enumerate()
            .take(current.saturating_sub(self.config.min_node_separation))
            .filter_map(|(i, p)| {
                let (dx, dy) = (p.x - cur.x, p.y - cur.y);
                let d2 = dx * dx + dy * dy;
                (d2 <= r2).then_some((d2, i))
            })
            .collect();
        found.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        found.into_iter().take(self.config.max_candidates).map(|(_, i)| i).collect()
    }

    /// Stages 1+2: find the best verified loop constraint for the keyframe
    /// `current`, or `None` if no candidate survives verification.
    ///
    /// # Errors
    ///
    /// Only hard failures propagate ([`SlamError::GridTooLarge`] from a
    /// mis-configured verification window); candidates that merely fail to
    /// match are skipped.
    pub fn detect<K: KeyframeLike>(
        &self,
        keyframes: &[K],
        current: usize,
    ) -> Result<Option<LoopConstraint>, SlamError> {
        let poses: Vec<Pose2> = keyframes.iter().map(KeyframeLike::pose).collect();
        let (Some(cur_kf), Some(cur_pose)) = (keyframes.get(current), poses.get(current)) else {
            return Ok(None);
        };
        let cur_scan = cur_kf.scan();
        let csm = CsmConfig {
            search_x_m: self.config.verification_window_m,
            search_y_m: self.config.verification_window_m,
            search_theta_rad: self.config.verification_theta_rad,
            // Loop closing needs speed more than sub-cm precision; the graph
            // optimization refines geometry anyway.
            angular_step_rad: 0.01,
            linear_step_m: 0.02,
            ..CsmConfig::default()
        };
        let matcher = CorrelativeMatcher::new(csm);

        let mut best: Option<LoopConstraint> = None;
        for i in self.candidates(&poses, current) {
            let (Some(ref_kf), Some(ref_pose)) = (keyframes.get(i), poses.get(i)) else {
                continue;
            };
            let guess = ref_pose.between(cur_pose);
            let result = match matcher.match_scans(ref_kf.scan(), cur_scan, &guess) {
                Ok(r) => r,
                Err(SlamError::MatchFailed { .. }) => continue,
                Err(e) => return Err(e),
            };
            if !result.converged || result.score < self.config.min_score {
                continue;
            }
            let information = invert_covariance(&result.covariance);
            if best.as_ref().is_none_or(|b| result.score > b.score) {
                best = Some(LoopConstraint {
                    node: i,
                    relative: result.pose,
                    information,
                    score: result.score,
                });
            }
        }
        Ok(best)
    }
}

/// Covariance → information with graceful degradation: a singular covariance
/// falls back to a weak diagonal information (the constraint still helps but
/// cannot dominate).
fn invert_covariance(cov: &Matrix3<f64>) -> Matrix3<f64> {
    cov.try_inverse()
        .filter(|inv| inv.iter().all(|v| v.is_finite()))
        .unwrap_or_else(|| Matrix3::from_diagonal_element(1.0))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::matcher::test_scenes::room_scan;

    #[test]
    fn candidate_search_respects_radius_and_separation() {
        let detector = LoopDetector::new(LoopClosureConfig {
            search_radius_m: 1.0,
            min_node_separation: 3,
            ..LoopClosureConfig::default()
        });
        // Walk right, then return: node 6 sits on top of node 0.
        let poses = [
            Pose2::identity(),
            Pose2::new(2.0, 0.0, 0.0),
            Pose2::new(4.0, 0.0, 0.0),
            Pose2::new(4.0, 2.0, 0.0),
            Pose2::new(2.0, 2.0, 0.0),
            Pose2::new(0.0, 2.0, 0.0),
            Pose2::new(0.1, 0.1, 0.0),
        ];
        let c = detector.candidates(&poses, 6);
        assert_eq!(c, vec![0], "only node 0 is near and old enough, got {c:?}");
        // Node 5 is nearby in index — excluded by separation.
        assert!(!c.contains(&5));
    }

    #[test]
    fn verification_accepts_true_loop_and_measures_it() {
        // Two scans of the same room from nearly the same pose, with a
        // drifted estimate: verification must find the true relative pose.
        let truth_a = Pose2::identity();
        let truth_b = Pose2::new(0.3, -0.2, 0.15);
        let mut kfs = vec![(truth_a, room_scan(truth_a))];
        // Padding keyframes far away so separation is satisfied.
        for i in 0..21 {
            kfs.push((Pose2::new(100.0 + f64::from(i), 100.0, 0.0), ScanCloud::default()));
        }
        // The *estimate* for the current keyframe has drifted ~0.5 m.
        kfs.push((Pose2::new(0.7, 0.3, 0.05), room_scan(truth_b)));
        let current = kfs.len() - 1;

        let detector = LoopDetector::new(LoopClosureConfig {
            min_node_separation: 5,
            ..LoopClosureConfig::default()
        });
        let found = detector.detect(&kfs, current).unwrap();
        let c = found.expect("true loop must be detected");
        assert_eq!(c.node, 0);
        // Measured relative pose ≈ truth_a.between(truth_b) = truth_b here.
        approx::assert_abs_diff_eq!(c.relative.x, truth_b.x, epsilon = 0.05);
        approx::assert_abs_diff_eq!(c.relative.y, truth_b.y, epsilon = 0.05);
        approx::assert_abs_diff_eq!(c.relative.theta, truth_b.theta, epsilon = 0.03);
    }

    #[test]
    fn verification_rejects_nonoverlapping_scenes() {
        // Current scan far from candidate: no consistent alignment exists.
        let mut kfs = vec![(Pose2::identity(), room_scan(Pose2::identity()))];
        for i in 0..21 {
            kfs.push((Pose2::new(100.0 + f64::from(i), 100.0, 0.0), ScanCloud::default()));
        }
        // A "scan" that is just a tight arc of points nothing like the room.
        let bogus = ScanCloud::new(
            (0..60)
                .map(|i| {
                    let a = f64::from(i) * 0.02;
                    crate::pose::Point2::new(11.0 + 0.2 * a.cos(), 0.3 * a.sin())
                })
                .collect(),
            0,
        );
        kfs.push((Pose2::new(0.2, 0.1, 0.0), bogus));
        let current = kfs.len() - 1;

        let detector = LoopDetector::new(LoopClosureConfig {
            min_node_separation: 5,
            ..LoopClosureConfig::default()
        });
        let found = detector.detect(&kfs, current).unwrap();
        assert!(
            found.is_none(),
            "a non-overlapping scene must not verify: {found:?}"
        );
    }
}
