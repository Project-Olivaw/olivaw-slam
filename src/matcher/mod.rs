//! Scan matching: ICP (scan→scan) and correlative scan matching (CSM,
//! scan→scan and scan→map).
//!
//! All matchers implement [`ScanMatcher`]: given a reference, a query, and an
//! initial guess, they return the pose of the query frame expressed in the
//! reference frame, with a covariance and a normalized score.
//!
//! - [`IcpMatcher`] — point-to-point ICP. Fast and accurate *given a good
//!   initial guess*; runtime varies with the data.
//! - [`CorrelativeMatcher`] — CSM per Olson 2009. Evaluates a bounded 3D
//!   search window (x, y, θ) over a likelihood field, so it needs no good
//!   initial guess and its runtime is predictable. This is what makes the
//!   system robust without odometry.
//! - [`ScanToMapMatcher`] — the same CSM machinery, but the likelihood field
//!   comes from the accumulated occupancy grid. This kills incremental drift.

mod csm;
mod icp;
mod likelihood;
mod scan_to_map;
#[cfg(test)]
pub(crate) mod test_scenes;

use nalgebra::Matrix3;

pub use csm::{CorrelativeMatcher, CsmConfig};
pub use icp::{IcpConfig, IcpMatcher};
pub use scan_to_map::{ScanToMapConfig, ScanToMapMatcher};

use crate::error::SlamError;
use crate::pose::Pose2;
use crate::scan::ScanCloud;

/// Result of a scan match.
#[derive(Debug, Clone)]
pub struct MatchResult {
    /// Estimated pose of the query frame in the reference frame
    /// (metres/radians).
    pub pose: Pose2,
    /// Covariance of the estimate over `(x, y, θ)` (m², m·rad, rad²).
    pub covariance: Matrix3<f64>,
    /// Normalized match quality in `0..=1`, higher is better.
    pub score: f64,
    /// Iterations (ICP) or candidate evaluations (CSM) performed.
    pub iterations: usize,
    /// `true` if the estimate is trustworthy: the optimization converged
    /// (ICP) or the maximum lies inside the search window with an acceptable
    /// score (CSM). A `false` result is still returned — the caller decides
    /// what to do with a weak match.
    pub converged: bool,
}

/// A scan-to-scan matcher.
pub trait ScanMatcher {
    /// Estimate the pose of `query`'s frame expressed in `reference`'s frame,
    /// starting from `initial_guess` (metres/radians).
    ///
    /// # Errors
    ///
    /// [`SlamError::MatchFailed`] when no estimate can be produced at all
    /// (degenerate input, singular normal equations). Low-quality matches are
    /// *not* errors: they come back as `Ok` with `converged = false`.
    fn match_scans(
        &self,
        reference: &ScanCloud,
        query: &ScanCloud,
        initial_guess: &Pose2,
    ) -> Result<MatchResult, SlamError>;
}
