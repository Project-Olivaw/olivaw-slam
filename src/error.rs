//! Crate-wide error type.

/// Errors returned by `olivaw-slam` APIs.
///
/// The enum is `#[non_exhaustive]`: later phases (scan matching, pose graph,
/// serialization) add variants without a breaking change.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum SlamError {
    /// A configuration field failed validation.
    #[error("invalid config field `{field}`: {reason}")]
    InvalidConfig {
        /// Name of the offending config field.
        field: &'static str,
        /// Why the value was rejected.
        reason: String,
    },

    /// An input scan exceeded the configured point-count limit.
    ///
    /// This is a denial-of-service guard: a malformed input returns an error
    /// instead of driving unbounded allocation.
    #[error("scan has {actual} points, exceeding limit {limit}")]
    ScanTooLarge {
        /// Number of points in the rejected scan.
        actual: usize,
        /// Configured maximum.
        limit: usize,
    },

    /// A grid configuration would allocate more cells than allowed.
    ///
    /// Like [`SlamError::ScanTooLarge`], this bounds allocation on bad input.
    #[error("grid of {cells} cells exceeds limit {limit}")]
    GridTooLarge {
        /// Requested total cell count (`width × height`).
        cells: usize,
        /// Configured maximum.
        limit: usize,
    },

    /// Scan matching failed to produce a usable estimate.
    #[error("scan match failed: {reason}")]
    MatchFailed {
        /// Why the match was rejected (too few points, no convergence, …).
        reason: String,
    },

    /// Pose-graph optimization failed.
    #[error("pose graph optimization failed: {reason}")]
    OptimizationFailed {
        /// The underlying solver failure.
        reason: String,
    },

    /// Saving or loading SLAM state failed (corrupt or incompatible data).
    #[error("serialization failed: {reason}")]
    Serialization {
        /// What went wrong while encoding or decoding.
        reason: String,
    },

    /// An underlying I/O operation failed (map export, serialization).
    #[error("i/o error")]
    Io(#[from] std::io::Error),
}
