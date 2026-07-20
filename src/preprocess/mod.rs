//! Scan preprocessing: range gating, outlier rejection, voxel subsampling.
//!
//! The pipeline (in this fixed order) is:
//!
//! 1. **Range gate** — drop returns closer than [`PreprocessConfig::min_range_m`]
//!    (sensor-cowl echoes) or farther than [`PreprocessConfig::max_range_m`].
//!    Non-finite points are dropped here too.
//! 2. **Outlier rejection** — drop isolated points with fewer than
//!    [`PreprocessConfig::outlier_min_neighbors`] neighbours within
//!    [`PreprocessConfig::outlier_radius_m`]. Runs *before* subsampling so
//!    neighbour counts reflect true point density.
//! 3. **Voxel subsample** — collapse all points in each square cell of side
//!    [`PreprocessConfig::voxel_size_m`] to their centroid. Output order is
//!    deterministic (first-seen cell order).
//!
//! [`Preprocessor`] owns reusable scratch buffers: the scan pipeline runs at
//! sensor rate, so after warm-up `process` performs no allocation.

use std::collections::HashMap;
use std::collections::hash_map::Entry;

use crate::convert::floor_to_i64;
use crate::error::SlamError;
use crate::pose::Point2;
use crate::scan::ScanCloud;

/// Tunables for scan preprocessing. All distances in metres.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct PreprocessConfig {
    /// Minimum accepted range, metres (inclusive). Returns closer than this are
    /// typically echoes off the sensor's own housing. Default `0.15`.
    pub min_range_m: f64,
    /// Maximum accepted range, metres (inclusive). Default `12.0`, the SLAMTEC
    /// C1's rated maximum.
    pub max_range_m: f64,
    /// Voxel (grid-cell) side length for subsampling, metres. All points in a
    /// cell are replaced by their centroid. Default `0.05`, matching the
    /// default occupancy-grid resolution.
    pub voxel_size_m: f64,
    /// Neighbourhood radius for outlier rejection, metres. Default `0.10`.
    pub outlier_radius_m: f64,
    /// Minimum number of neighbours (excluding the point itself) within
    /// [`Self::outlier_radius_m`] for a point to be kept. `0` disables
    /// outlier rejection. Default `2`.
    pub outlier_min_neighbors: usize,
    /// Maximum accepted input scan size, points. A denial-of-service guard:
    /// larger scans are rejected with [`SlamError::ScanTooLarge`] rather than
    /// processed. Default `8192` (the C1 delivers ~450 points per revolution).
    pub max_input_points: usize,
}

impl Default for PreprocessConfig {
    fn default() -> Self {
        Self {
            min_range_m: 0.15,
            max_range_m: 12.0,
            voxel_size_m: 0.05,
            outlier_radius_m: 0.10,
            outlier_min_neighbors: 2,
            max_input_points: 8192,
        }
    }
}

impl PreprocessConfig {
    /// Validate the configuration.
    ///
    /// # Errors
    ///
    /// Returns [`SlamError::InvalidConfig`] if any field is non-finite, a
    /// range or size is non-positive where it must be positive, or
    /// `max_range_m <= min_range_m`.
    pub fn validate(&self) -> Result<(), SlamError> {
        let finite = |field: &'static str, v: f64| -> Result<(), SlamError> {
            if v.is_finite() {
                Ok(())
            } else {
                Err(SlamError::InvalidConfig { field, reason: format!("must be finite, got {v}") })
            }
        };
        finite("min_range_m", self.min_range_m)?;
        finite("max_range_m", self.max_range_m)?;
        finite("voxel_size_m", self.voxel_size_m)?;
        finite("outlier_radius_m", self.outlier_radius_m)?;
        if self.min_range_m < 0.0 {
            return Err(SlamError::InvalidConfig {
                field: "min_range_m",
                reason: format!("must be >= 0, got {}", self.min_range_m),
            });
        }
        if self.max_range_m <= self.min_range_m {
            return Err(SlamError::InvalidConfig {
                field: "max_range_m",
                reason: format!(
                    "must exceed min_range_m ({}), got {}",
                    self.min_range_m, self.max_range_m
                ),
            });
        }
        if self.voxel_size_m <= 0.0 {
            return Err(SlamError::InvalidConfig {
                field: "voxel_size_m",
                reason: format!("must be > 0, got {}", self.voxel_size_m),
            });
        }
        if self.outlier_radius_m <= 0.0 {
            return Err(SlamError::InvalidConfig {
                field: "outlier_radius_m",
                reason: format!("must be > 0, got {}", self.outlier_radius_m),
            });
        }
        if self.max_input_points == 0 {
            return Err(SlamError::InvalidConfig {
                field: "max_input_points",
                reason: "must be > 0".to_owned(),
            });
        }
        Ok(())
    }
}

/// Scan preprocessor with reusable scratch buffers.
///
/// Create once, call [`Preprocessor::process`] per scan. Scratch buffers are
/// cleared and reused between calls, so steady-state processing allocates
/// nothing.
#[derive(Debug)]
pub struct Preprocessor {
    config: PreprocessConfig,
    /// Points that survived the range gate.
    gated: Vec<Point2>,
    /// Points that survived outlier rejection.
    kept: Vec<Point2>,
    /// Voxel key → slot in `accum`.
    cell_index: HashMap<(i64, i64), usize>,
    /// Per-voxel accumulator `(sum_x, sum_y, count)`, in first-seen order so
    /// output is deterministic (`HashMap` iteration order is never observed).
    accum: Vec<(f64, f64, u32)>,
}

impl Preprocessor {
    /// Create a preprocessor with the given configuration.
    ///
    /// # Errors
    ///
    /// Returns [`SlamError::InvalidConfig`] if the configuration fails
    /// [`PreprocessConfig::validate`].
    pub fn new(config: PreprocessConfig) -> Result<Self, SlamError> {
        config.validate()?;
        Ok(Self {
            config,
            gated: Vec::new(),
            kept: Vec::new(),
            cell_index: HashMap::new(),
            accum: Vec::new(),
        })
    }

    /// The active configuration.
    #[must_use]
    pub fn config(&self) -> &PreprocessConfig {
        &self.config
    }

    /// Run the full pipeline on `input`, writing the filtered scan into
    /// `output` (its point buffer is cleared and reused; the timestamp is
    /// copied over). Distances in metres throughout.
    ///
    /// # Errors
    ///
    /// Returns [`SlamError::ScanTooLarge`] if `input` has more than
    /// [`PreprocessConfig::max_input_points`] points. No other failure mode:
    /// degenerate points (non-finite, out of range) are silently dropped.
    pub fn process(&mut self, input: &ScanCloud, output: &mut ScanCloud) -> Result<(), SlamError> {
        if input.points.len() > self.config.max_input_points {
            return Err(SlamError::ScanTooLarge {
                actual: input.points.len(),
                limit: self.config.max_input_points,
            });
        }

        self.range_gate(&input.points);
        self.reject_outliers();
        self.voxel_subsample(&mut output.points);
        output.timestamp_ns = input.timestamp_ns;
        Ok(())
    }

    /// Stage 1: keep points with `min_range_m <= ||p|| <= max_range_m`
    /// (inclusive both ends). NaN/infinite coordinates fail the comparison and
    /// are dropped.
    fn range_gate(&mut self, points: &[Point2]) {
        let min2 = self.config.min_range_m * self.config.min_range_m;
        let max2 = self.config.max_range_m * self.config.max_range_m;
        self.gated.clear();
        self.gated.extend(points.iter().copied().filter(|p| {
            let r2 = p.x * p.x + p.y * p.y;
            r2 >= min2 && r2 <= max2
        }));
    }

    /// Stage 2: keep points with at least `outlier_min_neighbors` other points
    /// within `outlier_radius_m`. Brute force O(n²) on squared distances —
    /// at lidar scan sizes (~450 points) this is microseconds and cheaper than
    /// building a k-d tree per scan.
    fn reject_outliers(&mut self) {
        self.kept.clear();
        let min_neighbors = self.config.outlier_min_neighbors;
        if min_neighbors == 0 {
            self.kept.extend_from_slice(&self.gated);
            return;
        }
        let r2 = self.config.outlier_radius_m * self.config.outlier_radius_m;
        for (i, p) in self.gated.iter().enumerate() {
            let mut neighbors = 0usize;
            for (j, q) in self.gated.iter().enumerate() {
                if i == j {
                    continue;
                }
                let dx = p.x - q.x;
                let dy = p.y - q.y;
                if dx * dx + dy * dy <= r2 {
                    neighbors += 1;
                    if neighbors >= min_neighbors {
                        break;
                    }
                }
            }
            if neighbors >= min_neighbors {
                self.kept.push(*p);
            }
        }
    }

    /// Stage 3: collapse each voxel's points to their centroid, preserving
    /// first-seen cell order for deterministic output.
    fn voxel_subsample(&mut self, out: &mut Vec<Point2>) {
        self.cell_index.clear();
        self.accum.clear();
        let inv_voxel = 1.0 / self.config.voxel_size_m;
        for p in &self.kept {
            // Gating bounds coordinates well inside floor_to_i64's range;
            // `None` is unreachable in practice but drops the point safely.
            let (Some(cx), Some(cy)) = (floor_to_i64(p.x * inv_voxel), floor_to_i64(p.y * inv_voxel))
            else {
                continue;
            };
            match self.cell_index.entry((cx, cy)) {
                Entry::Occupied(slot) => {
                    if let Some((sx, sy, n)) = self.accum.get_mut(*slot.get()) {
                        *sx += p.x;
                        *sy += p.y;
                        *n += 1;
                    }
                }
                Entry::Vacant(slot) => {
                    slot.insert(self.accum.len());
                    self.accum.push((p.x, p.y, 1));
                }
            }
        }
        out.clear();
        out.extend(
            self.accum
                .iter()
                .map(|&(sx, sy, n)| Point2::new(sx / f64::from(n), sy / f64::from(n))),
        );
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;

    /// Config that isolates the range gate: no outlier rejection, voxels so
    /// small every point keeps its own cell.
    fn gate_only() -> PreprocessConfig {
        PreprocessConfig {
            outlier_min_neighbors: 0,
            voxel_size_m: 1e-4,
            ..PreprocessConfig::default()
        }
    }

    fn run(config: PreprocessConfig, points: Vec<Point2>) -> ScanCloud {
        let mut pre = Preprocessor::new(config).unwrap();
        let input = ScanCloud::new(points, 7);
        let mut output = ScanCloud::default();
        pre.process(&input, &mut output).unwrap();
        output
    }

    #[test]
    fn range_gate_is_inclusive_at_both_bounds() {
        let cfg = gate_only();
        let out = run(
            cfg,
            vec![
                Point2::new(0.15, 0.0),  // exactly min — kept
                Point2::new(0.149, 0.0), // just inside min — dropped
                Point2::new(12.0, 0.0),  // exactly max — kept
                Point2::new(12.01, 0.0), // just past max — dropped
                Point2::new(1.0, 1.0),   // ordinary — kept
            ],
        );
        assert_eq!(out.len(), 3);
        assert_eq!(out.timestamp_ns, 7);
    }

    #[test]
    fn range_gate_drops_non_finite() {
        let out = run(
            gate_only(),
            vec![
                Point2::new(f64::NAN, 1.0),
                Point2::new(f64::INFINITY, 0.0),
                Point2::new(1.0, 1.0),
            ],
        );
        assert_eq!(out.len(), 1);
    }

    #[test]
    fn isolated_point_dropped_cluster_kept() {
        let cfg = PreprocessConfig {
            outlier_min_neighbors: 2,
            outlier_radius_m: 0.10,
            voxel_size_m: 1e-4,
            ..PreprocessConfig::default()
        };
        // A tight 3-point cluster (mutual neighbours) and one loner 1 m away.
        let out = run(
            cfg,
            vec![
                Point2::new(1.00, 1.00),
                Point2::new(1.05, 1.00),
                Point2::new(1.00, 1.05),
                Point2::new(3.0, 3.0),
            ],
        );
        assert_eq!(out.len(), 3);
        assert!(out.points.iter().all(|p| p.x < 2.0));
    }

    #[test]
    fn voxel_collapses_to_centroid() {
        let cfg = PreprocessConfig {
            outlier_min_neighbors: 0,
            voxel_size_m: 0.1,
            ..PreprocessConfig::default()
        };
        // Both points fall in voxel cell (12, 12) at 0.1 m: [1.2, 1.3).
        let out = run(cfg, vec![Point2::new(1.22, 1.24), Point2::new(1.28, 1.26)]);
        assert_eq!(out.len(), 1);
        let p = out.points.first().unwrap();
        assert_relative_eq!(p.x, 1.25, epsilon = 1e-12);
        assert_relative_eq!(p.y, 1.25, epsilon = 1e-12);
    }

    #[test]
    fn output_order_is_deterministic_and_reuse_is_clean() {
        let points: Vec<Point2> = (0..300)
            .map(|i| {
                let a = f64::from(i) * 0.021;
                Point2::new(3.0 * a.cos(), 3.0 * a.sin())
            })
            .collect();
        let mut pre = Preprocessor::new(PreprocessConfig::default()).unwrap();
        let input = ScanCloud::new(points, 0);
        let mut first = ScanCloud::default();
        let mut second = ScanCloud::default();
        pre.process(&input, &mut first).unwrap();
        // Same preprocessor, reused scratch buffers — result must be identical.
        pre.process(&input, &mut second).unwrap();
        assert_eq!(first.len(), second.len());
        for (a, b) in first.points.iter().zip(&second.points) {
            assert_relative_eq!(a.x, b.x);
            assert_relative_eq!(a.y, b.y);
        }
    }

    #[test]
    fn oversized_scan_rejected() {
        let cfg = PreprocessConfig { max_input_points: 4, ..PreprocessConfig::default() };
        let mut pre = Preprocessor::new(cfg).unwrap();
        let input = ScanCloud::new(vec![Point2::new(1.0, 0.0); 5], 0);
        let mut output = ScanCloud::default();
        let err = pre.process(&input, &mut output).unwrap_err();
        assert!(matches!(err, SlamError::ScanTooLarge { actual: 5, limit: 4 }));
    }

    #[test]
    fn invalid_configs_rejected() {
        let bad_range =
            PreprocessConfig { max_range_m: 0.1, min_range_m: 0.2, ..PreprocessConfig::default() };
        assert!(matches!(bad_range.validate(), Err(SlamError::InvalidConfig { .. })));

        let bad_voxel = PreprocessConfig { voxel_size_m: 0.0, ..PreprocessConfig::default() };
        assert!(matches!(bad_voxel.validate(), Err(SlamError::InvalidConfig { .. })));

        let bad_nan = PreprocessConfig { min_range_m: f64::NAN, ..PreprocessConfig::default() };
        assert!(matches!(bad_nan.validate(), Err(SlamError::InvalidConfig { .. })));
    }
}
