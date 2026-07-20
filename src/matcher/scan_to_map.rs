//! Scan-to-map matching: CSM against the accumulated occupancy grid.
//!
//! Matching each incoming scan against the *map* rather than the previous
//! scan is what kills incremental drift: errors stop compounding because the
//! reference is the global model, not the last (already slightly wrong)
//! estimate.

use crate::error::SlamError;
use crate::grid::OccupancyGrid;
use crate::matcher::csm::{CsmConfig, search};
use crate::matcher::likelihood::LikelihoodField;
use crate::matcher::MatchResult;
use crate::pose::Pose2;
use crate::scan::ScanCloud;

/// Tunables for [`ScanToMapMatcher`].
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct ScanToMapConfig {
    /// The underlying correlative search. Defaults to [`CsmConfig::default`]
    /// with a smaller window (±0.3 m, ±0.25 rad) — incremental motion between
    /// consecutive scans is small.
    pub csm: CsmConfig,
    /// Occupancy probability above which a grid cell counts as an obstacle
    /// for the likelihood field. Default `0.65` (the PGM export convention).
    pub occupied_threshold: f32,
    /// Minimum occupied cells required for a meaningful match. Default `10`.
    pub min_occupied_cells: usize,
}

impl Default for ScanToMapConfig {
    fn default() -> Self {
        Self {
            csm: CsmConfig {
                search_x_m: 0.3,
                search_y_m: 0.3,
                search_theta_rad: 0.25,
                ..CsmConfig::default()
            },
            occupied_threshold: 0.65,
            min_occupied_cells: 10,
        }
    }
}

/// Matches an incoming scan against an [`OccupancyGrid`] using the CSM search.
#[derive(Debug, Clone, Default)]
pub struct ScanToMapMatcher {
    config: ScanToMapConfig,
}

impl ScanToMapMatcher {
    /// Create a matcher with the given configuration.
    #[must_use]
    pub fn new(config: ScanToMapConfig) -> Self {
        Self { config }
    }

    /// The active configuration.
    #[must_use]
    pub fn config(&self) -> &ScanToMapConfig {
        &self.config
    }

    /// Estimate the world pose of `query`'s frame by matching against `grid`,
    /// searching around `initial_guess` (metres/radians).
    ///
    /// # Errors
    ///
    /// [`SlamError::MatchFailed`] if the grid has too few occupied cells or
    /// the query is degenerate; [`SlamError::ScanTooLarge`] /
    /// [`SlamError::GridTooLarge`] on inputs beyond the configured guards.
    pub fn match_scan(
        &self,
        grid: &OccupancyGrid,
        query: &ScanCloud,
        initial_guess: &Pose2,
    ) -> Result<MatchResult, SlamError> {
        let occupied = grid.occupied_cell_centres(self.config.occupied_threshold);
        if occupied.len() < self.config.min_occupied_cells {
            return Err(SlamError::MatchFailed {
                reason: format!(
                    "grid has only {} occupied cells (need {})",
                    occupied.len(),
                    self.config.min_occupied_cells
                ),
            });
        }
        let cfg = &self.config.csm;
        let margin = cfg.search_x_m.max(cfg.search_y_m) + 4.0 * cfg.sigma_m + 0.2;
        let field = LikelihoodField::from_points(
            &occupied,
            cfg.field_resolution_m,
            cfg.sigma_m,
            margin,
            cfg.max_field_cells,
        )?;
        search(&field, &query.points, initial_guess, cfg)
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_abs_diff_eq;

    use super::*;
    use crate::grid::GridConfig;
    use crate::matcher::test_scenes::room_scan;

    #[test]
    fn localizes_scan_against_accumulated_grid() {
        // Build the map from a few identity-pose scans.
        let mut grid = OccupancyGrid::new(GridConfig::default()).unwrap();
        let map_scan = room_scan(Pose2::identity());
        for _ in 0..3 {
            grid.integrate_scan(&Pose2::identity(), &map_scan);
        }

        // A scan taken from a displaced pose must be located in the map.
        let truth = Pose2::new(0.15, -0.1, 0.1);
        let query = room_scan(truth);
        let matcher = ScanToMapMatcher::default();
        let result = matcher.match_scan(&grid, &query, &Pose2::identity()).unwrap();
        assert!(result.converged, "score {}", result.score);
        // Accuracy against a map is bounded by the grid resolution (0.05 m):
        // occupied evidence is quantized to cell centres.
        assert_abs_diff_eq!(result.pose.x, truth.x, epsilon = 0.04);
        assert_abs_diff_eq!(result.pose.y, truth.y, epsilon = 0.04);
        assert_abs_diff_eq!(result.pose.theta, truth.theta, epsilon = 0.02);
    }

    #[test]
    fn refuses_empty_map() {
        let grid = OccupancyGrid::new(GridConfig::default()).unwrap();
        let query = room_scan(Pose2::identity());
        let matcher = ScanToMapMatcher::default();
        assert!(matches!(
            matcher.match_scan(&grid, &query, &Pose2::identity()),
            Err(SlamError::MatchFailed { .. })
        ));
    }
}
