//! Log-odds occupancy grid with Bresenham ray casting.
//!
//! The grid is fixed-size with a configurable origin and extent (growth is the
//! job of submaps, later). Cells store log-odds occupancy in `f32` — memory-
//! bound, precision is ample. All world coordinates are metres.

mod bresenham;
mod cellmath;
#[cfg(feature = "viz")]
mod rerun_viz;

use std::fs::File;
use std::io::{self, BufWriter, Write};
use std::path::Path;

use crate::error::SlamError;
use crate::pose::{Point2, Pose2};
use crate::scan::ScanCloud;
use cellmath::{cell_in_bounds, clip_segment_to_aabb, world_to_cell};

/// Occupancy probability above which a cell is exported as *occupied*
/// (ROS `map_server` convention).
const PGM_OCCUPIED_THRESHOLD: f32 = 0.65;
/// Occupancy probability below which a cell is exported as *free*
/// (ROS `map_server` convention).
const PGM_FREE_THRESHOLD: f32 = 0.196;

/// Tunables for [`OccupancyGrid`]. Distances in metres; `l_*` fields are
/// log-odds increments/bounds.
///
/// Log-odds defaults follow the inverse sensor model in Thrun, Burgard & Fox,
/// *Probabilistic Robotics* ch. 9: `p_hit = 0.7`, `p_miss = 0.4`, prior `0.5`
/// (log-odds `0`). The asymmetric clamps mean a wall saturates after ~5 hits
/// and fully clears after ~9 misses, keeping cells responsive to change.
/// (Cartographer uses the gentler `p_hit 0.55 / p_miss 0.49` — an alternative
/// worth trying once scan matching exists.)
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct GridConfig {
    /// Cell side length, metres per cell. Default `0.05`.
    pub resolution_m: f64,
    /// Grid width, cells. Default `600` (30 m at default resolution).
    pub width_cells: usize,
    /// Grid height, cells. Default `600`.
    pub height_cells: usize,
    /// World coordinates (metres) of the outer corner of cell `(0, 0)`.
    /// Default `(-15, -15)`, centring the default grid on the sensor.
    pub origin: Point2,
    /// Log-odds added to a cell on a *hit* (beam endpoint). Must be `> 0`.
    /// Default `0.85` (= ln(0.7/0.3), `p_hit = 0.7`).
    pub l_hit: f32,
    /// Log-odds added to a cell on a *miss* (beam passes through). Must be
    /// `< 0`. Default `-0.4` (≈ ln(0.4/0.6), `p_miss = 0.4`).
    pub l_miss: f32,
    /// Lower clamp on cell log-odds. Must be `< 0`. Default `-2.0`
    /// (p ≈ 0.12) so free cells stay responsive to change.
    pub l_min: f32,
    /// Upper clamp on cell log-odds. Must be `> 0`. Default `3.5` (p ≈ 0.97).
    pub l_max: f32,
    /// Maximum total cell count (`width × height`) accepted at construction —
    /// a denial-of-service guard against absurd allocations. Default
    /// `16_777_216` (64 MiB of `f32`).
    pub max_cells: usize,
}

impl Default for GridConfig {
    fn default() -> Self {
        Self {
            resolution_m: 0.05,
            width_cells: 600,
            height_cells: 600,
            origin: Point2::new(-15.0, -15.0),
            l_hit: 0.85,
            l_miss: -0.4,
            l_min: -2.0,
            l_max: 3.5,
            max_cells: 16_777_216,
        }
    }
}

impl GridConfig {
    /// Validate the configuration.
    ///
    /// # Errors
    ///
    /// [`SlamError::GridTooLarge`] if `width_cells × height_cells` overflows
    /// or exceeds [`Self::max_cells`]; [`SlamError::InvalidConfig`] for any
    /// other invalid field (non-finite values, non-positive sizes, log-odds
    /// increments/clamps with the wrong sign).
    pub fn validate(&self) -> Result<(), SlamError> {
        if !(self.resolution_m.is_finite() && self.resolution_m > 0.0) {
            return Err(SlamError::InvalidConfig {
                field: "resolution_m",
                reason: format!("must be finite and > 0, got {}", self.resolution_m),
            });
        }
        if !(self.origin.x.is_finite() && self.origin.y.is_finite()) {
            return Err(SlamError::InvalidConfig {
                field: "origin",
                reason: format!("must be finite, got ({}, {})", self.origin.x, self.origin.y),
            });
        }
        if self.width_cells == 0 || self.height_cells == 0 {
            return Err(SlamError::InvalidConfig {
                field: "width_cells/height_cells",
                reason: "must be > 0".to_owned(),
            });
        }
        // Each dimension must fit in u32 so cell counts convert to f64/rerun
        // resolutions losslessly. (4 billion cells per side is far past any
        // sane 2D grid anyway.)
        if u32::try_from(self.width_cells).is_err() || u32::try_from(self.height_cells).is_err() {
            return Err(SlamError::InvalidConfig {
                field: "width_cells/height_cells",
                reason: "must fit in u32".to_owned(),
            });
        }
        let cells = self
            .width_cells
            .checked_mul(self.height_cells)
            .ok_or(SlamError::GridTooLarge { cells: usize::MAX, limit: self.max_cells })?;
        if cells > self.max_cells {
            return Err(SlamError::GridTooLarge { cells, limit: self.max_cells });
        }
        let signed = |field: &'static str, v: f32, positive: bool| -> Result<(), SlamError> {
            if v.is_finite() && ((positive && v > 0.0) || (!positive && v < 0.0)) {
                Ok(())
            } else {
                Err(SlamError::InvalidConfig {
                    field,
                    reason: format!(
                        "must be finite and {} 0, got {v}",
                        if positive { ">" } else { "<" }
                    ),
                })
            }
        };
        signed("l_hit", self.l_hit, true)?;
        signed("l_miss", self.l_miss, false)?;
        signed("l_min", self.l_min, false)?;
        signed("l_max", self.l_max, true)?;
        Ok(())
    }
}

/// Log-odds occupancy grid.
///
/// Cells start at the prior (log-odds `0`, probability `0.5` — *unknown*).
/// [`OccupancyGrid::integrate_scan`] ray-casts each beam from the sensor
/// origin: traversed cells get a miss update, the endpoint a hit update, all
/// clamped to `[l_min, l_max]`.
#[derive(Debug, Clone)]
pub struct OccupancyGrid {
    config: GridConfig,
    /// Flat, row-major (`index = y * width + x`); row 0 is at the origin
    /// (minimum y).
    log_odds: Vec<f32>,
    /// World coordinates of the grid's outer corner, pulled fractionally
    /// inside so clipped beam endpoints discretize to in-bounds cells.
    max_corner: Point2,
}

impl OccupancyGrid {
    /// Create a grid with all cells at the prior (unknown, probability 0.5).
    ///
    /// # Errors
    ///
    /// Propagates [`GridConfig::validate`] failures; notably
    /// [`SlamError::GridTooLarge`] if the requested dimensions exceed
    /// [`GridConfig::max_cells`].
    pub fn new(config: GridConfig) -> Result<Self, SlamError> {
        config.validate()?;
        // Validated to fit in u32, so the f64 conversions are lossless.
        let width_m = config.resolution_m * f64::from(u32::try_from(config.width_cells).unwrap_or(0));
        let height_m =
            config.resolution_m * f64::from(u32::try_from(config.height_cells).unwrap_or(0));
        // Shrink the clip box by a sliver of a cell so points clipped to the
        // outer edge floor to the last cell instead of one past it.
        let margin = config.resolution_m * 1e-9;
        let max_corner =
            Point2::new(config.origin.x + width_m - margin, config.origin.y + height_m - margin);
        let cells = config.width_cells * config.height_cells;
        Ok(Self { config, log_odds: vec![0.0; cells], max_corner })
    }

    /// Integrate a scan taken at `pose` (world frame, metres/radians).
    ///
    /// Each beam runs from the sensor position to the scan point transformed
    /// by `pose`. Beams are clipped to the grid: fully-outside beams are
    /// skipped, and a beam whose endpoint lies off-map contributes misses
    /// along its in-bounds portion but no hit (the obstacle itself is
    /// unobserved). Infallible by design — degenerate points are skipped.
    pub fn integrate_scan(&mut self, pose: &Pose2, cloud: &ScanCloud) {
        let sensor = Point2::new(pose.x, pose.y);
        let GridConfig {
            resolution_m,
            width_cells,
            height_cells,
            origin,
            l_hit,
            l_miss,
            l_min,
            l_max,
            ..
        } = self.config;
        let max_corner = self.max_corner;
        let log_odds = &mut self.log_odds;

        let mut apply = |cell: (i64, i64), delta: f32| {
            if let Some((cx, cy)) = cell_in_bounds(cell, width_cells, height_cells) {
                if let Some(l) = log_odds.get_mut(cy * width_cells + cx) {
                    *l = (*l + delta).clamp(l_min, l_max);
                }
            }
        };

        for p in &cloud.points {
            let hit_world = pose.transform_point(*p);
            let Some(seg) = clip_segment_to_aabb(sensor, hit_world, origin, max_corner) else {
                continue;
            };
            let Some(start_cell) = world_to_cell(seg.start, origin, resolution_m) else {
                continue;
            };
            let Some(end_cell) = world_to_cell(seg.end, origin, resolution_m) else {
                continue;
            };
            bresenham::trace(start_cell, end_cell, |c| apply(c, l_miss));
            if seg.end_clipped {
                // The clipped boundary cell is real traversed free space, but
                // the obstacle beyond it was never observed: miss, no hit.
                apply(end_cell, l_miss);
            } else {
                apply(end_cell, l_hit);
            }
        }
    }

    /// Occupancy probability (`0..1`) at world point `p` (metres);
    /// `None` outside the grid. Unknown cells read `0.5`.
    #[must_use]
    pub fn probability_at(&self, p: Point2) -> Option<f32> {
        self.log_odds_at(p).map(prob_from_log_odds)
    }

    /// Raw log-odds at world point `p` (metres); `None` outside the grid.
    #[must_use]
    pub fn log_odds_at(&self, p: Point2) -> Option<f32> {
        let cell = world_to_cell(p, self.config.origin, self.config.resolution_m)?;
        let (cx, cy) = cell_in_bounds(cell, self.config.width_cells, self.config.height_cells)?;
        self.log_odds.get(cy * self.config.width_cells + cx).copied()
    }

    /// Cell side length, metres.
    #[must_use]
    pub fn resolution_m(&self) -> f64 {
        self.config.resolution_m
    }

    /// Grid width, cells.
    #[must_use]
    pub fn width(&self) -> usize {
        self.config.width_cells
    }

    /// Grid height, cells.
    #[must_use]
    pub fn height(&self) -> usize {
        self.config.height_cells
    }

    /// World coordinates (metres) of the outer corner of cell `(0, 0)`.
    #[must_use]
    pub fn origin(&self) -> Point2 {
        self.config.origin
    }

    /// Raw log-odds cells, flat row-major (`index = y * width + x`), row 0 at
    /// the origin (minimum y).
    #[must_use]
    pub fn cells(&self) -> &[f32] {
        &self.log_odds
    }

    /// World-frame centres (metres) of every cell whose occupancy probability
    /// exceeds `threshold` (`0..1`). Used to build likelihood fields for
    /// scan-to-map matching, and handy for visualization.
    #[must_use]
    pub fn occupied_cell_centres(&self, threshold: f32) -> Vec<Point2> {
        let min_log_odds = (threshold.clamp(1e-6, 1.0 - 1e-6) / (1.0 - threshold.clamp(1e-6, 1.0 - 1e-6))).ln();
        let width = self.config.width_cells;
        let res = self.config.resolution_m;
        let origin = self.config.origin;
        self.log_odds
            .iter()
            .enumerate()
            .filter(|&(_, &l)| l > min_log_odds)
            .map(|(i, _)| {
                let (cx, cy) = (i % width, i / width);
                Point2::new(
                    origin.x + (cell_f64(cx) + 0.5) * res,
                    origin.y + (cell_f64(cy) + 0.5) * res,
                )
            })
            .collect()
    }

    /// Export the map in the standard ROS `map_server` file pair:
    /// `<stem>.pgm` (the image, via [`OccupancyGrid::to_pgm`]) plus
    /// `<stem>.yaml` with resolution, origin, and thresholds — loadable by
    /// any `map_server`-compatible consumer without ROS being involved here.
    ///
    /// # Errors
    ///
    /// Any I/O error from writing either file.
    pub fn export_map(&self, stem: &Path) -> io::Result<()> {
        let pgm = stem.with_extension("pgm");
        let yaml = stem.with_extension("yaml");
        self.to_pgm(&pgm)?;
        let image = pgm
            .file_name()
            .map_or_else(|| "map.pgm".to_owned(), |n| n.to_string_lossy().into_owned());
        let contents = format!(
            "image: {image}\n\
             resolution: {}\n\
             origin: [{}, {}, 0.0]\n\
             negate: 0\n\
             occupied_thresh: {PGM_OCCUPIED_THRESHOLD}\n\
             free_thresh: {PGM_FREE_THRESHOLD}\n",
            self.config.resolution_m, self.config.origin.x, self.config.origin.y,
        );
        std::fs::write(yaml, contents)
    }

    /// Export as binary (P5) PGM in the ROS `map_server` convention:
    /// occupied (p > 0.65) = 0 (black), free (p < 0.196) = 254, unknown = 205.
    /// The image is written top-row-first, so the grid's maximum-y row comes
    /// first (map origin bottom-left, image origin top-left).
    ///
    /// # Errors
    ///
    /// Any I/O error from creating or writing `path`.
    pub fn to_pgm(&self, path: &Path) -> io::Result<()> {
        let mut file = BufWriter::new(File::create(path)?);
        write!(file, "P5\n{} {}\n255\n", self.config.width_cells, self.config.height_cells)?;
        let mut row = vec![0_u8; self.config.width_cells];
        for y in (0..self.config.height_cells).rev() {
            let start = y * self.config.width_cells;
            let cells = self.log_odds.get(start..start + self.config.width_cells).unwrap_or(&[]);
            row.clear();
            row.extend(cells.iter().map(|&l| {
                let p = prob_from_log_odds(l);
                if p > PGM_OCCUPIED_THRESHOLD {
                    0
                } else if p < PGM_FREE_THRESHOLD {
                    254
                } else {
                    205
                }
            }));
            file.write_all(&row)?;
        }
        file.flush()
    }
}

/// Log-odds → probability: `p = 1 - 1 / (1 + eˡ)`.
#[inline]
fn prob_from_log_odds(l: f32) -> f32 {
    1.0 - 1.0 / (1.0 + l.exp())
}

/// Lossless cell-index → f64 (grid dimensions are validated to fit in u32).
fn cell_f64(i: usize) -> f64 {
    u32::try_from(i).map_or(f64::MAX, f64::from)
}

#[cfg(test)]
// Direct indexing is fine in tests (out-of-bounds is a test failure, not a
// robot crash) — the library-code rule doesn't apply here.
#[allow(clippy::indexing_slicing)]
mod tests {
    use approx::{assert_abs_diff_eq, assert_relative_eq};

    use super::*;

    /// 20×20 cells at 0.1 m, world [0, 2] × [0, 2].
    fn small_config() -> GridConfig {
        GridConfig {
            resolution_m: 0.1,
            width_cells: 20,
            height_cells: 20,
            origin: Point2::new(0.0, 0.0),
            ..GridConfig::default()
        }
    }

    #[test]
    fn single_beam_marks_ray_and_endpoint() {
        let mut grid = OccupancyGrid::new(small_config()).unwrap();
        // Sensor in cell (0, 0), beam along +x to a hit in cell (10, 0).
        let pose = Pose2::new(0.05, 0.05, 0.0);
        let cloud = ScanCloud::new(vec![Point2::new(0.95, 0.0)], 0);
        grid.integrate_scan(&pose, &cloud);

        // Cells along the ray: one miss each.
        let on_ray = grid.log_odds_at(Point2::new(0.55, 0.05)).unwrap();
        assert_relative_eq!(on_ray, -0.4, epsilon = 1e-6);
        // Endpoint: one hit.
        let endpoint = grid.log_odds_at(Point2::new(1.0, 0.05)).unwrap();
        assert_relative_eq!(endpoint, 0.85, epsilon = 1e-6);
        // Untouched cell: prior (unknown).
        let elsewhere = grid.probability_at(Point2::new(0.55, 0.55)).unwrap();
        assert_abs_diff_eq!(elsewhere, 0.5, epsilon = 1e-6);
        // Probabilities on the right side of 0.5.
        assert!(grid.probability_at(Point2::new(0.55, 0.05)).unwrap() < 0.5);
        assert!(grid.probability_at(Point2::new(1.0, 0.05)).unwrap() > 0.5);
    }

    #[test]
    fn repeated_integration_saturates_at_clamps() {
        let mut grid = OccupancyGrid::new(small_config()).unwrap();
        let pose = Pose2::new(0.05, 0.05, 0.0);
        let cloud = ScanCloud::new(vec![Point2::new(0.95, 0.0)], 0);
        for _ in 0..100 {
            grid.integrate_scan(&pose, &cloud);
        }
        let cfg = small_config();
        let endpoint = grid.log_odds_at(Point2::new(1.0, 0.05)).unwrap();
        assert_relative_eq!(endpoint, cfg.l_max, epsilon = 1e-6);
        let on_ray = grid.log_odds_at(Point2::new(0.55, 0.05)).unwrap();
        assert_relative_eq!(on_ray, cfg.l_min, epsilon = 1e-6);
    }

    #[test]
    fn off_map_endpoint_contributes_misses_only() {
        let mut grid = OccupancyGrid::new(small_config()).unwrap();
        let pose = Pose2::new(0.05, 0.05, 0.0);
        // Beam endpoint at x = 5 m, far beyond the 2 m grid.
        let cloud = ScanCloud::new(vec![Point2::new(5.0, 0.0)], 0);
        grid.integrate_scan(&pose, &cloud);
        // No cell anywhere went occupied.
        assert!(grid.cells().iter().all(|&l| l <= 0.0));
        // The in-bounds portion did get free-space evidence, including the
        // boundary cell the beam exited through.
        assert!(grid.log_odds_at(Point2::new(1.5, 0.05)).unwrap() < 0.0);
        assert!(grid.log_odds_at(Point2::new(1.95, 0.05)).unwrap() < 0.0);
    }

    #[test]
    fn beam_from_outside_the_grid_is_clipped_or_skipped() {
        let mut grid = OccupancyGrid::new(small_config()).unwrap();
        // Sensor 1 m left of the grid, beam crossing it horizontally.
        let pose = Pose2::new(-1.0, 0.55, 0.0);
        let cloud = ScanCloud::new(vec![Point2::new(2.5, 0.0)], 0);
        grid.integrate_scan(&pose, &cloud);
        // Entry-edge cell got a miss; endpoint inside got a hit.
        assert!(grid.log_odds_at(Point2::new(0.05, 0.55)).unwrap() < 0.0);
        assert!(grid.log_odds_at(Point2::new(1.5, 0.55)).unwrap() > 0.0);

        // Fully-outside beam: nothing happens.
        let mut untouched = OccupancyGrid::new(small_config()).unwrap();
        let far_pose = Pose2::new(-5.0, -5.0, 0.0);
        untouched.integrate_scan(&far_pose, &ScanCloud::new(vec![Point2::new(1.0, 0.0)], 0));
        assert!(untouched.cells().iter().all(|&l| l.abs() < f32::EPSILON));
    }

    #[test]
    fn oversized_grid_rejected() {
        let cfg = GridConfig { width_cells: 10_000, height_cells: 10_000, ..GridConfig::default() };
        assert!(matches!(
            OccupancyGrid::new(cfg),
            Err(SlamError::GridTooLarge { cells: 100_000_000, .. })
        ));
    }

    #[test]
    fn invalid_log_odds_config_rejected() {
        let bad = GridConfig { l_miss: 0.4, ..GridConfig::default() };
        assert!(matches!(bad.validate(), Err(SlamError::InvalidConfig { field: "l_miss", .. })));
        let bad = GridConfig { resolution_m: 0.0, ..GridConfig::default() };
        assert!(matches!(bad.validate(), Err(SlamError::InvalidConfig { .. })));
    }

    #[test]
    fn pgm_golden_4x4() {
        let cfg = GridConfig {
            resolution_m: 1.0,
            width_cells: 4,
            height_cells: 4,
            origin: Point2::new(0.0, 0.0),
            ..GridConfig::default()
        };
        let mut grid = OccupancyGrid::new(cfg).unwrap();
        // Beam along the bottom row: free (0,0)..(2,0), occupied (3,0).
        // 4 passes push the misses below the free threshold (p < 0.196).
        let pose = Pose2::new(0.5, 0.5, 0.0);
        let cloud = ScanCloud::new(vec![Point2::new(3.0, 0.0)], 0);
        for _ in 0..4 {
            grid.integrate_scan(&pose, &cloud);
        }

        let path = std::env::temp_dir().join("olivaw_slam_pgm_golden_4x4.pgm");
        grid.to_pgm(&path).unwrap();
        let bytes = std::fs::read(&path).unwrap();
        std::fs::remove_file(&path).ok();

        let header = b"P5\n4 4\n255\n";
        assert_eq!(&bytes[..header.len()], header);
        let pixels = &bytes[header.len()..];
        // Top three image rows (grid rows y=3..1): all unknown.
        assert_eq!(&pixels[..12], &[205; 12]);
        // Bottom image row (grid row y=0): free, free, free, occupied.
        assert_eq!(&pixels[12..], &[254, 254, 254, 0]);
    }
}
