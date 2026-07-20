//! Likelihood field: a rasterized "how likely is a lidar return here" map.
//!
//! Built by rasterizing reference points (or occupied grid cells), running a
//! chamfer distance transform, and mapping distance → `exp(-d²/2σ²)`. CSM
//! scores a candidate pose by summing lookups of the transformed query points.

use crate::convert::floor_to_i64;
use crate::error::SlamError;
use crate::pose::Point2;

/// Chamfer weights (in cell units) for the two-pass distance transform:
/// orthogonal step 1, diagonal step √2. Good to a few percent, which is far
/// below the smoothing σ.
const DIAG: f32 = std::f32::consts::SQRT_2;

/// A blurred occupancy-likelihood raster over a bounded region.
#[derive(Debug, Clone)]
pub(crate) struct LikelihoodField {
    resolution_m: f64,
    origin: Point2,
    width: usize,
    height: usize,
    /// Row-major likelihood values in `0..=1`.
    values: Vec<f32>,
}

impl LikelihoodField {
    /// Build a field from reference points.
    ///
    /// The raster covers the bounding box of the finite points expanded by
    /// `margin_m` on every side, at `resolution_m` per cell, smoothed with
    /// Gaussian σ = `sigma_m`.
    ///
    /// # Errors
    ///
    /// [`SlamError::MatchFailed`] if no finite points remain;
    /// [`SlamError::GridTooLarge`] if the raster would exceed `max_cells`.
    pub fn from_points(
        points: &[Point2],
        resolution_m: f64,
        sigma_m: f64,
        margin_m: f64,
        max_cells: usize,
    ) -> Result<Self, SlamError> {
        let mut min_x = f64::INFINITY;
        let mut min_y = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut max_y = f64::NEG_INFINITY;
        for p in points {
            if p.x.is_finite() && p.y.is_finite() {
                min_x = min_x.min(p.x);
                min_y = min_y.min(p.y);
                max_x = max_x.max(p.x);
                max_y = max_y.max(p.y);
            }
        }
        if !(min_x.is_finite() && min_y.is_finite()) {
            return Err(SlamError::MatchFailed {
                reason: "likelihood field: no finite reference points".to_owned(),
            });
        }
        let origin = Point2::new(min_x - margin_m, min_y - margin_m);
        let span_x = (max_x + margin_m) - origin.x;
        let span_y = (max_y + margin_m) - origin.y;
        let width = cells_for_span(span_x, resolution_m);
        let height = cells_for_span(span_y, resolution_m);
        let cells = width
            .checked_mul(height)
            .filter(|&c| c <= max_cells)
            .ok_or(SlamError::GridTooLarge { cells: usize::MAX, limit: max_cells })?;

        // Distance transform buffer, metres. Occupied cells seed at 0.
        let mut dist = vec![f32::INFINITY; cells];
        for p in points {
            let Some((cx, cy)) = point_to_cell(*p, origin, resolution_m) else { continue };
            if let Some((cx, cy)) = in_bounds(cx, cy, width, height) {
                if let Some(d) = dist.get_mut(cy * width + cx) {
                    *d = 0.0;
                }
            }
        }
        chamfer(&mut dist, width, height);

        // Distance (cells) → likelihood.
        #[allow(clippy::cast_possible_truncation)]
        // Lossy f64→f32 is fine: likelihood precision is far below f32 eps.
        let (res_f32, sigma_f32) = (resolution_m as f32, sigma_m as f32);
        let inv_two_sigma2 = 1.0 / (2.0 * sigma_f32 * sigma_f32);
        let values = dist
            .iter()
            .map(|&d_cells| {
                let d = d_cells * res_f32;
                (-d * d * inv_two_sigma2).exp()
            })
            .collect();

        Ok(Self { resolution_m, origin, width, height, values })
    }

    /// Nearest-cell likelihood at world point `p`; 0 outside the raster.
    #[inline]
    pub fn lookup(&self, p: Point2) -> f32 {
        let Some((cx, cy)) = point_to_cell(p, self.origin, self.resolution_m) else {
            return 0.0;
        };
        match in_bounds(cx, cy, self.width, self.height) {
            Some((cx, cy)) => self.values.get(cy * self.width + cx).copied().unwrap_or(0.0),
            None => 0.0,
        }
    }

    /// Bilinearly interpolated likelihood at world point `p`; 0 outside.
    /// Used in the fine refinement stage so scores vary smoothly below the
    /// raster resolution.
    pub fn lookup_bilinear(&self, p: Point2) -> f32 {
        // Sample positions are cell centres: shift by half a cell.
        let gx = (p.x - self.origin.x) / self.resolution_m - 0.5;
        let gy = (p.y - self.origin.y) / self.resolution_m - 0.5;
        let (Some(x0), Some(y0)) = (floor_to_i64(gx), floor_to_i64(gy)) else { return 0.0 };
        #[allow(clippy::cast_possible_truncation)]
        // Exact: gx - floor(gx) ∈ [0, 1).
        let (fx, fy) = ((gx - gx.floor()) as f32, (gy - gy.floor()) as f32);
        let sample = |cx: i64, cy: i64| -> f32 {
            match in_bounds(cx, cy, self.width, self.height) {
                Some((cx, cy)) => self.values.get(cy * self.width + cx).copied().unwrap_or(0.0),
                None => 0.0,
            }
        };
        let v00 = sample(x0, y0);
        let v10 = sample(x0 + 1, y0);
        let v01 = sample(x0, y0 + 1);
        let v11 = sample(x0 + 1, y0 + 1);
        let top = v00 + (v10 - v00) * fx;
        let bottom = v01 + (v11 - v01) * fx;
        top + (bottom - top) * fy
    }

    /// Coarse field: block-max pooling by `factor`. Every coarse cell holds
    /// the maximum of its block, so coarse scores are optimistic (an upper
    /// bound at block-aligned offsets) — candidates are never pruned away by
    /// pooling, only over-approximated.
    pub fn max_pool(&self, factor: usize) -> Self {
        let factor = factor.max(1);
        let width = self.width.div_ceil(factor);
        let height = self.height.div_ceil(factor);
        let mut values = vec![0.0_f32; width * height];
        for y in 0..self.height {
            let row = y / factor;
            for x in 0..self.width {
                let src = self.values.get(y * self.width + x).copied().unwrap_or(0.0);
                if let Some(dst) = values.get_mut(row * width + x / factor) {
                    if src > *dst {
                        *dst = src;
                    }
                }
            }
        }
        Self {
            resolution_m: self.resolution_m * as_f64(factor),
            origin: self.origin,
            width,
            height,
            values,
        }
    }
}

/// Two-pass chamfer distance transform, distances in cell units.
fn chamfer(dist: &mut [f32], width: usize, height: usize) {
    // Forward pass: left/up/diagonal neighbours.
    for y in 0..height {
        for x in 0..width {
            let Some(&(mut d)) = dist.get(y * width + x) else { continue };
            let mut relax = |idx: Option<usize>, step: f32| {
                if let Some(&n) = idx.and_then(|i| dist.get(i)) {
                    if n + step < d {
                        d = n + step;
                    }
                }
            };
            relax((x > 0).then(|| y * width + x - 1), 1.0);
            relax((y > 0).then(|| (y - 1) * width + x), 1.0);
            relax((x > 0 && y > 0).then(|| (y - 1) * width + x - 1), DIAG);
            relax((x + 1 < width && y > 0).then(|| (y - 1) * width + x + 1), DIAG);
            if let Some(slot) = dist.get_mut(y * width + x) {
                *slot = d;
            }
        }
    }
    // Backward pass: right/down/diagonal neighbours.
    for y in (0..height).rev() {
        for x in (0..width).rev() {
            let Some(&(mut d)) = dist.get(y * width + x) else { continue };
            let mut relax = |idx: Option<usize>, step: f32| {
                if let Some(&n) = idx.and_then(|i| dist.get(i)) {
                    if n + step < d {
                        d = n + step;
                    }
                }
            };
            relax((x + 1 < width).then(|| y * width + x + 1), 1.0);
            relax((y + 1 < height).then(|| (y + 1) * width + x), 1.0);
            relax((x + 1 < width && y + 1 < height).then(|| (y + 1) * width + x + 1), DIAG);
            relax((x > 0 && y + 1 < height).then(|| (y + 1) * width + x - 1), DIAG);
            if let Some(slot) = dist.get_mut(y * width + x) {
                *slot = d;
            }
        }
    }
}

/// Cells needed to cover `span` metres (at least 1).
fn cells_for_span(span: f64, resolution: f64) -> usize {
    floor_to_i64((span / resolution).ceil())
        .and_then(|c| usize::try_from(c).ok())
        .map_or(1, |c| c.max(1))
}

fn point_to_cell(p: Point2, origin: Point2, resolution: f64) -> Option<(i64, i64)> {
    Some((floor_to_i64((p.x - origin.x) / resolution)?, floor_to_i64((p.y - origin.y) / resolution)?))
}

fn in_bounds(cx: i64, cy: i64, width: usize, height: usize) -> Option<(usize, usize)> {
    let cx = usize::try_from(cx).ok()?;
    let cy = usize::try_from(cy).ok()?;
    (cx < width && cy < height).then_some((cx, cy))
}

/// Lossless usize→f64 for small values (pool factors, cell counts per side).
fn as_f64(v: usize) -> f64 {
    u32::try_from(v).map_or(f64::MAX, f64::from)
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;

    #[test]
    fn peak_at_reference_point_and_decays() {
        let points = [Point2::new(1.0, 1.0)];
        let field = LikelihoodField::from_points(&points, 0.05, 0.1, 1.0, 1_000_000).unwrap();
        let at_peak = field.lookup(Point2::new(1.0, 1.0));
        assert!(at_peak > 0.99, "likelihood at the point should be ~1, got {at_peak}");
        let near = field.lookup(Point2::new(1.1, 1.0));
        let far = field.lookup(Point2::new(1.5, 1.0));
        assert!(at_peak > near && near > far, "{at_peak} > {near} > {far} expected");
        assert!(far < 0.01);
        // Outside the raster entirely.
        assert_relative_eq!(field.lookup(Point2::new(50.0, 50.0)), 0.0);
    }

    #[test]
    fn bilinear_is_smooth_between_cells() {
        let points = [Point2::new(0.0, 0.0)];
        let field = LikelihoodField::from_points(&points, 0.1, 0.2, 1.0, 1_000_000).unwrap();
        // The point lands in the cell spanning [0.0, 0.1)², whose centre is
        // (0.05, 0.05) — the interpolated peak. Walk away from it.
        let a = field.lookup_bilinear(Point2::new(0.05, 0.05));
        let b = field.lookup_bilinear(Point2::new(0.10, 0.05));
        let c = field.lookup_bilinear(Point2::new(0.15, 0.05));
        assert!(a >= b && b >= c, "monotone decay expected: {a} {b} {c}");
        assert!(a > c, "interpolation should distinguish sub-cell positions");
    }

    #[test]
    fn max_pool_is_optimistic() {
        let points = [Point2::new(0.3, 0.3)];
        let field = LikelihoodField::from_points(&points, 0.05, 0.05, 0.5, 1_000_000).unwrap();
        let coarse = field.max_pool(4);
        // The coarse cell containing the peak must not underestimate it.
        assert!(coarse.lookup(Point2::new(0.3, 0.3)) >= field.lookup(Point2::new(0.3, 0.3)));
    }

    #[test]
    fn rejects_degenerate_input() {
        assert!(matches!(
            LikelihoodField::from_points(&[], 0.05, 0.1, 1.0, 1_000_000),
            Err(SlamError::MatchFailed { .. })
        ));
        let huge = [Point2::new(0.0, 0.0), Point2::new(1e6, 1e6)];
        assert!(matches!(
            LikelihoodField::from_points(&huge, 0.05, 0.1, 1.0, 1_000_000),
            Err(SlamError::GridTooLarge { .. })
        ));
    }
}
