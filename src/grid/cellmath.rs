//! World↔cell coordinate conversion and segment clipping for the grid.
//!
//! All world coordinates are metres (`f64`); cells are integer indices.

use crate::convert::floor_to_i64;
use crate::pose::Point2;

/// Convert a world point (metres) to signed cell coordinates.
///
/// `None` if the point is non-finite or so far from `origin` that the cell
/// index is not exactly representable (see [`floor_to_i64`]).
#[inline]
pub(crate) fn world_to_cell(p: Point2, origin: Point2, resolution_m: f64) -> Option<(i64, i64)> {
    let cx = floor_to_i64((p.x - origin.x) / resolution_m)?;
    let cy = floor_to_i64((p.y - origin.y) / resolution_m)?;
    Some((cx, cy))
}

/// Check signed cell coordinates against grid bounds, returning unsigned
/// indices if inside. The `usize` conversion is lossless (`try_from`).
#[inline]
pub(crate) fn cell_in_bounds(cell: (i64, i64), width: usize, height: usize) -> Option<(usize, usize)> {
    let cx = usize::try_from(cell.0).ok()?;
    let cy = usize::try_from(cell.1).ok()?;
    if cx < width && cy < height { Some((cx, cy)) } else { None }
}

/// A segment clipped to an axis-aligned bounding box.
#[derive(Debug, Clone, Copy)]
pub(crate) struct ClippedSegment {
    /// Clipped start point (world metres).
    pub start: Point2,
    /// Clipped end point (world metres).
    pub end: Point2,
    /// `true` if the original end point lay outside the box (the segment was
    /// shortened) — for a lidar beam this means the obstacle is off-map and no
    /// hit update must be applied.
    pub end_clipped: bool,
}

/// Clip the segment `a → b` to the AABB `[min, max]` (Liang–Barsky).
///
/// Returns `None` if the segment lies entirely outside the box or any input
/// is non-finite.
pub(crate) fn clip_segment_to_aabb(
    a: Point2,
    b: Point2,
    min: Point2,
    max: Point2,
) -> Option<ClippedSegment> {
    if !(a.x.is_finite() && a.y.is_finite() && b.x.is_finite() && b.y.is_finite()) {
        return None;
    }
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    let mut t0 = 0.0_f64;
    let mut t1 = 1.0_f64;

    // Each (p, q) pair encodes one box edge; `clip_edge` tightens [t0, t1].
    for (p, q) in [
        (-dx, a.x - min.x),
        (dx, max.x - a.x),
        (-dy, a.y - min.y),
        (dy, max.y - a.y),
    ] {
        if p.abs() < f64::EPSILON {
            // Segment parallel to this edge: outside the slab → no intersection.
            if q < 0.0 {
                return None;
            }
        } else {
            let r = q / p;
            if p < 0.0 {
                if r > t1 {
                    return None;
                }
                if r > t0 {
                    t0 = r;
                }
            } else {
                if r < t0 {
                    return None;
                }
                if r < t1 {
                    t1 = r;
                }
            }
        }
    }

    Some(ClippedSegment {
        start: Point2::new(a.x + t0 * dx, a.y + t0 * dy),
        end: Point2::new(a.x + t1 * dx, a.y + t1 * dy),
        end_clipped: t1 < 1.0,
    })
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;

    const ORIGIN: Point2 = Point2::new(-1.0, -1.0);

    #[test]
    fn world_to_cell_round_trips() {
        // Cell (0,0) spans [-1.0, -0.9) at 0.1 m resolution.
        assert_eq!(world_to_cell(Point2::new(-1.0, -1.0), ORIGIN, 0.1), Some((0, 0)));
        assert_eq!(world_to_cell(Point2::new(-0.95, -0.85), ORIGIN, 0.1), Some((0, 1)));
        assert_eq!(world_to_cell(Point2::new(0.0, 0.0), ORIGIN, 0.1), Some((10, 10)));
        // Negative cell indices for points left of the origin.
        assert_eq!(world_to_cell(Point2::new(-1.05, -1.0), ORIGIN, 0.1), Some((-1, 0)));
        assert_eq!(world_to_cell(Point2::new(f64::NAN, 0.0), ORIGIN, 0.1), None);
    }

    #[test]
    fn bounds_check() {
        assert_eq!(cell_in_bounds((0, 0), 10, 10), Some((0, 0)));
        assert_eq!(cell_in_bounds((9, 9), 10, 10), Some((9, 9)));
        assert_eq!(cell_in_bounds((10, 9), 10, 10), None);
        assert_eq!(cell_in_bounds((-1, 5), 10, 10), None);
    }

    #[test]
    fn clip_fully_inside_is_unchanged() {
        let min = Point2::new(0.0, 0.0);
        let max = Point2::new(10.0, 10.0);
        let c = clip_segment_to_aabb(Point2::new(1.0, 1.0), Point2::new(9.0, 5.0), min, max)
            .unwrap();
        assert_relative_eq!(c.start.x, 1.0);
        assert_relative_eq!(c.end.x, 9.0);
        assert!(!c.end_clipped);
    }

    #[test]
    fn clip_shortens_exiting_segment() {
        let min = Point2::new(0.0, 0.0);
        let max = Point2::new(10.0, 10.0);
        // Horizontal beam from (5,5) exiting through the right edge.
        let c = clip_segment_to_aabb(Point2::new(5.0, 5.0), Point2::new(20.0, 5.0), min, max)
            .unwrap();
        assert_relative_eq!(c.end.x, 10.0, epsilon = 1e-12);
        assert_relative_eq!(c.end.y, 5.0, epsilon = 1e-12);
        assert!(c.end_clipped);
    }

    #[test]
    fn clip_rejects_fully_outside() {
        let min = Point2::new(0.0, 0.0);
        let max = Point2::new(10.0, 10.0);
        assert!(
            clip_segment_to_aabb(Point2::new(-5.0, -5.0), Point2::new(-1.0, -2.0), min, max)
                .is_none()
        );
        // Parallel to an edge, outside its slab.
        assert!(
            clip_segment_to_aabb(Point2::new(0.0, 11.0), Point2::new(10.0, 11.0), min, max)
                .is_none()
        );
    }

    #[test]
    fn clip_crossing_segment_clips_both_ends() {
        let min = Point2::new(0.0, 0.0);
        let max = Point2::new(10.0, 10.0);
        let c = clip_segment_to_aabb(Point2::new(-5.0, 5.0), Point2::new(15.0, 5.0), min, max)
            .unwrap();
        assert_relative_eq!(c.start.x, 0.0, epsilon = 1e-12);
        assert_relative_eq!(c.end.x, 10.0, epsilon = 1e-12);
        assert!(c.end_clipped);
    }

    #[test]
    fn clip_degenerate_zero_length() {
        let min = Point2::new(0.0, 0.0);
        let max = Point2::new(10.0, 10.0);
        let inside =
            clip_segment_to_aabb(Point2::new(3.0, 3.0), Point2::new(3.0, 3.0), min, max).unwrap();
        assert!(!inside.end_clipped);
        assert!(
            clip_segment_to_aabb(Point2::new(-3.0, 3.0), Point2::new(-3.0, 3.0), min, max)
                .is_none()
        );
    }

    #[test]
    fn clip_rejects_non_finite() {
        let min = Point2::new(0.0, 0.0);
        let max = Point2::new(10.0, 10.0);
        assert!(
            clip_segment_to_aabb(Point2::new(f64::NAN, 3.0), Point2::new(3.0, 3.0), min, max)
                .is_none()
        );
    }
}
