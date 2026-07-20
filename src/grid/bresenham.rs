//! Integer Bresenham line traversal over grid cells.

/// Visit every cell on the line from `from` to `to`, **excluding** `to`.
///
/// The endpoint is deliberately excluded: for a lidar beam the cells along the
/// ray get a *miss* (free-space) update while the endpoint gets a *hit* update,
/// and including the endpoint here would apply both to the hit cell — the
/// classic occupancy-grid bug. Applying updates through a closure keeps the
/// traversal allocation-free.
///
/// If `from == to`, nothing is visited.
pub(crate) fn trace(from: (i64, i64), to: (i64, i64), mut visit: impl FnMut((i64, i64))) {
    let (mut x, mut y) = from;
    let dx = (to.0 - x).abs();
    let dy = -(to.1 - y).abs();
    let sx: i64 = if x < to.0 { 1 } else { -1 };
    let sy: i64 = if y < to.1 { 1 } else { -1 };
    let mut err = dx + dy;

    while (x, y) != to {
        visit((x, y));
        let e2 = 2 * err;
        if e2 >= dy {
            err += dy;
            x += sx;
        }
        if e2 <= dx {
            err += dx;
            y += sy;
        }
    }
}

#[cfg(test)]
// Direct indexing is fine in tests (out-of-bounds is a test failure, not a
// robot crash) — the library-code rule doesn't apply here.
#[allow(clippy::indexing_slicing)]
mod tests {
    use super::*;

    fn collect(from: (i64, i64), to: (i64, i64)) -> Vec<(i64, i64)> {
        let mut cells = Vec::new();
        trace(from, to, |c| cells.push(c));
        cells
    }

    #[test]
    fn horizontal_and_vertical() {
        assert_eq!(collect((0, 0), (3, 0)), vec![(0, 0), (1, 0), (2, 0)]);
        assert_eq!(collect((0, 0), (0, 3)), vec![(0, 0), (0, 1), (0, 2)]);
        assert_eq!(collect((3, 0), (0, 0)), vec![(3, 0), (2, 0), (1, 0)]);
        assert_eq!(collect((0, 3), (0, 0)), vec![(0, 3), (0, 2), (0, 1)]);
    }

    #[test]
    fn diagonals() {
        assert_eq!(collect((0, 0), (3, 3)), vec![(0, 0), (1, 1), (2, 2)]);
        assert_eq!(collect((0, 0), (-3, -3)), vec![(0, 0), (-1, -1), (-2, -2)]);
        assert_eq!(collect((0, 0), (3, -3)), vec![(0, 0), (1, -1), (2, -2)]);
    }

    #[test]
    fn endpoint_excluded_and_degenerate_empty() {
        assert!(!collect((0, 0), (5, 2)).contains(&(5, 2)));
        assert!(collect((4, 4), (4, 4)).is_empty());
    }

    #[test]
    fn shallow_and_steep_slopes_form_connected_paths() {
        for &(from, to) in &[
            ((0_i64, 0_i64), (7_i64, 2_i64)),  // shallow
            ((0, 0), (2, 7)),                  // steep
            ((5, 5), (-3, 1)),                 // negative x direction
            ((1, -2), (-4, 6)),                // mixed signs
        ] {
            let cells = collect(from, to);
            // Correct cell count: one per Chebyshev step, endpoint excluded.
            let expected = (to.0 - from.0).abs().max((to.1 - from.1).abs());
            assert_eq!(cells.len(), usize::try_from(expected).unwrap(), "{from:?}→{to:?}");
            assert_eq!(cells.first(), Some(&from));
            // Consecutive cells are 8-adjacent (a connected ray, no gaps).
            for pair in cells.windows(2) {
                let [(ax, ay), (bx, by)] = [pair[0], pair[1]];
                assert!((ax - bx).abs() <= 1 && (ay - by).abs() <= 1, "gap in {from:?}→{to:?}");
            }
        }
    }
}
