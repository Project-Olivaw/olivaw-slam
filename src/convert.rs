//! Audited numeric conversions.
//!
//! This crate forbids ad-hoc lossy `as` casts. The single place a float→integer
//! conversion is permitted is [`floor_to_i64`], which validates its input first
//! so the cast is exact.

/// Largest magnitude at which every integer is exactly representable in `f64`
/// (2⁵³). Beyond this, `floor` results are not trustworthy cell indices.
const MAX_EXACT: f64 = 9_007_199_254_740_992.0;

/// Floor `v` and convert to `i64`, or `None` if the value is non-finite or its
/// magnitude is ≥ 2⁵³ (outside the exactly-representable integer range).
///
/// This is the only float→integer cast in the crate; after the finiteness and
/// range checks the `floor`ed value is an exact integer, so the cast is lossless.
#[inline]
pub(crate) fn floor_to_i64(v: f64) -> Option<i64> {
    if !v.is_finite() || v.abs() >= MAX_EXACT {
        return None;
    }
    // Exact: |v| < 2^53, so v.floor() is an integer representable in both
    // f64 and i64, and float→int `as` is defined and value-preserving here.
    #[allow(clippy::cast_possible_truncation)]
    Some(v.floor() as i64)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn floors_positives_and_negatives() {
        assert_eq!(floor_to_i64(0.0), Some(0));
        assert_eq!(floor_to_i64(1.99), Some(1));
        assert_eq!(floor_to_i64(-0.5), Some(-1));
        assert_eq!(floor_to_i64(-2.0), Some(-2));
        assert_eq!(floor_to_i64(41.999_999), Some(41));
    }

    #[test]
    fn rejects_non_finite() {
        assert_eq!(floor_to_i64(f64::NAN), None);
        assert_eq!(floor_to_i64(f64::INFINITY), None);
        assert_eq!(floor_to_i64(f64::NEG_INFINITY), None);
    }

    #[test]
    fn boundary_at_2_pow_53() {
        assert_eq!(floor_to_i64(MAX_EXACT), None);
        assert_eq!(floor_to_i64(-MAX_EXACT), None);
        assert_eq!(floor_to_i64(MAX_EXACT - 1.0), Some(9_007_199_254_740_991));
        assert_eq!(floor_to_i64(-(MAX_EXACT - 1.0)), Some(-9_007_199_254_740_991));
    }
}
