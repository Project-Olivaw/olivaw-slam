//! 2D rigid-body pose ([`Pose2`], SE(2)) and angle normalization.

use std::f64::consts::PI;

use nalgebra::{Isometry2, Vector2};

/// A point in 2D, metres. x forward, y left, right-handed.
pub type Point2 = nalgebra::Point2<f64>;

/// Normalize an angle in radians to the half-open interval `(-π, π]`.
///
/// This is the *only* angle-wrapping function in the crate; every [`Pose2`]
/// operation routes through it so the `theta ∈ (-π, π]` invariant lives in one
/// place. Non-finite inputs propagate (NaN in → NaN out).
///
/// Boundary behaviour: `+π → π`, `-π → π`, `±2π → 0`, `±3π → π`.
#[must_use]
#[inline]
pub fn normalize_angle(theta: f64) -> f64 {
    // rem_euclid maps into [0, 2π); the negations shift that to (-π, π] with
    // the boundary landing on +π (so -π normalizes to π, not -π).
    -((-theta + PI).rem_euclid(2.0 * PI) - PI)
}

/// 2D rigid-body pose, internally SE(2).
///
/// Units: `x`, `y` in metres; `theta` in radians, always normalized to
/// `(-π, π]` (counter-clockwise positive). Construct via [`Pose2::new`] to get
/// normalization; if you set `theta` directly you are responsible for it.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct Pose2 {
    /// Translation x, metres.
    pub x: f64,
    /// Translation y, metres.
    pub y: f64,
    /// Heading, radians, normalized to `(-π, π]`.
    pub theta: f64,
}

impl Pose2 {
    /// Create a pose from `x`, `y` (metres) and `theta` (radians).
    /// `theta` is normalized to `(-π, π]`.
    #[must_use]
    pub fn new(x: f64, y: f64, theta: f64) -> Self {
        Self { x, y, theta: normalize_angle(theta) }
    }

    /// The identity pose (origin, zero heading).
    #[must_use]
    pub fn identity() -> Self {
        Self { x: 0.0, y: 0.0, theta: 0.0 }
    }

    /// Pose composition `self ⊕ other`: apply `other` in the frame of `self`.
    #[must_use]
    pub fn compose(&self, other: &Pose2) -> Pose2 {
        let (s, c) = self.theta.sin_cos();
        Pose2 {
            x: self.x + c * other.x - s * other.y,
            y: self.y + s * other.x + c * other.y,
            theta: normalize_angle(self.theta + other.theta),
        }
    }

    /// The inverse pose: `self ⊕ self.inverse() == identity`.
    #[must_use]
    pub fn inverse(&self) -> Pose2 {
        let (s, c) = self.theta.sin_cos();
        Pose2 {
            x: -(c * self.x + s * self.y),
            y: s * self.x - c * self.y,
            theta: normalize_angle(-self.theta),
        }
    }

    /// Relative transform from `self` to `other`: `self ⊕ result == other`.
    #[must_use]
    pub fn between(&self, other: &Pose2) -> Pose2 {
        self.inverse().compose(other)
    }

    /// Transform a point (metres) from this pose's frame into the world frame.
    #[must_use]
    #[inline]
    pub fn transform_point(&self, p: Point2) -> Point2 {
        let (s, c) = self.theta.sin_cos();
        Point2::new(self.x + c * p.x - s * p.y, self.y + s * p.x + c * p.y)
    }

    /// Convert to an [`nalgebra::Isometry2`] (same translation and rotation).
    #[must_use]
    pub fn to_isometry(&self) -> Isometry2<f64> {
        Isometry2::new(Vector2::new(self.x, self.y), self.theta)
    }
}

impl Default for Pose2 {
    fn default() -> Self {
        Self::identity()
    }
}

#[cfg(test)]
mod tests {
    use approx::{assert_abs_diff_eq, assert_relative_eq};

    use super::*;

    const EPS: f64 = 1e-12;

    #[test]
    fn normalize_angle_boundaries() {
        assert_abs_diff_eq!(normalize_angle(0.0), 0.0, epsilon = EPS);
        // +π and -π both map to +π (half-open interval (-π, π]).
        assert_abs_diff_eq!(normalize_angle(PI), PI, epsilon = EPS);
        assert_abs_diff_eq!(normalize_angle(-PI), PI, epsilon = EPS);
        assert_abs_diff_eq!(normalize_angle(2.0 * PI), 0.0, epsilon = EPS);
        assert_abs_diff_eq!(normalize_angle(-2.0 * PI), 0.0, epsilon = EPS);
        assert_abs_diff_eq!(normalize_angle(3.0 * PI), PI, epsilon = EPS);
        assert_abs_diff_eq!(normalize_angle(-3.0 * PI), PI, epsilon = EPS);
    }

    #[test]
    fn normalize_angle_near_boundaries() {
        let eps = 1e-9;
        assert_abs_diff_eq!(normalize_angle(PI - eps), PI - eps, epsilon = EPS);
        // Just past +π wraps to just above -π.
        assert_abs_diff_eq!(normalize_angle(PI + eps), -PI + eps, epsilon = 1e-9);
        assert_abs_diff_eq!(normalize_angle(-PI + eps), -PI + eps, epsilon = EPS);
        assert_abs_diff_eq!(normalize_angle(10.0 * PI + 0.5), 0.5, epsilon = 1e-9);
    }

    #[test]
    fn normalize_angle_propagates_nan() {
        assert!(normalize_angle(f64::NAN).is_nan());
    }

    #[test]
    fn compose_hand_computed() {
        // Move 1 m forward, turn 90° left, then move 1 m "forward" again:
        // ends up at (1, 1) facing +y.
        let a = Pose2::new(1.0, 0.0, PI / 2.0);
        let b = Pose2::new(1.0, 0.0, 0.0);
        let c = a.compose(&b);
        assert_relative_eq!(c.x, 1.0, epsilon = 1e-12);
        assert_relative_eq!(c.y, 1.0, epsilon = 1e-12);
        assert_relative_eq!(c.theta, PI / 2.0, epsilon = 1e-12);
    }

    #[test]
    fn inverse_of_pi_heading_composes_to_identity() {
        let p = Pose2::new(2.0, -3.0, PI);
        let id = p.compose(&p.inverse());
        assert_abs_diff_eq!(id.x, 0.0, epsilon = 1e-12);
        assert_abs_diff_eq!(id.y, 0.0, epsilon = 1e-12);
        assert_abs_diff_eq!(normalize_angle(id.theta), 0.0, epsilon = 1e-12);
    }

    #[test]
    fn transform_point_matches_isometry() {
        let pose = Pose2::new(0.5, -1.25, 0.7);
        let p = Point2::new(2.0, 3.0);
        let ours = pose.transform_point(p);
        let theirs = pose.to_isometry().transform_point(&p);
        assert_relative_eq!(ours.x, theirs.x, epsilon = 1e-12);
        assert_relative_eq!(ours.y, theirs.y, epsilon = 1e-12);
    }

    #[test]
    fn between_recovers_relative_pose() {
        let a = Pose2::new(1.0, 2.0, 0.3);
        let rel = Pose2::new(0.4, -0.1, -0.2);
        let b = a.compose(&rel);
        let recovered = a.between(&b);
        assert_relative_eq!(recovered.x, rel.x, epsilon = 1e-12);
        assert_relative_eq!(recovered.y, rel.y, epsilon = 1e-12);
        assert_relative_eq!(recovered.theta, rel.theta, epsilon = 1e-12);
    }
}
