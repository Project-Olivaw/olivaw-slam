//! Property tests on the `Pose2` algebra.
//!
//! The geometry must be provably correct before anything depends on it: every
//! matcher, grid update, and graph edge composes poses. Angle wrapping bugs in
//! particular produce maps that look *almost* right.

use std::f64::consts::PI;

use olivaw_slam::{Point2, Pose2, normalize_angle};
use proptest::prelude::*;

const EPS: f64 = 1e-9;

fn pose_strategy() -> impl Strategy<Value = Pose2> {
    (-100.0..100.0f64, -100.0..100.0f64, (-10.0 * PI)..(10.0 * PI))
        .prop_map(|(x, y, theta)| Pose2::new(x, y, theta))
}

fn point_strategy() -> impl Strategy<Value = Point2> {
    (-100.0..100.0f64, -100.0..100.0f64).prop_map(|(x, y)| Point2::new(x, y))
}

/// Angle-aware pose comparison: translations by absolute difference, headings
/// by normalized angular difference (so π and -π compare equal).
fn assert_pose_close(a: &Pose2, b: &Pose2) {
    assert!((a.x - b.x).abs() < EPS, "x differs: {} vs {}", a.x, b.x);
    assert!((a.y - b.y).abs() < EPS, "y differs: {} vs {}", a.y, b.y);
    let dtheta = normalize_angle(a.theta - b.theta);
    assert!(dtheta.abs() < EPS, "theta differs: {} vs {}", a.theta, b.theta);
}

proptest! {
    #[test]
    fn theta_always_normalized(p in pose_strategy()) {
        prop_assert!(p.theta > -PI && p.theta <= PI);
    }

    #[test]
    fn normalize_angle_lands_in_half_open_interval(theta in (-40.0 * PI)..(40.0 * PI)) {
        let n = normalize_angle(theta);
        prop_assert!(n > -PI && n <= PI, "normalize_angle({theta}) = {n}");
        // Normalization preserves the angle modulo 2π.
        let diff = (theta - n).rem_euclid(2.0 * PI);
        prop_assert!(diff < EPS || (2.0 * PI - diff) < EPS);
    }

    #[test]
    fn compose_inverse_round_trips(p in pose_strategy()) {
        assert_pose_close(&p.compose(&p.inverse()), &Pose2::identity());
        assert_pose_close(&p.inverse().compose(&p), &Pose2::identity());
    }

    #[test]
    fn identity_is_neutral(p in pose_strategy()) {
        let id = Pose2::identity();
        assert_pose_close(&id.compose(&p), &p);
        assert_pose_close(&p.compose(&id), &p);
    }

    #[test]
    fn between_is_compose_inverse(a in pose_strategy(), b in pose_strategy()) {
        // a ⊕ (a.between(b)) == b
        assert_pose_close(&a.compose(&a.between(&b)), &b);
    }

    #[test]
    fn compose_is_associative(
        a in pose_strategy(),
        b in pose_strategy(),
        c in pose_strategy(),
    ) {
        assert_pose_close(&a.compose(&b).compose(&c), &a.compose(&b.compose(&c)));
    }

    #[test]
    fn transform_point_round_trips_through_inverse(
        pose in pose_strategy(),
        q in point_strategy(),
    ) {
        let back = pose.inverse().transform_point(pose.transform_point(q));
        prop_assert!((back.x - q.x).abs() < EPS);
        prop_assert!((back.y - q.y).abs() < EPS);
    }

    #[test]
    fn transform_point_agrees_with_isometry(
        pose in pose_strategy(),
        q in point_strategy(),
    ) {
        let ours = pose.transform_point(q);
        let theirs = pose.to_isometry().transform_point(&q);
        prop_assert!((ours.x - theirs.x).abs() < EPS);
        prop_assert!((ours.y - theirs.y).abs() < EPS);
    }
}
