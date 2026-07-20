//! Scan data as consumed by the SLAM pipeline.

use crate::pose::Point2;

/// A lidar scan in the sensor frame, already converted to Cartesian.
///
/// Units: metres, x forward, y left, counter-clockwise positive — convert from
/// sensor-native units (e.g. `olivaw-lidar`'s millimetres and clockwise
/// degrees) *before* constructing this type, and never convert again.
///
/// This struct is deliberately message-shaped (public fields, no internal
/// state) so a future bridge crate can convert it to/from ROS2 or other
/// middleware messages trivially.
#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct ScanCloud {
    /// Scan points in the sensor frame, metres.
    pub points: Vec<Point2>,
    /// Capture time in nanoseconds since an arbitrary, monotonic session epoch
    /// (e.g. the first scan of a recording). Not wall-clock time.
    pub timestamp_ns: u64,
}

impl ScanCloud {
    /// Create a scan cloud from points (metres, sensor frame) and a timestamp
    /// (nanoseconds since the session epoch).
    #[must_use]
    pub fn new(points: Vec<Point2>, timestamp_ns: u64) -> Self {
        Self { points, timestamp_ns }
    }

    /// Number of points in the scan.
    #[must_use]
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// `true` if the scan contains no points.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn len_and_empty() {
        let empty = ScanCloud::default();
        assert!(empty.is_empty());
        assert_eq!(empty.len(), 0);

        let cloud = ScanCloud::new(vec![Point2::new(1.0, 0.0)], 42);
        assert!(!cloud.is_empty());
        assert_eq!(cloud.len(), 1);
        assert_eq!(cloud.timestamp_ns, 42);
    }
}
