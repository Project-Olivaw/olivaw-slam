//! 2D lidar SLAM in pure Rust — no ROS2, no C++ dependencies.
//!
//! `olivaw-slam` takes lidar scans (from `olivaw-lidar` or any other source) and
//! produces a consistent occupancy-grid map plus a pose estimate. Every layer
//! (preprocessing, scan matching, grid mapping, pose graph) is independently
//! usable; depend only on what you need.
//!
//! # Units — read this first
//!
//! **Everything inside this crate is metres and radians.** Sensor drivers such as
//! `olivaw-lidar` report millimetres and degrees — convert once at the boundary,
//! before constructing a [`ScanCloud`], and never again. Angles are radians in
//! the half-open interval `(-π, π]`, counter-clockwise positive, in an x-forward,
//! y-left, right-handed frame.
//!
//! Poses use `f64` (composition accumulates error); grid cells use `f32`
//! (memory-bound, precision is ample).

#![forbid(unsafe_code)]

mod convert;
mod error;
pub mod graph;
pub mod grid;
pub mod loop_closure;
pub mod matcher;
mod pose;
pub mod preprocess;
mod scan;
mod slam;

pub use error::SlamError;
pub use graph::{NodeId, PoseGraph};
pub use grid::{GridConfig, OccupancyGrid};
pub use loop_closure::LoopClosureConfig;
pub use matcher::{MatchResult, ScanMatcher};
pub use pose::{Point2, Pose2, normalize_angle};
pub use scan::ScanCloud;
pub use slam::{Keyframe, KeyframeConfig, Slam, SlamConfig};
