//! The high-level SLAM orchestrator: preprocessing → scan-to-map matching →
//! keyframes → pose graph → loop closure, behind one [`Slam`] type.
//!
//! The loop is exposed as discrete callable steps (`process_scan` per scan,
//! no hidden threads) so an external orchestrator — a dora-rs node, a ROS2
//! bridge, a plain binary — can drive it.

use nalgebra::Matrix3;

use crate::error::SlamError;
use crate::graph::{NodeId, PoseGraph};
use crate::grid::{GridConfig, OccupancyGrid};
use crate::loop_closure::{KeyframeLike, LoopClosureConfig, LoopDetector};
use crate::matcher::{ScanToMapConfig, ScanToMapMatcher};
use crate::pose::{Pose2, normalize_angle};
use crate::preprocess::{PreprocessConfig, Preprocessor};
use crate::scan::ScanCloud;

/// Top-level configuration: every stage's tunables in one place.
#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct SlamConfig {
    /// Scan preprocessing (range gate, outlier rejection, subsampling).
    pub preprocess: PreprocessConfig,
    /// Occupancy grid geometry and sensor model.
    pub grid: GridConfig,
    /// Scan-to-map matcher (CSM window, likelihood field).
    pub matcher: ScanToMapConfig,
    /// Loop-closure detection and gating.
    pub loop_closure: LoopClosureConfig,
    /// Keyframe policy and optimization settings.
    pub keyframes: KeyframeConfig,
}

/// Keyframe policy and graph-optimization tunables.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct KeyframeConfig {
    /// Insert a keyframe after moving this far, metres. Default `0.3`.
    pub distance_m: f64,
    /// Insert a keyframe after rotating this much, radians. Default `0.3`.
    pub angle_rad: f64,
    /// Gauss-Newton iterations per optimization. Default `50`.
    pub optimize_iterations: usize,
    /// Scans whose match score falls below this are tracked but neither
    /// integrated into the map nor eligible as keyframes. Default `0.2`.
    pub min_integration_score: f64,
}

impl Default for KeyframeConfig {
    fn default() -> Self {
        Self {
            distance_m: 0.3,
            angle_rad: 0.3,
            optimize_iterations: 50,
            min_integration_score: 0.2,
        }
    }
}

/// A stored keyframe: a graph node with its (preprocessed) scan.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct Keyframe {
    /// Current pose estimate (kept in sync with the graph).
    pub pose: Pose2,
    /// The preprocessed scan, sensor frame, metres.
    pub cloud: ScanCloud,
}

impl KeyframeLike for Keyframe {
    fn pose(&self) -> Pose2 {
        self.pose
    }
    fn scan(&self) -> &ScanCloud {
        &self.cloud
    }
}

/// Operating mode. See [`Slam::set_localization_mode`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Mode {
    /// Build the map while tracking (default).
    Mapping,
    /// Track against a frozen map; never modify it.
    Localization,
}

/// The SLAM system. Feed it scans; it returns poses and maintains the map.
///
/// ```no_run
/// use olivaw_slam::{Slam, SlamConfig, ScanCloud};
/// # fn scans() -> Vec<ScanCloud> { Vec::new() }
/// let mut slam = Slam::new(SlamConfig::default()).unwrap();
/// for cloud in scans() {
///     let pose = slam.process_scan(&cloud).unwrap();
///     println!("pose: {pose:?}");
/// }
/// ```
#[derive(Debug)]
pub struct Slam {
    config: SlamConfig,
    preprocessor: Preprocessor,
    matcher: ScanToMapMatcher,
    detector: LoopDetector,
    graph: PoseGraph,
    keyframes: Vec<Keyframe>,
    grid: OccupancyGrid,
    pose: Pose2,
    mode: Mode,
    /// Scratch buffer reused across scans.
    filtered: ScanCloud,
    /// Total accepted loop closures (observability/testing).
    loops_closed: usize,
}

impl Slam {
    /// Create a SLAM system with the given configuration.
    ///
    /// # Errors
    ///
    /// [`SlamError::InvalidConfig`] / [`SlamError::GridTooLarge`] if any
    /// stage's configuration fails validation.
    pub fn new(config: SlamConfig) -> Result<Self, SlamError> {
        Ok(Self {
            preprocessor: Preprocessor::new(config.preprocess.clone())?,
            matcher: ScanToMapMatcher::new(config.matcher.clone()),
            detector: LoopDetector::new(config.loop_closure.clone()),
            grid: OccupancyGrid::new(config.grid.clone())?,
            graph: PoseGraph::new(),
            keyframes: Vec::new(),
            pose: Pose2::identity(),
            mode: Mode::Mapping,
            filtered: ScanCloud::default(),
            loops_closed: 0,
            config,
        })
    }

    /// Process one scan (sensor frame, metres) and return the updated world
    /// pose of the sensor.
    ///
    /// In mapping mode this may insert a keyframe, extend the pose graph,
    /// close loops, and update the grid. In localization mode the map is
    /// never touched.
    ///
    /// # Errors
    ///
    /// [`SlamError::ScanTooLarge`] for oversized input;
    /// [`SlamError::OptimizationFailed`] if the backend fails. A merely *weak*
    /// match is not an error — the previous pose is carried forward.
    pub fn process_scan(&mut self, cloud: &ScanCloud) -> Result<Pose2, SlamError> {
        let mut filtered = std::mem::take(&mut self.filtered);
        self.preprocessor.process(cloud, &mut filtered)?;
        let result = match self.mode {
            Mode::Mapping => self.track_and_map(&filtered),
            Mode::Localization => self.track_only(&filtered),
        };
        self.filtered = filtered;
        result
    }

    /// Current pose estimate (world frame, metres/radians).
    #[must_use]
    pub fn pose(&self) -> Pose2 {
        self.pose
    }

    /// The occupancy grid map.
    #[must_use]
    pub fn grid(&self) -> &OccupancyGrid {
        &self.grid
    }

    /// The stored keyframes (poses stay in sync with the graph).
    #[must_use]
    pub fn keyframes(&self) -> &[Keyframe] {
        &self.keyframes
    }

    /// The underlying pose graph.
    #[must_use]
    pub fn graph(&self) -> &PoseGraph {
        &self.graph
    }

    /// The configuration this system was built with.
    #[must_use]
    pub fn config(&self) -> &SlamConfig {
        &self.config
    }

    /// Number of loop closures accepted so far.
    #[must_use]
    pub fn loops_closed(&self) -> usize {
        self.loops_closed
    }

    /// Switch between mapping (default) and localization mode. In
    /// localization mode the map and graph are frozen; scans only update the
    /// pose estimate. Typically used after [`Slam::load`]-ing a saved map.
    pub fn set_localization_mode(&mut self, enabled: bool) {
        self.mode = if enabled { Mode::Localization } else { Mode::Mapping };
    }

    /// `true` when in localization mode.
    #[must_use]
    pub fn is_localization_mode(&self) -> bool {
        self.mode == Mode::Localization
    }

    // ---- internals ----

    fn track_and_map(&mut self, filtered: &ScanCloud) -> Result<Pose2, SlamError> {
        // First scan bootstraps the map at the origin.
        if self.keyframes.is_empty() {
            self.pose = Pose2::identity();
            self.insert_keyframe(filtered, None);
            return Ok(self.pose);
        }

        let matched = match self.matcher.match_scan(&self.grid, filtered, &self.pose) {
            Ok(r) => r,
            // A failed match (e.g. featureless input) is survivable: carry
            // the previous estimate forward and skip map updates.
            Err(SlamError::MatchFailed { .. }) => return Ok(self.pose),
            Err(e) => return Err(e),
        };
        if !matched.converged || matched.score < self.config.keyframes.min_integration_score {
            return Ok(self.pose);
        }
        self.pose = matched.pose;

        // Keyframe policy: bound graph growth by distance/angle travelled.
        let last = self.keyframes.len() - 1;
        let delta = self
            .keyframes
            .last()
            .map_or_else(Pose2::identity, |kf| kf.pose.between(&self.pose));
        let moved = (delta.x * delta.x + delta.y * delta.y).sqrt();
        if moved >= self.config.keyframes.distance_m
            || delta.theta.abs() >= self.config.keyframes.angle_rad
        {
            self.insert_keyframe(filtered, Some((last, delta, &matched.covariance)));
            self.try_close_loop()?;
        }
        Ok(self.pose)
    }

    fn track_only(&mut self, filtered: &ScanCloud) -> Result<Pose2, SlamError> {
        match self.matcher.match_scan(&self.grid, filtered, &self.pose) {
            Ok(r) if r.converged && r.score >= self.config.keyframes.min_integration_score => {
                self.pose = r.pose;
                Ok(self.pose)
            }
            Ok(_) | Err(SlamError::MatchFailed { .. }) => Ok(self.pose),
            Err(e) => Err(e),
        }
    }

    /// Store a keyframe: add the graph node (and odometry-style edge from the
    /// previous keyframe when given) and integrate the scan into the grid.
    fn insert_keyframe(
        &mut self,
        filtered: &ScanCloud,
        edge_from: Option<(usize, Pose2, &Matrix3<f64>)>,
    ) {
        let id = self.graph.add_node(self.pose);
        if let Some((prev, delta, covariance)) = edge_from {
            let information = covariance
                .try_inverse()
                .filter(|m| m.iter().all(|v| v.is_finite()))
                .unwrap_or_else(|| Matrix3::from_diagonal_element(1.0));
            self.graph.add_edge(node(prev), id, delta, information);
        }
        self.grid.integrate_scan(&self.pose, filtered);
        self.keyframes.push(Keyframe { pose: self.pose, cloud: filtered.clone() });
    }

    /// Loop-closure stages 1–3. Detection runs on the freshly added keyframe;
    /// gating optimizes a *copy* of the graph and adopts it only if the loop
    /// edge stays consistent — a rejected constraint leaves no trace.
    fn try_close_loop(&mut self) -> Result<(), SlamError> {
        let current = self.keyframes.len() - 1;
        let Some(constraint) = self.detector.detect(&self.keyframes, current)? else {
            return Ok(());
        };

        let mut trial = self.graph.clone();
        trial.add_robust_edge(
            node(constraint.node),
            node(current),
            constraint.relative,
            constraint.information,
        );
        if trial.optimize(self.config.keyframes.optimize_iterations).is_err() {
            return Ok(()); // solver refused: reject the constraint
        }

        // Gate: the loop edge residual after optimization must be consistent.
        let (Some(pi), Some(pj)) =
            (trial.node_pose(node(constraint.node)), trial.node_pose(node(current)))
        else {
            return Ok(());
        };
        let observed = pi.between(&pj);
        let r = nalgebra::Vector3::new(
            observed.x - constraint.relative.x,
            observed.y - constraint.relative.y,
            normalize_angle(observed.theta - constraint.relative.theta),
        );
        let chi2 = r.dot(&(constraint.information * r));
        if chi2 > self.detector.config().residual_gate {
            return Ok(()); // inconsistent: a false positive folds the map — reject
        }

        // Accept: adopt the optimized graph, refresh keyframes, rebuild map.
        self.graph = trial;
        for (kf, pose) in self.keyframes.iter_mut().zip(self.graph.node_poses()) {
            kf.pose = *pose;
        }
        self.pose = self.keyframes.last().map_or(self.pose, |kf| kf.pose);
        self.rebuild_grid()?;
        self.loops_closed += 1;
        Ok(())
    }

    /// Re-integrate every keyframe scan at its (optimized) pose.
    fn rebuild_grid(&mut self) -> Result<(), SlamError> {
        let mut grid = OccupancyGrid::new(self.config.grid.clone())?;
        for kf in &self.keyframes {
            grid.integrate_scan(&kf.pose, &kf.cloud);
        }
        self.grid = grid;
        Ok(())
    }
}

/// Keyframe index → graph node id (they are inserted in lockstep).
fn node(index: usize) -> NodeId {
    // NodeId is dense from 0 in insertion order; keyframes and nodes are
    // created together, so the index mapping is the identity.
    PoseGraph::node_id_for_index(index)
}

/// Serialized SLAM state (`serialize` feature). The grid is *not* stored:
/// rebuilding it from the keyframes is deterministic, so a reload reproduces
/// identical geometry from a fraction of the bytes.
#[cfg(feature = "serialize")]
#[derive(serde::Serialize, serde::Deserialize)]
struct SlamState {
    config: SlamConfig,
    keyframes: Vec<Keyframe>,
    edges: Vec<crate::graph::GraphEdge>,
    pose: Pose2,
    loops_closed: usize,
}

#[cfg(feature = "serialize")]
const STATE_MAGIC: &[u8; 8] = b"OLIVSLAM";
#[cfg(feature = "serialize")]
const STATE_VERSION: u32 = 1;

#[cfg(feature = "serialize")]
impl Slam {
    /// Save the full SLAM state (config, keyframes with scans, pose graph)
    /// to `path` as a compact binary file. Load it back with [`Slam::load`]
    /// to continue mapping or to localize (lifelong mapping).
    ///
    /// # Errors
    ///
    /// [`SlamError::Serialization`] on encoding failure, [`SlamError::Io`] on
    /// write failure.
    pub fn save(&self, path: &std::path::Path) -> Result<(), SlamError> {
        let state = SlamState {
            config: self.config.clone(),
            keyframes: self.keyframes.clone(),
            edges: self.graph.edges().to_vec(),
            pose: self.pose,
            loops_closed: self.loops_closed,
        };
        let body = postcard::to_allocvec(&state)
            .map_err(|e| SlamError::Serialization { reason: e.to_string() })?;
        let mut out = Vec::with_capacity(body.len() + 12);
        out.extend_from_slice(STATE_MAGIC);
        out.extend_from_slice(&STATE_VERSION.to_le_bytes());
        out.extend_from_slice(&body);
        std::fs::write(path, out)?;
        Ok(())
    }

    /// Load a SLAM state saved by [`Slam::save`]. The grid is rebuilt from
    /// the stored keyframes with identical geometry. The system loads in
    /// mapping mode; call [`Slam::set_localization_mode`] to localize in the
    /// map without extending it.
    ///
    /// # Errors
    ///
    /// [`SlamError::Serialization`] for wrong magic/version or corrupt data;
    /// [`SlamError::Io`] on read failure.
    pub fn load(path: &std::path::Path) -> Result<Self, SlamError> {
        let data = std::fs::read(path)?;
        let (header, body) = data.split_at_checked(12).ok_or_else(|| {
            SlamError::Serialization { reason: "file too short".to_owned() }
        })?;
        if header.get(..8) != Some(STATE_MAGIC.as_slice()) {
            return Err(SlamError::Serialization {
                reason: "not an olivaw-slam state file (bad magic)".to_owned(),
            });
        }
        let version = header
            .get(8..12)
            .and_then(|b| <[u8; 4]>::try_from(b).ok())
            .map(u32::from_le_bytes);
        if version != Some(STATE_VERSION) {
            return Err(SlamError::Serialization {
                reason: format!("unsupported state version {version:?} (want {STATE_VERSION})"),
            });
        }
        let state: SlamState = postcard::from_bytes(body)
            .map_err(|e| SlamError::Serialization { reason: e.to_string() })?;

        let mut slam = Self::new(state.config)?;
        slam.graph = PoseGraph::from_parts(
            state.keyframes.iter().map(|k| k.pose).collect(),
            state.edges,
        );
        slam.keyframes = state.keyframes;
        slam.pose = state.pose;
        slam.loops_closed = state.loops_closed;
        slam.rebuild_grid()?;
        Ok(slam)
    }
}
