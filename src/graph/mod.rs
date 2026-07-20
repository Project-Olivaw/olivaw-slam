//! Pose graph backed by [factrs](https://docs.rs/factrs).
//!
//! Nodes are keyframe poses; edges are relative-pose constraints with
//! information matrices from the scan matcher. The graph stores its own nodes
//! and edges and rebuilds the factrs problem on every [`PoseGraph::optimize`]
//! call — at house scale (hundreds of keyframes) the rebuild is microseconds,
//! and it makes speculative optimization (loop-closure gating on a clone)
//! trivial.
//!
//! Conventions at the factrs boundary, handled here once:
//! - our `Pose2` is `(x, y, θ)`; factrs `SE2::new` takes `(θ, x, y)`;
//! - our covariances/information are ordered `[x, y, θ]`; factrs tangent
//!   space is rotation-first `[θ, x, y]` — information matrices are permuted;
//! - factrs uses nalgebra 0.33 internally, we use 0.34 — matrices cross the
//!   boundary element-wise, never by type.

use factrs::containers::FactorBuilder;
use factrs::core::{BetweenResidual, GaussNewton, GaussianNoise, Graph, Huber, PriorResidual, SE2, Values};
use factrs::optimizers::OptError;
use factrs::traits::Optimizer;
use nalgebra::Matrix3;

use crate::error::SlamError;
use crate::pose::Pose2;

#[allow(missing_docs)]
mod symbols {
    use factrs::assign_symbols;
    use factrs::core::SE2;
    assign_symbols!(X: SE2);
}
use symbols::X;

/// Handle to a node in a [`PoseGraph`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct NodeId(usize);

impl NodeId {
    /// The node's index (nodes are numbered densely from 0 in insertion
    /// order).
    #[must_use]
    pub fn index(self) -> usize {
        self.0
    }
}

/// A relative-pose constraint between two nodes.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct GraphEdge {
    /// Source node.
    pub from: NodeId,
    /// Target node.
    pub to: NodeId,
    /// Measured relative pose: `from.between(to)` (metres/radians).
    pub measurement: Pose2,
    /// Information matrix (inverse covariance) over `[x, y, θ]`.
    pub information: Matrix3<f64>,
    /// If `true` the edge uses a Huber robust kernel, so a bad constraint
    /// degrades the solution gracefully instead of catastrophically. Use for
    /// loop closures.
    pub robust: bool,
}

/// A prior (absolute) constraint on a single node.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct GraphPrior {
    /// Constrained node.
    pub node: NodeId,
    /// Prior pose in the world frame (metres/radians).
    pub pose: Pose2,
    /// Information matrix over `[x, y, θ]`.
    pub information: Matrix3<f64>,
}

/// 2D pose graph. See the module docs for conventions.
#[derive(Debug, Clone, Default)]
pub struct PoseGraph {
    nodes: Vec<Pose2>,
    edges: Vec<GraphEdge>,
    priors: Vec<GraphPrior>,
}

impl PoseGraph {
    /// Create an empty graph.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Rebuild a graph from saved parts. Nodes take the given poses in order;
    /// edge validity is checked lazily at the next [`PoseGraph::optimize`].
    #[must_use]
    pub fn from_parts(poses: Vec<Pose2>, edges: Vec<GraphEdge>) -> Self {
        Self { nodes: poses, edges, priors: Vec::new() }
    }

    /// The [`NodeId`] for a dense insertion index (nodes are numbered from 0
    /// in insertion order). The caller is responsible for the index being in
    /// range — out-of-range ids are rejected at optimization time.
    #[must_use]
    pub fn node_id_for_index(index: usize) -> NodeId {
        NodeId(index)
    }

    /// Add a node with an initial pose estimate; returns its handle.
    pub fn add_node(&mut self, pose: Pose2) -> NodeId {
        self.nodes.push(pose);
        NodeId(self.nodes.len() - 1)
    }

    /// Add a relative-pose edge. `measurement` is `from.between(to)`;
    /// `information` is the inverse covariance over `[x, y, θ]`.
    pub fn add_edge(&mut self, from: NodeId, to: NodeId, measurement: Pose2, information: Matrix3<f64>) {
        self.edges.push(GraphEdge { from, to, measurement, information, robust: false });
    }

    /// Like [`PoseGraph::add_edge`] but with a Huber robust kernel — use for
    /// loop-closure constraints, which may be wrong.
    pub fn add_robust_edge(
        &mut self,
        from: NodeId,
        to: NodeId,
        measurement: Pose2,
        information: Matrix3<f64>,
    ) {
        self.edges.push(GraphEdge { from, to, measurement, information, robust: true });
    }

    /// Add an absolute prior on a node.
    pub fn add_prior(&mut self, node: NodeId, pose: Pose2, information: Matrix3<f64>) {
        self.priors.push(GraphPrior { node, pose, information });
    }

    /// Remove and return the most recently added edge (used to revert a
    /// speculative loop closure).
    pub fn pop_edge(&mut self) -> Option<GraphEdge> {
        self.edges.pop()
    }

    /// Current pose estimate of `id`, if it exists.
    #[must_use]
    pub fn node_pose(&self, id: NodeId) -> Option<Pose2> {
        self.nodes.get(id.0).copied()
    }

    /// All node poses in insertion order.
    #[must_use]
    pub fn node_poses(&self) -> &[Pose2] {
        &self.nodes
    }

    /// All edges in insertion order.
    #[must_use]
    pub fn edges(&self) -> &[GraphEdge] {
        &self.edges
    }

    /// Number of nodes.
    #[must_use]
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    /// Optimize the graph in place with Gauss-Newton and return the final
    /// objective value. If no explicit prior exists, the first node is
    /// anchored automatically (gauge freedom).
    ///
    /// Hitting the iteration limit still adopts the (partially optimized)
    /// estimate — standard practice in an online SLAM loop.
    ///
    /// # Errors
    ///
    /// [`SlamError::OptimizationFailed`] if the problem is structurally
    /// invalid or the solver cannot make a step; node poses are left
    /// unchanged in that case.
    pub fn optimize(&mut self, max_iterations: usize) -> Result<f64, SlamError> {
        if self.nodes.is_empty() {
            return Ok(0.0);
        }
        let (graph, values) = self.build_factrs()?;
        let mut opt: GaussNewton = GaussNewton::new(graph);
        opt.params.max_iterations = max_iterations;
        let result = match opt.optimize(values) {
            // Hitting the iteration cap still yields a usable estimate.
            Ok(v) | Err(OptError::MaxIterations(v)) => v,
            Err(e) => {
                return Err(SlamError::OptimizationFailed { reason: format!("{e:?}") });
            }
        };
        let cost = opt.error(&result);
        for (i, slot) in self.nodes.iter_mut().enumerate() {
            let Ok(key) = u32::try_from(i) else { continue };
            if let Some(se2) = result.get(X(key)) {
                let se2: &SE2 = se2;
                *slot = Pose2::new(se2.x(), se2.y(), se2.theta());
            }
        }
        Ok(cost)
    }

    /// Total objective at the current estimate (without optimizing).
    ///
    /// # Errors
    ///
    /// [`SlamError::OptimizationFailed`] if the graph cannot be built
    /// (non-positive-definite information matrix, too many nodes).
    pub fn error(&self) -> Result<f64, SlamError> {
        let (graph, values) = self.build_factrs()?;
        Ok(graph.error(&values))
    }

    /// Build the factrs problem from the stored nodes, edges, and priors.
    fn build_factrs(&self) -> Result<(Graph, Values), SlamError> {
        let mut values = Values::new();
        for (i, pose) in self.nodes.iter().enumerate() {
            let key = u32::try_from(i).map_err(|_| SlamError::OptimizationFailed {
                reason: "more than u32::MAX nodes".to_owned(),
            })?;
            values.insert(X(key), SE2::new(pose.theta, pose.x, pose.y));
        }

        let mut graph = Graph::with_capacity(self.edges.len() + self.priors.len() + 1);
        if self.priors.is_empty() {
            // Anchor the gauge: strong prior on the first node.
            let first = self.nodes.first().copied().unwrap_or_default();
            let noise = GaussianNoise::<3>::from_diag_sigmas(1e-4, 1e-4, 1e-4);
            let residual = PriorResidual::new(SE2::new(first.theta, first.x, first.y));
            graph.add_factor(FactorBuilder::new1(residual, X(0)).noise(noise).build());
        }
        for p in &self.priors {
            let key = node_key(p.node, self.nodes.len())?;
            let noise = information_to_noise(&p.information)?;
            let residual = PriorResidual::new(SE2::new(p.pose.theta, p.pose.x, p.pose.y));
            graph.add_factor(FactorBuilder::new1(residual, X(key)).noise(noise).build());
        }
        for e in &self.edges {
            let from = node_key(e.from, self.nodes.len())?;
            let to = node_key(e.to, self.nodes.len())?;
            let noise = information_to_noise(&e.information)?;
            let m = e.measurement;
            let residual = BetweenResidual::new(SE2::new(m.theta, m.x, m.y));
            let builder = FactorBuilder::new2(residual, X(from), X(to)).noise(noise);
            let factor =
                if e.robust { builder.robust(Huber::default()).build() } else { builder.build() };
            graph.add_factor(factor);
        }
        Ok((graph, values))
    }
}

/// Validate a node id against the node count and convert to a factrs key.
fn node_key(id: NodeId, count: usize) -> Result<u32, SlamError> {
    if id.0 >= count {
        return Err(SlamError::OptimizationFailed {
            reason: format!("edge references missing node {}", id.0),
        });
    }
    u32::try_from(id.0)
        .map_err(|_| SlamError::OptimizationFailed { reason: "node id exceeds u32".to_owned() })
}

/// Convert an `[x, y, θ]` information matrix into a factrs noise model
/// (rotation-first `[θ, x, y]` ordering, nalgebra-0.33 types), regularizing
/// minimally if it is not positive definite.
fn information_to_noise(
    information: &Matrix3<f64>,
) -> Result<GaussianNoise<3>, SlamError> {
    // Permute [x,y,θ] → [θ,x,y]: new[i][j] = old[m(i)][m(j)], m = [2,0,1].
    const M: [usize; 3] = [2, 0, 1];
    let mut permuted = Matrix3::<f64>::zeros();
    for (i, &mi) in M.iter().enumerate() {
        for (j, &mj) in M.iter().enumerate() {
            let (Some(src), Some(dst)) = (information.get((mi, mj)), permuted.get_mut((i, j)))
            else {
                continue;
            };
            *dst = *src;
        }
    }
    // factrs Choleskys the information matrix; make sure it is PD first,
    // regularizing with a growing diagonal if needed.
    let mut reg = 0.0_f64;
    for _ in 0..6 {
        let candidate = permuted + Matrix3::from_diagonal_element(reg);
        if candidate.cholesky().is_some() {
            let f = factrs::linalg::Matrix3::new(
                candidate.m11, candidate.m12, candidate.m13,
                candidate.m21, candidate.m22, candidate.m23,
                candidate.m31, candidate.m32, candidate.m33,
            );
            return Ok(GaussianNoise::from_matrix_inf(f.as_view()));
        }
        reg = if reg == 0.0 { 1e-9 } else { reg * 1000.0 };
    }
    Err(SlamError::OptimizationFailed {
        reason: "information matrix is not positive definite".to_owned(),
    })
}

#[cfg(test)]
mod tests {
    use approx::assert_abs_diff_eq;

    use super::*;

    fn diag_info(lin: f64, ang: f64) -> Matrix3<f64> {
        Matrix3::from_diagonal(&nalgebra::Vector3::new(lin, lin, ang))
    }

    #[test]
    fn odometry_chain_with_perfect_measurements_is_consistent() {
        let mut g = PoseGraph::new();
        let a = g.add_node(Pose2::identity());
        let b = g.add_node(Pose2::new(1.1, 0.1, 0.0)); // deliberately off
        let c = g.add_node(Pose2::new(2.2, -0.1, 0.0));
        g.add_edge(a, b, Pose2::new(1.0, 0.0, 0.0), diag_info(100.0, 100.0));
        g.add_edge(b, c, Pose2::new(1.0, 0.0, 0.0), diag_info(100.0, 100.0));
        let cost = g.optimize(50).unwrap();
        assert!(cost < 1e-9, "perfect chain should reach ~zero cost, got {cost}");
        let pb = g.node_pose(b).unwrap();
        let pc = g.node_pose(c).unwrap();
        assert_abs_diff_eq!(pb.x, 1.0, epsilon = 1e-6);
        assert_abs_diff_eq!(pb.y, 0.0, epsilon = 1e-6);
        assert_abs_diff_eq!(pc.x, 2.0, epsilon = 1e-6);
    }

    #[test]
    fn loop_constraint_distributes_error() {
        // Square loop with drifted odometry; the loop edge pulls it closed.
        let mut g = PoseGraph::new();
        let n0 = g.add_node(Pose2::identity());
        let n1 = g.add_node(Pose2::new(1.0, 0.0, std::f64::consts::FRAC_PI_2));
        let n2 = g.add_node(Pose2::new(1.1, 1.1, std::f64::consts::PI));
        let n3 = g.add_node(Pose2::new(0.2, 1.2, -std::f64::consts::FRAC_PI_2));
        let side = Pose2::new(1.0, 0.0, std::f64::consts::FRAC_PI_2);
        let info = diag_info(50.0, 50.0);
        g.add_edge(n0, n1, side, info);
        g.add_edge(n1, n2, side, info);
        g.add_edge(n2, n3, side, info);
        // Loop closure: n3 → n0 must also be one side of the square.
        g.add_robust_edge(n3, n0, side, info);
        let cost = g.optimize(100).unwrap();
        assert!(cost < 1e-6, "consistent square must close, got {cost}");
        let p3 = g.node_pose(n3).unwrap();
        assert_abs_diff_eq!(p3.x, 0.0, epsilon = 1e-4);
        assert_abs_diff_eq!(p3.y, 1.0, epsilon = 1e-4);
    }

    #[test]
    fn invalid_edge_is_reported() {
        let mut g = PoseGraph::new();
        let a = g.add_node(Pose2::identity());
        g.add_edge(a, NodeId(7), Pose2::identity(), diag_info(1.0, 1.0));
        assert!(matches!(g.optimize(10), Err(SlamError::OptimizationFailed { .. })));
    }

    #[test]
    fn priors_pin_nodes() {
        let mut g = PoseGraph::new();
        let a = g.add_node(Pose2::identity());
        let b = g.add_node(Pose2::new(0.9, 0.0, 0.0));
        g.add_prior(a, Pose2::identity(), diag_info(1e6, 1e6));
        g.add_prior(b, Pose2::new(2.0, 0.0, 0.0), diag_info(1e6, 1e6));
        g.add_edge(a, b, Pose2::new(1.0, 0.0, 0.0), diag_info(1.0, 1.0));
        g.optimize(100).unwrap();
        // Strong priors dominate the weak odometry edge.
        let pb = g.node_pose(b).unwrap();
        assert_abs_diff_eq!(pb.x, 2.0, epsilon = 1e-2);
    }
}
