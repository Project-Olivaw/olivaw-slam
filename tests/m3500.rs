//! Backend correctness against the canonical M3500 pose-graph benchmark
//! (Olson's 3500-node Manhattan world). If our wrapper reproduces the known
//! optimum through factrs, the backend is correct.

#![allow(clippy::indexing_slicing)] // tests may index freely

use std::path::Path;

use nalgebra::Matrix3;
use olivaw_slam::{Pose2, PoseGraph};

/// Minimal g2o parser for `VERTEX_SE2` / `EDGE_SE2` lines.
///
/// `EDGE_SE2 i j dx dy dθ I11 I12 I13 I22 I23 I33` — the information matrix
/// is the upper triangle in `[x, y, θ]` order, which is exactly our
/// `PoseGraph` convention.
fn load_g2o(path: &Path) -> PoseGraph {
    let text = std::fs::read_to_string(path).expect("fixture readable");
    let mut graph = PoseGraph::new();
    let mut nodes = Vec::new();
    for line in text.lines() {
        let mut it = line.split_whitespace();
        match it.next() {
            Some("VERTEX_SE2") => {
                let vals: Vec<f64> = it.skip(1).map(|v| v.parse().unwrap()).collect();
                let [x, y, theta] = vals[..] else { panic!("bad vertex line: {line}") };
                nodes.push(graph.add_node(Pose2::new(x, y, theta)));
            }
            Some("EDGE_SE2") => {
                let from: usize = it.next().unwrap().parse().unwrap();
                let to: usize = it.next().unwrap().parse().unwrap();
                let vals: Vec<f64> = it.map(|v| v.parse().unwrap()).collect();
                let [dx, dy, dtheta, i11, i12, i13, i22, i23, i33] = vals[..] else {
                    panic!("bad edge line: {line}")
                };
                let info = Matrix3::new(i11, i12, i13, i12, i22, i23, i13, i23, i33);
                graph.add_edge(nodes[from], nodes[to], Pose2::new(dx, dy, dtheta), info);
            }
            _ => {}
        }
    }
    graph
}

#[test]
fn m3500_converges_to_published_objective() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/M3500.g2o");
    let mut graph = load_g2o(&path);
    assert_eq!(graph.node_count(), 3500);
    assert_eq!(graph.edges().len(), 5453);

    let initial = graph.error().unwrap();
    let final_cost = graph.optimize(100).unwrap();
    println!("M3500: initial objective {initial:.1}, optimized {final_cost:.3}");

    // The raw odometry initialization is wildly inconsistent; optimization
    // must reduce the objective by orders of magnitude.
    assert!(final_cost < initial / 100.0, "{final_cost} vs initial {initial}");
    // The published converged χ² for M3500 is ≈ 138 (GTSAM, g2o). factrs uses
    // the 0.5·Σ rᵀΩr convention, so the expected objective is ≈ 69. We land
    // at 68.96 — matching the published optimum.
    assert!(
        (65.0..75.0).contains(&final_cost),
        "objective {final_cost} does not match the published M3500 optimum (~69 in ½χ²)"
    );
}
