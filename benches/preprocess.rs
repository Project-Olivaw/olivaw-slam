//! Criterion benchmark for the preprocessing pipeline.
//!
//! The pipeline runs at sensor rate (~10 Hz × ~450 points on a SLAMTEC C1);
//! a regression here is a bug.

// The criterion_group! macro expands to an undocumented function; benches are
// not library code, so the missing-docs rule does not apply here.
#![allow(missing_docs)]

use criterion::{Criterion, black_box, criterion_group, criterion_main};
use olivaw_slam::preprocess::{PreprocessConfig, Preprocessor};
use olivaw_slam::{Point2, ScanCloud};

/// Synthetic single-revolution scan: a noisy ring at ~3 m, C1-like density.
/// Deterministic (no RNG) so runs are comparable.
fn synthetic_scan(points: usize) -> ScanCloud {
    let cloud: Vec<Point2> = (0..points)
        .map(|i| {
            let angle = f64::from(u32::try_from(i).unwrap()) / f64::from(u32::try_from(points).unwrap())
                * std::f64::consts::TAU;
            // Deterministic pseudo-noise from a fast-oscillating sine.
            let r = 3.0 + 0.02 * (angle * 57.0).sin();
            Point2::new(r * angle.cos(), r * angle.sin())
        })
        .collect();
    ScanCloud::new(cloud, 0)
}

fn bench_process(c: &mut Criterion) {
    let input = synthetic_scan(450);
    let mut pre = Preprocessor::new(PreprocessConfig::default()).unwrap();
    let mut output = ScanCloud::default();

    c.bench_function("preprocess_450pt_scan", |b| {
        b.iter(|| {
            pre.process(black_box(&input), &mut output).unwrap();
            black_box(&output);
        });
    });
}

criterion_group!(benches, bench_process);
criterion_main!(benches);
