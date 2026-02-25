// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

//! Benchmarks for 3D spatial hash DBSCAN clustering.
//!
//! Measures:
//! - Spatial hash build time
//! - Full DBSCAN clustering at various point cloud sizes
//! - Warmup behavior (buffer reuse after first frame)
//!
//! Run with: cargo bench --bench cluster_bench
//!
//! For on-target profiling, cross-compile and run:
//!   cargo bench --bench cluster_bench --target aarch64-unknown-linux-gnu

use criterion::{BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use edgefirst_lidarpub::cluster::{ClusterData, cluster_};

/// Generate a synthetic point cloud with clustered structure.
///
/// Creates `n_clusters` spherical clusters of `points_per_cluster` points each,
/// spread across a 100m × 100m × 10m volume (typical outdoor LiDAR scene).
/// Remaining points up to `total` are set to origin (invalid returns).
fn generate_scene(total: usize, n_clusters: usize, points_per_cluster: usize) -> PointCloud {
    let mut x = vec![0.0f32; total];
    let mut y = vec![0.0f32; total];
    let mut z = vec![0.0f32; total];

    let mut idx = 0;
    for c in 0..n_clusters {
        // Spread cluster centers across the scene
        let cx = (c as f32 / n_clusters as f32) * 100.0 - 50.0;
        let cy = ((c * 7) as f32 % 100.0) - 50.0;
        let cz = ((c * 3) as f32 % 10.0) - 5.0;

        for p in 0..points_per_cluster {
            if idx >= total {
                break;
            }
            // Distribute points within ~0.2m radius of cluster center
            let angle = p as f32 * 2.399; // golden angle
            let r = 0.05 * (p as f32 / points_per_cluster as f32).sqrt();
            x[idx] = cx + r * angle.cos();
            y[idx] = cy + r * angle.sin();
            z[idx] = cz + r * 0.3 * ((p as f32 * 1.7).sin());
            idx += 1;
        }
    }
    // Remaining points stay at origin (invalid returns)

    PointCloud { x, y, z }
}

struct PointCloud {
    x: Vec<f32>,
    y: Vec<f32>,
    z: Vec<f32>,
}

/// Benchmark full DBSCAN at different point cloud sizes.
fn bench_cluster_sizes(c: &mut Criterion) {
    let mut group = c.benchmark_group("dbscan_full");

    // Sizes representing: small scene, E1R typical, Ouster 512x10, Ouster 1024x10
    for &n_points in &[10_000, 26_000, 32_768, 65_536] {
        // ~20% of points in clusters, rest are invalid returns (origin)
        let n_clusters = 30;
        let pts_per_cluster = n_points / 5 / n_clusters;
        let scene = generate_scene(n_points, n_clusters, pts_per_cluster);

        group.throughput(Throughput::Elements(n_points as u64));
        group.bench_with_input(
            BenchmarkId::new("points", n_points),
            &scene,
            |b, scene| {
                let mut data = ClusterData::new(0.256, 4);
                // Warmup: one call to allocate buffers
                let _ = cluster_(&mut data, &scene.x, &scene.y, &scene.z);

                b.iter(|| cluster_(&mut data, &scene.x, &scene.y, &scene.z));
            },
        );
    }

    group.finish();
}

/// Benchmark the worst case: all points valid and densely packed (single cluster).
fn bench_dense_cluster(c: &mut Criterion) {
    let mut group = c.benchmark_group("dbscan_dense");

    for &n_points in &[1_000, 5_000, 10_000] {
        // All points in a tight ball — worst case for neighbor queries
        let mut x = Vec::with_capacity(n_points);
        let mut y = Vec::with_capacity(n_points);
        let mut z = Vec::with_capacity(n_points);

        for i in 0..n_points {
            let angle = i as f32 * 2.399;
            let r = 0.1 * (i as f32 / n_points as f32).sqrt();
            x.push(r * angle.cos());
            y.push(r * angle.sin());
            z.push(0.01 * (i as f32 * 0.1).sin());
        }

        group.throughput(Throughput::Elements(n_points as u64));
        group.bench_with_input(
            BenchmarkId::new("dense", n_points),
            &(x.clone(), y.clone(), z.clone()),
            |b, (x, y, z)| {
                let mut data = ClusterData::new(0.256, 4);
                let _ = cluster_(&mut data, x, y, z);
                b.iter(|| cluster_(&mut data, x, y, z));
            },
        );
    }

    group.finish();
}

/// Benchmark the best case: all origin points (all noise, no clustering work).
fn bench_all_noise(c: &mut Criterion) {
    let mut group = c.benchmark_group("dbscan_noise");

    for &n_points in &[26_000, 65_536] {
        let x = vec![0.0f32; n_points];
        let y = vec![0.0f32; n_points];
        let z = vec![0.0f32; n_points];

        group.throughput(Throughput::Elements(n_points as u64));
        group.bench_with_input(
            BenchmarkId::new("noise", n_points),
            &(x.clone(), y.clone(), z.clone()),
            |b, (x, y, z)| {
                let mut data = ClusterData::new(0.256, 4);
                let _ = cluster_(&mut data, x, y, z);
                b.iter(|| cluster_(&mut data, x, y, z));
            },
        );
    }

    group.finish();
}

/// Benchmark warmup vs steady-state to verify zero-allocation claim.
fn bench_warmup(c: &mut Criterion) {
    let mut group = c.benchmark_group("dbscan_warmup");

    let n_points = 26_000;
    let scene = generate_scene(n_points, 30, 170);

    // Cold start: fresh ClusterData each iteration
    group.throughput(Throughput::Elements(n_points as u64));
    group.bench_function("cold", |b| {
        b.iter(|| {
            let mut data = ClusterData::new(0.256, 4);
            cluster_(&mut data, &scene.x, &scene.y, &scene.z)
        });
    });

    // Warm: reuse ClusterData across iterations
    group.bench_function("warm", |b| {
        let mut data = ClusterData::new(0.256, 4);
        let _ = cluster_(&mut data, &scene.x, &scene.y, &scene.z);
        b.iter(|| cluster_(&mut data, &scene.x, &scene.y, &scene.z));
    });

    group.finish();
}

criterion_group!(
    benches,
    bench_cluster_sizes,
    bench_dense_cluster,
    bench_all_noise,
    bench_warmup,
);
criterion_main!(benches);
