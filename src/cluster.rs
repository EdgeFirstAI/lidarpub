// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

//! 3D Spatial Hash DBSCAN clustering for point clouds.
//!
//! Divides 3D space into voxels of size `eps`, hashes each point into its voxel,
//! and queries the 27 adjacent voxels (3×3×3) for neighbor searches. This gives
//! O(n) average performance for uniform distributions.

use std::collections::HashMap;

/// 3D spatial hash for O(1) average neighbor queries.
///
/// Points are binned into voxels of size `1/inv_cell_size`. Neighbor queries
/// check the 27 adjacent voxels (3×3×3) and filter by squared Euclidean
/// distance.
struct SpatialHash {
    cells: HashMap<(i32, i32, i32), Vec<usize>>,
    inv_cell_size: f32,
}

impl SpatialHash {
    fn new(cell_size: f32) -> Self {
        Self {
            cells: HashMap::new(),
            inv_cell_size: 1.0 / cell_size,
        }
    }

    fn clear(&mut self) {
        for v in self.cells.values_mut() {
            v.clear();
        }
    }

    fn voxel_key(&self, x: f32, y: f32, z: f32) -> (i32, i32, i32) {
        (
            (x * self.inv_cell_size).floor() as i32,
            (y * self.inv_cell_size).floor() as i32,
            (z * self.inv_cell_size).floor() as i32,
        )
    }

    fn insert(&mut self, idx: usize, x: f32, y: f32, z: f32) {
        let key = self.voxel_key(x, y, z);
        self.cells.entry(key).or_default().push(idx);
    }

    #[allow(clippy::too_many_arguments)]
    fn query_neighbors(
        &self,
        qx: f32,
        qy: f32,
        qz: f32,
        x: &[f32],
        y: &[f32],
        z: &[f32],
        eps_sq: f32,
        neighbors: &mut Vec<usize>,
    ) {
        neighbors.clear();
        let (cx, cy, cz) = self.voxel_key(qx, qy, qz);
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let key = (cx + dx, cy + dy, cz + dz);
                    if let Some(indices) = self.cells.get(&key) {
                        for &idx in indices {
                            let ddx = x[idx] - qx;
                            let ddy = y[idx] - qy;
                            let ddz = z[idx] - qz;
                            let dist_sq = ddx * ddx + ddy * ddy + ddz * ddz;
                            if dist_sq < eps_sq {
                                neighbors.push(idx);
                            }
                        }
                    }
                }
            }
        }
    }
}

/// Reusable state for spatial hash DBSCAN clustering.
///
/// All internal buffers are retained between calls to avoid allocation after
/// the first frame (warmup).
pub struct ClusterData {
    cluster_ids: Vec<u32>,
    eps_sq: f32,
    min_pts: usize,
    spatial_hash: SpatialHash,
    queue: Vec<usize>,
    neighbors: Vec<usize>,
}

impl ClusterData {
    /// Create a new `ClusterData` with the given epsilon (meters) and minimum
    /// points per cluster.
    pub fn new(eps_m: f32, min_pts: usize) -> Self {
        Self {
            cluster_ids: Vec::new(),
            eps_sq: eps_m * eps_m,
            min_pts,
            spatial_hash: SpatialHash::new(eps_m),
            queue: Vec::new(),
            neighbors: Vec::new(),
        }
    }
}

const VISITED: u32 = u32::MAX;
const UNVISITED: u32 = 0;

/// Run DBSCAN clustering on the given point cloud.
///
/// Returns a `Vec<u32>` where 0 = noise and non-zero values are cluster IDs.
/// Origin points (0,0,0) are treated as invalid sensor returns and marked as
/// noise.
pub fn cluster_(data: &mut ClusterData, x: &[f32], y: &[f32], z: &[f32]) -> Vec<u32> {
    let n = x.len();

    // Reset cluster IDs
    data.cluster_ids.clear();
    data.cluster_ids.resize(n, UNVISITED);

    // Build spatial hash — skip origin points (invalid sensor returns)
    data.spatial_hash.clear();
    for i in 0..n {
        if x[i] == 0.0 && y[i] == 0.0 && z[i] == 0.0 {
            data.cluster_ids[i] = VISITED; // mark as visited (noise)
            continue;
        }
        data.spatial_hash.insert(i, x[i], y[i], z[i]);
    }

    let mut id: u32 = 0;
    for i in 0..n {
        if data.cluster_ids[i] != UNVISITED {
            continue;
        }

        // Query neighbors of point i
        data.spatial_hash.query_neighbors(
            x[i],
            y[i],
            z[i],
            x,
            y,
            z,
            data.eps_sq,
            &mut data.neighbors,
        );

        if data.neighbors.len() < data.min_pts {
            data.cluster_ids[i] = VISITED; // noise
            continue;
        }

        // New cluster
        id += 1;
        expand_cluster(data, i, id, x, y, z);
    }

    // Convert VISITED (noise) markers back to 0
    for cid in &mut data.cluster_ids {
        if *cid == VISITED {
            *cid = 0;
        }
    }

    let mut result = Vec::new();
    std::mem::swap(&mut result, &mut data.cluster_ids);
    result
}

fn expand_cluster(
    data: &mut ClusterData,
    seed: usize,
    id: u32,
    x: &[f32],
    y: &[f32],
    z: &[f32],
) {
    data.cluster_ids[seed] = id;

    // Seed the queue from the current neighbors buffer
    data.queue.clear();
    for &ni in &data.neighbors {
        if data.cluster_ids[ni] == UNVISITED {
            data.cluster_ids[ni] = id;
            data.queue.push(ni);
        } else if data.cluster_ids[ni] == VISITED {
            // noise point absorbed into cluster as border point
            data.cluster_ids[ni] = id;
        }
    }

    // BFS expansion using index cursor to avoid borrow conflicts
    let mut qi = 0;
    while qi < data.queue.len() {
        let pt = data.queue[qi];
        qi += 1;

        data.spatial_hash.query_neighbors(
            x[pt],
            y[pt],
            z[pt],
            x,
            y,
            z,
            data.eps_sq,
            &mut data.neighbors,
        );

        if data.neighbors.len() < data.min_pts {
            continue;
        }

        for &ni in &data.neighbors {
            if data.cluster_ids[ni] == UNVISITED {
                data.cluster_ids[ni] = id;
                data.queue.push(ni);
            } else if data.cluster_ids[ni] == VISITED {
                data.cluster_ids[ni] = id;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{ClusterData, cluster_};

    #[test]
    fn test_cluster_spatial() {
        // Two clusters in 3D space + one noise point
        // Cluster A: 3 points near (1.0, 0.0, 0.0)
        // Cluster B: 3 points near (5.0, 0.0, 0.0)
        // Noise: origin (0,0,0) is treated as invalid
        let x = vec![1.0, 1.05, 1.1, 5.0, 5.05, 5.1, 0.0];
        let y = vec![0.0, 0.05, 0.0, 0.0, 0.05, 0.0, 0.0];
        let z = vec![0.0, 0.0, 0.05, 0.0, 0.0, 0.05, 0.0];

        let mut data = ClusterData::new(0.2, 2);

        let clusters = cluster_(&mut data, &x, &y, &z);

        // Origin point should be noise (0)
        assert_eq!(clusters[6], 0, "Origin point should be noise");

        // Cluster A points should share the same non-zero ID
        assert!(clusters[0] > 0, "Point 0 should be in a cluster");
        assert_eq!(clusters[0], clusters[1]);
        assert_eq!(clusters[0], clusters[2]);

        // Cluster B points should share the same non-zero ID
        assert!(clusters[3] > 0, "Point 3 should be in a cluster");
        assert_eq!(clusters[3], clusters[4]);
        assert_eq!(clusters[3], clusters[5]);

        // The two clusters should have different IDs
        assert_ne!(
            clusters[0], clusters[3],
            "Cluster A and B should have different IDs"
        );
    }

    #[test]
    fn test_cluster_25k_spatial() {
        // All-zero points = all invalid = all noise
        let n = 1024 * 24;
        let x = vec![0.0f32; n];
        let y = vec![0.0f32; n];
        let z = vec![0.0f32; n];

        let mut data = ClusterData::new(0.256, 4);

        let start = std::time::Instant::now();
        let clusters = cluster_(&mut data, &x, &y, &z);
        let elapsed = start.elapsed();
        println!("Clustering 25k all-zero points took: {:?}", elapsed);

        for c in clusters {
            assert_eq!(c, 0, "All-zero points should be noise (cluster ID 0)");
        }
    }

    #[test]
    fn test_cluster_unstructured() {
        // Simulate E1R-like variable-count unstructured point cloud
        // 5 points forming a cluster + 3 scattered noise points
        let x = vec![2.0, 2.01, 2.02, 2.0, 2.01, 10.0, 20.0, 30.0];
        let y = vec![3.0, 3.01, 3.0, 3.02, 3.01, 10.0, 20.0, 30.0];
        let z = vec![1.0, 1.0, 1.01, 1.0, 1.01, 10.0, 20.0, 30.0];

        let mut data = ClusterData::new(0.1, 3);

        let clusters = cluster_(&mut data, &x, &y, &z);

        // First 5 points should form one cluster
        assert!(clusters[0] > 0, "Dense points should be clustered");
        for i in 1..5 {
            assert_eq!(
                clusters[0], clusters[i],
                "Points 0..5 should be in same cluster"
            );
        }

        // Scattered points should be noise
        for i in 5..8 {
            assert_eq!(clusters[i], 0, "Scattered point {} should be noise", i);
        }
    }
}
