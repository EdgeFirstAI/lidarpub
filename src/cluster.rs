// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

//! 3D Spatial Hash DBSCAN clustering for point clouds.
//!
//! Divides 3D space into voxels of size `eps`, hashes each point into its voxel,
//! and queries the 27 adjacent voxels (3×3×3) for neighbor searches. This gives
//! O(n) average performance for uniform distributions.
//!
//! Two spatial hash implementations are provided:
//! - `SpatialHash` — HashMap-based (default, backward compatible)
//! - `FlatSpatialHash` — Open-addressing with contiguous point storage
//!
//! On aarch64, neighbor distance checks use NEON SIMD intrinsics to process
//! 4 points per iteration.

use std::collections::HashMap;

#[cfg(target_arch = "aarch64")]
use std::arch::aarch64::*;

// ── Section 1: SpatialHash (HashMap-based) ──────────────────────────────────

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
                        #[cfg(target_arch = "aarch64")]
                        Self::query_neighbors_neon(
                            indices, qx, qy, qz, x, y, z, eps_sq, neighbors,
                        );
                        #[cfg(not(target_arch = "aarch64"))]
                        Self::query_neighbors_scalar(
                            indices, qx, qy, qz, x, y, z, eps_sq, neighbors,
                        );
                    }
                }
            }
        }
    }

    #[cfg(not(target_arch = "aarch64"))]
    #[allow(clippy::too_many_arguments)]
    fn query_neighbors_scalar(
        indices: &[usize],
        qx: f32,
        qy: f32,
        qz: f32,
        x: &[f32],
        y: &[f32],
        z: &[f32],
        eps_sq: f32,
        neighbors: &mut Vec<usize>,
    ) {
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

    /// NEON-vectorized distance check: processes 4 points per iteration.
    ///
    /// Gathers scattered point coordinates into stack arrays, loads them into
    /// NEON registers, and uses fused multiply-add for distance computation.
    #[cfg(target_arch = "aarch64")]
    #[allow(clippy::too_many_arguments)]
    fn query_neighbors_neon(
        indices: &[usize],
        qx: f32,
        qy: f32,
        qz: f32,
        x: &[f32],
        y: &[f32],
        z: &[f32],
        eps_sq: f32,
        neighbors: &mut Vec<usize>,
    ) {
        let len = indices.len();
        let n_simd = len - len % 4;

        // SAFETY: NEON intrinsics are always available on aarch64.
        // All index accesses are bounded by the indices slice length and
        // the x/y/z coordinate arrays (caller guarantees indices are valid).
        unsafe {
            let qx_v = vdupq_n_f32(qx);
            let qy_v = vdupq_n_f32(qy);
            let qz_v = vdupq_n_f32(qz);
            let eps_sq_v = vdupq_n_f32(eps_sq);

            for i in (0..n_simd).step_by(4) {
                let i0 = indices[i];
                let i1 = indices[i + 1];
                let i2 = indices[i + 2];
                let i3 = indices[i + 3];

                // Gather coordinates into stack arrays and load into NEON
                let xbuf: [f32; 4] = [x[i0], x[i1], x[i2], x[i3]];
                let ybuf: [f32; 4] = [y[i0], y[i1], y[i2], y[i3]];
                let zbuf: [f32; 4] = [z[i0], z[i1], z[i2], z[i3]];

                let xv = vld1q_f32(xbuf.as_ptr());
                let yv = vld1q_f32(ybuf.as_ptr());
                let zv = vld1q_f32(zbuf.as_ptr());

                // Vectorized distance: dx*dx + dy*dy + dz*dz
                let ddx = vsubq_f32(xv, qx_v);
                let ddy = vsubq_f32(yv, qy_v);
                let ddz = vsubq_f32(zv, qz_v);
                let dist_sq =
                    vfmaq_f32(vfmaq_f32(vmulq_f32(ddx, ddx), ddy, ddy), ddz, ddz);

                // Compare: dist_sq < eps_sq → 0xFFFFFFFF per lane
                let mask = vcltq_f32(dist_sq, eps_sq_v);
                let mask_u32 = vreinterpretq_u32_f32(vreinterpretq_f32_u32(mask));

                if vgetq_lane_u32::<0>(mask_u32) != 0 {
                    neighbors.push(i0);
                }
                if vgetq_lane_u32::<1>(mask_u32) != 0 {
                    neighbors.push(i1);
                }
                if vgetq_lane_u32::<2>(mask_u32) != 0 {
                    neighbors.push(i2);
                }
                if vgetq_lane_u32::<3>(mask_u32) != 0 {
                    neighbors.push(i3);
                }
            }
        }

        // Scalar remainder for last 1-3 points
        for &idx in &indices[n_simd..] {
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

// ── Section 2: FlatSpatialHash (open-addressing, contiguous storage) ────────

const SENTINEL: (i32, i32, i32) = (i32::MIN, i32::MIN, i32::MIN);
const INITIAL_TABLE_SIZE: usize = 4096;
/// Maximum load factor before table must grow. At 0.75, linear probing
/// averages ~2.5 probes per lookup — beyond this, probe chains degrade fast.
const MAX_LOAD_FACTOR: f32 = 0.75;

/// Flat-array spatial hash with open-addressing and contiguous point storage.
///
/// Eliminates HashMap's SipHash overhead and pointer chasing by storing voxel
/// keys in an open-addressed table and point indices in a contiguous array
/// (grouped by voxel). Built in two passes: count, then scatter.
struct FlatSpatialHash {
    /// Voxel key per slot (SENTINEL = empty).
    bucket_keys: Vec<(i32, i32, i32)>,
    /// Start index into `point_indices` for each bucket.
    bucket_offsets: Vec<u32>,
    /// Number of points in each bucket.
    bucket_counts: Vec<u32>,
    /// Write cursor per bucket (used during scatter pass, then reset).
    bucket_cursors: Vec<u32>,
    /// Contiguous per-voxel point indices.
    point_indices: Vec<u32>,
    /// `table_size - 1` for fast modulo.
    mask: u32,
    inv_cell_size: f32,
    /// Indices of occupied buckets for fast clear.
    dirty_buckets: Vec<u32>,
    /// Reverse map: table slot → index into `dirty_buckets` (u32::MAX = empty).
    slot_to_dirty: Vec<u32>,
    /// Current table capacity (always a power of 2).
    table_size: usize,
}

const SLOT_EMPTY: u32 = u32::MAX;

impl FlatSpatialHash {
    fn new(cell_size: f32) -> Self {
        Self {
            bucket_keys: vec![SENTINEL; INITIAL_TABLE_SIZE],
            bucket_offsets: vec![0; INITIAL_TABLE_SIZE],
            bucket_counts: vec![0; INITIAL_TABLE_SIZE],
            bucket_cursors: vec![0; INITIAL_TABLE_SIZE],
            point_indices: Vec::new(),
            mask: (INITIAL_TABLE_SIZE - 1) as u32,
            inv_cell_size: 1.0 / cell_size,
            dirty_buckets: Vec::new(),
            slot_to_dirty: vec![SLOT_EMPTY; INITIAL_TABLE_SIZE],
            table_size: INITIAL_TABLE_SIZE,
        }
    }

    fn clear(&mut self) {
        // Only reset occupied buckets — O(occupied) not O(table_size)
        for &slot in &self.dirty_buckets {
            let s = slot as usize;
            self.bucket_keys[s] = SENTINEL;
            self.bucket_counts[s] = 0;
            self.bucket_cursors[s] = 0;
            self.slot_to_dirty[s] = SLOT_EMPTY;
        }
        self.dirty_buckets.clear();
    }

    /// Ensure the table is large enough for `n_points` without needing to
    /// grow during build. Worst case: every point is in a unique voxel.
    fn ensure_capacity(&mut self, n_points: usize) {
        let required = ((n_points as f32 / MAX_LOAD_FACTOR) as usize).next_power_of_two();
        if required > self.table_size {
            self.resize_table(required);
        }
    }

    /// Resize the table to `new_size` (must be a power of 2). Rehashes all
    /// occupied buckets into fresh arrays.
    fn resize_table(&mut self, new_size: usize) {
        debug_assert!(new_size.is_power_of_two());
        let old_keys = std::mem::replace(&mut self.bucket_keys, vec![SENTINEL; new_size]);
        let old_counts = std::mem::replace(&mut self.bucket_counts, vec![0; new_size]);
        self.bucket_offsets = vec![0; new_size];
        self.bucket_cursors = vec![0; new_size];
        self.slot_to_dirty = vec![SLOT_EMPTY; new_size];
        self.mask = (new_size - 1) as u32;
        self.table_size = new_size;

        let old_dirty = std::mem::take(&mut self.dirty_buckets);
        for &old_slot in &old_dirty {
            let os = old_slot as usize;
            let key = old_keys[os];
            if key == SENTINEL {
                continue;
            }
            let new_slot = self.insert_slot_unchecked(key);
            self.bucket_counts[new_slot] = old_counts[os];
        }
    }

    /// Insert a key into the table without checking load factor.
    /// Used by `resize_table` and `find_or_insert_slot` after the check.
    #[inline(always)]
    fn insert_slot_unchecked(&mut self, key: (i32, i32, i32)) -> usize {
        let mut slot = self.hash_key(key) as usize;
        loop {
            if self.bucket_keys[slot] == key {
                return slot;
            }
            if self.bucket_keys[slot] == SENTINEL {
                self.bucket_keys[slot] = key;
                let dirty_idx = self.dirty_buckets.len() as u32;
                self.dirty_buckets.push(slot as u32);
                self.slot_to_dirty[slot] = dirty_idx;
                return slot;
            }
            slot = (slot + 1) & (self.mask as usize);
        }
    }

    fn voxel_key(&self, x: f32, y: f32, z: f32) -> (i32, i32, i32) {
        (
            (x * self.inv_cell_size).floor() as i32,
            (y * self.inv_cell_size).floor() as i32,
            (z * self.inv_cell_size).floor() as i32,
        )
    }

    /// FxHash-style hash for voxel keys: wrapping multiply + XOR fold.
    #[inline(always)]
    fn hash_key(&self, key: (i32, i32, i32)) -> u32 {
        let mut h = (key.0 as u32).wrapping_mul(0x9e37_79b9);
        h ^= (key.1 as u32).wrapping_mul(0x517c_c1b7);
        h ^= (key.2 as u32).wrapping_mul(0x6a09_e667);
        h & self.mask
    }

    /// Find or insert a slot for `key` via linear probing. Returns the slot index.
    /// Grows the table if load factor exceeds MAX_LOAD_FACTOR.
    fn find_or_insert_slot(&mut self, key: (i32, i32, i32)) -> usize {
        // Check if we need to grow before inserting
        if (self.dirty_buckets.len() + 1) as f32 > self.table_size as f32 * MAX_LOAD_FACTOR {
            self.resize_table(self.table_size * 2);
        }
        self.insert_slot_unchecked(key)
    }

    /// Find a slot for `key` (read-only lookup). Returns `None` if not found.
    #[inline(always)]
    fn find_slot(&self, key: (i32, i32, i32)) -> Option<usize> {
        let mut slot = self.hash_key(key) as usize;
        loop {
            if self.bucket_keys[slot] == key {
                return Some(slot);
            }
            if self.bucket_keys[slot] == SENTINEL {
                return None;
            }
            slot = (slot + 1) & (self.mask as usize);
        }
    }

    /// Two-pass build: count points per voxel, compute offsets, scatter indices.
    fn build(&mut self, x: &[f32], y: &[f32], z: &[f32], valid: &[bool]) {
        let n = x.len();

        // Pre-size table to avoid growth during insert
        self.ensure_capacity(n);

        // Pass 1: Count points per voxel
        for i in 0..n {
            if !valid[i] {
                continue;
            }
            let key = self.voxel_key(x[i], y[i], z[i]);
            let slot = self.find_or_insert_slot(key);
            self.bucket_counts[slot] += 1;
        }

        // Prefix sum to compute offsets
        let mut total: u32 = 0;
        for &slot in &self.dirty_buckets {
            let s = slot as usize;
            self.bucket_offsets[s] = total;
            self.bucket_cursors[s] = total;
            total += self.bucket_counts[s];
        }

        // Allocate contiguous storage
        self.point_indices.resize(total as usize, 0);

        // Pass 2: Scatter point indices into contiguous ranges
        for i in 0..n {
            if !valid[i] {
                continue;
            }
            let key = self.voxel_key(x[i], y[i], z[i]);
            // Slot is guaranteed to exist from pass 1
            let slot = self.find_slot(key).unwrap();
            let cursor = self.bucket_cursors[slot] as usize;
            self.point_indices[cursor] = i as u32;
            self.bucket_cursors[slot] += 1;
        }
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
                    if let Some(slot) = self.find_slot(key) {
                        let offset = self.bucket_offsets[slot] as usize;
                        let count = self.bucket_counts[slot] as usize;
                        let indices = &self.point_indices[offset..offset + count];

                        #[cfg(target_arch = "aarch64")]
                        Self::query_neighbors_neon(
                            indices, qx, qy, qz, x, y, z, eps_sq, neighbors,
                        );
                        #[cfg(not(target_arch = "aarch64"))]
                        Self::query_neighbors_scalar(
                            indices, qx, qy, qz, x, y, z, eps_sq, neighbors,
                        );
                    }
                }
            }
        }
    }

    #[cfg(not(target_arch = "aarch64"))]
    #[allow(clippy::too_many_arguments)]
    fn query_neighbors_scalar(
        indices: &[u32],
        qx: f32,
        qy: f32,
        qz: f32,
        x: &[f32],
        y: &[f32],
        z: &[f32],
        eps_sq: f32,
        neighbors: &mut Vec<usize>,
    ) {
        for &idx_u32 in indices {
            let idx = idx_u32 as usize;
            let ddx = x[idx] - qx;
            let ddy = y[idx] - qy;
            let ddz = z[idx] - qz;
            let dist_sq = ddx * ddx + ddy * ddy + ddz * ddz;
            if dist_sq < eps_sq {
                neighbors.push(idx);
            }
        }
    }

    /// NEON-vectorized distance check for contiguous u32 index slices.
    #[cfg(target_arch = "aarch64")]
    #[allow(clippy::too_many_arguments)]
    fn query_neighbors_neon(
        indices: &[u32],
        qx: f32,
        qy: f32,
        qz: f32,
        x: &[f32],
        y: &[f32],
        z: &[f32],
        eps_sq: f32,
        neighbors: &mut Vec<usize>,
    ) {
        let len = indices.len();
        let n_simd = len - len % 4;

        // SAFETY: NEON intrinsics are always available on aarch64.
        // All index accesses are bounded by the indices slice length and
        // the x/y/z coordinate arrays (caller guarantees indices are valid).
        unsafe {
            let qx_v = vdupq_n_f32(qx);
            let qy_v = vdupq_n_f32(qy);
            let qz_v = vdupq_n_f32(qz);
            let eps_sq_v = vdupq_n_f32(eps_sq);

            for i in (0..n_simd).step_by(4) {
                let i0 = indices[i] as usize;
                let i1 = indices[i + 1] as usize;
                let i2 = indices[i + 2] as usize;
                let i3 = indices[i + 3] as usize;

                // Gather coordinates into stack arrays and load into NEON
                let xbuf: [f32; 4] = [x[i0], x[i1], x[i2], x[i3]];
                let ybuf: [f32; 4] = [y[i0], y[i1], y[i2], y[i3]];
                let zbuf: [f32; 4] = [z[i0], z[i1], z[i2], z[i3]];

                let xv = vld1q_f32(xbuf.as_ptr());
                let yv = vld1q_f32(ybuf.as_ptr());
                let zv = vld1q_f32(zbuf.as_ptr());

                // Vectorized distance: dx*dx + dy*dy + dz*dz
                let ddx = vsubq_f32(xv, qx_v);
                let ddy = vsubq_f32(yv, qy_v);
                let ddz = vsubq_f32(zv, qz_v);
                let dist_sq =
                    vfmaq_f32(vfmaq_f32(vmulq_f32(ddx, ddx), ddy, ddy), ddz, ddz);

                // Compare: dist_sq < eps_sq → 0xFFFFFFFF per lane
                let mask = vcltq_f32(dist_sq, eps_sq_v);
                let mask_u32 = vreinterpretq_u32_f32(vreinterpretq_f32_u32(mask));

                if vgetq_lane_u32::<0>(mask_u32) != 0 {
                    neighbors.push(i0);
                }
                if vgetq_lane_u32::<1>(mask_u32) != 0 {
                    neighbors.push(i1);
                }
                if vgetq_lane_u32::<2>(mask_u32) != 0 {
                    neighbors.push(i2);
                }
                if vgetq_lane_u32::<3>(mask_u32) != 0 {
                    neighbors.push(i3);
                }
            }
        }

        // Scalar remainder for last 1-3 points
        for &idx_u32 in &indices[n_simd..] {
            let idx = idx_u32 as usize;
            let ddx = x[idx] - qx;
            let ddy = y[idx] - qy;
            let ddz = z[idx] - qz;
            let dist_sq = ddx * ddx + ddy * ddy + ddz * ddz;
            if dist_sq < eps_sq {
                neighbors.push(idx);
            }
        }
    }

    /// Number of occupied voxel buckets.
    fn occupied_count(&self) -> usize {
        self.dirty_buckets.len()
    }

    /// Slot index of the i-th occupied bucket.
    fn dirty_bucket_slot(&self, i: usize) -> usize {
        self.dirty_buckets[i] as usize
    }

    /// Voxel key at the given table slot.
    fn key_at_slot(&self, slot: usize) -> (i32, i32, i32) {
        self.bucket_keys[slot]
    }

    /// Number of points in the given table slot.
    fn count_at_slot(&self, slot: usize) -> usize {
        self.bucket_counts[slot] as usize
    }

    /// Point indices for the given table slot.
    fn indices_at_slot(&self, slot: usize) -> &[u32] {
        let offset = self.bucket_offsets[slot] as usize;
        let count = self.bucket_counts[slot] as usize;
        &self.point_indices[offset..offset + count]
    }

    /// Reverse-lookup: find the dirty_bucket index for a given table slot. O(1).
    #[inline(always)]
    fn dirty_bucket_index(&self, slot: usize) -> Option<usize> {
        let idx = self.slot_to_dirty[slot];
        if idx == SLOT_EMPTY { None } else { Some(idx as usize) }
    }
}

// ── Section 3: SpatialHashKind enum dispatch ────────────────────────────────

enum SpatialHashKind {
    HashMap(SpatialHash),
    Flat(FlatSpatialHash),
}

impl SpatialHashKind {
    fn clear(&mut self) {
        match self {
            Self::HashMap(h) => h.clear(),
            Self::Flat(f) => f.clear(),
        }
    }

    fn insert_hashmap(&mut self, idx: usize, x: f32, y: f32, z: f32) {
        if let Self::HashMap(h) = self {
            h.insert(idx, x, y, z);
        }
    }

    fn build_flat(&mut self, x: &[f32], y: &[f32], z: &[f32], valid: &[bool]) {
        if let Self::Flat(f) = self {
            f.build(x, y, z, valid);
        }
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
        match self {
            Self::HashMap(h) => h.query_neighbors(qx, qy, qz, x, y, z, eps_sq, neighbors),
            Self::Flat(f) => f.query_neighbors(qx, qy, qz, x, y, z, eps_sq, neighbors),
        }
    }
}

// ── Section 4: ClusterData + cluster_ ───────────────────────────────────────

/// Reusable state for spatial hash DBSCAN clustering.
///
/// All internal buffers are retained between calls to avoid allocation after
/// the first frame (warmup).
pub struct ClusterData {
    cluster_ids: Vec<u32>,
    eps_sq: f32,
    min_pts: usize,
    spatial_hash: SpatialHashKind,
    queue: Vec<usize>,
    neighbors: Vec<usize>,
    valid: Vec<bool>,
}

impl ClusterData {
    /// Create a new `ClusterData` using HashMap-based spatial hash.
    pub fn new(eps_m: f32, min_pts: usize) -> Self {
        Self {
            cluster_ids: Vec::new(),
            eps_sq: eps_m * eps_m,
            min_pts,
            spatial_hash: SpatialHashKind::HashMap(SpatialHash::new(eps_m)),
            queue: Vec::new(),
            neighbors: Vec::new(),
            valid: Vec::new(),
        }
    }

    /// Create a new `ClusterData` using flat-array spatial hash.
    ///
    /// Uses open-addressing with contiguous point storage for better cache
    /// behavior and no SipHash overhead.
    pub fn new_flat(eps_m: f32, min_pts: usize) -> Self {
        Self {
            cluster_ids: Vec::new(),
            eps_sq: eps_m * eps_m,
            min_pts,
            spatial_hash: SpatialHashKind::Flat(FlatSpatialHash::new(eps_m)),
            queue: Vec::new(),
            neighbors: Vec::new(),
            valid: Vec::new(),
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

    match &mut data.spatial_hash {
        SpatialHashKind::HashMap(_) => {
            for i in 0..n {
                if x[i] == 0.0 && y[i] == 0.0 && z[i] == 0.0 {
                    data.cluster_ids[i] = VISITED;
                    continue;
                }
                data.spatial_hash.insert_hashmap(i, x[i], y[i], z[i]);
            }
        }
        SpatialHashKind::Flat(_) => {
            // Mark invalid points, then batch-build
            data.valid.clear();
            data.valid.resize(n, true);
            for i in 0..n {
                if x[i] == 0.0 && y[i] == 0.0 && z[i] == 0.0 {
                    data.cluster_ids[i] = VISITED;
                    data.valid[i] = false;
                }
            }
            data.spatial_hash.build_flat(x, y, z, &data.valid);
        }
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

// ── Section 4b: VoxelClusterData + voxel_cluster ────────────────────────────

/// Reusable state for voxel connected-component clustering.
///
/// All internal buffers are retained between calls to avoid allocation after
/// the first frame (warmup).
pub struct VoxelClusterData {
    flat_hash: FlatSpatialHash,
    cluster_ids: Vec<u32>,
    /// Per-dirty-bucket visited flag for BFS.
    voxel_visited: Vec<bool>,
    /// BFS queue storing dirty_bucket indices (not table slots).
    voxel_queue: Vec<usize>,
    min_pts: usize,
    valid: Vec<bool>,
}

impl VoxelClusterData {
    pub fn new(cell_size_m: f32, min_pts: usize) -> Self {
        Self {
            flat_hash: FlatSpatialHash::new(cell_size_m),
            cluster_ids: Vec::new(),
            voxel_visited: Vec::new(),
            voxel_queue: Vec::new(),
            min_pts,
            valid: Vec::new(),
        }
    }
}

/// Run voxel connected-component clustering on the given point cloud.
///
/// Returns a `Vec<u32>` where 0 = noise and non-zero values are cluster IDs.
/// Points are binned into voxels of size `cell_size_m`. Adjacent occupied
/// voxels (26-connected) with >= `min_pts` points are merged into clusters.
/// Origin points (0,0,0) are treated as invalid sensor returns and marked noise.
pub fn voxel_cluster(
    data: &mut VoxelClusterData,
    x: &[f32],
    y: &[f32],
    z: &[f32],
) -> Vec<u32> {
    let n = x.len();

    // Reset output
    data.cluster_ids.clear();
    data.cluster_ids.resize(n, 0);

    // Build spatial hash (marks invalid origin points)
    data.flat_hash.clear();
    data.valid.clear();
    data.valid.resize(n, true);
    for i in 0..n {
        if x[i] == 0.0 && y[i] == 0.0 && z[i] == 0.0 {
            data.valid[i] = false;
        }
    }
    data.flat_hash.build(x, y, z, &data.valid);

    let n_occupied = data.flat_hash.occupied_count();
    if n_occupied == 0 {
        return std::mem::take(&mut data.cluster_ids);
    }

    // Reset BFS state
    data.voxel_visited.clear();
    data.voxel_visited.resize(n_occupied, false);

    let mut cluster_id: u32 = 0;

    // BFS over voxels (not points)
    for start in 0..n_occupied {
        if data.voxel_visited[start] {
            continue;
        }

        let start_slot = data.flat_hash.dirty_bucket_slot(start);

        // Skip voxels below min_pts threshold
        if data.flat_hash.count_at_slot(start_slot) < data.min_pts {
            data.voxel_visited[start] = true;
            continue;
        }

        // New cluster — BFS through 26-connected neighbors
        cluster_id += 1;
        data.voxel_queue.clear();
        data.voxel_queue.push(start);
        data.voxel_visited[start] = true;

        // Label seed voxel's points
        for &pt in data.flat_hash.indices_at_slot(start_slot) {
            data.cluster_ids[pt as usize] = cluster_id;
        }

        let mut qi = 0;
        while qi < data.voxel_queue.len() {
            let current = data.voxel_queue[qi];
            qi += 1;

            let current_slot = data.flat_hash.dirty_bucket_slot(current);
            let (cx, cy, cz) = data.flat_hash.key_at_slot(current_slot);

            // Check 26-connected neighbors
            for dx in -1..=1 {
                for dy in -1..=1 {
                    for dz in -1..=1 {
                        if dx == 0 && dy == 0 && dz == 0 {
                            continue;
                        }
                        let neighbor_key = (cx + dx, cy + dy, cz + dz);
                        if let Some(neighbor_slot) = data.flat_hash.find_slot(neighbor_key) {
                            if let Some(nb_idx) = data.flat_hash.dirty_bucket_index(neighbor_slot) {
                                if !data.voxel_visited[nb_idx]
                                    && data.flat_hash.count_at_slot(neighbor_slot)
                                        >= data.min_pts
                                {
                                    data.voxel_visited[nb_idx] = true;
                                    data.voxel_queue.push(nb_idx);

                                    for &pt in data.flat_hash.indices_at_slot(neighbor_slot) {
                                        data.cluster_ids[pt as usize] = cluster_id;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    std::mem::take(&mut data.cluster_ids)
}

// ── Section 5: Tests ────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(clippy::needless_range_loop)]
mod tests {
    use super::{ClusterData, VoxelClusterData, cluster_, voxel_cluster};

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
    fn test_cluster_spatial_flat() {
        let x = vec![1.0, 1.05, 1.1, 5.0, 5.05, 5.1, 0.0];
        let y = vec![0.0, 0.05, 0.0, 0.0, 0.05, 0.0, 0.0];
        let z = vec![0.0, 0.0, 0.05, 0.0, 0.0, 0.05, 0.0];

        let mut data = ClusterData::new_flat(0.2, 2);

        let clusters = cluster_(&mut data, &x, &y, &z);

        assert_eq!(clusters[6], 0, "Origin point should be noise");
        assert!(clusters[0] > 0, "Point 0 should be in a cluster");
        assert_eq!(clusters[0], clusters[1]);
        assert_eq!(clusters[0], clusters[2]);
        assert!(clusters[3] > 0, "Point 3 should be in a cluster");
        assert_eq!(clusters[3], clusters[4]);
        assert_eq!(clusters[3], clusters[5]);
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
    fn test_cluster_25k_spatial_flat() {
        let n = 1024 * 24;
        let x = vec![0.0f32; n];
        let y = vec![0.0f32; n];
        let z = vec![0.0f32; n];

        let mut data = ClusterData::new_flat(0.256, 4);

        let clusters = cluster_(&mut data, &x, &y, &z);

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

    #[test]
    fn test_cluster_unstructured_flat() {
        let x = vec![2.0, 2.01, 2.02, 2.0, 2.01, 10.0, 20.0, 30.0];
        let y = vec![3.0, 3.01, 3.0, 3.02, 3.01, 10.0, 20.0, 30.0];
        let z = vec![1.0, 1.0, 1.01, 1.0, 1.01, 10.0, 20.0, 30.0];

        let mut data = ClusterData::new_flat(0.1, 3);

        let clusters = cluster_(&mut data, &x, &y, &z);

        assert!(clusters[0] > 0, "Dense points should be clustered");
        for i in 1..5 {
            assert_eq!(
                clusters[0], clusters[i],
                "Points 0..5 should be in same cluster"
            );
        }
        for i in 5..8 {
            assert_eq!(clusters[i], 0, "Scattered point {} should be noise", i);
        }
    }

    #[test]
    fn test_cluster_hashmap_vs_flat() {
        // Verify both implementations produce identical cluster IDs on same input
        let x = vec![
            1.0, 1.05, 1.1, 1.02, 1.08, // cluster A
            5.0, 5.05, 5.1, 5.02, 5.08, // cluster B
            0.0, 20.0, 30.0, // noise + origin
        ];
        let y = vec![
            0.0, 0.05, 0.0, 0.02, 0.03, 0.0, 0.05, 0.0, 0.02, 0.03, 0.0, 20.0, 30.0,
        ];
        let z = vec![
            0.0, 0.0, 0.05, 0.01, 0.02, 0.0, 0.0, 0.05, 0.01, 0.02, 0.0, 20.0, 30.0,
        ];

        let mut data_hashmap = ClusterData::new(0.2, 3);
        let mut data_flat = ClusterData::new_flat(0.2, 3);

        let clusters_hashmap = cluster_(&mut data_hashmap, &x, &y, &z);
        let clusters_flat = cluster_(&mut data_flat, &x, &y, &z);

        assert_eq!(
            clusters_hashmap.len(),
            clusters_flat.len(),
            "Both should produce same number of cluster IDs"
        );

        // Cluster IDs may differ in numbering but the groupings must match.
        // Check that points in the same cluster in hashmap are also in the
        // same cluster in flat, and vice versa.
        for i in 0..clusters_hashmap.len() {
            for j in (i + 1)..clusters_hashmap.len() {
                let same_hashmap = clusters_hashmap[i] == clusters_hashmap[j];
                let same_flat = clusters_flat[i] == clusters_flat[j];
                assert_eq!(
                    same_hashmap, same_flat,
                    "Points {} and {} grouping mismatch: hashmap={}, flat={}",
                    i, j, clusters_hashmap[i], clusters_flat[j]
                );
            }
        }

        // Noise points must be the same
        for i in 0..clusters_hashmap.len() {
            assert_eq!(
                clusters_hashmap[i] == 0,
                clusters_flat[i] == 0,
                "Point {} noise mismatch: hashmap={}, flat={}",
                i,
                clusters_hashmap[i],
                clusters_flat[i]
            );
        }
    }

    #[test]
    fn test_voxel_cluster_two_groups() {
        let x = vec![1.0, 1.05, 1.1, 5.0, 5.05, 5.1, 0.0];
        let y = vec![0.0, 0.05, 0.0, 0.0, 0.05, 0.0, 0.0];
        let z = vec![0.0, 0.0, 0.05, 0.0, 0.0, 0.05, 0.0];

        let mut data = VoxelClusterData::new(0.2, 2);
        let clusters = voxel_cluster(&mut data, &x, &y, &z);

        assert_eq!(clusters[6], 0, "Origin point should be noise");
        assert!(clusters[0] > 0, "Point 0 should be in a cluster");
        assert_eq!(clusters[0], clusters[1]);
        assert_eq!(clusters[0], clusters[2]);
        assert!(clusters[3] > 0, "Point 3 should be in a cluster");
        assert_eq!(clusters[3], clusters[4]);
        assert_eq!(clusters[3], clusters[5]);
        assert_ne!(clusters[0], clusters[3], "Two groups should differ");
    }

    #[test]
    fn test_voxel_cluster_all_noise() {
        let n = 1024 * 24;
        let x = vec![0.0f32; n];
        let y = vec![0.0f32; n];
        let z = vec![0.0f32; n];

        let mut data = VoxelClusterData::new(0.256, 4);
        let clusters = voxel_cluster(&mut data, &x, &y, &z);

        for c in clusters {
            assert_eq!(c, 0, "All-zero points should be noise");
        }
    }

    #[test]
    fn test_voxel_cluster_unstructured() {
        let x = vec![2.0, 2.01, 2.02, 2.0, 2.01, 10.0, 20.0, 30.0];
        let y = vec![3.0, 3.01, 3.0, 3.02, 3.01, 10.0, 20.0, 30.0];
        let z = vec![1.0, 1.0, 1.01, 1.0, 1.01, 10.0, 20.0, 30.0];

        let mut data = VoxelClusterData::new(0.1, 3);
        let clusters = voxel_cluster(&mut data, &x, &y, &z);

        assert!(clusters[0] > 0, "Dense points should be clustered");
        for i in 1..5 {
            assert_eq!(clusters[0], clusters[i], "Points 0..5 same cluster");
        }
        for i in 5..8 {
            assert_eq!(clusters[i], 0, "Scattered point {} should be noise", i);
        }
    }

    #[test]
    fn test_voxel_cluster_adjacent_voxels_merge() {
        // At cell_size=0.5, points at (0.1,0,0) and (0.6,0,0) are in
        // neighboring voxels and should merge into one cluster.
        let x = vec![0.1, 0.2, 0.3, 0.6, 0.7, 0.8];
        let y = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let z = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

        let mut data = VoxelClusterData::new(0.5, 2);
        let clusters = voxel_cluster(&mut data, &x, &y, &z);

        assert!(clusters[0] > 0, "Should be clustered");
        for i in 1..6 {
            assert_eq!(clusters[0], clusters[i], "All should merge via adjacency");
        }
    }

    #[test]
    fn test_voxel_cluster_reuse_second_call() {
        // Reproduce bug: voxel_cluster works on first call but crashes on second
        let x1 = vec![1.0, 1.05, 1.1, 5.0, 5.05, 5.1, 0.0];
        let y1 = vec![0.0, 0.05, 0.0, 0.0, 0.05, 0.0, 0.0];
        let z1 = vec![0.0, 0.0, 0.05, 0.0, 0.0, 0.05, 0.0];

        let mut data = VoxelClusterData::new(0.2, 2);

        // First call should work
        let c1 = voxel_cluster(&mut data, &x1, &y1, &z1);
        assert!(c1[0] > 0, "First call: point 0 should be clustered");

        // Second call with different data should also work
        let x2 = vec![2.0, 2.01, 2.02, 2.0, 2.01, 10.0, 20.0, 30.0];
        let y2 = vec![3.0, 3.01, 3.0, 3.02, 3.01, 10.0, 20.0, 30.0];
        let z2 = vec![1.0, 1.0, 1.01, 1.0, 1.01, 10.0, 20.0, 30.0];

        let c2 = voxel_cluster(&mut data, &x2, &y2, &z2);
        assert!(c2[0] > 0, "Second call: point 0 should be clustered");
        for i in 1..5 {
            assert_eq!(c2[0], c2[i], "Second call: points 0..5 same cluster");
        }
        for i in 5..8 {
            assert_eq!(c2[i], 0, "Second call: point {} should be noise", i);
        }

        // Third call with same data as first
        let c3 = voxel_cluster(&mut data, &x1, &y1, &z1);
        assert!(c3[0] > 0, "Third call: point 0 should be clustered");
        assert_eq!(c3[0], c3[1]);
        assert_eq!(c3[0], c3[2]);
    }

    #[test]
    fn test_voxel_vs_dbscan_agreement() {
        let x = vec![
            1.0, 1.05, 1.1, 1.02, 1.08,
            5.0, 5.05, 5.1, 5.02, 5.08,
            0.0, 20.0, 30.0,
        ];
        let y = vec![
            0.0, 0.05, 0.0, 0.02, 0.03,
            0.0, 0.05, 0.0, 0.02, 0.03,
            0.0, 20.0, 30.0,
        ];
        let z = vec![
            0.0, 0.0, 0.05, 0.01, 0.02,
            0.0, 0.0, 0.05, 0.01, 0.02,
            0.0, 20.0, 30.0,
        ];

        let mut dbscan = ClusterData::new_flat(0.2, 3);
        let mut voxel = VoxelClusterData::new(0.2, 3);

        let c_dbscan = cluster_(&mut dbscan, &x, &y, &z);
        let c_voxel = voxel_cluster(&mut voxel, &x, &y, &z);

        assert_eq!(c_dbscan.len(), c_voxel.len());

        for i in 0..c_dbscan.len() {
            for j in (i + 1)..c_dbscan.len() {
                let same_d = c_dbscan[i] == c_dbscan[j];
                let same_v = c_voxel[i] == c_voxel[j];
                assert_eq!(same_d, same_v,
                    "Points {} and {} grouping mismatch: dbscan={}, voxel={}",
                    i, j, c_dbscan[i], c_voxel[j]);
            }
        }

        for i in 0..c_dbscan.len() {
            assert_eq!(c_dbscan[i] == 0, c_voxel[i] == 0,
                "Point {} noise mismatch", i);
        }
    }

    /// Load a binary PCD file (FIELDS x y z intensity, 13 bytes/point).
    /// Returns (x, y, z) coordinate vectors. Skips origin (0,0,0) points.
    fn load_pcd(path: &str) -> Option<(Vec<f32>, Vec<f32>, Vec<f32>)> {
        let data = std::fs::read(path).ok()?;

        // Find end of header ("DATA binary\n")
        let header_end = data.windows(12)
            .position(|w| w == b"DATA binary\n")?
            + 12;

        let body = &data[header_end..];
        let point_size = 13; // 4+4+4+1
        if body.len() % point_size != 0 {
            return None;
        }
        let n = body.len() / point_size;

        let mut x = Vec::with_capacity(n);
        let mut y = Vec::with_capacity(n);
        let mut z = Vec::with_capacity(n);

        for i in 0..n {
            let off = i * point_size;
            let px = f32::from_le_bytes(body[off..off + 4].try_into().ok()?);
            let py = f32::from_le_bytes(body[off + 4..off + 8].try_into().ok()?);
            let pz = f32::from_le_bytes(body[off + 8..off + 12].try_into().ok()?);
            x.push(px);
            y.push(py);
            z.push(pz);
        }

        Some((x, y, z))
    }

    macro_rules! require_pcd {
        ($path:expr) => {
            match load_pcd($path) {
                Some(v) => v,
                None => {
                    eprintln!("Skipping: {} not found", $path);
                    return;
                }
            }
        };
    }

    #[test]
    fn test_voxel_cluster_e1r_real_data() {
        let (x, y, z) = require_pcd!("testdata/e1r_frame0.pcd");
        assert!(x.len() > 10_000, "E1R frame should have >10k points");

        let mut data = VoxelClusterData::new(0.256, 4);

        let t = std::time::Instant::now();
        let clusters = voxel_cluster(&mut data, &x, &y, &z);
        let elapsed = t.elapsed();
        assert_eq!(clusters.len(), x.len());

        let max_id = clusters.iter().max().copied().unwrap_or(0);
        let n_clustered = clusters.iter().filter(|c| **c > 0).count();
        println!(
            "E1R voxel: {} points, {} clustered, {} clusters in {:.2?}",
            x.len(), n_clustered, max_id, elapsed
        );
        assert!(max_id > 0, "Should produce at least 1 cluster");

        // Second call with different frame to verify state reuse
        let (x2, y2, z2) = require_pcd!("testdata/e1r_frame5.pcd");
        let t = std::time::Instant::now();
        let clusters2 = voxel_cluster(&mut data, &x2, &y2, &z2);
        let elapsed2 = t.elapsed();
        assert_eq!(clusters2.len(), x2.len());
        let max_id2 = clusters2.iter().max().copied().unwrap_or(0);
        println!(
            "E1R voxel (reuse): {} points, {} clusters in {:.2?}",
            x2.len(), max_id2, elapsed2
        );
        assert!(max_id2 > 0, "Second call should also produce clusters");
    }

    #[test]
    fn test_dbscan_cluster_e1r_real_data() {
        let (x, y, z) = require_pcd!("testdata/e1r_frame0.pcd");
        assert!(x.len() > 10_000);

        let mut data = ClusterData::new_flat(0.256, 4);

        let t = std::time::Instant::now();
        let clusters = cluster_(&mut data, &x, &y, &z);
        let elapsed = t.elapsed();
        assert_eq!(clusters.len(), x.len());

        let max_id = clusters.iter().max().copied().unwrap_or(0);
        let n_clustered = clusters.iter().filter(|c| **c > 0).count();
        println!(
            "E1R DBSCAN: {} points, {} clustered, {} clusters in {:.2?}",
            x.len(), n_clustered, max_id, elapsed
        );
        assert!(max_id > 0, "Should produce at least 1 cluster");
    }

    #[test]
    fn test_voxel_cluster_ouster_real_data() {
        let (x, y, z) = require_pcd!("testdata/os1_frame0.pcd");
        assert!(x.len() > 50_000, "Ouster frame should have >50k points");

        let mut data = VoxelClusterData::new(0.256, 4);

        let t = std::time::Instant::now();
        let clusters = voxel_cluster(&mut data, &x, &y, &z);
        let elapsed = t.elapsed();
        assert_eq!(clusters.len(), x.len());

        let max_id = clusters.iter().max().copied().unwrap_or(0);
        let n_clustered = clusters.iter().filter(|c| **c > 0).count();
        println!(
            "Ouster voxel: {} points, {} clustered, {} clusters in {:.2?}",
            x.len(), n_clustered, max_id, elapsed
        );
        assert!(max_id > 0, "Should produce at least 1 cluster");
    }

    // Note: DBSCAN on Ouster 128k points takes ~375s on x86_64 in debug — too slow
    // for a unit test. Use voxel clustering for Ouster-scale point clouds.
    // The DBSCAN algorithm is O(n * neighbors) and doesn't scale well past ~30k.
}
