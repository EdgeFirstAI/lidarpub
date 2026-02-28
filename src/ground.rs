// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

//! IMU-guided ground plane removal for LiDAR point clouds.
//!
//! 1. **Detect ground height** — Per-patch PCA on a polar grid identifies flat,
//!    upright surfaces. Seeds from accepted patches are pooled and their median
//!    height (along the gravity axis) is the ground level. This step is skipped
//!    when a known `sensor_height` is provided.
//!
//! 2. **Normal from IMU** — The gravity vector defines the ground plane normal.
//!    In the gravity-aligned frame this is simply [0,0,1].
//!
//! 3. **Classify** — Every point whose elevation above the ground plane is less
//!    than `thickness_m` is marked as ground. One-sided: anything at or below
//!    the ground surface is also ground.

/// Polar grid: azimuth sectors.
const N_AZIMUTH: usize = 16;
/// Polar grid: range rings.
const N_RANGE: usize = 8;
/// Total patches.
const N_PATCHES: usize = N_AZIMUTH * N_RANGE;
/// Max horizontal range for grid (meters).
const MAX_RANGE: f32 = 30.0;
/// Range ring width (meters).
const RANGE_BIN_SIZE: f32 = MAX_RANGE / N_RANGE as f32;
/// Azimuth bin width (radians).
const AZIMUTH_BIN_SIZE: f32 = std::f32::consts::TAU / N_AZIMUTH as f32;

/// Seeds per patch for PCA.
const K_SEEDS: usize = 20;
/// Minimum seeds for a valid PCA patch.
const MIN_SEEDS: usize = 5;
/// Uprightness: |normal_z| must exceed this (cos 32°).
const UPRIGHTNESS_THRESH: f32 = 0.85;
/// Flatness: smallest eigenvalue ratio below this.
const FLATNESS_THRESH: f32 = 0.03;
/// Minimum range for ground detection (meters).
const MIN_RANGE_M: f32 = 0.5;
/// Elevation tolerance: reject patches whose centroid height differs
/// from the ground reference by more than this (meters).
const ELEVATION_TOLERANCE: f32 = 1.0;
/// Temporal EMA smoothing factor.
const EMA_ALPHA: f32 = 0.5;
/// Minimum accepted patches to trust ground detection.
const MIN_ACCEPTED_PATCHES: usize = 3;

/// IMU-guided ground plane filter.
pub struct GroundFilter {
    /// Per-point height along gravity (dot(point, gravity_unit)).
    heights: Vec<f32>,
    /// Horizontal coordinates for polar binning [h0, h1] per point.
    horiz: Vec<[f32; 2]>,
    /// Per-patch point index lists.
    patch_points: Vec<Vec<usize>>,
    /// Scratch buffer for seed extraction.
    seed_buf: Vec<(f32, usize)>,
    /// Temporally-smoothed ground height (EMA).
    ema_ground_height: Option<f32>,
}

impl GroundFilter {
    /// Current smoothed ground height (if detected).
    pub fn ground_height(&self) -> Option<f32> {
        self.ema_ground_height
    }

    #[must_use]
    pub fn new() -> Self {
        Self {
            heights: Vec::new(),
            horiz: Vec::new(),
            patch_points: (0..N_PATCHES).map(|_| Vec::new()).collect(),
            seed_buf: Vec::new(),
            ema_ground_height: None,
        }
    }

    /// Remove ground-plane points from the validity mask.
    ///
    /// * `sensor_height` — If `Some(h)`, skip detection and use `h` meters as
    ///   the known sensor height above ground. If `None`, detect automatically.
    /// * `accel` — IMU accelerometer `(ax, ay, az)` in the LiDAR frame.
    /// * `thickness_m` — Points within this distance above the ground plane
    ///   (and anything at/below it) are marked as ground.
    #[allow(clippy::too_many_arguments)]
    pub fn filter(
        &mut self,
        x: &[f32],
        y: &[f32],
        z: &[f32],
        range: &[f32],
        accel: (f32, f32, f32),
        sensor_height: Option<f32>,
        thickness_m: f32,
        valid: &mut [bool],
    ) {
        let n = x.len();
        assert_eq!(n, y.len());
        assert_eq!(n, z.len());
        assert_eq!(n, range.len());
        assert_eq!(n, valid.len());

        // ── Gravity direction ────────────────────────────────────────────
        let (ax, ay, az) = accel;
        let mag = (ax * ax + ay * ay + az * az).sqrt();
        if mag < 1e-6 {
            return;
        }
        let inv_mag = 1.0 / mag;
        let gx = ax * inv_mag;
        let gy = ay * inv_mag;
        let gz = az * inv_mag;

        // ── Per-point height along gravity ───────────────────────────────
        self.heights.clear();
        self.heights.reserve(n);
        for i in 0..n {
            self.heights.push(x[i] * gx + y[i] * gy + z[i] * gz);
        }

        // ── Determine ground height ─────────────────────────────────────
        let ground_height = if let Some(h) = sensor_height {
            // Known sensor height → ground is at height h in gravity direction
            h
        } else {
            match self.detect_ground(x, y, z, range, valid, gx, gy, gz) {
                Some(h) => h,
                None => {
                    // Detection failed. Use EMA fallback if available.
                    match self.ema_ground_height {
                        Some(h) => h,
                        None => return,
                    }
                }
            }
        };

        // ── EMA smooth ──────────────────────────────────────────────────
        let smooth_height = match self.ema_ground_height {
            Some(prev) if (prev - ground_height).abs() < 0.5 => {
                EMA_ALPHA * prev + (1.0 - EMA_ALPHA) * ground_height
            }
            _ => ground_height,
        };
        self.ema_ground_height = Some(smooth_height);

        // ── Classify: one-sided threshold ────────────────────────────────
        // ground plane is at height = smooth_height (in gravity direction).
        // Elevation above ground = smooth_height - point_height.
        // Mark as ground if elevation < thickness_m (i.e. at ground level
        // or up to thickness_m above it, including anything below ground).
        // NOTE: No range gate here — near-field floor points must also be
        // filtered. The range gate in detect_ground() is only for PCA quality.
        let cutoff = smooth_height - thickness_m;
        for (v, h) in valid.iter_mut().zip(self.heights.iter()).take(n) {
            if *v && *h >= cutoff {
                *v = false;
            }
        }
    }

    /// Detect ground height using per-patch PCA.
    ///
    /// Returns the median seed height from accepted ground patches, or None
    /// if not enough evidence was found.
    #[allow(clippy::too_many_arguments)]
    fn detect_ground(
        &mut self,
        x: &[f32],
        y: &[f32],
        z: &[f32],
        range: &[f32],
        valid: &[bool],
        gx: f32,
        gy: f32,
        gz: f32,
    ) -> Option<f32> {
        let n = x.len();

        // Build gravity-aligned horizontal frame for polar binning
        let (hx0, hy0, hz0, hx1, hy1, hz1) = gravity_frame(gx, gy, gz);

        self.horiz.clear();
        self.horiz.reserve(n);
        for i in 0..n {
            self.horiz.push([
                x[i] * hx0 + y[i] * hy0 + z[i] * hz0,
                x[i] * hx1 + y[i] * hy1 + z[i] * hz1,
            ]);
        }

        // Bin valid points into polar grid
        for patch in self.patch_points.iter_mut() {
            patch.clear();
        }
        for i in 0..n {
            if !valid[i] || range[i] < MIN_RANGE_M {
                continue;
            }
            let [h0, h1] = self.horiz[i];
            let horiz_range = (h0 * h0 + h1 * h1).sqrt();
            if !(0.01..MAX_RANGE).contains(&horiz_range) {
                continue;
            }
            let azimuth = h1.atan2(h0) + std::f32::consts::PI;
            let az_bin = ((azimuth / AZIMUTH_BIN_SIZE) as usize).min(N_AZIMUTH - 1);
            let r_bin = ((horiz_range / RANGE_BIN_SIZE) as usize).min(N_RANGE - 1);
            self.patch_points[az_bin * N_RANGE + r_bin].push(i);
        }

        // Per-patch PCA: find flat upright patches, collect seed heights
        let mut accepted_patches = 0usize;
        let mut max_centroid_h = f32::NEG_INFINITY;
        // Store (centroid_h, vec of seed heights) per accepted patch
        struct PatchInfo {
            centroid_h: f32,
            seed_heights: Vec<f32>,
        }
        let mut patch_infos: Vec<PatchInfo> = Vec::new();

        for patch_idx in 0..N_PATCHES {
            let pts = &self.patch_points[patch_idx];
            if pts.len() < MIN_SEEDS {
                continue;
            }

            // Extract K seeds with LARGEST heights (closest to ground)
            self.seed_buf.clear();
            for &pi in pts.iter() {
                self.seed_buf.push((self.heights[pi], pi));
            }
            let k = K_SEEDS.min(self.seed_buf.len());
            self.seed_buf[..].select_nth_unstable_by(k.saturating_sub(1), |a, b| {
                b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal)
            });
            let seeds = &self.seed_buf[..k];

            // Covariance in GA frame: [horiz0, horiz1, height]
            let inv_k = 1.0 / k as f32;
            let mut cx = 0.0f32;
            let mut cy = 0.0f32;
            let mut ch = 0.0f32;
            for &(_, pi) in seeds {
                cx += self.horiz[pi][0];
                cy += self.horiz[pi][1];
                ch += self.heights[pi];
            }
            cx *= inv_k;
            cy *= inv_k;
            ch *= inv_k;

            let mut cov = [0.0f32; 6];
            for &(_, pi) in seeds {
                let dx = self.horiz[pi][0] - cx;
                let dy = self.horiz[pi][1] - cy;
                let dh = self.heights[pi] - ch;
                cov[0] += dx * dx;
                cov[1] += dx * dy;
                cov[2] += dx * dh;
                cov[3] += dy * dy;
                cov[4] += dy * dh;
                cov[5] += dh * dh;
            }
            for c in cov.iter_mut() {
                *c *= inv_k;
            }

            let (eigenvalues, eigenvectors) = sym3x3_eigen(cov);
            let min_idx = smallest_eigenvalue_index(eigenvalues);
            let normal = eigenvectors[min_idx];

            // Uprightness: normal should be along height axis (index 2)
            if normal[2].abs() < UPRIGHTNESS_THRESH {
                continue;
            }

            // Flatness
            let eig_sum = eigenvalues[0] + eigenvalues[1] + eigenvalues[2];
            if eig_sum < 1e-12 {
                continue;
            }
            if eigenvalues[min_idx] / eig_sum > FLATNESS_THRESH {
                continue;
            }

            // This patch is a flat, upright surface
            let seed_heights: Vec<f32> = seeds.iter().map(|&(h, _)| h).collect();
            if ch > max_centroid_h {
                max_centroid_h = ch;
            }
            patch_infos.push(PatchInfo {
                centroid_h: ch,
                seed_heights,
            });
        }

        if patch_infos.is_empty() {
            return None;
        }

        // Elevation filter: keep only patches near the ground (max centroid)
        // to reject ceilings/tables
        let mut all_seed_heights: Vec<f32> = Vec::new();
        for info in &patch_infos {
            if (info.centroid_h - max_centroid_h).abs() <= ELEVATION_TOLERANCE {
                accepted_patches += 1;
                all_seed_heights.extend_from_slice(&info.seed_heights);
            }
        }

        if accepted_patches < MIN_ACCEPTED_PATCHES || all_seed_heights.is_empty() {
            return None;
        }

        // Ground height = median of all pooled seed heights from accepted patches
        all_seed_heights
            .sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let ground_h = all_seed_heights[all_seed_heights.len() / 2];

        Some(ground_h)
    }
}

impl Default for GroundFilter {
    fn default() -> Self {
        Self::new()
    }
}

fn smallest_eigenvalue_index(eigenvalues: [f32; 3]) -> usize {
    if eigenvalues[0] <= eigenvalues[1] && eigenvalues[0] <= eigenvalues[2] {
        0
    } else if eigenvalues[1] <= eigenvalues[2] {
        1
    } else {
        2
    }
}

/// Build an orthonormal frame from a gravity unit vector.
fn gravity_frame(gx: f32, gy: f32, gz: f32) -> (f32, f32, f32, f32, f32, f32) {
    let (sx, sy, sz) = if gz.abs() < 0.9 {
        (0.0, 0.0, 1.0)
    } else {
        (1.0, 0.0, 0.0)
    };

    let h0x = sy * gz - sz * gy;
    let h0y = sz * gx - sx * gz;
    let h0z = sx * gy - sy * gx;
    let h0_mag = (h0x * h0x + h0y * h0y + h0z * h0z).sqrt();
    let inv_h0 = 1.0 / h0_mag;
    let h0x = h0x * inv_h0;
    let h0y = h0y * inv_h0;
    let h0z = h0z * inv_h0;

    let h1x = gy * h0z - gz * h0y;
    let h1y = gz * h0x - gx * h0z;
    let h1z = gx * h0y - gy * h0x;

    (h0x, h0y, h0z, h1x, h1y, h1z)
}

/// Eigendecomposition of a 3×3 symmetric matrix.
///
/// Uses the analytical method described by Smith (1961) / Kopp (2008).
fn sym3x3_eigen(cov: [f32; 6]) -> ([f32; 3], [[f32; 3]; 3]) {
    let a11 = cov[0] as f64;
    let a12 = cov[1] as f64;
    let a13 = cov[2] as f64;
    let a22 = cov[3] as f64;
    let a23 = cov[4] as f64;
    let a33 = cov[5] as f64;

    let q = (a11 + a22 + a33) / 3.0;
    let p1 = a12 * a12 + a13 * a13 + a23 * a23;

    let eigenvalues = if p1 < 1e-30 {
        let mut evs = [a11 as f32, a22 as f32, a33 as f32];
        evs.sort_unstable_by(|a, b| b.partial_cmp(a).unwrap_or(std::cmp::Ordering::Equal));
        evs
    } else {
        let p2 = (a11 - q) * (a11 - q) + (a22 - q) * (a22 - q) + (a33 - q) * (a33 - q) + 2.0 * p1;
        let p = (p2 / 6.0).sqrt();
        let inv_p = 1.0 / p;

        let b11 = (a11 - q) * inv_p;
        let b12 = a12 * inv_p;
        let b13 = a13 * inv_p;
        let b22 = (a22 - q) * inv_p;
        let b23 = a23 * inv_p;
        let b33 = (a33 - q) * inv_p;

        let half_det = (b11 * b22 * b33 + 2.0 * b12 * b13 * b23
            - b11 * b23 * b23
            - b22 * b13 * b13
            - b33 * b12 * b12)
            / 2.0;

        let phi = half_det.clamp(-1.0, 1.0).acos() / 3.0;

        let e1 = (q + 2.0 * p * phi.cos()) as f32;
        let e3 = (q + 2.0 * p * (phi + std::f64::consts::TAU / 3.0).cos()) as f32;
        let e2 = 3.0 * q as f32 - e1 - e3;
        [e1, e2, e3]
    };

    let mat = [a11, a12, a13, a22, a23, a33];
    let eigenvectors = [
        eigenvector_for(mat, eigenvalues[0] as f64),
        eigenvector_for(mat, eigenvalues[1] as f64),
        eigenvector_for(mat, eigenvalues[2] as f64),
    ];

    (eigenvalues, eigenvectors)
}

/// Compute eigenvector for a 3×3 symmetric matrix at a given eigenvalue.
fn eigenvector_for(mat: [f64; 6], lambda: f64) -> [f32; 3] {
    let m00 = mat[0] - lambda;
    let m01 = mat[1];
    let m02 = mat[2];
    let m11 = mat[3] - lambda;
    let m12 = mat[4];
    let m22 = mat[5] - lambda;

    let v0x = m01 * m12 - m02 * m11;
    let v0y = m02 * m01 - m00 * m12;
    let v0z = m00 * m11 - m01 * m01;

    let v1x = m01 * m22 - m02 * m12;
    let v1y = m02 * m02 - m00 * m22;
    let v1z = m00 * m12 - m01 * m02;

    let v2x = m11 * m22 - m12 * m12;
    let v2y = m12 * m02 - m01 * m22;
    let v2z = m01 * m12 - m11 * m02;

    let mag0 = v0x * v0x + v0y * v0y + v0z * v0z;
    let mag1 = v1x * v1x + v1y * v1y + v1z * v1z;
    let mag2 = v2x * v2x + v2y * v2y + v2z * v2z;

    let (vx, vy, vz, mag) = if mag0 >= mag1 && mag0 >= mag2 {
        (v0x, v0y, v0z, mag0)
    } else if mag1 >= mag2 {
        (v1x, v1y, v1z, mag1)
    } else {
        (v2x, v2y, v2z, mag2)
    };

    if mag < 1e-30 {
        return [0.0, 0.0, 1.0];
    }

    let inv = 1.0 / mag.sqrt();
    [(vx * inv) as f32, (vy * inv) as f32, (vz * inv) as f32]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn compute_range(x: &[f32], y: &[f32], z: &[f32]) -> Vec<f32> {
        x.iter()
            .zip(y.iter())
            .zip(z.iter())
            .map(|((&xi, &yi), &zi)| (xi * xi + yi * yi + zi * zi).sqrt())
            .collect()
    }

    /// Sensor at 1.5m, gravity down. Floor should be fully removed.
    #[test]
    fn test_ground_filter_flat_floor() {
        let n_floor = 400;
        let n_objects = 100;
        let n_origin = 10;
        let n = n_floor + n_objects + n_origin;

        let mut x = vec![0.0f32; n];
        let mut y = vec![0.0f32; n];
        let mut z = vec![0.0f32; n];
        let mut valid = vec![true; n];

        // Floor at z=-1.5 (height in GA = 1.5)
        for i in 0..n_floor {
            let angle = (i as f32 / n_floor as f32) * std::f32::consts::TAU;
            let r = 3.0 + (i as f32 % 20.0) * 0.05;
            x[i] = r * angle.cos();
            y[i] = r * angle.sin();
            z[i] = -1.5 + (i as f32 % 7.0) * 0.003;
        }

        // Objects at z=0 (height in GA = 0, well above floor)
        for i in 0..n_objects {
            let idx = n_floor + i;
            let angle = 0.3 + (i as f32) * 0.02;
            let r = 3.0 + (i as f32) * 0.05;
            x[idx] = r * angle.cos();
            y[idx] = r * angle.sin();
            z[idx] = (i as f32 % 5.0) * 0.02;
        }

        // Origin points (pre-invalidated)
        for i in 0..n_origin {
            let idx = n_floor + n_objects + i;
            valid[idx] = false;
        }

        let range = compute_range(&x, &y, &z);
        let mut gf = GroundFilter::new();
        gf.filter(
            &x,
            &y,
            &z,
            &range,
            (0.0, 0.0, -9.81),
            None,
            0.10,
            &mut valid,
        );

        let floor_filtered = (0..n_floor).filter(|&i| !valid[i]).count();
        assert!(
            floor_filtered > n_floor * 3 / 4,
            "Expected most floor points filtered, got {floor_filtered}/{n_floor}"
        );

        let objects_valid = (n_floor..n_floor + n_objects).filter(|&i| valid[i]).count();
        assert!(
            objects_valid > n_objects * 2 / 3,
            "Expected most objects to survive, got {objects_valid}/{n_objects}"
        );

        for (i, &v) in valid.iter().enumerate().take(n).skip(n_floor + n_objects) {
            assert!(!v, "Origin point {i} should remain invalid");
        }
    }

    /// With known sensor_height, skip detection entirely.
    #[test]
    fn test_ground_filter_known_height() {
        let n = 200;
        let mut x = vec![0.0f32; n];
        let mut y = vec![0.0f32; n];
        let mut z = vec![0.0f32; n];
        let mut valid = vec![true; n];

        // Floor at z=-1.2 → GA height = 1.2
        for i in 0..100 {
            let angle = (i as f32 / 100.0) * std::f32::consts::TAU;
            let r = 2.0 + (i as f32 % 10.0) * 0.1;
            x[i] = r * angle.cos();
            y[i] = r * angle.sin();
            z[i] = -1.2 + (i as f32 % 5.0) * 0.002;
        }
        // Objects at z=0
        for i in 100..200 {
            let angle = (i as f32) * 0.03;
            let r = 3.0;
            x[i] = r * angle.cos();
            y[i] = r * angle.sin();
            z[i] = 0.0;
        }

        let range = compute_range(&x, &y, &z);
        let mut gf = GroundFilter::new();
        // sensor_height = 1.2 (known)
        gf.filter(
            &x,
            &y,
            &z,
            &range,
            (0.0, 0.0, -9.81),
            Some(1.2),
            0.10,
            &mut valid,
        );

        let floor_filtered = (0..100).filter(|&i| !valid[i]).count();
        assert!(
            floor_filtered > 90,
            "With known height, should filter nearly all floor: {floor_filtered}/100"
        );
        let objects_valid = (100..200).filter(|&i| valid[i]).count();
        assert!(
            objects_valid > 90,
            "Objects should survive: {objects_valid}/100"
        );
    }

    /// Walls should NOT be classified as ground.
    #[test]
    fn test_ground_filter_rejects_walls() {
        let n_floor = 200;
        let n_wall = 100;
        let n = n_floor + n_wall;

        let mut x = vec![0.0f32; n];
        let mut y = vec![0.0f32; n];
        let mut z = vec![0.0f32; n];
        let mut valid = vec![true; n];

        for i in 0..n_floor {
            let angle = (i as f32 / n_floor as f32) * std::f32::consts::TAU;
            let r = 2.0 + (i as f32 % 20.0) * 0.05;
            x[i] = r * angle.cos();
            y[i] = r * angle.sin();
            z[i] = -1.5 + (i as f32 % 5.0) * 0.003;
        }

        // Wall: vertical surface at y ≈ 3.0, spanning z from -1.5 to +1.5
        for i in 0..n_wall {
            let idx = n_floor + i;
            x[idx] = (i as f32 % 20.0) * 0.15 - 1.5;
            y[idx] = 3.0 + (i as f32 % 10.0) * 0.01;
            z[idx] = -1.5 + (i as f32 / n_wall as f32) * 3.0;
        }

        let range = compute_range(&x, &y, &z);
        let mut gf = GroundFilter::new();
        gf.filter(
            &x,
            &y,
            &z,
            &range,
            (0.0, 0.0, -9.81),
            None,
            0.10,
            &mut valid,
        );

        // Wall points above the floor should survive
        let wall_above_floor = (n_floor..n).filter(|&i| z[i] > -1.3).count();
        let wall_above_survived = (n_floor..n).filter(|&i| valid[i] && z[i] > -1.3).count();
        assert!(
            wall_above_survived > wall_above_floor * 3 / 4,
            "Wall points above floor should survive: {wall_above_survived}/{wall_above_floor}"
        );
    }

    /// 45-degree sensor tilt. Verifies IMU pose compensation.
    #[test]
    fn test_ground_filter_tilted_sensor() {
        let accel = (0.0f32, -6.94f32, -6.94f32);
        let mag = (accel.1 * accel.1 + accel.2 * accel.2).sqrt();
        let gy = accel.1 / mag;
        let gz = accel.2 / mag;

        let n_floor = 400;
        let n_objects = 100;
        let n = n_floor + n_objects;

        let mut x = vec![0.0f32; n];
        let mut y = vec![0.0f32; n];
        let mut z = vec![0.0f32; n];
        let mut valid = vec![true; n];

        let ground_h = 1.5f32;
        let h1y = gz;
        let h1z = -gy;
        for i in 0..n_floor {
            let angle = (i as f32 / n_floor as f32) * std::f32::consts::TAU;
            let r = 3.0 + (i as f32 % 20.0) * 0.05;
            let horiz_x = r * angle.cos();
            let horiz_perp = r * angle.sin();
            x[i] = horiz_x;
            y[i] = ground_h * gy + horiz_perp * h1y;
            z[i] = ground_h * gz + horiz_perp * h1z;
            let noise = (i as f32 % 5.0) * 0.002;
            y[i] += noise * gy;
            z[i] += noise * gz;
        }

        for i in 0..n_objects {
            let idx = n_floor + i;
            let angle = 0.5 + (i as f32) * 0.01;
            let r = 3.0 + (i as f32) * 0.02;
            x[idx] = r * angle.cos();
            let horiz_perp = r * angle.sin();
            y[idx] = -0.5 * gy + horiz_perp * h1y;
            z[idx] = -0.5 * gz + horiz_perp * h1z;
        }

        let range = compute_range(&x, &y, &z);
        let mut gf = GroundFilter::new();
        gf.filter(&x, &y, &z, &range, accel, None, 0.10, &mut valid);

        let floor_filtered = (0..n_floor).filter(|&i| !valid[i]).count();
        assert!(
            floor_filtered > n_floor / 2,
            "Tilted sensor: {floor_filtered}/{n_floor}"
        );

        let objects_valid = (n_floor..n).filter(|&i| valid[i]).count();
        assert!(
            objects_valid > n_objects * 3 / 4,
            "Objects with tilted sensor: {objects_valid}/{n_objects}"
        );
    }

    /// Zero accelerometer = no filtering.
    #[test]
    fn test_ground_filter_no_imu() {
        let n = 20;
        let x: Vec<f32> = (0..n).map(|i| (i as f32 + 1.0) * 0.5).collect();
        let y = vec![2.0f32; n];
        let z = vec![-1.5f32; n];
        let mut valid = vec![true; n];

        let range = compute_range(&x, &y, &z);
        let mut gf = GroundFilter::new();
        gf.filter(&x, &y, &z, &range, (0.0, 0.0, 0.0), None, 0.10, &mut valid);
        assert!(valid.iter().all(|&v| v));
    }

    /// Buffer reuse across multiple calls.
    #[test]
    fn test_ground_filter_reuse() {
        let mut gf = GroundFilter::new();

        let make_floor = |n: usize, floor_z: f32| -> (Vec<f32>, Vec<f32>, Vec<f32>) {
            let mut x = Vec::with_capacity(n);
            let mut y = Vec::with_capacity(n);
            let mut z = Vec::with_capacity(n);
            for i in 0..n {
                let angle = (i as f32 / n as f32) * std::f32::consts::TAU;
                let r = 3.0 + (i as f32 % 20.0) * 0.05;
                x.push(r * angle.cos());
                y.push(r * angle.sin());
                z.push(floor_z + (i as f32 % 5.0) * 0.002);
            }
            (x, y, z)
        };

        {
            let (x, y, z) = make_floor(300, -1.0);
            let range = compute_range(&x, &y, &z);
            let mut valid = vec![true; 300];
            gf.filter(
                &x,
                &y,
                &z,
                &range,
                (0.0, 0.0, -9.81),
                None,
                0.10,
                &mut valid,
            );
            let filtered = valid.iter().filter(|&&v| !v).count();
            assert!(filtered > 150, "First: got {filtered}/300");
        }

        {
            let (mut x, mut y, mut z) = make_floor(300, -2.0);
            let n_floor = x.len();
            for i in 0..100 {
                let angle = (i as f32) * 0.05 + 1.0;
                let r = 3.0 + (i as f32) * 0.02;
                x.push(r * angle.cos());
                y.push(r * angle.sin());
                z.push(0.5 + (i as f32 % 5.0) * 0.01);
            }
            let n = x.len();
            let range = compute_range(&x, &y, &z);
            let mut valid = vec![true; n];
            gf.filter(
                &x,
                &y,
                &z,
                &range,
                (0.0, 0.0, -9.81),
                None,
                0.10,
                &mut valid,
            );

            let floor_filtered = (0..n_floor).filter(|&i| !valid[i]).count();
            assert!(
                floor_filtered > n_floor / 2,
                "Second floor: {floor_filtered}/{n_floor}"
            );
            let obj_valid = (n_floor..n).filter(|&i| valid[i]).count();
            assert!(obj_valid > 75, "Second objects: {obj_valid}/100");
        }
    }

    fn load_pcd(path: &str) -> Option<(Vec<f32>, Vec<f32>, Vec<f32>)> {
        let data = std::fs::read(path).ok()?;
        let header_end = data.windows(12).position(|w| w == b"DATA binary\n")? + 12;
        let body = &data[header_end..];
        let point_size = 13;
        if body.len() % point_size != 0 {
            return None;
        }
        let n = body.len() / point_size;
        let mut x = Vec::with_capacity(n);
        let mut y = Vec::with_capacity(n);
        let mut z = Vec::with_capacity(n);
        for i in 0..n {
            let off = i * point_size;
            x.push(f32::from_le_bytes(body[off..off + 4].try_into().unwrap()));
            y.push(f32::from_le_bytes(
                body[off + 4..off + 8].try_into().unwrap(),
            ));
            z.push(f32::from_le_bytes(
                body[off + 8..off + 12].try_into().unwrap(),
            ));
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

    /// Real-data test: ground filter on E1R PCD.
    #[test]
    fn test_ground_filter_e1r_real_data() {
        let (x, y, z) = require_pcd!("testdata/e1r_frame0.pcd");
        let n = x.len();
        assert!(n > 10_000);

        let mut valid = vec![true; n];
        for i in 0..n {
            if x[i] == 0.0 && y[i] == 0.0 && z[i] == 0.0 {
                valid[i] = false;
            }
        }
        let pre_valid = valid.iter().filter(|&&v| v).count();

        let range = compute_range(&x, &y, &z);
        let mut filter = GroundFilter::new();
        let t = std::time::Instant::now();
        filter.filter(
            &x,
            &y,
            &z,
            &range,
            (0.0, 0.0, -9.81),
            None,
            0.15,
            &mut valid,
        );
        let elapsed = t.elapsed();

        let post_valid = valid.iter().filter(|&&v| v).count();
        let ground_removed = pre_valid - post_valid;

        eprintln!(
            "E1R ground filter: {} points, {} valid before, {} ground removed, {} remaining in {:.2?}",
            n, pre_valid, ground_removed, post_valid, elapsed
        );
        eprintln!("Detected ground height: {:?}", filter.ema_ground_height);

        assert!(
            ground_removed > 100,
            "Expected some ground points removed, got {ground_removed}"
        );
        assert!(post_valid > n / 4, "Too many points removed");
    }
}
