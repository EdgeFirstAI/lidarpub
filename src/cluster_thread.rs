// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

use crate::{
    Args,
    formats::{clustered_xyz_fields, format_clustered_17byte},
    lidar::Points,
};
use edgefirst_lidarpub::cluster::{
    CLUSTER_ID_FIRST, CLUSTER_ID_GROUND, ClusterData, VoxelClusterData, cluster_, voxel_cluster,
};
use edgefirst_lidarpub::ground::GroundFilter;
use edgefirst_schemas::{
    builtin_interfaces::Time, sensor_msgs::PointCloud2, serde_cdr, std_msgs::Header,
};
use kanal::Receiver;
use std::time::Instant;
use tracing::{error, info, info_span, instrument};
use zenoh::bytes::{Encoding, ZBytes};

enum ClusteringKind {
    Dbscan(ClusterData),
    Voxel(VoxelClusterData),
}

// If the receiver is empty, waits for the next message, otherwise returns the
// most recent message on this receiver. If the receiver is closed, returns None
async fn drain_recv<T>(rx: &mut Receiver<T>) -> Option<T> {
    let mut msg = match rx.try_recv() {
        Err(_) => {
            return None;
        }
        Ok(Some(v)) => v,
        Ok(None) => return rx.recv().map_or(None, |x| Some(x)),
    };
    while let Ok(Some(v)) = rx.try_recv() {
        msg = v;
    }
    Some(msg)
}

#[allow(clippy::type_complexity)]
pub async fn cluster_thread(
    mut rx: Receiver<(Vec<f32>, Points, Time, Option<(f32, f32, f32)>)>,
    publ: zenoh::pubsub::Publisher<'_>,
    args: Args,
) {
    let eps_m = args.clustering_eps as f32 / 1000.0;
    let min_pts = args.clustering_minpts;

    let bridge_pts = if args.clustering_bridge == 0 {
        min_pts
    } else {
        args.clustering_bridge
    };
    let mut kind = match args.clustering.as_str() {
        "voxel" => {
            let mut data = VoxelClusterData::new(eps_m, min_pts);
            data.set_bridge_pts(bridge_pts);
            ClusteringKind::Voxel(data)
        }
        _ => {
            let mut data = ClusterData::new_flat(eps_m, min_pts);
            data.set_bridge_pts(bridge_pts);
            ClusteringKind::Dbscan(data)
        }
    };

    let mut ground_filter = GroundFilter::new();
    let ground_thickness_m = args.ground_thickness as f32 / 1000.0;
    let sensor_height_m: Option<f32> = args.sensor_height.map(|h| h as f32 / 1000.0);
    let mut logged_imu = false;

    // Pipeline timing accumulators (microseconds)
    let mut frame_count: u64 = 0;
    let mut sum_valid_us: u64 = 0;
    let mut sum_ground_us: u64 = 0;
    let mut sum_cluster_us: u64 = 0;
    let mut sum_relabel_us: u64 = 0;
    let mut sum_format_us: u64 = 0;
    let mut sum_publish_us: u64 = 0;
    let mut sum_total_us: u64 = 0;
    let report_interval: u64 = 100;

    loop {
        let (ranges, points, time, imu_accel) = match drain_recv(&mut rx).await {
            Some(v) => v,
            None => return,
        };

        let t_frame = Instant::now();
        let n_points = points.x.len();

        // ── Stage 1: Validity mask ──────────────────────────────────────
        let t0 = Instant::now();
        let mut valid = info_span!("valid_mask").in_scope(|| {
            edgefirst_lidarpub::cluster::compute_valid(&points.x, &points.y, &points.z)
        });
        let dt_valid = t0.elapsed();

        // ── Stage 2: Ground filter ──────────────────────────────────────
        let t0 = Instant::now();
        let pre_ground = info_span!("ground_filter").in_scope(|| {
            if let Some(accel) = imu_accel.filter(|_| args.ground_filter) {
                let snapshot = valid.clone();
                ground_filter.filter(
                    &points.x, &points.y, &points.z,
                    &ranges,
                    accel,
                    sensor_height_m,
                    ground_thickness_m,
                    &mut valid,
                );
                if !logged_imu {
                    let mag = (accel.0 * accel.0 + accel.1 * accel.1 + accel.2 * accel.2).sqrt();
                    let gx = accel.0 / mag;
                    let gy = accel.1 / mag;
                    let gz = accel.2 / mag;
                    let ground_removed = snapshot.iter().zip(valid.iter())
                        .filter(|&(pre, post)| *pre && !*post).count();
                    let mut heights: Vec<f32> = (0..n_points)
                        .filter(|&i| snapshot[i])
                        .map(|i| points.x[i] * gx + points.y[i] * gy + points.z[i] * gz)
                        .collect();
                    heights.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
                    let n_h = heights.len();
                    let p = |pct: usize| heights[n_h * pct / 100];
                    info!(
                        "IMU raw=({:.3}, {:.3}, {:.3}) gravity=({:.3}, {:.3}, {:.3}) ground_height={:?} ground_removed={} heights: min={:.2} p10={:.2} p50={:.2} p90={:.2} max={:.2}",
                        accel.0, accel.1, accel.2, gx, gy, gz,
                        ground_filter.ground_height(), ground_removed,
                        heights[0], p(10), p(50), p(90), heights[n_h - 1]
                    );
                    logged_imu = true;
                }
                Some(snapshot)
            } else {
                None
            }
        });
        let dt_ground = t0.elapsed();

        // ── Stage 3: Clustering ─────────────────────────────────────────
        let t0 = Instant::now();
        let mut clusters = info_span!("clustering").in_scope(|| match &mut kind {
            ClusteringKind::Dbscan(data) => cluster_(data, &points.x, &points.y, &points.z, &valid),
            ClusteringKind::Voxel(data) => {
                voxel_cluster(data, &points.x, &points.y, &points.z, &valid)
            }
        });
        let dt_cluster = t0.elapsed();

        // ── Stage 4: Relabel cluster IDs ────────────────────────────────
        let t0 = Instant::now();
        info_span!("relabel").in_scope(|| {
            if let Some(ref pre) = pre_ground {
                for i in 0..n_points {
                    if clusters[i] > 0 {
                        clusters[i] += CLUSTER_ID_FIRST - 1;
                    } else if pre[i] && !valid[i] {
                        clusters[i] = CLUSTER_ID_GROUND;
                    }
                }
            }
        });
        let dt_relabel = t0.elapsed();

        // ── Stage 5: Format PointCloud2 ─────────────────────────────────
        let t0 = Instant::now();
        let (msg, enc) = match format_points_clustered(
            &points,
            &clusters,
            n_points,
            time,
            args.frame_id.clone(),
        ) {
            Ok(v) => v,
            Err(e) => {
                error!("Could not encode clustered PCD: {:?}", e);
                continue;
            }
        };
        let dt_format = t0.elapsed();

        // ── Stage 6: Publish ────────────────────────────────────────────
        let t0 = Instant::now();
        info_span!("publish")
            .in_scope(|| async {
                match publ.put(msg).encoding(enc).await {
                    Ok(_) => {}
                    Err(e) => error!("cluster publish error: {:?}", e),
                }
            })
            .await;
        let dt_publish = t0.elapsed();

        let dt_total = t_frame.elapsed();

        // ── Accumulate and report ───────────────────────────────────────
        frame_count += 1;
        sum_valid_us += dt_valid.as_micros() as u64;
        sum_ground_us += dt_ground.as_micros() as u64;
        sum_cluster_us += dt_cluster.as_micros() as u64;
        sum_relabel_us += dt_relabel.as_micros() as u64;
        sum_format_us += dt_format.as_micros() as u64;
        sum_publish_us += dt_publish.as_micros() as u64;
        sum_total_us += dt_total.as_micros() as u64;

        if frame_count.is_multiple_of(report_interval) {
            let n = report_interval as f64;
            info!(
                "pipeline avg over {} frames ({} pts): valid={:.1}ms ground={:.1}ms cluster={:.1}ms relabel={:.1}ms format={:.1}ms publish={:.1}ms total={:.1}ms",
                report_interval,
                n_points,
                sum_valid_us as f64 / n / 1000.0,
                sum_ground_us as f64 / n / 1000.0,
                sum_cluster_us as f64 / n / 1000.0,
                sum_relabel_us as f64 / n / 1000.0,
                sum_format_us as f64 / n / 1000.0,
                sum_publish_us as f64 / n / 1000.0,
                sum_total_us as f64 / n / 1000.0,
            );
            sum_valid_us = 0;
            sum_ground_us = 0;
            sum_cluster_us = 0;
            sum_relabel_us = 0;
            sum_format_us = 0;
            sum_publish_us = 0;
            sum_total_us = 0;
        }
    }
}

/// Format clustered point cloud data into a PointCloud2 message.
///
/// Uses the shared SIMD formatters from the formats module.
#[instrument(skip_all)]
fn format_points_clustered(
    points: &Points,
    cluster_ids: &[u32],
    n_points: usize,
    timestamp: Time,
    frame_id: String,
) -> Result<(ZBytes, Encoding), serde_cdr::Error> {
    let fields = clustered_xyz_fields();

    // Use the shared SIMD formatter from formats module
    let data = format_clustered_17byte(
        &points.x,
        &points.y,
        &points.z,
        cluster_ids,
        &points.intensity,
        n_points,
    );

    let msg = PointCloud2 {
        header: Header {
            stamp: timestamp,
            frame_id,
        },
        height: 1,
        width: n_points as u32,
        fields,
        is_bigendian: false,
        point_step: 17,
        row_step: 17 * n_points as u32,
        data,
        is_dense: true,
    };

    let msg = ZBytes::from(serde_cdr::serialize(&msg)?);
    let enc = Encoding::APPLICATION_CDR.with_schema("sensor_msgs/msg/PointCloud2");

    Ok((msg, enc))
}
