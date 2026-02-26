// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

use crate::{
    Args,
    formats::{clustered_xyz_fields, format_clustered_17byte},
    lidar::Points,
};
use edgefirst_schemas::{
    builtin_interfaces::Time, sensor_msgs::PointCloud2, serde_cdr, std_msgs::Header,
};
use edgefirst_lidarpub::cluster::{ClusterData, VoxelClusterData, cluster_, voxel_cluster};
use kanal::Receiver;
use tracing::{error, info_span, instrument};
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

pub async fn cluster_thread(
    mut rx: Receiver<(Vec<f32>, Points, Time)>,
    publ: zenoh::pubsub::Publisher<'_>,
    args: Args,
) {
    let eps_m = args.clustering_eps as f32 / 1000.0;
    let min_pts = args.clustering_minpts;

    let mut kind = match args.clustering.as_str() {
        "voxel" => ClusteringKind::Voxel(VoxelClusterData::new(eps_m, min_pts)),
        _ => ClusteringKind::Dbscan(ClusterData::new_flat(eps_m, min_pts)),
    };

    loop {
        let (_ranges, points, time) = match drain_recv(&mut rx).await {
            Some(v) => v,
            None => return,
        };

        let n_points = points.x.len();
        let clusters = info_span!("clustering").in_scope(|| match &mut kind {
            ClusteringKind::Dbscan(data) => cluster_(data, &points.x, &points.y, &points.z),
            ClusteringKind::Voxel(data) => voxel_cluster(data, &points.x, &points.y, &points.z),
        });

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

        match publ.put(msg).encoding(enc).await {
            Ok(_) => {}
            Err(e) => error!("cluster publish error: {:?}", e),
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
