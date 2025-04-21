use std::{
    ops::Add,
    simd::{Simd, ToBytes},
};
use tracing::trace;
type RangeType = u16;
pub struct ClusterData {
    range: Vec<RangeType>,
    rows: usize,
    cols: usize,
    cluster_ids: Vec<u32>, // 0 = noise, otherwise cluster_id
    eps: RangeType,
    min_pts: usize,
    wrap: bool,
}

const OFFSETS: [Coord; 8] = [
    Coord(-1, -1),
    Coord(-1, 0),
    Coord(-1, 1),
    Coord(0, -1),
    // Coord(0, 0),
    Coord(0, 1),
    Coord(1, -1),
    Coord(1, 0),
    Coord(1, 1),
];

#[derive(Debug, Clone, Copy)]
struct Coord(isize, isize);

impl ClusterData {
    fn set_id(&mut self, c: Coord, id: u32) {
        let ind = self.get_index(c);
        if let Some(ind) = ind {
            self.cluster_ids[ind] = id;
        }
    }

    fn get_id(&mut self, c: Coord) -> Option<u32> {
        let ind = self.get_index(c)?;
        Some(self.cluster_ids[ind])
    }

    fn get_range(&self, c: Coord) -> Option<RangeType> {
        let ind = self.get_index(c)?;
        Some(self.range[ind])
    }

    fn get_index(&self, c: Coord) -> Option<usize> {
        if self.wrap {
            let ind = (c.0 + self.rows as isize) % self.rows as isize * self.cols as isize
                + (c.1 + self.cols as isize) % self.cols as isize;
            Some(ind as usize)
        } else if 0 <= c.0 && c.0 < self.rows as isize && 0 <= c.1 && c.1 < self.cols as isize {
            Some(c.0 as usize * self.cols + c.1 as usize)
        } else {
            None
        }
    }
}

impl Add for Coord {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Coord(self.0 + rhs.0, self.1 + rhs.1)
    }
}

use cdr::{CdrLe, Infinite};
use edgefirst_schemas::{
    builtin_interfaces::Time,
    sensor_msgs::{PointCloud2, PointField},
    std_msgs::Header,
};
use kanal::Receiver;
use lidarpub::ouster::Points;
use tracing::error;
use zenoh::bytes::{Encoding, ZBytes};

use crate::PointFieldType;

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
    rows: usize,
    cols: usize,
    frame_id: String,
) {
    let mut data = ClusterData {
        cluster_ids: Vec::new(),
        range: Vec::new(),
        rows,
        cols,
        eps: 256,
        min_pts: 4,
        wrap: false,
    };
    loop {
        let (ranges, points, time) = match drain_recv(&mut rx).await {
            Some(v) => v,
            None => return,
        };
        let start = std::time::Instant::now();
        data.range = ranges.into_iter().map(|v| v as RangeType).collect();
        let clusters = cluster_(&mut data);
        trace!("Clustering takes {:?}", start.elapsed());

        let (msg, enc) = match format_points_clustered(
            &points,
            &clusters,
            rows * cols,
            time,
            frame_id.clone(),
        ) {
            Ok(v) => v,
            Err(e) => {
                error!("Could not encode clustered PCD: {:?}", e);
                return;
            }
        };
        match publ.put(msg).encoding(enc).await {
            Ok(_) => {}
            Err(e) => error!("cluster publish error: {:?}", e),
        }
    }
}

pub fn cluster_(data: &mut ClusterData) -> Vec<u32> {
    assert!(data.range.len() == data.rows * data.cols);
    // let index = |r, c| r * cols_ + c;
    data.cluster_ids = vec![0; data.rows * data.cols];
    let mut id = 1;
    for row in 0..data.rows {
        for col in 0..data.cols {
            let c = Coord(row as isize, col as isize);
            if data.get_id(c).unwrap() > 0 {
                continue;
            }
            if expand_cluster(data, c, id) {
                id += 1;
            }
        }
    }
    let mut c = Vec::new();
    std::mem::swap(&mut c, &mut data.cluster_ids);
    c
}

fn get_valid_neighbours(data: &ClusterData, coord: Coord) -> Vec<Coord> {
    let r = data.get_range(coord).unwrap();
    // if r <= 1.0 {
    //     return Vec::new();
    // }
    OFFSETS
        .iter()
        .filter_map(|x| {
            let new_coord = coord + *x;
            let r_ = data.get_range(new_coord)?;
            if r.abs_diff(r_) < data.eps {
                // println!("r-r_ = {}", r - r_);
                Some(new_coord)
            } else {
                None
            }
        })
        .collect()
}

fn expand_cluster(data: &mut ClusterData, coord: Coord, id: u32) -> bool {
    let mut queue = vec![];
    let valid = get_valid_neighbours(data, coord);
    if valid.len() >= data.min_pts {
        data.set_id(coord, id);
        for c in valid {
            // println!("\t\tc={:?}", c);
            // all the connected points are at least an Edge
            if data.get_id(c).unwrap() == 0 {
                data.set_id(c, id);
                queue.push(c);
            }
        }
    } else {
        return false;
    }
    while let Some(coord) = queue.pop() {
        let valid = get_valid_neighbours(data, coord);
        // println!("2. Current={:?}\tValid={:?}", coord, valid);

        // this coordinate is a Core
        if valid.len() >= data.min_pts {
            for c in valid {
                // println!("\t\tc={:?}", c);
                // all the connected points are at least an Edge
                if data.get_id(c).unwrap() == 0 {
                    data.set_id(c, id);
                    queue.push(c);
                }
            }
        }
    }
    true
}

fn format_points_clustered(
    points: &Points,
    cluster_ids: &[u32],
    n_points: usize,
    timestamp: Time,
    frame_id: String,
) -> Result<(ZBytes, Encoding), cdr::Error> {
    const N: usize = 4;

    let fields = vec![
        PointField {
            name: String::from("x"),
            offset: 0,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        PointField {
            name: String::from("y"),
            offset: 4,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        PointField {
            name: String::from("z"),
            offset: 8,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        PointField {
            name: String::from("cluster_id"),
            offset: 12,
            datatype: PointFieldType::UINT32 as u8,
            count: 1,
        },
        PointField {
            name: String::from("reflect"),
            offset: 16,
            datatype: PointFieldType::UINT8 as u8,
            count: 1,
        },
    ];

    let mut data = vec![0u8; 17 * n_points];

    for index in (0..n_points).step_by(N) {
        let x = Simd::<f32, N>::from_slice(&points.x[index..index + N]);
        let x = x.to_le_bytes();
        let y = Simd::<f32, N>::from_slice(&points.y[index..index + N]);
        let y = y.to_le_bytes();
        let z = Simd::<f32, N>::from_slice(&points.z[index..index + N]);
        let z = z.to_le_bytes();
        let id = Simd::<u32, N>::from_slice(&cluster_ids[index..index + N]);
        let id = id.to_le_bytes();

        let out = index * 17;
        data[out..out + 4].copy_from_slice(&x[..4]);
        data[out + 4..out + 8].copy_from_slice(&y[..4]);
        data[out + 8..out + 12].copy_from_slice(&z[..4]);
        data[out + 12..out + 16].copy_from_slice(&id[..4]);
        data[out + 16] = points.l[index];

        let out = (index + 1) * 17;
        data[out..out + 4].copy_from_slice(&x[4..8]);
        data[out + 4..out + 8].copy_from_slice(&y[4..8]);
        data[out + 8..out + 12].copy_from_slice(&z[4..8]);
        data[out + 12..out + 16].copy_from_slice(&id[4..8]);
        data[out + 16] = points.l[index];

        let out = (index + 2) * 17;
        data[out..out + 4].copy_from_slice(&x[8..12]);
        data[out + 4..out + 8].copy_from_slice(&y[8..12]);
        data[out + 8..out + 12].copy_from_slice(&z[8..12]);
        data[out + 12..out + 16].copy_from_slice(&id[8..12]);
        data[out + 16] = points.l[index];

        let out = (index + 3) * 17;
        data[out..out + 4].copy_from_slice(&x[12..16]);
        data[out + 4..out + 8].copy_from_slice(&y[12..16]);
        data[out + 8..out + 12].copy_from_slice(&z[12..16]);
        data[out + 12..out + 16].copy_from_slice(&id[12..16]);
        data[out + 16] = points.l[index];
    }

    #[allow(clippy::needless_range_loop)]
    for index in n_points - n_points % N..n_points {
        let x = points.x[index].to_le_bytes();
        let y = points.y[index].to_le_bytes();
        let z = points.z[index].to_le_bytes();
        let id = cluster_ids[index].to_le_bytes();
        let l = points.l[index];

        let out = index * 13;
        data[out..out + 4].copy_from_slice(&x);
        data[out + 4..out + 8].copy_from_slice(&y);
        data[out + 8..out + 12].copy_from_slice(&z);
        data[out + 12..out + 16].copy_from_slice(&id);
        data[out + 16] = l;
    }

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

    let msg = ZBytes::from(cdr::serialize::<_, _, CdrLe>(&msg, Infinite)?);
    let enc = Encoding::APPLICATION_CDR.with_schema("sensor_msgs/msg/PointCloud2");

    Ok((msg, enc))
}

#[cfg(test)]
mod tests {
    use std::io::Write;

    use crate::cluster::RangeType;

    use super::{cluster_, ClusterData};

    #[test]
    fn test_cluster() {
        let mut r1 = vec![00, 10, 10, 00, 00, 00, 00, 00];
        let mut r2 = vec![00, 10, 10, 00, 10, 10, 00, 00];
        let mut r3 = vec![00, 10, 10, 10, 10, 10, 00, 00];
        let mut r4 = vec![00, 10, 10, 10, 10, 10, 10, 10];

        let mut c1 = vec![0, 2, 2, 0, 0, 1, 1, 1];
        let mut c2 = vec![0, 2, 2, 0, 2, 2, 1, 1];
        let mut c3 = vec![0, 2, 2, 2, 2, 2, 1, 1];
        let mut c4 = vec![0, 2, 2, 2, 2, 2, 2, 0];

        let mut range = Vec::new();
        range.append(&mut r1);
        range.append(&mut r2);
        range.append(&mut r3);
        range.append(&mut r4);

        let mut data = ClusterData {
            cluster_ids: Vec::new(),
            range,
            rows: 4,
            cols: 8,
            eps: 1,
            min_pts: 4,
            wrap: false,
        };

        let mut c = Vec::new();
        c.append(&mut c1);
        c.append(&mut c2);
        c.append(&mut c3);
        c.append(&mut c4);

        let clusters = cluster_(&mut data);
        assert_eq!(clusters, c,);
    }

    #[test]
    fn test_cluster_25k() {
        let mut data = ClusterData {
            cluster_ids: vec![0; 1024 * 24],
            range: vec![0; 1024 * 24],
            rows: 64,
            cols: 384,
            eps: 3,
            min_pts: 4,
            wrap: false,
        };
        let start = std::time::Instant::now();
        let clusters = cluster_(&mut data);
        println!("Elapsed: {:?}", start.elapsed());
        for c in clusters {
            assert_eq!(c, 1);
        }
    }

    #[test]
    fn test_cluster_data() {
        let d = include_bytes!("../depth.l16");
        let ptr = d as *const u8 as *const u16;
        let d_range_type: &[u16] = unsafe { std::slice::from_raw_parts(ptr, 64 * 382) };
        println!("d_RangeType.len()={}", d_range_type.len());
        let range: Vec<RangeType> = d_range_type.iter().map(|v| (*v) as RangeType).collect();
        let mut data = ClusterData {
            cluster_ids: vec![0; d_range_type.len()],
            range: vec![0; range.len()],
            rows: 64,
            cols: 382,
            eps: 256,
            min_pts: 4,
            wrap: false,
        };
        let _ = cluster_(&mut data);
        let _ = cluster_(&mut data);
        data.range = range;
        let clusters = cluster_(&mut data);
        print!("{:?}", clusters);
        let mut file = std::fs::File::create("testdata/clustered.l8").unwrap();
        // Write a slice of bytes to the file
        for c in clusters {
            file.write_all(&((c % 256) as u8).to_be_bytes()).unwrap();
        }
    }
}
