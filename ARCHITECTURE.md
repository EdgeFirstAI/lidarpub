# EdgeFirst LiDAR Publisher - Architecture Documentation

**Version:** 2.1
**Last Updated:** 2026-02-27
**Project:** EdgeFirst LiDAR Publisher  
**License:** Apache-2.0

---

## Table of Contents

1. [Overview](#overview)
2. [System Architecture](#system-architecture)
3. [Module Structure](#module-structure)
4. [Data Flow](#data-flow)
5. [Threading Model](#threading-model)
6. [Message Publishing](#message-publishing)
7. [SIMD Optimization](#simd-optimization)
8. [Ground Plane Filter](#ground-plane-filter)
9. [Clustering Algorithms](#clustering-algorithms)
10. [Pipeline Instrumentation](#pipeline-instrumentation)
11. [Error Handling](#error-handling)
12. [Configuration](#configuration)

---

## Overview

The EdgeFirst LiDAR Publisher is a Rust application that receives UDP packets from Ouster or Robosense LiDAR sensors, transforms the data into 3D point clouds, optionally applies ground plane removal and spatial clustering, and publishes ROS2-compatible messages via Zenoh.

### Technology Stack

- **Language:** Rust 2024 edition (stable toolchain)
- **Async Runtime:** Tokio (multi-threaded)
- **Messaging:** Zenoh 1.7
- **Serialization:** CDR (ROS2 Common Data Representation)
- **SIMD:** NEON intrinsics on aarch64 for clustering distance checks
- **Profiling:** Tracy integration via `tracing-tracy`
- **Dependencies:** See `Cargo.toml` for complete list

### Target Platforms

**Primary:**
- Maivin (NXP i.MX 8M Plus)
- Raivin (NXP i.MX 8M Plus + SmartMicro DRVEGRD-169 radar)

**Development:**
- x86_64 Linux
- ARM64 Linux (generic)

---

## System Architecture

### Component Overview

```mermaid
graph TB
    subgraph Sensor["Ouster OS1-64 LiDAR"]
        UDP["UDP Packets<br/>Port 7502/7503"]
    end
    
    subgraph Publisher["LiDAR Publisher Process"]
        Receiver["UDP Receiver<br/>src/main.rs"]
        FrameReader["Frame Reader<br/>src/ouster.rs"]
        FrameBuilder["Frame Builder<br/>src/ouster.rs"]
        Clustering["Clustering Thread<br/>src/cluster.rs"]
        Serializer["CDR Serializer<br/>src/main.rs"]
        ZenohPub["Zenoh Publisher<br/>src/main.rs"]
        TFStatic["TF Static Loop<br/>src/main.rs"]
    end
    
    subgraph Output["Published Topics"]
        Points["/lidar/points<br/>PointCloud2"]
        Depth["/lidar/depth<br/>Image mono16"]
        Reflect["/lidar/reflect<br/>Image mono8"]
        Clusters["/lidar/clusters<br/>PointCloud2"]
        TF["rt/tf_static<br/>TransformStamped"]
    end
    
    UDP --> Receiver
    Receiver --> FrameReader
    FrameReader --> FrameBuilder
    FrameBuilder --> Clustering
    FrameBuilder --> Serializer
    Clustering --> Serializer
    Serializer --> ZenohPub
    ZenohPub --> Points
    ZenohPub --> Depth
    ZenohPub --> Reflect
    ZenohPub --> Clusters
    TFStatic --> TF
```

### Key Components

| Component | Location | Responsibility |
|-----------|----------|----------------|
| UDP Receiver | `src/main.rs::udp_receiver()` | Binds UdpSocket, receives packets asynchronously |
| Frame Reader | `src/ouster.rs::FrameReader` | Parses RNG15_RFL8_NIR8 packets, assembles frames |
| Frame Builder | `src/ouster.rs::FrameBuilder` | Applies SIMD transforms, generates point clouds |
| Clustering | `src/cluster.rs::cluster_thread()` | DBSCAN spatial clustering (optional) |
| CDR Serializer | `src/main.rs::format_points()` | Converts to ROS2 message formats |
| Zenoh Publisher | `src/main.rs::frame_processor()` | Publishes messages with QoS |
| TF Static Loop | `src/main.rs::tf_static_loop()` | Broadcasts transform frames |
| CLI Parser | `src/args.rs::Args` | Parses command-line arguments |

---

## Module Structure

### File Organization

```
src/
├── main.rs              - Entry point, sensor drivers, publishing logic
├── ouster.rs            - Ouster protocol implementation
├── robosense.rs         - Robosense E1R protocol and DIFOP/MSOP parsing
├── cluster.rs           - DBSCAN and voxel clustering (NEON SIMD)
├── cluster_thread.rs    - Clustering pipeline with instrumentation
├── ground.rs            - IMU-guided PCA ground plane filter
├── formats.rs           - PointCloud2 CDR serialization (SIMD)
├── args.rs              - CLI configuration
├── common.rs            - Shared utilities
├── lidar.rs             - Sensor type abstraction
├── packet_source.rs     - UDP/pcap packet source abstraction
└── lib.rs               - Library exports

benches/
└── cluster_bench.rs     - Criterion benchmarks for clustering

testdata/
└── e1r_frame0.pcd       - Real E1R point cloud for testing (20,512 pts)
```

### Module Dependencies

```mermaid
graph LR
    main["main.rs"]
    ouster["ouster.rs"]
    cluster["cluster.rs"]
    args["args.rs"]
    common["common.rs"]
    
    main --> args
    main --> ouster
    main --> cluster
    main --> common
    
    ouster --> common
    
    cluster --> args
    cluster --> ouster
```

### main.rs - Application Entry

**Functions:**

| Function | Description |
|----------|-------------|
| `main()` | Tokio runtime entry, Zenoh session setup |
| `udp_receiver()` | Async UDP packet reception loop |
| `frame_processor()` | Processes frames, publishes messages |
| `format_points()` | Serializes PointCloud2 (XYZR fields) |
| `format_depth()` | Serializes depth Image (mono16) |
| `format_reflect()` | Serializes reflectivity Image (mono8) |
| `tf_static_loop()` | Publishes TF transforms at 1Hz |

**Key Data Structures:**

- `PointFieldType` enum - ROS2 field type constants
- Zenoh publishers with QoS configuration
- Kanal channels for frame passing

### ouster.rs - Sensor Protocol

**Structures:**

| Type | Lines | Purpose |
|------|-------|---------|
| `Config` | 13-35 | Sensor configuration (UDP dest, mode, etc.) |
| `SensorInfo` | 37-45 | Sensor metadata (serial, firmware, etc.) |
| `LidarDataFormat` | 47-56 | Packet format description |
| `BeamIntrinsics` | 58-63 | Calibration angles and transform |
| `Parameters` | 65-70 | Complete sensor parameters |
| `Header` | 177-211 | Packet header fields |
| `Points` | 414-430 | Point cloud storage (XYZL) |
| `FrameReader` | 432-502 | Frame assembly from packets |
| `FrameBuilder` | 504-712 | SIMD point transformation |

**Key Functions:**

| Function | Lines | Description |
|----------|-------|-------------|
| `FrameReader::new()` | 444-460 | Initializes frame reader |
| `FrameReader::update()` | 462-499 | Processes packets, detects frame completion |
| `FrameBuilder::new()` | 551-629 | Initializes builder with intrinsics |
| `FrameBuilder::update()` | 631-634 | Updates images and transforms points |
| `FrameBuilder::calculate_points()` | 669-710 | SIMD-accelerated Cartesian transform |

### cluster.rs - DBSCAN Implementation

**Structures:**

| Type | Purpose |
|------|---------|-------|
| `ClusterData` | Clustering input/output data |
| `Coord` | 2D coordinate in range image |

**Functions:**

| Function | Description |
|----------|-------------|
| `cluster_thread()` | Async clustering task |
| `drain_recv()` | Receives latest message from channel |
| `cluster_()` | DBSCAN main algorithm |
| `expand_cluster()` | Recursive cluster expansion |
| `get_valid_neighbours()` | Finds nearby points within epsilon |
| `format_points_clustered()` | Serializes clustered PointCloud2 |

**Algorithm:** DBSCAN (Density-Based Spatial Clustering of Applications with Noise)
- Epsilon: Distance threshold in mm (default: 256)
- Min points: Minimum cluster size (default: 4)
- Operates on 2D range image for efficiency

### args.rs - Configuration

**CLI Arguments:**

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `--sensor-type` | Enum | `ouster` | Sensor type: `ouster` or `robosense` |
| `TARGET` | String | (optional) | Sensor IP, hostname, or pcap file |
| `--azimuth` | Vec<u32> | `0 360` | FOV start/stop degrees (Ouster) |
| `--lidar-mode` | String | `1024x10` | Column×Hz configuration (Ouster) |
| `--timestamp-mode` | Enum | `internal` | Timestamp source (Ouster) |
| `--msop-port` | u16 | `6699` | MSOP UDP port (Robosense) |
| `--difop-port` | u16 | `7788` | DIFOP UDP port (Robosense) |
| `--include-noisy` | bool | `false` | Include noisy points (Robosense) |
| `--discover` | bool | `false` | Discover sensors and exit (Robosense) |
| `--tf-vec` | Vec<f64> | `0 0 0` | Transform translation (meters) |
| `--tf-quat` | Vec<f64> | `0 0 0 1` | Transform rotation quaternion |
| `--base-frame-id` | String | `base_link` | Base frame name |
| `--frame-id` | String | `lidar` | LiDAR child frame name |
| `--lidar-topic` | String | `rt/lidar` | Topic prefix |
| `--rust-log` | Level | `info` | Log level |
| `--tracy` | bool | `false` | Enable Tracy profiler |
| `--clustering` | String | `""` | Algorithm: `""`, `dbscan`, `voxel` |
| `--clustering-eps` | u16 | `200` | Cluster distance threshold (mm) |
| `--clustering-minpts` | usize | `4` | Min points per cluster |
| `--clustering-bridge` | usize | `0` | Bridge threshold (0 = same as minpts) |
| `--ground-filter` | bool | `false` | Enable IMU ground plane removal |
| `--ground-thickness` | u16 | `150` | Ground slab thickness (mm) |
| `--sensor-height` | u16 | (auto) | Fixed sensor height (mm) |
| `--mode` | WhatAmI | `peer` | Zenoh mode |
| `--connect` | Vec<String> | `[]` | Zenoh endpoints |
| `--listen` | Vec<String> | `[]` | Zenoh listen endpoints |
| `--no-multicast-scouting` | bool | `false` | Disable multicast |

---

## Data Flow

### Frame Processing Pipeline

```mermaid
sequenceDiagram
    participant Sensor as Ouster OS1-64
    participant UDP as UDP Receiver (async)
    participant Reader as Frame Reader (ouster.rs)
    participant Builder as Frame Builder (ouster.rs)
    participant Cluster as Clustering (optional)
    participant Serialize as CDR Serializer
    participant Zenoh as Zenoh Publisher
    
    loop Every 100ms at 10Hz
        Sensor->>UDP: 128 UDP packets (12608 bytes each)
        UDP->>Reader: Raw packet bytes
        Reader->>Reader: Parse RNG15_RFL8_NIR8
        Reader->>Reader: Assemble 64x1024 frame
        Reader->>Builder: Array2 u16 depth, Array2 u8 reflect
        Builder->>Builder: SIMD transform to Cartesian XYZ
        Builder->>Cluster: Range data + points
        Cluster->>Cluster: DBSCAN clustering
        Builder->>Serialize: Points, depth, reflect
        Cluster->>Serialize: Clustered points
        Serialize->>Serialize: Convert to ROS2 CDR
        Serialize->>Zenoh: PointCloud2, Images
        Zenoh->>Zenoh: Publish to topics
    end
```

### Packet to Point Cloud

```mermaid
flowchart TD
    A["UDP Packet: 12608 bytes"] --> B{"Parse Header: 32 bytes"}
    B --> C["Extract: frame_id, timestamp, status"]
    
    B --> D["Parse Columns: 16 per packet"]
    D --> E["For each column: parse 64 rows"]
    E --> F["Extract per pixel: range 15-bit, reflect 8-bit, NIR 8-bit"]
    
    F --> G["Build Array2: 64 rows x 1024 cols"]
    G --> H["Apply pixel shift per beam"]
    H --> I["SIMD Transform to Cartesian"]
    
    I --> J["Points struct: x, y, z, l vectors"]
```

### Message Types

**Published Messages:**

```mermaid
graph LR
    subgraph PointCloud2
        PC_Header["Header: timestamp, frame_id"]
        PC_Fields["Fields: x,y,z,reflect"]
        PC_Data["Data: byte array"]
    end
    
    subgraph Image_Depth
        ID_Header["Header"]
        ID_Encoding["mono16"]
        ID_Data["16-bit depth: 64x1024"]
    end
    
    subgraph Image_Reflect
        IR_Header["Header"]
        IR_Encoding["mono8"]
        IR_Data["8-bit reflect: 64x1024"]
    end
    
    subgraph TransformStamped
        TF_Header["Header"]
        TF_Child["child_frame_id"]
        TF_Transform["translation, rotation"]
    end
```

---

## Threading Model

### Thread Architecture

```mermaid
graph TB
    subgraph Main["Main Thread"]
        Init["Initialize Tokio Runtime, Parse CLI args, Configure Zenoh"]
    end
    
    subgraph Tokio["Tokio Thread Pool (4 workers)"]
        Worker1["Worker 1: UDP recv_from"]
        Worker2["Worker 2: Zenoh publishing"]
        Worker3["Worker 3: HTTP requests"]
        Worker4["Worker 4: Logging/tracing"]
    end
    
    subgraph Dedicated["Dedicated Threads"]
        FrameProc["Frame Processor: SIMD transforms, blocking compute"]
        ClusterT["Clustering Thread: DBSCAN algorithm, blocking compute"]
    end
    
    Main --> Tokio
    Tokio --> FrameProc
    FrameProc --> ClusterT
    
    style Main fill:#e1f5ff
    style Tokio fill:#fff4e1
    style Dedicated fill:#ffe1e1
```

### Channel Communication

The application uses `kanal` bounded channels for inter-thread communication:

**Frame Channel** (created in `main()`):
- Producer: `udp_receiver()`
- Consumer: `frame_processor()`
- Capacity: Unbounded
- Data: `(u64 timestamp, u16 frame_id, Array2<u16> depth, Array2<u8> reflect)`

**Clustering Channel** (created in `frame_processor()`):
- Producer: `frame_processor()`
- Consumer: `cluster_thread()`
- Capacity: 8 frames
- Data: `(Vec<u16> range, Points, Time timestamp)`

---

## Message Publishing

### Zenoh Configuration

**Session Setup** (in `main()`):
```rust
let session = zenoh::open(args.clone()).await.unwrap();
```

**QoS Settings:**

| Topic | Priority | Congestion Control | Rationale |
|-------|----------|-------------------|-----------|----------|
| `/lidar/points` | DataHigh | Drop | Real-time sensor data |
| `/lidar/depth` | DataHigh | Drop | Real-time sensor data |
| `/lidar/reflect` | DataHigh | Drop | Real-time sensor data |
| `/lidar/clusters` | DataHigh | Drop | Real-time sensor data |
| `rt/tf_static` | Background | Drop | Low-frequency broadcast |

See `frame_processor()` for publisher declarations.

### Topic Structure

```mermaid
graph TD
    Base["Base Topic: --lidar-topic arg, default rt/lidar"]
    
    Base --> Points["/points: sensor_msgs/PointCloud2, XYZR fields"]
    Base --> Depth["/depth: sensor_msgs/Image, mono16 encoding"]
    Base --> Reflect["/reflect: sensor_msgs/Image, mono8 encoding"]
    Base --> Clusters["/clusters: sensor_msgs/PointCloud2, XYZR + cluster_id"]
    
    Root["Root Topic: rt/"] --> TF["tf_static: geometry_msgs/TransformStamped"]
```

### CDR Serialization

**PointCloud2 Format** (`format_points()`):
- Fields: x (FLOAT32), y (FLOAT32), z (FLOAT32), reflect (UINT8)
- Point step: 13 bytes (4+4+4+1)
- Data layout: Interleaved XYZR

**Image Format** (`format_depth()` and `format_reflect()`):
- Depth: mono16, 2 bytes per pixel, little-endian
- Reflectivity: mono8, 1 byte per pixel
- Dimensions: 64 rows × 1024 cols (configurable by mode)

**Transform Format** (`tf_static_loop()`):
- Published at 1Hz
- Static transform from base_link to lidar frame
- Configurable via `--tf-vec` and `--tf-quat` args

---

## SIMD Optimization

### Implementation

The SIMD implementation uses Rust's `portable_simd` feature for cross-platform vectorization.

**Location:** `FrameBuilder::calculate_points()` in `src/ouster.rs`

**Algorithm:**
```mermaid
flowchart TD
    A["Input: range Array2 u16, reflect Array2 u8"] --> B["Iterate rows 0..64"]
    B --> C["For each column in crop range"]
    C --> D["Extract: range value, cached x_range, y_range, altitude angle"]
    
    D --> E{"Process in SIMD chunks N=4"}
    E -->|Full chunk| F["Simd f32x4: r, x_range, y_range, altitude"]
    E -->|Remainder| G["Scalar f32: process individually"]
    
    F --> H["Compute: x, y, z from range and angles"]
    
    G --> H
    
    H --> I["Scale by 0.001: mm to meters"]
    I --> J["Copy to Points: x, y, z vectors"]
```

**SIMD Lane Count:**
- Const generic `N=4` for f32x4 (128-bit SIMD)
- Supported platforms: ARM NEON, x86 SSE
- Fallback: Scalar loop for remainder points

**Precomputation:**
The implementation caches intermediate values (in `FrameBuilder::update_points()`):
- `x_range_cache`, `y_range_cache`: Range component per pixel
- `x_delta_cache`, `y_delta_cache`: Delta component per pixel
- `altitude`: Sine of altitude angle per row

This reduces per-point computation and improves SIMD efficiency.

---

## Ground Plane Filter

**Implementation:** `src/ground.rs` — `GroundFilter` struct

The ground plane filter uses IMU data to determine the gravity direction, then applies
region-wise PCA (inspired by the Patchwork algorithm) to detect and remove floor points
before clustering. This prevents nearby objects from being connected through shared
floor points.

### Algorithm

1. **Gravity vector** — The IMU accelerometer reading (remapped to LiDAR frame
   coordinates) is normalized to a unit gravity vector. All subsequent height
   calculations project points onto this axis via dot product.

2. **Polar grid binning** — Valid points are binned into a polar grid of 16 azimuth
   sectors × 8 range rings (out to 30m). Points closer than 0.5m are excluded from
   detection to avoid sensor housing artifacts.

3. **Per-patch PCA** — For each occupied patch, the lowest-K points along the gravity
   axis (K=20) are selected as seeds. A 3×3 covariance matrix is computed from the
   seeds and its eigenvalues are found using Cardano's closed-form formula for cubic
   roots (no iterative solver needed). The patch is accepted as ground if:
   - **Uprightness** > 0.85 — the smallest eigenvector aligns with gravity
   - **Flatness** < 0.03 — the smallest eigenvalue ratio indicates a plane
   - **Elevation** within ±1.0m of the current ground height estimate

4. **Ground height** — The median seed height across all accepted patches establishes
   the ground level. An EMA (alpha=0.5) smooths this across frames with a 0.5m jump
   gate for fast convergence on large changes.

5. **Classification** — Every point whose height along the gravity axis is at or below
   `ground_height - thickness_m` is marked as ground. This is one-sided: anything below
   the ground surface is always removed. No range gate is applied during classification
   so near-field floor points are also caught.

### Sensor Height Override

When `--sensor-height` is set, steps 2–4 are skipped and the provided height is used
directly. This is useful when the sensor is in a fixed mount and auto-detection is
unreliable (e.g. looking straight down with few ground patches visible).

### IMU Axis Remap (Robosense E1R)

The E1R's built-in IMU axes differ from its LiDAR point cloud frame. The remap
applied in `src/main.rs` is:

```
LiDAR = (imu_z, -imu_x, -imu_y)
```

This was empirically validated by tilting the sensor and confirming the ground filter
removes the correct region.

### Cluster ID Scheme

When the ground filter is active, cluster IDs follow a reserved scheme:

| ID | Meaning |
|----|---------|
| 0  | Noise (invalid returns, too-few-neighbors) |
| 1  | Ground plane |
| 2+ | Real clusters |

Constants `CLUSTER_ID_NOISE`, `CLUSTER_ID_GROUND`, and `CLUSTER_ID_FIRST` are defined
in `src/cluster.rs`.

---

## Clustering Algorithms

Two clustering algorithms are available, selected via `--clustering`:

### DBSCAN (`--clustering=dbscan`)

**Implementation:** `cluster_()` and `expand_cluster()` in `src/cluster.rs`

A full 3D DBSCAN using a spatial hash for O(n) average neighbor queries. Points are
binned into voxels of size `eps`, and neighbor searches check the 27 adjacent voxels
(3×3×3 cube). On aarch64, distance checks use NEON SIMD intrinsics to process 4
points per iteration.

**Spatial hash variants:**
- `SpatialHash` — `HashMap<(i32,i32,i32), Vec<usize>>` (default, backward compatible)
- `FlatSpatialHash` — Open-addressing hash table with contiguous point storage for
  better cache locality. Used by `ClusterData::new_flat()`.

**Parameters:**
- `eps` (CLUSTERING_EPS): 3D Euclidean distance threshold in mm (default: 200)
- `min_pts` (CLUSTERING_MINPTS): Minimum neighbors to be a core point (default: 4)
- `bridge_pts` (CLUSTERING_BRIDGE): Minimum neighbors for a point to **propagate**
  during BFS expansion. When > min_pts, border points with fewer neighbors are
  assigned to the cluster but don't expand it, preventing thin structures (ropes,
  wires) from merging distinct dense clusters.

**Performance:** ~70ms per frame on E1R 25k points (aarch64 Cortex-A).

### Voxel Connected-Component (`--clustering=voxel`)

**Implementation:** `voxel_cluster()` in `src/cluster.rs`

A faster, coarser alternative that operates at voxel granularity rather than individual
points. Points are binned into the same `FlatSpatialHash`, then a BFS over 26-connected
occupied voxels merges adjacent voxels into clusters.

**Bridge support:** When `bridge_pts > min_pts`, voxels with fewer than `bridge_pts`
points are labeled (assigned to the nearest cluster) but don't propagate BFS expansion.
A second pass absorbs unvisited sparse voxels that are adjacent to labeled clusters,
analogous to DBSCAN noise absorption.

**Trade-off:** Objects within one voxel width of each other will always merge (the
algorithm cannot distinguish them). For 200mm eps, this means objects closer than
200mm are always one cluster.

**Performance:** ~13ms per frame on E1R 25k points (aarch64 Cortex-A) with bridge=10.

### Output Format

**Clustered PointCloud2** (published to `{lidar_topic}/clusters`):
- Fields: x (FLOAT32), y (FLOAT32), z (FLOAT32), cluster_id (UINT32), intensity (UINT8)
- Point step: 17 bytes (4+4+4+4+1)
- Cluster IDs: 0 = noise, 1 = ground (when filter active), 2+ = real clusters

---

## Pipeline Instrumentation

**Implementation:** `src/cluster_thread.rs`

Each stage of the clustering pipeline is wrapped in a `tracing::info_span!()` that
integrates with both the tracing subscriber (journald, console) and Tracy when enabled
via `--tracy`. Additionally, `std::time::Instant` accumulators track per-stage timing
and log an average summary every 100 frames at INFO level.

### Pipeline Stages

| Stage | Span Name | Description |
|-------|-----------|-------------|
| 1 | `valid_mask` | Mark origin (0,0,0) points as invalid |
| 2 | `ground_filter` | PCA ground detection + classification |
| 3 | `clustering` | DBSCAN or voxel clustering |
| 4 | `relabel` | Remap cluster IDs (ground=1, real=2+) |
| 5 | `format_points_clustered` | CDR serialize PointCloud2 |
| 6 | `publish` | Zenoh put |

### Example Timing Log

```
pipeline avg over 100 frames (24967 pts): valid=0.2ms ground=9.3ms cluster=13.2ms relabel=0.5ms format=3.4ms publish=2.6ms total=29.1ms
```

### Tracy Integration

When running with `--tracy`, all spans appear as zones in the Tracy profiler GUI.
Frame marks are emitted at the end of each LiDAR frame. The `profiling` build feature
adds allocation tracking and system-level tracing.

---

## Error Handling

### Error Types

**ouster::Error** (defined in `src/ouster.rs`):

| Variant | Source | Recovery |
|---------|--------|----------|
| `IoError` | Network I/O | Log and retry |
| `SystemTimeError` | Timestamp | Log and continue |
| `ShapeError` | ndarray operations | Fatal error |
| `UnsupportedDataFormat` | Sensor config | Fatal error |
| `UnexpectedEndOfSlice` | Packet parsing | Skip packet |
| `UnknownPacketType` | Packet parsing | Skip packet |
| `TooManyColumns` | Frame assembly | Skip packet |
| `InsufficientColumns` | Frame assembly | Wait for more |
| `UnsupportedRows` | Sensor config | Fatal error |

### Error Propagation

```mermaid
flowchart TD
    A["UDP Packet Received"] --> B{"Parse Header"}
    B -->|Error| C["Log Warning, Continue to Next"]
    B -->|Success| D{"Valid Packet Type?"}
    
    D -->|No| C
    D -->|Yes| E{"Parse Columns"}
    
    E -->|Error| C
    E -->|Success| F{"Frame Complete?"}
    
    F -->|No| A
    F -->|Yes| G{"Transform Points"}
    
    G -->|Error| H["Log Error, Skip Frame"]
    G -->|Success| I{"Publish Messages"}
    
    I -->|Error| J["Log Error, Continue"]
    I -->|Success| A
```

**Strategy:**
- Network errors: Log and continue (resilient to packet loss)
- Parsing errors: Skip packet, wait for next
- Publishing errors: Log and continue (subscriber may reconnect)
- Configuration errors: Fatal (exit with error message)

### Logging

**Levels** (configured via `--rust-log` argument):
- `error`: Fatal errors, application exits
- `warn`: Recoverable errors, packet drops
- `info`: Frame processing, publication events
- `debug`: Detailed diagnostics
- `trace`: Per-packet logging

**Subscribers** (configured in `main()`):
- `tracing-subscriber`: Console output (pretty format)
- `tracing-journald`: systemd journal (production)
- `tracing-tracy`: Tracy profiler (optional)

---

## Configuration

### Sensor Configuration

**HTTP API** (accessed in `main()`):
- Endpoint: `http://{target}/api/v1/sensor`
- Methods:
  - `GET /sensor/metadata`: Retrieve sensor parameters
  - `POST /sensor/config`: Update sensor configuration

**Config Structure** (defined in `src/ouster.rs`):
```rust
pub struct Config {
    pub udp_dest: String,           // Destination IP
    pub udp_port_lidar: u16,        // Data port (7502)
    pub udp_profile_lidar: String,  // RNG15_RFL8_NIR8
    pub lidar_mode: String,         // 512x10, 1024x10, 2048x10
    pub azimuth_window: [u32; 2],   // [0, 360000] (millidegrees)
    pub timestamp_mode: String,     // Internal OSC, PTP, etc.
}
```

### Runtime Parameters

**Environment Variables:**
All CLI arguments support environment variable equivalents:
```bash
export TARGET="192.168.1.100"
export LIDAR_MODE="1024x10"
export RUST_LOG="info"
export CLUSTERING="true"
```

**Zenoh Configuration** (via `Args` to `Config` conversion):
- Mode: peer (default), client, router
- Connect endpoints: `--connect tcp/127.0.0.1:7447`
- Listen endpoints: `--listen tcp/0.0.0.0:7447`
- Multicast scouting: Enabled by default, disable with `--no-multicast-scouting`

### Build Profiles

**Cargo Profiles** (defined in `Cargo.toml`):

| Profile | Optimization | LTO | Debug Symbols | Strip | Use Case |
|---------|-------------|-----|---------------|-------|----------|
| `debug` | 0 | No | Yes | No | Development |
| `release` | 3 | Yes | No | Yes | Production |
| `profiling` | 3 | Yes | Yes | No | Tracy profiling |

**Features:**
- `default`: Includes Tracy (lightweight overhead)
- `rerun`: Enables Rerun visualization dependencies
- `tracy`: Tracy profiler integration
- `profiling`: Tracy with sampling and system tracing

---

## Deployment

### Cross-Compilation

**ARM64 Target:**
```bash
# Install cross tool
cargo install cross

# Build for aarch64
cross build --release --target aarch64-unknown-linux-gnu

# Output location
target/aarch64-unknown-linux-gnu/release/lidarpub
```

### Systemd Service

**Example Unit File:**
```ini
[Unit]
Description=EdgeFirst LiDAR Publisher
After=network.target

[Service]
Type=simple
ExecStart=/usr/local/bin/lidarpub \
    --target 192.168.1.100 \
    --clustering \
    --clustering-eps 256 \
    --clustering-minpts 4
Restart=always
RestartSec=5
Environment="RUST_LOG=info"

[Install]
WantedBy=multi-user.target
```

### Network Setup

**Sensor Network:**
- Ouster default IP: 192.168.1.100
- UDP ports: 7502 (LiDAR data), 7503 (IMU data)
- Firewall: Allow incoming UDP on 7502/7503

**Zenoh Network:**
- Default: Peer mode with multicast scouting
- Production: Client mode with explicit router endpoints
- Ports: TCP 7447 (default Zenoh router)

---

## Testing

**Unit Tests:**
- `src/cluster.rs` test module: DBSCAN clustering validation
  - `test_cluster()`: Verifies clustering behavior
  - `test_cluster_25k()`: Performance test with large dataset

**Running Tests:**
```bash
# All tests
cargo test

# With output
cargo test -- --nocapture

# Specific test
cargo test test_cluster
```

**Current Coverage:** ~5% (2 tests)  
**Target Coverage:** 70% minimum per SPS-07

---

## Tracy Profiling

### Usage

**Build with Tracy:**
```bash
cargo build --release --features tracy
```

**Run with Tracy:**
```bash
./target/release/lidarpub --target 192.168.1.100 --tracy
```

**Frame Markers:**
The application marks frame boundaries using `tracy_client::frame_mark()` at the end of each iteration in `frame_processor()`.

**Instrumentation:**
- `tracing` spans automatically captured by `tracing-tracy`
- Manual instrumentation: `#[instrument]` attribute on functions
- Allocations tracked when built with `profiling` feature

---

## Code Locations Reference

### Key Functions by File

**src/main.rs:**
- `main()` - Application entry, Zenoh session
- `udp_receiver()` - UDP packet reception loop
- `frame_processor()` - Frame processing and publishing
- `format_points()` - PointCloud2 serialization
- `format_depth()` - Depth image serialization
- `format_reflect()` - Reflectivity image serialization
- `tf_static_loop()` - TF static broadcasting

**src/ouster.rs:**
- `FrameReader::new()` - Initialize frame reader
- `FrameReader::update()` - Process packets, detect frames
- `FrameBuilder::new()` - Initialize with intrinsics
- `FrameBuilder::update()` - Update images and points
- `FrameBuilder::calculate_points()` - SIMD transform

**src/cluster.rs:**
- `cluster_thread()` - Clustering async task
- `cluster_()` - DBSCAN main algorithm
- `expand_cluster()` - Recursive expansion
- `format_points_clustered()` - Clustered PC2 serialization

**src/args.rs:**
- `Args` struct - CLI argument definitions
- `From<Args> for Config` - Zenoh config conversion
