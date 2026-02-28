# Testing

This document describes the testing strategy for EdgeFirst LiDAR Publisher, including
automated CI testing, unit tests for each module, and manual hardware testing procedures.

## Overview

EdgeFirst LiDAR Publisher uses a three-tier testing approach:

1. **Automated Testing (CI)**: Unit tests, linting, and static analysis on GitHub runners
2. **Unit Tests**: Per-module tests covering clustering, ground filtering, formats, and drivers
3. **Manual Testing**: Hardware integration testing with Ouster and Robosense LiDAR sensors

## Automated Testing (CI)

The GitHub Actions workflow (`.github/workflows/test.yml`) runs on every push and PR:

| Job | Description |
|-----|-------------|
| Format Check | Verifies code formatting with `cargo fmt` |
| Clippy Lint | Static analysis with `cargo clippy` |
| Unit Tests | Runs `cargo test` with coverage collection |
| SonarCloud | Uploads coverage and performs code quality analysis |
| Security Audit | Checks dependencies for known vulnerabilities |
| License Check | Verifies all dependencies have compatible licenses |

### Running Tests Locally

```bash
# Run all unit tests
cargo test

# Run tests with verbose output
cargo test -- --nocapture

# Run a specific test module
cargo test cluster::tests
cargo test ground::tests

# Run a specific test by name
cargo test test_ground_filter_e1r_real_data -- --nocapture

# Check formatting
cargo fmt --check

# Run Clippy lints
cargo clippy -- -D warnings
```

## Unit Test Modules

### Clustering (`src/cluster.rs`)

Tests for both DBSCAN and voxel clustering algorithms:

| Test | Description |
|------|-------------|
| `test_cluster_spatial` | DBSCAN with HashMap spatial hash on structured grid |
| `test_cluster_spatial_flat` | DBSCAN with flat spatial hash on structured grid |
| `test_cluster_unstructured` | DBSCAN with HashMap on irregular point positions |
| `test_cluster_unstructured_flat` | DBSCAN with flat hash on irregular point positions |
| `test_cluster_25k_spatial` | DBSCAN performance with 25k points (HashMap) |
| `test_cluster_25k_spatial_flat` | DBSCAN performance with 25k points (flat hash) |
| `test_cluster_hashmap_vs_flat` | Verifies HashMap and flat hash produce identical results |
| `test_dbscan_cluster_e1r_real_data` | DBSCAN on real E1R point cloud (`testdata/e1r_frame0.pcd`) |
| `test_voxel_cluster_two_groups` | Voxel BFS separates two distant point groups |
| `test_voxel_cluster_adjacent_voxels_merge` | Adjacent occupied voxels merge into one cluster |
| `test_voxel_cluster_all_noise` | Sparse points below min_pts are labeled noise |
| `test_voxel_cluster_reuse_second_call` | State correctly resets between frames |
| `test_voxel_cluster_unstructured` | Voxel clustering on irregular point positions |
| `test_voxel_cluster_e1r_real_data` | Voxel on real E1R point cloud (`testdata/e1r_frame0.pcd`) |
| `test_voxel_cluster_ouster_real_data` | Voxel on real Ouster point cloud (`testdata/ouster_frame0.pcd`) |
| `test_voxel_vs_dbscan_agreement` | Verifies voxel and DBSCAN produce structurally similar results |

```bash
# Run all clustering tests
cargo test cluster::tests -- --nocapture

# Run only DBSCAN tests
cargo test cluster::tests::test_cluster -- --nocapture
cargo test cluster::tests::test_dbscan -- --nocapture

# Run only voxel tests
cargo test cluster::tests::test_voxel -- --nocapture

# Run real-data tests (requires testdata/ PCD files)
cargo test cluster::tests::test_dbscan_cluster_e1r_real_data -- --nocapture
cargo test cluster::tests::test_voxel_cluster_e1r_real_data -- --nocapture
```

### Ground Filter (`src/ground.rs`)

Tests for the IMU-guided ground plane removal:

| Test | Description |
|------|-------------|
| `test_ground_filter_flat_floor` | Synthetic flat floor at known height is detected and removed |
| `test_ground_filter_tilted_sensor` | Ground detection works with a tilted gravity vector |
| `test_ground_filter_no_imu` | Filter is a no-op when no IMU data is available |
| `test_ground_filter_known_height` | `sensor_height` override bypasses PCA detection |
| `test_ground_filter_rejects_walls` | Vertical surfaces are not classified as ground |
| `test_ground_filter_reuse` | State correctly resets between frames |
| `test_ground_filter_e1r_real_data` | Ground filter on real E1R point cloud with IMU data |

```bash
# Run all ground filter tests
cargo test ground::tests -- --nocapture

# Run the real-data ground filter test
cargo test ground::tests::test_ground_filter_e1r_real_data -- --nocapture
```

The real-data test loads `testdata/e1r_frame0.pcd` and verifies:
- Ground points are detected and labeled with cluster ID 1
- A reasonable number of floor points are removed (typically 300-1500 for E1R)
- Non-ground points are preserved for clustering

### Formats (`src/formats.rs`)

Tests for the PointCloud2 serialization:

| Test | Description |
|------|-------------|
| `test_format_points_13byte` | Unclustered format: 13 bytes/point (x, y, z, intensity) |
| `test_format_clustered_17byte` | Clustered format: 17 bytes/point (x, y, z, intensity, cluster_id) |
| `test_format_into_preallocated` | Buffer reuse across frames |
| `test_point_field_builders` | PointField descriptor construction |

```bash
cargo test formats::tests -- --nocapture
```

### Robosense Driver (`src/robosense.rs`)

Tests for E1R packet parsing and frame assembly:

```bash
cargo test robosense::tests -- --nocapture
```

### Ouster Driver (`src/ouster.rs`)

Tests for Ouster packet parsing:

```bash
cargo test ouster::tests -- --nocapture
```

### Test Data

Real-data tests use PCD files in the `testdata/` directory:

| File | Description |
|------|-------------|
| `testdata/e1r_frame0.pcd` | Robosense E1R frame, ~20,512 points |
| `testdata/ouster_frame0.pcd` | Ouster OS1 frame |

These files are checked into the repository. Tests that depend on them are skipped
if the file is not found (e.g., in stripped CI environments).

## Manual Hardware Testing

Integration testing requires a LiDAR sensor connected to the network.

### Prerequisites

- Linux system with network access to the sensor
- Ouster OS0/OS1/OS2 sensor (firmware v2.x or v3.x), or Robosense E1R
- Gigabit Ethernet connection (recommended)
- Rust stable toolchain

### Building

```bash
# Build release binary (native)
cargo build --release

# Cross-compile for aarch64 (Maivin / Torizon)
cargo zigbuild --target aarch64-unknown-linux-gnu.2.35 --release
```

### Sensor Connectivity

#### Ouster

```bash
# Ping the sensor
ping os-122122000149.local

# Query sensor metadata via HTTP API
curl http://os-122122000149.local/api/v1/sensor/metadata

# Check current sensor configuration
curl http://os-122122000149.local/api/v1/sensor/metadata | jq '.config_params'
```

#### Robosense E1R

```bash
# Discover Robosense sensors on the network
SENSOR_TYPE=robosense DISCOVER=true ./target/release/edgefirst-lidarpub

# Expected output:
# Robosense E1R device info, serial: XXXXXXXXXXXX, firmware: X.X.X, ...
```

The E1R broadcasts UDP packets to 255.255.255.255 by default — no TARGET is
needed. If you have multiple E1R sensors, set `TARGET` to filter by source IP.

### Test Scenarios

#### 1. Basic Point Cloud (No Clustering)

```bash
# Ouster
SENSOR_TYPE=ouster TARGET=os-122122000149.local \
  ./target/release/edgefirst-lidarpub

# Robosense E1R
SENSOR_TYPE=robosense \
  ./target/release/edgefirst-lidarpub
```

Verify: Zenoh topic `rt/lidar/points` receives PointCloud2 messages at the
sensor's frame rate.

#### 2. Voxel Clustering

```bash
SENSOR_TYPE=robosense CLUSTERING=voxel CLUSTERING_EPS=200 CLUSTERING_MINPTS=4 \
  ./target/release/edgefirst-lidarpub
```

Verify: `rt/lidar/clusters` publishes clustered point clouds with cluster IDs
(17 bytes/point). Pipeline timing log appears every 100 frames.

#### 3. DBSCAN Clustering

```bash
SENSOR_TYPE=robosense CLUSTERING=dbscan CLUSTERING_EPS=200 CLUSTERING_MINPTS=4 \
  ./target/release/edgefirst-lidarpub
```

Verify: Same output as voxel but with finer-grained clusters. Expect higher
cluster time (~70ms on E1R aarch64 vs ~13ms for voxel).

#### 4. Ground Filter + Clustering

```bash
SENSOR_TYPE=robosense CLUSTERING=voxel GROUND_FILTER=true GROUND_THICKNESS=150 \
  ./target/release/edgefirst-lidarpub
```

Verify the first-frame diagnostic log:
```
IMU raw=(X, Y, Z) gravity=(gx, gy, gz) ground_height=Some(H) ground_removed=N ...
```

Check that:
- `gravity` vector is roughly unit-length and points along expected axis
- `ground_height` is a reasonable distance in meters
- `ground_removed` shows floor points being filtered (typically 300-1500 for E1R)
- Pipeline timing shows `ground` stage taking ~5-9ms on E1R

#### 5. Bridge Threshold

```bash
SENSOR_TYPE=robosense CLUSTERING=voxel CLUSTERING_BRIDGE=10 GROUND_FILTER=true \
  ./target/release/edgefirst-lidarpub
```

Verify: Thin structures (railings, ropes, wires) no longer merge separate
objects into a single cluster. Compare with `CLUSTERING_BRIDGE=0` to see the
difference.

#### 6. Known Sensor Height

```bash
SENSOR_TYPE=robosense CLUSTERING=voxel GROUND_FILTER=true SENSOR_HEIGHT=680 \
  ./target/release/edgefirst-lidarpub
```

Verify: The first-frame log shows the fixed `ground_height` matching the
provided value (680mm = 0.68m), without PCA auto-detection.

#### 7. Pipeline Timing Verification

With any clustering configuration, the pipeline summary log prints every 100
frames:

```
pipeline avg over 100 frames (24967 pts): valid=0.2ms ground=9.3ms cluster=13.2ms relabel=0.5ms format=3.4ms publish=2.6ms total=29.1ms
```

Expected timing ranges on aarch64 Cortex-A with E1R (~25k points):

| Stage | Voxel | DBSCAN |
|-------|-------|--------|
| valid | <1ms | <1ms |
| ground | 5-10ms | 5-10ms |
| cluster | 10-15ms | 60-75ms |
| relabel | <1ms | <1ms |
| format | 2-4ms | 2-4ms |
| publish | 1-3ms | 1-3ms |
| **total** | **25-35ms** | **75-95ms** |

Both algorithms are well within the 100ms (10Hz) frame budget.

#### 8. Tracy Profiler

```bash
# Build with Tracy feature (if not default)
cargo build --release --features profiling

# Run with Tracy broadcast enabled
SENSOR_TYPE=robosense CLUSTERING=voxel TRACY=true \
  ./target/release/edgefirst-lidarpub
```

Connect the Tracy profiler GUI to the device. You should see named spans for
each pipeline stage: `valid_mask`, `ground_filter`, `clustering`, `relabel`,
`publish`.

### On-Target Deployment Testing

For testing on the Maivin (aarch64 Torizon):

```bash
# Cross-compile
cargo zigbuild --target aarch64-unknown-linux-gnu.2.35 --release

# Stop the running service
ssh torizon@10.10.41.236 "sudo systemctl stop lidarpub"

# Deploy
scp target/aarch64-unknown-linux-gnu/release/edgefirst-lidarpub torizon@10.10.41.236:~/lidarpub

# Run manually with desired configuration
ssh torizon@10.10.41.236 "SENSOR_TYPE=robosense CLUSTERING=voxel GROUND_FILTER=true ~/lidarpub"

# Or restart the service (uses /etc/default/lidarpub for configuration)
ssh torizon@10.10.41.236 "sudo systemctl start lidarpub"
```

To run unit tests on target:

```bash
# Cross-compile tests
cargo zigbuild --target aarch64-unknown-linux-gnu.2.35 --tests

# Deploy the test binary
scp target/aarch64-unknown-linux-gnu/debug/deps/edgefirst_lidarpub-* torizon@10.10.41.236:~/lidarpub-tests
scp -r testdata/ torizon@10.10.41.236:~/testdata

# Run tests on target
ssh torizon@10.10.41.236 "cd ~ && ./lidarpub-tests --test-threads=1 --nocapture"
```

### Verifying Zenoh Output

```bash
# Subscribe to all lidar topics
z_sub -k "rt/lidar/**"

# Subscribe to point cloud only
z_sub -k "rt/lidar/points"

# Subscribe to clustered point cloud
z_sub -k "rt/lidar/clusters"
```

## Troubleshooting

### No LiDAR Data Received

**Ouster:**

1. Verify the sensor is reachable: `ping <sensor_hostname>`
2. Check sensor status: `curl http://<sensor>/api/v1/sensor/metadata | jq '.sensor_info.status'`
3. Verify UDP destination matches your IP: `curl http://<sensor>/api/v1/sensor/metadata | jq '.config_params.udp_dest'`
4. Check firewall: `sudo ufw allow 7502/udp && sudo ufw allow 7503/udp`
5. Verify lidar mode matches: `curl http://<sensor>/api/v1/sensor/metadata | jq '.config_params.lidar_mode'`

**Robosense E1R:**

1. Check the E1R is powered and on the network
2. Use `DISCOVER=true` to verify DIFOP packets are being received
3. Check firewall allows UDP ports 6699 (MSOP) and 7788 (DIFOP)
4. If using a TARGET filter, verify the E1R's source IP matches

### Ground Filter Not Removing Floor

1. Verify IMU data is available (check first-frame log for `IMU raw=` line)
2. If `ground_height=None`, the PCA auto-detection could not find a ground plane — try setting `SENSOR_HEIGHT` manually
3. If floor points remain, increase `GROUND_THICKNESS` (e.g., 200-300)
4. If non-floor objects are being removed, decrease `GROUND_THICKNESS`
5. Check that the sensor is reasonably level; extreme tilt angles may affect detection

### Performance Issues

1. Increase network buffer size:
   ```bash
   sudo sysctl -w net.core.rmem_max=8388608
   sudo sysctl -w net.core.rmem_default=8388608
   ```

2. Use voxel clustering instead of DBSCAN for ~5x speedup

3. If total pipeline exceeds the frame period, disable ground filter or increase
   `CLUSTERING_EPS` to reduce neighbor search overhead

4. Use the pipeline timing log to identify the bottleneck stage

### Zenoh Communication Issues

1. Check Zenoh is discovering peers: `z_scout`
2. Disable multicast if the network doesn't support it:
   ```bash
   ./target/release/edgefirst-lidarpub --no-multicast-scouting --connect tcp/<router>:7447
   ```

## See Also

- [README.md](README.md) - Usage documentation and quick start
- [CONTRIBUTING.md](CONTRIBUTING.md) - Development guidelines
- [ARCHITECTURE.md](ARCHITECTURE.md) - System architecture
