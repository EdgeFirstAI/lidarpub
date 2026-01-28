# Testing

This document describes the testing strategy for EdgeFirst LiDAR Publisher, including
automated CI testing and manual hardware testing procedures.

## Overview

EdgeFirst LiDAR Publisher uses a two-tier testing approach:

1. **Automated Testing (CI)**: Unit tests and static analysis on GitHub runners
2. **Manual Testing**: Hardware integration testing with Ouster LiDAR sensors

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

# Run specific test
cargo test cluster::tests::test_cluster

# Check formatting
cargo fmt --check

# Run Clippy lints
cargo clippy -- -D warnings
```

## Manual Hardware Testing

Integration testing requires an Ouster LiDAR sensor connected to the network.

### Prerequisites

- Linux system with network access to the sensor
- Ouster OS0/OS1/OS2 sensor (firmware v2.x or v3.x)
- Gigabit Ethernet connection (recommended)
- Rust toolchain (nightly)

### Building

```bash
# Build release binary
cargo build --release

# The binary is at target/release/edgefirst-lidarpub
```

### Verifying Sensor Connectivity

Before running the publisher, verify the sensor is accessible:

```bash
# Ping the sensor (use hostname or IP)
ping os-122122000149.local

# Query sensor metadata via HTTP API
curl http://os-122122000149.local/api/v1/sensor/metadata

# Check current sensor configuration
curl http://os-122122000149.local/api/v1/sensor/metadata | jq '.config_params'
```

### Running the Publisher

```bash
# Basic usage - sensor hostname/IP is a positional argument
./target/release/edgefirst-lidarpub os-122122000149.local

# Match the sensor's configured lidar mode
./target/release/edgefirst-lidarpub os-122122000149.local --lidar-mode 512x20

# Enable point cloud clustering
./target/release/edgefirst-lidarpub os-122122000149.local --clustering

# Specify azimuth field of view (degrees)
./target/release/edgefirst-lidarpub os-122122000149.local --azimuth 0 180

# Enable Tracy profiler for performance analysis
./target/release/edgefirst-lidarpub os-122122000149.local --tracy

# Connect to a specific Zenoh router
./target/release/edgefirst-lidarpub os-122122000149.local --connect tcp/192.168.1.1:7447
```

### Verifying Output

Use Zenoh tools to verify messages are being published:

```bash
# Install zenoh tools if needed
cargo install zenoh

# Subscribe to point cloud messages
z_sub -k "rt/lidar/points"

# Subscribe to all lidar topics
z_sub -k "rt/lidar/**"

# Check depth image output
z_sub -k "rt/lidar/depth"
```

### Expected Console Output

When running correctly, you should see:

```
INFO zenoh::net::runtime: Using ZID: ...
INFO zenoh::net::runtime::orchestrator: Zenoh can be reached at: tcp/...
INFO lidarpub: Config { udp_dest: "...", udp_port_lidar: 7502, ... }
Waiting for LiDAR to initialize....
```

After initialization completes, frame processing begins and point clouds are published.

### Common Test Scenarios

#### 1. Basic Connectivity Test

```bash
# Verify the sensor responds and publisher initializes
timeout 30 ./target/release/edgefirst-lidarpub os-122122000149.local --lidar-mode 512x20
```

#### 2. Different LiDAR Modes

```bash
# Test each supported mode (must match sensor configuration)
./target/release/edgefirst-lidarpub <sensor> --lidar-mode 512x10
./target/release/edgefirst-lidarpub <sensor> --lidar-mode 1024x10
./target/release/edgefirst-lidarpub <sensor> --lidar-mode 2048x10
./target/release/edgefirst-lidarpub <sensor> --lidar-mode 512x20
./target/release/edgefirst-lidarpub <sensor> --lidar-mode 1024x20
```

#### 3. Clustering Performance

```bash
# Test with clustering enabled (CPU intensive)
./target/release/edgefirst-lidarpub <sensor> --clustering --clustering-eps 256 --clustering-minpts 4
```

#### 4. Frame Transformation

```bash
# Test with custom TF transform (translation + rotation)
./target/release/edgefirst-lidarpub <sensor> --tf-vec 0.1 0 0.5 --tf-quat 0 0 0.707 0.707
```

## Troubleshooting

### No LiDAR Data Received

1. **Check network connectivity:**
   ```bash
   ping <sensor_hostname>
   ```

2. **Verify sensor is running:**
   ```bash
   curl http://<sensor>/api/v1/sensor/metadata | jq '.sensor_info.status'
   # Should return "RUNNING"
   ```

3. **Check UDP destination matches your IP:**
   ```bash
   curl http://<sensor>/api/v1/sensor/metadata | jq '.config_params.udp_dest'
   ```

4. **Verify firewall allows UDP ports 7502-7503:**
   ```bash
   sudo ufw allow 7502/udp
   sudo ufw allow 7503/udp
   ```

5. **Check lidar mode matches sensor configuration:**
   ```bash
   curl http://<sensor>/api/v1/sensor/metadata | jq '.config_params.lidar_mode'
   ```

### Performance Issues

1. **Increase network buffer size:**
   ```bash
   sudo sysctl -w net.core.rmem_max=8388608
   sudo sysctl -w net.core.rmem_default=8388608
   ```

2. **Reduce resolution if CPU-bound:**
   ```bash
   # Use lower resolution mode
   ./target/release/edgefirst-lidarpub <sensor> --lidar-mode 512x10
   ```

3. **Disable clustering if not needed:**
   ```bash
   # Run without --clustering flag
   ./target/release/edgefirst-lidarpub <sensor>
   ```

### Zenoh Communication Issues

1. **Check Zenoh is discovering peers:**
   ```bash
   z_scout
   ```

2. **Disable multicast if network doesn't support it:**
   ```bash
   ./target/release/edgefirst-lidarpub <sensor> --no-multicast-scouting --connect tcp/<router>:7447
   ```

## Unit Tests

The clustering module (`src/cluster.rs`) contains unit tests:

- `test_cluster`: Tests DBSCAN clustering with known patterns
- `test_cluster_25k`: Performance test with 25,000 points

Run with:
```bash
cargo test cluster::tests
```

## See Also

- [README.md](README.md) - Usage documentation and quick start
- [CONTRIBUTING.md](CONTRIBUTING.md) - Development guidelines
- [ARCHITECTURE.md](ARCHITECTURE.md) - System architecture
