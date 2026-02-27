# EdgeFirst LiDAR Publisher

[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Rust](https://img.shields.io/badge/Rust-2024_Edition-orange.svg)](https://www.rust-lang.org/)
[![EdgeFirst Studio](https://img.shields.io/badge/EdgeFirst-Studio-green)](https://edgefirst.studio)

High-performance LiDAR point cloud publisher connecting Ouster sensors to the Zenoh messaging framework with ROS2-compatible serialization, optimized for edge AI perception pipelines on resource-constrained platforms.

**Part of the [EdgeFirst Perception Middleware](https://doc.edgefirst.ai/test/perception/)** — A collection of highly-optimized Rust services delivering the building blocks for spatial perception systems on edge hardware. The EdgeFirst Perception stack provides ROS2-compatible messaging over Zenoh for cameras, LiDAR, radar, IMU, GPS, and AI inference services.

## Features

- **Multi-Sensor Support** - Ouster OS1-64 and Robosense E1R with native UDP protocol implementations
- **Zenoh Messaging** - Low-latency pub/sub with configurable QoS (priority, congestion control)
- **ROS2 Compatible** - CDR-serialized sensor_msgs/PointCloud2, sensor_msgs/Image, geometry_msgs/TransformStamped
- **Point Cloud Clustering** - DBSCAN (accurate) and voxel (fast) spatial clustering with NEON SIMD acceleration
- **Ground Plane Removal** - IMU-guided PCA ground filter prevents floor from merging nearby objects
- **Bridge Threshold** - Prevents thin structures (ropes, wires, railings) from merging separate clusters
- **Pipeline Instrumentation** - Per-stage timing logs and Tracy profiler integration for performance analysis
- **Hardware Optimized** - NEON SIMD on aarch64 for distance checks and point formatting
- **EdgeFirst Studio Integration** - Seamless deployment and monitoring
- **Performance Profiling** - Tracy profiler integration for optimization

## Sensor Support

**Ouster:**
- OS1-64 RevD (firmware 2.5.3)
- Packet format: RNG15_RFL8_NIR8 (15-bit range, 8-bit reflectivity, 8-bit NIR)
- HTTP API for configuration and calibration

**Robosense:**
- E1R solid-state LiDAR (~20k points per frame at 10Hz)
- MSOP (point cloud data) and DIFOP (device info + IMU) UDP protocols
- Built-in IMU for ground plane detection

**References:**
- [Ouster HTTP API v1](https://static.ouster.dev/sensor-docs/image_route1/image_route2/common_sections/API/http-api-v1.html)
- [Ouster Sensor Data](https://static.ouster.dev/sensor-docs/image_route1/image_route2/sensor_data/sensor-data.html)

## Quick Start

### Prerequisites

- Rust toolchain (see `rust-toolchain.toml` for version)
- Ouster LiDAR sensor on the network (static IP recommended: 192.168.1.x)
- Zenoh router (optional, for multi-node deployments)

### Installation

```bash
# Clone the repository
git clone https://github.com/EdgeFirstAI/lidarpub.git
cd lidarpub

# Build the project
cargo build --release

# The binary will be available at
./target/release/lidarpub
```

### Basic Usage

```bash
# Ouster: connect to sensor and publish point clouds
lidarpub --sensor-type ouster <SENSOR_IP>

# Robosense E1R: listen for broadcast UDP packets
lidarpub --sensor-type robosense

# Enable voxel clustering (fast, recommended for real-time)
lidarpub --sensor-type robosense --clustering voxel

# Enable DBSCAN clustering (accurate, higher CPU)
lidarpub --sensor-type robosense --clustering dbscan

# Full pipeline: ground filter + voxel clustering with bridge separation
lidarpub --sensor-type robosense \
    --clustering voxel \
    --ground-filter \
    --ground-thickness 300 \
    --clustering-bridge 10
```

### Configuration Options

All parameters can be set via CLI arguments or environment variables:

```bash
# View all options
lidarpub --help

# Clustering parameters:
# --clustering <ALG>        Algorithm: "" (disabled), "dbscan", "voxel"
# --clustering-eps <MM>     Distance threshold in mm (default: 200)
# --clustering-minpts <N>   Min points per cluster (default: 4)
# --clustering-bridge <N>   Bridge threshold (default: 0, see below)

# Ground filter parameters:
# --ground-filter           Enable IMU-guided ground plane removal
# --ground-thickness <MM>   Slab thickness above ground to remove (default: 150)
# --sensor-height <MM>      Fixed sensor height (skips auto-detection)

# Or use environment variables (useful for systemd services):
SENSOR_TYPE=robosense CLUSTERING=voxel GROUND_FILTER=true lidarpub
```

### Understanding Bridge Threshold

The `--clustering-bridge` parameter controls how easily thin structures can connect
two separate clusters. When set higher than `--clustering-minpts`:

- Points/voxels with fewer than `bridge` neighbors are **labeled** as part of a
  cluster but don't **expand** it further
- This prevents ropes, wires, railings, and other thin structures from merging
  two dense objects into one cluster
- Recommended starting value: 8-12 for indoor scenes

### Ground Filter

The `--ground-filter` flag enables automatic floor removal using the sensor's IMU.
This is important for clustering because without it, nearby objects standing on the
same floor will be connected through shared floor points and merged into one cluster.

The filter uses the IMU accelerometer to determine which direction is "down", then
applies PCA-based ground detection on a polar grid to find the floor surface. Points
at or below the floor (plus a configurable thickness margin) are removed.

**First-frame diagnostics** are logged at INFO level showing the detected gravity
vector, ground height, and number of ground points removed — useful for verifying
correct operation.

**Pipeline timing** is logged every 100 frames showing average per-stage milliseconds.
Example:
```
pipeline avg over 100 frames (24967 pts): valid=0.2ms ground=9.3ms cluster=13.2ms relabel=0.5ms format=3.4ms publish=2.6ms total=29.1ms
```

## Documentation

- 📚 **[User Guide](docs/)** - Detailed setup and configuration
- 🔧 **[API Documentation](https://docs.rs/lidarpub)** - Rust API reference
- 🎓 **[EdgeFirst Studio Integration](https://docs.edgefirst.ai/studio/integration)** - Deploy to production
- 💻 **[Hardware Optimization](https://docs.edgefirst.ai/hardware)** - Platform-specific tuning

## Architecture

The LiDAR Publisher implements an event-driven pipeline with async processing:

**Data Flow:**
1. **UDP Receiver** - Captures raw LiDAR packets from Ouster sensor
2. **Frame Builder** - Assembles packets into complete point cloud frames
3. **Point Cloud Processor** - Applies transformations and optional clustering
4. **Zenoh Publisher** - Distributes processed data via pub/sub messaging
5. **Transform Publisher** - Broadcasts sensor TF frames for localization

**Message Format:**
- **Serialization**: CDR (Common Data Representation) for ROS2 compatibility
- **Published Topics**:
  - `{lidar_topic}/points` → `sensor_msgs/msg/PointCloud2` (XYZ + reflectivity)
  - `{lidar_topic}/depth` → `sensor_msgs/msg/Image` (range image, mono16)
  - `{lidar_topic}/reflect` → `sensor_msgs/msg/Image` (reflectivity image, mono8)
  - `{lidar_topic}/clusters` → `sensor_msgs/msg/PointCloud2` (clustered points, when enabled)
  - `rt/tf_static` → `geometry_msgs/msg/TransformStamped` (sensor transform)

**Transport:**
- Zenoh pub/sub with configurable QoS
- Priority: `DataHigh` for point clouds, `Background` for transforms
- Congestion control: `Drop` to prevent backpressure

**ROS2 Integration:**
- Messages are CDR-serialized and compatible with ROS2 Humble
- Schemas from [ros2/common_interfaces](https://github.com/ros2/common_interfaces/tree/humble)
- Can be consumed directly by ROS2 nodes via Zenoh bridge

## EdgeFirst Perception Middleware

This LiDAR publisher is one component of the **EdgeFirst Perception Middleware** — a modular software stack designed as a collection of services communicating over Zenoh with ROS2-compatible message encoding.

**Related Services:**
- **Camera Service** - ISP integration, H.265 encoding, streaming
- **Radar Publisher** - Radar sensor integration and processing
- **Vision Model Service** - AI inference for object detection and segmentation
- **Fusion Model Service** - Multi-sensor fusion for 3D perception
- **IMU Publisher** - Inertial measurement unit data
- **GPS/NavSat Publisher** - GNSS positioning data
- **Recorder Service** - MCAP recording for data capture and replay
- **Studio Client** - EdgeFirst Studio integration and telemetry

**Key Benefits:**
- Highly-optimized Rust implementation for low latency and minimal resource usage
- ROS2 CDR message encoding for ecosystem compatibility
- Zenoh pub/sub for efficient, scalable communication
- Modular architecture - use only the services you need
- Designed for edge AI perception on resource-constrained platforms

**Learn More:** [EdgeFirst Perception Documentation](https://doc.edgefirst.ai/test/perception/)

## Hardware Platforms

EdgeFirst LiDAR Publisher is optimized for edge AI platforms with resource-constrained environments:

### Supported Platforms

**Maivin** - Vision Module
- **Processor:** NXP i.MX 8M Plus
- **Network:** Gigabit Ethernet
- **OS:** Embedded Linux

**Raivin** - Vision + Radar Module
- **Processor:** NXP i.MX 8M Plus
- **Radar:** SmartMicro DRVEGRD-169
- **Network:** Gigabit Ethernet
- **OS:** Embedded Linux

**Generic ARM64** - Development and Testing
- **Examples:** NVIDIA Jetson, Raspberry Pi 4/5, embedded Linux boards
- **Requirements:** ARM64 architecture, Gigabit Ethernet, Linux kernel 5.x+

### Sensor Compatibility

**Ouster OS1-64 LiDAR** (Tested and Optimized)
- **Firmware:** 2.5.3 (tested), likely compatible with 2.x series
- **Packet Format:** RNG15_RFL8_NIR8 (15-bit range, 8-bit reflectivity, 8-bit NIR)
- **Resolution Modes:** 512x10, 1024x10, 2048x10 (columns × Hz)
- **Network:** Gigabit Ethernet (UDP ports 7502/7503)
- **Configuration:** HTTP API for sensor setup and calibration

**Future Support Planned:**
- Additional Ouster packet formats (RNG19_RFL8_SIG16_NIR16, LEGACY)
- Newer Ouster firmware (3.x series)
- Multi-vendor support (Velodyne, Livox under consideration)

### Cross-Compilation

Build for ARM64 platforms from x86_64 development machines:

```bash
# Install cross-compilation tool
cargo install cross

# Build for ARM64 (Maivin, Raivin, NXP i.MX8, Jetson)
cross build --release --target aarch64-unknown-linux-gnu

# Deploy to target device
scp target/aarch64-unknown-linux-gnu/release/lidarpub user@target-device:/usr/local/bin/
```

## Support

### Community Resources

- 📚 **[Documentation](https://doc.edgefirst.ai/perception/lidar)** - Complete LiDAR integration guides, API reference, and tutorials
- 💬 **[GitHub Discussions](https://github.com/EdgeFirstAI/lidarpub/discussions)** - Ask questions, share ideas, and connect with the community
- 🐛 **[Issue Tracker](https://github.com/EdgeFirstAI/lidarpub/issues)** - Report bugs, request features, and track development
- 📖 **[Contributing Guide](CONTRIBUTING.md)** - Learn how to contribute to the project

### EdgeFirst Ecosystem

The **EdgeFirst LiDAR Publisher** is part of a comprehensive spatial perception platform:

- **[EdgeFirst Perception Middleware](https://doc.edgefirst.ai/test/perception/)** - Complete modular perception stack
  - **Camera Service** - ISP integration, H.265 encoding, multi-camera support
  - **LiDAR Publisher** - This project - Ouster sensor integration with ROS2 messaging
  - **Radar Publisher** - Radar integration and processing
  - **Vision Model Service** - AI inference for object detection, segmentation, tracking
  - **Fusion Model Service** - Multi-sensor fusion for enhanced 3D spatial perception
  - **IMU Publisher** - Inertial measurement unit data streaming
  - **GPS/NavSat Publisher** - GNSS positioning and navigation data
  - **Recorder Service** - MCAP-based recording for data capture and replay
  - **Studio Client** - EdgeFirst Studio telemetry and deployment integration
  
  All services communicate via **Zenoh** with **ROS2 CDR message encoding** for maximum interoperability.

- **[EdgeFirst Studio](https://edgefirst.studio?utm_source=github&utm_medium=readme&utm_campaign=lidarpub)** - Complete MLOps Platform for Edge AI
  - Deploy and manage perception pipelines at scale across your device fleet
  - Real-time performance monitoring, analytics, and alerting
  - Model versioning, A/B testing, and automated rollback
  - Visual workflow builder for perception pipelines
  - Free tier available for development and small deployments
  - Enterprise support with SLAs and dedicated engineering

- **[EdgeFirst Modules](https://www.edgefirst.ai/edgefirstmodules?utm_source=github&utm_medium=readme&utm_campaign=lidarpub)** - Rugged Sensor Platforms
  - **Maivin** - Vision module (NXP i.MX 8M Plus processor)
  - **Raivin** - Vision + radar module (NXP i.MX 8M Plus processor, SmartMicro DRVEGRD-169 radar)
  - **Deployment:** Standard configurations, development kits, and fully custom options available
  - **Use Cases:** Autonomous vehicles, robotics, industrial automation, surveillance

### Professional Services

Au-Zone Technologies offers comprehensive support for production deployments of spatial perception systems:

- **Training & Workshops** - Accelerate your team's perception expertise with hands-on training
  - EdgeFirst Perception architecture and best practices
  - Sensor calibration and integration (LiDAR, radar, cameras)
  - ROS2 and Zenoh messaging patterns
  - Performance optimization for edge platforms

- **Custom Development** - Extend perception capabilities for your autonomous systems
  - Custom sensor drivers and integration
  - Specialized AI models for your domain (agriculture, mining, construction, etc.)
  - Multi-sensor fusion algorithms
  - Real-time object tracking and prediction

- **Integration Services** - Connect EdgeFirst Perception with your existing systems
  - ROS2 ecosystem integration
  - Custom middleware and protocol bridges
  - Cloud platform connectivity (AWS IoT, Azure IoT, Google Cloud)
  - Legacy system modernization

- **Enterprise Support** - Production-grade support with guaranteed response times
  - SLA-backed support (4-hour, 24-hour, or custom response times)
  - Priority bug fixes and security patches
  - Dedicated technical account manager
  - Direct access to EdgeFirst engineering team
  - Custom feature development roadmap alignment

📧 **Contact:** support@au-zone.com | 🌐 **Learn more:** [au-zone.com](https://au-zone.com?utm_source=github&utm_medium=readme&utm_campaign=lidarpub)

## Performance

Optimized for real-time performance on edge platforms:

- **Low Latency** - <10ms frame processing target
- **Efficient Memory** - Zero-copy data paths where possible
- **SIMD Acceleration** - Vectorized point transformations
- **Configurable Threading** - Tokio async runtime for concurrent operations

## Contributing

We welcome contributions! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for development setup and guidelines.

This project follows our [Code of Conduct](CODE_OF_CONDUCT.md).

## Troubleshooting

### Sensor Connection Issues

**Ouster sensor not reachable:**
```bash
# Verify network connectivity
ping 192.168.1.100

# Check sensor HTTP API
curl http://192.168.1.100/api/v1/sensor/metadata

# Verify firewall allows UDP ports 7502-7503
sudo iptables -L | grep 750
```

**Robosense E1R not receiving data:**
- The E1R broadcasts UDP packets — no `TARGET` is needed
- Verify MSOP port (default 6699) and DIFOP port (default 7788) are not blocked
- Use `--discover` to confirm the sensor is visible on the network
- If DIFOP bind fails ("Address already in use"), kill the stale process:
  `sudo fuser -k 7788/udp`

**No point cloud data:**
- Ensure sensor is powered and fully booted
- For Ouster: verify UDP ports 7502/7503 are accessible and lidar mode matches
- For Robosense: check that no other process is bound to the MSOP/DIFOP ports

### Ground Filter Issues

**Floor still visible after enabling `--ground-filter`:**
- Check the first-frame IMU diagnostic log for `ground_removed` count
- If `ground_removed` is low, increase `--ground-thickness` (try 300mm)
- If the gravity vector looks wrong, the IMU axis remap may need adjustment
- Near-field floor should be caught — the filter has no range gate on classification

**Ground filter removing too many points (low objects disappearing):**
- Decrease `--ground-thickness` (e.g. 100mm)
- Objects shorter than the thickness above the floor will be removed

**Wrong region being cleared (e.g. wall instead of floor):**
- Check the `gravity=` values in the first-frame diagnostic log
- Gravity should point toward the floor (largest component in the expected axis)
- If 90° off, the IMU axis remap in `src/main.rs` may need correction

**Delay in floor detection during fast motion:**
- The IMU updates at ~10Hz (DIFOP packet rate)
- EMA smoothing (alpha=0.5) adds ~300ms convergence time
- For fixed mounts this is not an issue; for moving platforms consider
  reducing `EMA_ALPHA` in `src/ground.rs`

### Clustering Issues

**Objects merging that should be separate:**
- Increase `--clustering-bridge` (try 8-12) to prevent thin connections
- Decrease `--clustering-eps` for tighter distance threshold
- Enable `--ground-filter` if floor points are connecting objects

**Too many small clusters (fragmentation):**
- Increase `--clustering-eps` for more aggressive merging
- Decrease `--clustering-minpts` to accept smaller groups

**Clustering too slow (frame budget exceeded):**
- Switch from `dbscan` to `voxel` (~5x faster on typical scenes)
- Check the pipeline timing log for the `cluster=` value
- DBSCAN on E1R ~25k points: ~70ms; Voxel: ~13ms (aarch64)

### Performance Issues

**High CPU usage:**
- Use `--clustering=voxel` instead of `dbscan`
- Disable clustering entirely if not needed (`--clustering=""`)
- Check pipeline timing log for bottleneck identification
- Enable Tracy (`--tracy`) for detailed profiling

**Frame drops:**
- Increase network buffer size: `sudo sysctl -w net.core.rmem_max=8388608`
- Ensure Gigabit Ethernet connection
- Check Zenoh subscriber backpressure

### Build Issues

**Cross-compilation for aarch64:**
```bash
# Using cargo-zigbuild (recommended)
cargo install cargo-zigbuild
cargo zigbuild --target aarch64-unknown-linux-gnu.2.35 --release

# Deploy to target
scp target/aarch64-unknown-linux-gnu/release/edgefirst-lidarpub user@target:~/
```

### Getting Help

- **Documentation:** https://doc.edgefirst.ai/perception/lidar
- **GitHub Issues:** https://github.com/EdgeFirstAI/lidarpub/issues
- **GitHub Discussions:** https://github.com/EdgeFirstAI/lidarpub/discussions
- **Email Support:** support@au-zone.com

## Security

For security vulnerabilities, please see [SECURITY.md](SECURITY.md) or email support@au-zone.com with subject "Security Vulnerability".

## License

Apache License 2.0 - see [LICENSE](LICENSE) for details.

Copyright 2025 Au-Zone Technologies

## Acknowledgments

- Ouster sensor SDK and documentation
- Zenoh project for high-performance messaging
- ROS2 community for sensor message standards
- Open source contributors (see [NOTICE](NOTICE))
