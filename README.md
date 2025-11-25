# EdgeFirst LiDAR Publisher

[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Rust](https://img.shields.io/badge/Rust-2024_Edition-orange.svg)](https://www.rust-lang.org/)
[![EdgeFirst Studio](https://img.shields.io/badge/EdgeFirst-Studio-green)](https://edgefirst.studio)

High-performance LiDAR point cloud publisher connecting Ouster sensors to the Zenoh messaging framework with ROS2-compatible serialization, optimized for edge AI perception pipelines on resource-constrained platforms.

**Part of the [EdgeFirst Perception Middleware](https://doc.edgefirst.ai/test/perception/)** — A collection of highly-optimized Rust services delivering the building blocks for spatial perception systems on edge hardware. The EdgeFirst Perception stack provides ROS2-compatible messaging over Zenoh for cameras, LiDAR, radar, IMU, GPS, and AI inference services.

## Features

- ✨ **Ouster LiDAR Support** - Native integration with Ouster OS1-64 (firmware 2.5.3, RNG15_RFL8_NIR8 packet format)
- 🚀 **Zenoh Messaging** - Low-latency pub/sub with configurable QoS (priority, congestion control)
- 📊 **ROS2 Compatible** - CDR-serialized sensor_msgs/PointCloud2, sensor_msgs/Image, geometry_msgs/TransformStamped
- 🎯 **Point Cloud Clustering** - Built-in spatial clustering for object detection
- ⚡ **Hardware Optimized** - SIMD-accelerated processing for edge platforms
- 🔌 **EdgeFirst Studio Integration** - Seamless deployment and monitoring
- 🔍 **Rerun Visualization** - Optional real-time 3D visualization support
- 📈 **Performance Profiling** - Tracy profiler integration for optimization

## Sensor Support

**Current:**
- Ouster OS1-64 RevD (firmware 2.5.3)
- Packet format: RNG15_RFL8_NIR8 (15-bit range, 8-bit reflectivity, 8-bit NIR)

**Future:**
- Additional Ouster packet formats (RNG19_RFL8_SIG16_NIR16, LEGACY)
- Newer Ouster firmware (3.x)
- Multi-vendor support (Velodyne, Livox)

**References:**
- [Ouster HTTP API v1](https://static.ouster.dev/sensor-docs/image_route1/image_route2/common_sections/API/http-api-v1.html)
- [Ouster Sensor Data](https://static.ouster.dev/sensor-docs/image_route1/image_route2/sensor_data/sensor-data.html)
- [Lidar Packet Format](https://static.ouster.dev/sensor-docs/image_route1/image_route2/sensor_data/sensor-data.html#lidar-data-packet-format)

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
# Connect to Ouster sensor and publish point clouds
lidarpub --target <SENSOR_IP>

# Specify custom lidar mode
lidarpub --target <SENSOR_IP> --lidar-mode 1024x10

# Enable clustering
lidarpub --target <SENSOR_IP> --cluster

# With Rerun visualization (example)
cargo run --example lidar_rerun --features rerun -- --target <SENSOR_IP>
```

### Configuration Options

```bash
# View all options
lidarpub --help

# Common parameters:
# --target <IP>           Ouster sensor IP address
# --lidar-mode <MODE>     Sensor resolution mode (512x10, 1024x10, 2048x10)
# --timestamp-mode <MODE> TIME_FROM_INTERNAL_OSC, TIME_FROM_SYNC_PULSE_IN, etc.
# --cluster               Enable point cloud clustering
# --cluster-threshold <M> Distance threshold for clustering (default: 0.5m)
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

**Sensor not reachable:**
```bash
# Verify network connectivity
ping 192.168.1.100

# Check sensor HTTP API
curl http://192.168.1.100/api/v1/sensor/metadata

# Verify firewall allows UDP ports 7502-7503
sudo iptables -L | grep 750
```

**No point cloud data:**
- Ensure sensor is powered and fully booted (LED indicators)
- Verify UDP ports 7502 (LiDAR data) and 7503 (IMU) are accessible
- Check lidar mode matches sensor configuration
- Try resetting sensor via HTTP API: `curl -X POST http://192.168.1.100/api/v1/system/reset`

### Performance Issues

**High CPU usage:**
- Disable clustering if not needed (remove `--cluster` flag)
- Reduce sensor resolution (use 512x10 instead of 2048x10)
- Check for other processes consuming CPU
- Verify SIMD optimizations compiled (check `portable_simd` feature)

**Frame drops:**
- Increase network buffer size: `sudo sysctl -w net.core.rmem_max=8388608`
- Ensure Gigabit Ethernet connection (not 100Mbps)
- Check Zenoh subscriber backpressure
- Monitor with `--log-level debug` for diagnostics

### Zenoh Communication Issues

**Messages not received by subscribers:**
```bash
# Test Zenoh connectivity
zenoh-ping -l tcp/127.0.0.1:7447

# Check if messages are being published
zenoh-cli --mode client --connect tcp/127.0.0.1:7447 subscribe '/lidar/*'
```

**ROS2 bridge not working:**
- Verify Zenoh-DDS bridge is running
- Check topic name mapping configuration
- Validate CDR serialization with ROS2 tools: `ros2 topic echo /lidar/points`

### Build Issues

**Cross-compilation fails:**
```bash
# Ensure cross tool is up to date
cargo install cross --force

# Use Docker-based cross-compilation
cross build --release --target aarch64-unknown-linux-gnu
```

**SIMD compilation errors:**
- Ensure nightly Rust toolchain: `rustup default nightly`
- Check `portable_simd` feature is enabled in Cargo.toml
- Fallback to scalar implementation by disabling feature (reduced performance)

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
