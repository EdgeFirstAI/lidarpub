# EdgeFirst LiDAR Publisher

[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![EdgeFirst Studio](https://img.shields.io/badge/EdgeFirst-Studio-green)](https://edgefirst.studio)

High-performance LiDAR point cloud publisher connecting Ouster sensors to the Zenoh messaging framework with ROS2-compatible serialization, optimized for edge AI perception pipelines.

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
- Ouster LiDAR sensor on the network
- Zenoh router (optional, for multi-node deployments)

### Installation

```bash
# Clone the repository
git clone https://bitbucket.org/au-zone/lidarpub.git
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

# With Rerun visualization
cargo run --release --features rerun --bin lidar-rerun -- --target <SENSOR_IP>
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

## Support

### Community Resources

- 📚 **[Documentation](docs/)** - Start here for guides and examples
- 💬 **[GitHub Discussions](https://bitbucket.org/au-zone/lidarpub/discussions)** - Ask questions and share ideas
- 🐛 **[Issue Tracker](https://bitbucket.org/au-zone/lidarpub/issues)** - Report bugs and request features

### EdgeFirst Ecosystem

- **[EdgeFirst Perception Middleware](https://doc.edgefirst.ai/test/perception/)** - Complete spatial perception stack
  - Camera, LiDAR, radar, IMU, GPS integration
  - AI inference services (vision, fusion models)
  - Recording, replay, and visualization tools

- **[EdgeFirst Studio](https://edgefirst.studio)** - Complete MLOps Platform
  - Deploy and manage perception pipelines at scale
  - Real-time performance monitoring and analytics
  - Model versioning and A/B testing
  - Free tier available for development

- **[EdgeFirst Modules](https://www.edgefirst.ai/edgefirstmodules)** - Rugged Sensor Platforms
  - **Maivin** - Vision-only module (4K/8MP, EdgeFirst Perception Engine, ROS2 interface)
  - **Raivin** - Maivin + integrated 4D radar with sensor fusion for enhanced spatial perception
  - Both modules: IP67-rated, -40°C to +65°C, designed for harsh environments
  - Standard configurations, development kits, and full custom options

### Professional Services

Au-Zone Technologies offers comprehensive support for production deployments:

- **Training & Workshops** - Accelerate your team's perception expertise
- **Custom Development** - Extend capabilities for autonomous systems
- **Integration Services** - Connect with ROS2, custom middleware, and cloud platforms
- **Enterprise Support** - SLAs, priority fixes, and dedicated engineering access

📧 Contact: support@au-zone.com | 🌐 Learn more: [au-zone.com](https://au-zone.com)

## Performance

Optimized for real-time performance on edge platforms:

- **Low Latency** - <10ms frame processing on Maivin platforms
- **Efficient Memory** - Zero-copy data paths where possible
- **SIMD Acceleration** - Vectorized point transformations
- **Configurable Threading** - Tokio async runtime for concurrent operations

Benchmarks on Maivin platform (Renesas RZ/V2H):
- 1024x10 mode @ 10Hz: ~8ms per frame
- Point cloud clustering: ~2ms per frame
- Total CPU usage: <15% single core

## Contributing

We welcome contributions! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for development setup and guidelines.

This project follows our [Code of Conduct](CODE_OF_CONDUCT.md).

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
