# Changelog

All notable changes to the EdgeFirst LiDAR Publisher will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [2.0.1] - 2026-02-24

### Added
- `lidarpub.default` configuration file for `/etc/default/lidarpub` with documented
  settings for both Ouster and Robosense sensor types, published as a release artifact

### Fixed
- Empty `TARGET` environment variable causing silent startup failure in Robosense
  mode when set via `/etc/default/lidarpub` (empty string parsed as invalid IP)

### Changed
- Updated dependencies: clap 4.5.60, edgefirst-schemas 1.5.5, env_logger 0.11.9,
  libc 0.2.182, rerun 0.29.2, ureq 3.2.0, kanal rev 9a4b98d

## [2.0.0] - 2026-02-23

### Added
- **Robosense E1R solid-state LiDAR support** with MSOP/DIFOP packet parsing,
  120° H × 90° V FOV, ~26k points/frame at 10Hz
- **Multi-sensor `LidarFrame` trait abstraction** with client-owned frame pattern
  enabling sensor-agnostic processing pipeline
- **`LidarDriver` trait** providing unified `process()` interface for all sensor types
- `--sensor-type` CLI argument to select between `ouster` and `robosense` drivers
- `--msop-port` / `--difop-port` CLI arguments for Robosense port configuration
- `--include-noisy` flag to include filtered Robosense noisy points (PointAttribute == 2)
- `--discover` flag for parallel multi-sensor network discovery (Robosense via passive
  DIFOP listening, Ouster via mDNS `_roger._tcp`)
- Robosense DIFOP parsing: serial number, firmware version, network config, time sync,
  IMU data (accelerometer + gyroscope) published on `rt/lidar/imu`
- Robosense return mode extraction from MSOP header (Dual/Strongest/Last/Nearest)
- PCAP replay support via `PacketSource` trait with IP fragment reassembly for
  testing LiDAR drivers without hardware (`pcap` feature)
- `pcap_viewer` example for offline PCAP replay with Rerun visualization
- Criterion benchmarks for both drivers: full pipeline, per-packet throughput,
  frame operations, and cross-driver comparison (`benches/driver_bench.rs`)
- Format points benchmark (`benches/format_points_bench.rs`)
- Integration tests using PCAP captures for both Ouster and Robosense drivers
- Git LFS test data: `os1_frames.pcap`, `e1r_frames.pcap`, `os1_sensor_info.json`

### Changed
- **BREAKING:** Replaced `FrameBuilder` with `LidarFrame` / `LidarFrameWriter` trait
  pattern — clients now own frame objects and provide mutable references to drivers
- **BREAKING:** Removed legacy `buffer.rs` module (replaced by trait-based architecture)
- Switched toolchain from nightly to stable Rust; nightly remains optional for
  `portable_simd` on non-aarch64 targets
- Ouster driver refactored to true zero-copy architecture — SIMD
  `calculate_points_fused_into()` writes directly to frame buffers
- Fused single-pass XYZ calculation with pre-computed azimuth/elevation coefficients
- Native NEON SIMD intrinsics on aarch64 for vectorized point cloud generation
- `target` argument now optional for Robosense (required for Ouster); when provided,
  filters MSOP packets by source IP
- Simplified `lidar_viewer` example (replaced multi-file `lidar_rerun/` directory)
- Updated CI workflows to use stable Rust and explicit feature lists
- Removed nightly-only rustfmt options; reformatted with stable rustfmt
- Updated CycloneDX CLI checksum in SBOM workflow
- Added pcap feature dependencies to NOTICE file

### Fixed
- Robosense frame boundary reset bug: frames no longer accumulate across boundaries
  when packet counter wraps (added `needs_reset` flag)
- Robosense DIFOP temperature overflow in parsing
- Frame buffer overflow warning when push exceeds capacity

### Removed
- Legacy `FrameBuilder` and `buffer.rs` module
- 14 unused fields and 9 legacy methods from Ouster driver
- `.cargo/config.toml` build configuration

### Performance
- Zero-copy point cloud processing: computation now dominates (~20%) rather than
  memory copies on imx8mp-frdm
- Pre-allocated `pending_packet_buf` eliminates allocations in main processing loop
- Ouster `FrameReader` permanently owns depth/reflect buffers (no per-frame allocation)

## [1.4.2] - 2026-01-28

### Changed
- **Binary renamed**: `lidarpub` → `edgefirst-lidarpub` for consistency with EdgeFirst naming convention
- Package name updated to `edgefirst-lidarpub` in Cargo.toml

## [1.4.1] - 2026-01-28

### Added
- TESTING.md documentation with CI overview and manual hardware testing procedures

### Changed
- Migrated CDR serialization to use `edgefirst_schemas::serde_cdr` API instead of direct `cdr` crate usage
- Updated edgefirst-schemas to 1.5.2
- Removed direct `cdr` dependency (now transitive through edgefirst-schemas)
- Updated build.yml to use native GitHub runners (ubuntu-22.04, ubuntu-22.04-arm) instead of cross-compilation
- Simplified test.yml with separated format/clippy jobs and streamlined coverage collection
- Updated release.yml to use lewagon/wait-on-check-action for cleaner build/release separation

## [1.4.0] - 2025-11-25

### Added
- Complete GitHub Actions CI/CD workflow suite
  - Build workflow for x86_64 and aarch64 targets
  - Test and coverage workflow with SonarCloud integration
  - SBOM generation and license compliance validation
  - Release workflow with automated GitHub releases
- Comprehensive ARCHITECTURE.md with Mermaid diagrams
- GitHub issue templates (bug reports, feature requests, questions)
- Pull request template with checklist
- SBOM generation scripts (CycloneDX format)
- License policy validation (check_license_policy.py)
- NOTICE file validation (validate_notice.py)

### Changed
- **BREAKING:** Static version handling (1.4.0) - removed dynamic versioning from build scripts
- Migrated from Bitbucket to GitHub (EdgeFirstAI organization)
- Updated repository URL: https://github.com/EdgeFirstAI/lidarpub
- Moved Rerun visualization from binary to example (`examples/lidar_rerun/`)
- Architecture documentation now references functions instead of line numbers
- Fixed all clippy warnings (while_let_loop, len_zero)
- Updated AGENTS.md with project-specific guidelines (868 lines)
- Enhanced documentation for SPS v2.0 compliance

### Fixed
- Cluster test assertions now algorithm-agnostic
- Code formatting (cargo +nightly fmt)
- Removed all unverified hardware claims from documentation

### Removed
- Bitbucket Pipelines configuration (replaced with GitHub Actions)
- Dynamic version extraction from git tags in build process

## [1.3.1] - 2025-11-12

### Changed
- Updated Rust edition to 2024
- Updated all dependencies to latest compatible versions
- Fixed Rerun API usage for latest version

### Fixed
- Adjusted cluster ID to use uint16 format (EDGEAI-729)

## [1.3.0] - 2024-XX-XX

### Added
- Point cloud clustering support (DBSCAN algorithm)
- Optional clustering with `--cluster` CLI flag
- Cluster distance threshold configuration
- Published `/lidar/clusters` topic for clustered point clouds

### Changed
- Updated cluster topic name to `/lidar/clusters` (EDGEAI-574)
- Improved timestamp error handling
- Updated Bitbucket Pipelines configuration

### Fixed
- Timestamp functions now return proper `ouster::Error` types
- Fixed timestamp precision on Linux platforms

## [1.2.0] - 2024-XX-XX

### Added
- Initial open source release preparation
- Apache-2.0 license
- CODE_OF_CONDUCT.md (Contributor Covenant v2.1)
- CONTRIBUTING.md for development guidelines
- SECURITY.md for vulnerability reporting
- NOTICE file for third-party attributions

### Changed
- Prepared documentation for public release
- Updated dependencies for security and compatibility

## [1.1.0] - 2024-XX-XX

### Added
- Ouster OS1-64 LiDAR support (firmware 2.5.3)
- RNG15_RFL8_NIR8 packet format parsing
- Zenoh pub/sub messaging integration
- ROS2 CDR message serialization
- Published topics:
  - `/lidar/points` (sensor_msgs/PointCloud2)
  - `/lidar/depth` (sensor_msgs/Image, mono16)
  - `/lidar/reflect` (sensor_msgs/Image, mono8)
  - `rt/tf_static` (geometry_msgs/TransformStamped)
- SIMD-accelerated 3D point transformations
- Tracy profiler integration
- Optional Rerun 3D visualization
- Configurable lidar modes (512x10, 1024x10, 2048x10)
- Configurable timestamp modes

### Performance
- <10ms frame processing on edge platforms
- ~2ms clustering latency

## [1.0.0] - 2024-XX-XX

### Added
- Initial internal release
- Core Ouster LiDAR protocol implementation
- Basic Zenoh publishing pipeline
- Command-line configuration interface

---

## Version History Notes

- **1.3.x** - Open source preparation, clustering support, dependency updates
- **1.2.x** - Documentation and license preparation
- **1.1.x** - Feature-complete internal release
- **1.0.x** - Initial implementation

---

**Maintained by:** Au-Zone Technologies  
**Contact:** support@au-zone.com  
**Documentation:** https://doc.edgefirst.ai/perception/lidar
