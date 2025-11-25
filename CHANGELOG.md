# Changelog

All notable changes to the EdgeFirst LiDAR Publisher will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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
