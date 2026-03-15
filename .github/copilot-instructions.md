# AI Assistant Development Guidelines

**Purpose:** Project-specific instructions for AI coding assistants (GitHub Copilot, Claude, Cursor, etc.)

**Organization Standards:** See [Au-Zone SPS 05-copilot-instructions.md](https://github.com/au-zone/sps/blob/main/05-copilot-instructions.md) for universal rules

**Version:** 3.0
**Last Updated:** 2026-03-12
**Project:** EdgeFirst LiDAR Publisher

---

## Overview

This file provides **project-specific** guidelines for the EdgeFirst LiDAR Publisher. ALL contributors must also follow:

- **Organization-wide:** Au-Zone SPS 05-copilot-instructions.md - License policy, security, Git/JIRA
- **Process docs:** Au-Zone Software Process Specification (SPS) repository
- **This file:** LiDAR Publisher conventions, module structure, naming patterns, domain specifics

**Hierarchy:** Org standards (mandatory) > SPS processes (required) > This file (project-specific)

**EdgeFirst LiDAR Publisher** is a high-performance Rust-based point cloud publisher connecting Ouster and Robosense LiDAR sensors to the Zenoh messaging framework with ROS2-compatible serialization. Part of the EdgeFirst Perception Middleware — a collection of highly-optimized services for spatial perception systems on edge hardware (Maivin, Raivin platforms).

---

**Branch:** `<type>/EDGEAI-###[-desc]` (feature/bugfix/hotfix, JIRA key required)
**Commit:** `EDGEAI-###: Brief description` (50-72 chars, what done not how)
**PR:** main=2 approvals, develop=1. Link JIRA, squash features, merge commits for releases.

**Example:**
```bash
# Good branch names
feature/EDGEAI-854-add-velodyne-support
bugfix/EDGEAI-857-fix-packet-loss

# Good commits
EDGEAI-854: Add Velodyne VLP-16 packet parser
EDGEAI-857: Fix UDP packet reassembly race condition
```

**External Contributors:** Use `feature/issue-###-description` format referencing GitHub issues.

---

## CRITICAL RULES

### #1: NEVER Use cd Commands

Modern build tools work from project root. Changing directories causes AI assistants to lose context and creates non-reproducible workflows.

```bash
# CORRECT: Modern Rust workflow (stay in project root)
cargo build --release
cargo test --workspace
cargo clippy -- -D warnings
cargo doc --no-deps --open

# CORRECT: Cross-compilation from root
cargo zigbuild --target aarch64-unknown-linux-gnu.2.35 --release

# WRONG: Changing directories
cd target && ./debug/edgefirst-lidarpub  # AI loses context!
cd src && cargo build                    # Breaks workspace structure!
```

**Rationale:**
- AI assistants lose working directory context after `cd`
- Cargo works perfectly from project root
- Commands become non-reproducible
- Recovery from subdirectories is error-prone

**Rare Exception - Subshell Pattern:**
```bash
# If you MUST run a legacy tool requiring specific directory
(cd subdir && legacy-tool)
# Subshell exits automatically, returns to original directory
```

**Project-Specific:** NEVER use `cd` in this project. Cargo handles all paths correctly from root.

---

### #2: ALWAYS Use Python venv (If Applicable)

**Note:** This project is pure Rust and doesn't use Python. This rule applies if you add Python tooling (e.g., code generation, analysis scripts, license validation).

```bash
# If adding Python scripts, create venv first
python3 -m venv venv
venv/bin/pip install -r requirements-dev.txt

# Direct invocation (no activation needed)
venv/bin/python scripts/generate_bindings.py
venv/bin/pytest tests/

# WRONG: System Python pollution
python scripts/generate_bindings.py  # Which Python?
pip install package                  # Pollutes system!
```

**Current Status:** Python scripts exist for CI validation (`check_license_policy.py`, `validate_notice.py`). Use venv if modifying or adding Python tooling.

---

### #3: env.sh for Integration Tests (Optional)

Integration tests MAY require external configuration (sensor IP addresses, Zenoh endpoints). Manage through optional `env.sh`.

```bash
# env.sh - LOCAL ONLY (.gitignore, NEVER commit!)
export OUSTER_IP="192.168.1.100"            # Test LiDAR sensor
export ZENOH_ENDPOINT="tcp/127.0.0.1:7447"  # Local Zenoh router
export TEST_TIMEOUT="30"                     # Test timeout seconds
```

**Security:**
- Ephemeral config, no secrets
- MUST be in `.gitignore` (already added)
- NEVER commit sensor IPs or network config
- NO passwords or API keys (use test sensors only)

**Tests without env.sh:**
- Unit tests: Always run (no external dependencies)
- PCAP-based integration tests: Run with `cargo test --features pcap`
- Hardware integration tests: Skip if env vars or sensors are missing
- CI/CD: Uses GitHub Secrets for sensor configuration

---

## Code Quality Standards

### Edge-First Development

This project targets **resource-constrained edge platforms**:

- **Memory:** Typical 512MB-2GB RAM, minimize allocations
- **CPU:** ARM Cortex-A series, optimize hot paths with NEON SIMD
- **Latency:** <10ms frame processing target at 10Hz
- **Lifespan:** 5-10 year deployment lifecycle

**Platform Targets:**
- **Maivin** - Vision module (NXP i.MX 8M Plus)
- **Raivin** - Vision + radar module (NXP i.MX 8M Plus, SmartMicro DRVEGRD-169 radar)
- **Generic ARM64** - Jetson, Raspberry Pi, embedded Linux

### Rust Standards

**Toolchain:**
- Rust edition: 2024 (see `rust-toolchain.toml`)
- Toolchain: **stable** (nightly only needed for `portable_simd` feature on non-aarch64)
- On aarch64: native NEON intrinsics are always used regardless of features

**Code Quality:**
```bash
# Format (MUST pass before commit)
cargo fmt --check

# Lint (MUST pass with zero warnings)
cargo clippy -- -D warnings

# Audit dependencies (check for CVEs)
cargo audit

# Check all targets
cargo check --workspace --all-targets
```

**Performance:**
- **Stack over heap:** Use stack allocation where possible
- **Zero-copy:** Minimize memory copies in hot paths; use fused write-into-buffer patterns
- **NEON SIMD:** Native NEON intrinsics on aarch64 for distance checks and point formatting
- **Pre-computation:** Cache intermediate trigonometric values (see `FrameBuilder`)
- **Profile on target:** Use Tracy profiler on actual hardware (Maivin/Raivin)

**Async Runtime:**
- Tokio with `rt-multi-thread` feature
- Dedicated threads for blocking compute (frame processor, clustering)
- Async tasks for I/O-bound work (UDP receiver, HTTP requests, Zenoh publishing)
- Kanal channels for inter-thread communication

---

## Testing Standards

### Coverage Requirements

- **Minimum:** 70% line coverage (enforced in CI/CD)
- **Critical paths:** 90%+ coverage (packet parsing, point transforms, clustering, ground filter)
- **Edge cases:** Explicit tests (null, bounds, concurrency)
- **Error paths:** Validate error handling and recovery

### Test Organization

**Unit Tests:**
- Co-located in `#[cfg(test)] mod tests` at end of implementation files
- Test naming: `test_<function>_<scenario>`
- Fast execution (< 1s per test suite)
- No external dependencies (mock sensors, Zenoh)

**Existing test modules:**
- `cluster.rs` - 16 tests: DBSCAN, voxel, spatial hash variants, real-data, cross-algorithm
- `ground.rs` - 7 tests: flat floor, tilt, no IMU, known height, wall rejection, reuse, real E1R
- `formats.rs` - 4 tests: 13-byte unclustered, 17-byte clustered, preallocated buffer, field builders
- `ouster.rs` - Ouster packet parsing tests
- `robosense.rs` - E1R packet parsing and frame assembly tests

**Integration Tests:**
- `tests/ouster_pcap_test.rs` - Ouster driver via PCAP replay (requires `pcap` feature)
- `tests/robosense_pcap_test.rs` - Robosense driver via PCAP replay (requires `pcap` feature)

**Benchmark Tests:**
- `benches/cluster_bench.rs` - Criterion benchmarks for clustering algorithms
- `benches/format_points_bench.rs` - PointCloud2 serialization benchmarks
- `benches/driver_bench.rs` - Full driver pipeline benchmarks (requires `pcap` feature)

**Test Data:**
- `testdata/e1r_frame0.pcd` / `e1r_frame5.pcd` - Real Robosense E1R point clouds
- `testdata/os1_frame0.pcd` / `os1_frame3.pcd` - Real Ouster OS1 point clouds
- `testdata/e1r_frames.pcap` - Robosense E1R PCAP capture
- `testdata/os1_frames.pcap` - Ouster OS1 PCAP capture (IP-fragmented)
- `testdata/os1_sensor_info.json` - Ouster sensor metadata
- Tests skip gracefully if data files are missing

### Running Tests

```bash
# All unit tests
cargo test

# Specific module
cargo test cluster::tests
cargo test ground::tests
cargo test formats::tests

# Integration tests (require pcap feature)
cargo test --features pcap

# With verbose output
cargo test -- --nocapture

# Benchmarks
cargo bench

# All checks (format + lint + test)
cargo fmt --check && cargo clippy -- -D warnings && cargo test
```

---

## License Policy (ZERO TOLERANCE)

**Project License:** Apache-2.0

**Allowed Dependencies:**
- MIT, Apache-2.0, BSD-2-Clause, BSD-3-Clause
- ISC, 0BSD, Unlicense, Zlib, BSL-1.0 (Boost)

**Conditional (External deps ONLY):**
- MPL-2.0, EPL-2.0 (Mozilla/Eclipse - library dependencies only)
- **LGPL FORBIDDEN in Rust** (static linking conflicts with Apache-2.0)

**BLOCKED (Never Use):**
- GPL (any version), AGPL (any version)
- SSPL, Commons Clause, proprietary without approval

**Verification:**
```bash
# Check all dependencies for license compliance
cargo install cargo-license
cargo license --json | jq '.[] | select(.license | contains("GPL"))'

# CI runs license validation automatically
```

**Before Adding Dependencies:**
1. Check crates.io for license
2. Verify no GPL in dependency tree
3. Document in commit message: "Adds dependency X (MIT license) for Y"
4. CI/CD will block GPL violations automatically

**Current Dependencies:** All Rust crates are MIT/Apache-2.0 dual-licensed (standard Rust ecosystem).

---

## Security Practices

### Input Validation

**Network Inputs (UDP packets from sensor):**
- Validate packet format and size before parsing
- Bound checks on all array accesses
- Reject malformed packets early
- Rate limiting for packet processing

**Configuration Inputs:**
- Validate sensor IP addresses (RFC 1918 private ranges expected)
- Sanitize topic names (no special characters)
- Bounds on numeric parameters (cluster threshold, timeout values)

### Secrets Management

**NO secrets in this project:**
- No API keys or passwords
- Sensor communication uses local network (no auth)
- Zenoh uses unauthenticated mode (local deployment)

**If adding external services:**
- Use environment variables for configuration
- Document in `env.sh.template` (not `env.sh`)
- Add `env.sh` to `.gitignore` (already done)
- Use short-lived tokens (<48h) if authentication added

### Dependency Security

```bash
# Check for known vulnerabilities
cargo audit

# Update dependencies carefully
cargo update  # Updates within Cargo.toml ranges
cargo outdated  # Check for newer versions

# Run before commits
cargo clippy -- -D warnings -W clippy::all
```

### Vulnerability Reporting

**For security issues:**
- Email: support@au-zone.com with subject "Security Vulnerability - LiDAR Publisher"
- See [SECURITY.md](../SECURITY.md) for full process
- Expected response: 48 hours acknowledgment, 7 days assessment

---

## Project-Specific Guidelines

### Technology Stack

**Language & Toolchain:**
- Rust edition: 2024 (see `rust-toolchain.toml`)
- Toolchain: stable (see `rust-toolchain.toml`)
- Nightly only needed for `portable_simd` feature on non-aarch64 targets
- Binary name: `edgefirst-lidarpub`

**Features:**
- `default` - Includes Tracy (lightweight overhead)
- `tracy` - Tracy profiler integration
- `profiling` - Tracy with sampling and system tracing
- `pcap` - PCAP file replay for testing without hardware
- `rerun` - Rerun 3D visualization (implies pcap)
- `discovery` - mDNS-based Ouster sensor discovery
- `portable_simd` - Portable SIMD for non-aarch64 targets (nightly only)

**Build System:**
- Cargo workspace (single crate)
- Cross-compilation: `cargo-zigbuild` for ARM targets (preferred over `cross`)
- Target platforms:
  - `x86_64-unknown-linux-gnu` (development)
  - `aarch64-unknown-linux-gnu` (Maivin, Raivin, generic ARM64)

**Key Dependencies:**
- **tokio** (1.49+) - Async runtime with multi-thread feature
- **zenoh** (1.7.2) - High-performance pub/sub messaging
- **edgefirst-schemas** (1.5.5) - ROS2 message definitions and CDR serialization
- **kanal** - Bounded channels for inter-thread communication
- **ndarray** (0.17) - N-dimensional arrays for frame data
- **clap** (4.5) - CLI argument parsing with derive and env features
- **tracing** + **tracing-tracy** - Instrumentation and profiling
- **criterion** (0.6) - Benchmarking framework (dev-dependency)

**Sensor Support:**
- **Ouster** - OS1-64 RevD (firmware 2.5.3), RNG15_RFL8_NIR8 packet format
- **Robosense** - E1R solid-state LiDAR, MSOP/DIFOP UDP protocols, built-in IMU

**Target Platforms:**
- **Maivin** - Vision module (NXP i.MX 8M Plus, EdgeFirst Perception)
- **Raivin** - Vision + radar module (NXP i.MX 8M Plus, SmartMicro DRVEGRD-169 radar)
- **Generic ARM64** - Jetson, Raspberry Pi, embedded Linux

### Architecture

**Pattern:** Event-driven async pipeline with dedicated compute threads

**Components:**
1. **UDP Receiver** - Async packet capture from sensor (Ouster: 7502/7503, Robosense: 6699/7788)
2. **Frame Builder** - Assembles packets into complete frames (`LidarFrame` trait)
3. **Point Cloud Processor** - Applies 3D transformations (NEON SIMD-accelerated on aarch64)
4. **Ground Filter** - Optional IMU-guided PCA ground plane removal
5. **Clustering Engine** - Optional DBSCAN or voxel spatial clustering with bridge threshold
6. **Zenoh Publisher** - Distributes sensor_msgs/PointCloud2, sensor_msgs/Image
7. **Transform Publisher** - Broadcasts geometry_msgs/TransformStamped for TF

**Data Flow:**
```
LiDAR Sensor (UDP)
    |
UDP Receiver (tokio::net::UdpSocket)
    |
Frame Builder (LidarFrame trait + LidarDriver)
    |
Point Processor (NEON SIMD on aarch64)
    |
[Optional] Ground Filter (IMU-guided PCA)
    |
[Optional] Clustering (DBSCAN or Voxel BFS)
    |
Zenoh Publisher (CDR-serialized ROS2 messages)
    |
Perception Middleware / ROS2 Nodes
```

**Threading Model:**
- Main thread: Argument parsing, initialization
- Tokio runtime: UDP receiver, HTTP requests, Zenoh publishing (async)
- Frame processor thread: Point transformations (dedicated, blocking compute)
- Cluster thread: Ground filter + clustering (dedicated, blocking compute)
- Kanal channels between threads (bounded for clustering, unbounded for frames)

**Error Handling:**
- Result<T, E> types throughout
- Custom error type: `ouster::Error`
- Graceful degradation: Log errors, continue processing next frame
- Network resilience: Reconnect on sensor disconnect

### Module Structure

```
src/
├── main.rs              (870 lines)  - Entry point, sensor drivers, publishing
├── ouster.rs           (1118 lines)  - Ouster protocol, packet parsing, SIMD transforms
├── robosense.rs         (820 lines)  - Robosense E1R MSOP/DIFOP parsing, IMU
├── cluster.rs          (1537 lines)  - DBSCAN and voxel clustering (NEON SIMD)
├── cluster_thread.rs    (288 lines)  - Clustering pipeline with instrumentation
├── ground.rs            (871 lines)  - IMU-guided PCA ground plane filter
├── formats.rs           (703 lines)  - PointCloud2 CDR serialization (SIMD)
├── lidar.rs             (321 lines)  - LidarFrame/LidarDriver trait abstractions
├── packet_source.rs     (296 lines)  - UDP/pcap packet source abstraction
├── pcap_source.rs       (610 lines)  - PCAP replay with IP fragment reassembly
├── args.rs              (215 lines)  - CLI argument parsing with clap
├── common.rs             (84 lines)  - Shared utilities and constants
└── lib.rs                (88 lines)  - Library exports for testing

tests/
├── ouster_pcap_test.rs              - Ouster driver integration (pcap feature)
└── robosense_pcap_test.rs           - Robosense driver integration (pcap feature)

benches/
├── cluster_bench.rs                 - Clustering algorithm benchmarks
├── format_points_bench.rs           - PointCloud2 serialization benchmarks
└── driver_bench.rs                  - Full driver pipeline benchmarks (pcap feature)

testdata/
├── e1r_frame0.pcd / e1r_frame5.pcd  - Real E1R point clouds for unit tests
├── os1_frame0.pcd / os1_frame3.pcd  - Real Ouster point clouds for unit tests
├── e1r_frames.pcap                  - E1R PCAP capture for integration tests
├── os1_frames.pcap                  - Ouster PCAP capture (IP-fragmented)
└── os1_sensor_info.json             - Ouster sensor metadata

examples/
├── lidar_viewer.rs                  - Real-time Rerun visualization (rerun feature)
└── pcap_viewer.rs                   - Offline PCAP replay with Rerun (rerun feature)
```

**Key Module Responsibilities:**

**ouster.rs** - Ouster LiDAR Protocol
- `Config`, `SensorInfo`, `LidarDataFormat`, `BeamIntrinsics` - Sensor config types
- `FrameReader` - UDP packet parsing and frame assembly
- `FrameBuilder` - SIMD-accelerated Cartesian point transforms
- Range image and reflectivity image generation

**robosense.rs** - Robosense E1R Protocol
- MSOP packet parsing (point cloud data, port 6699)
- DIFOP packet parsing (device info, IMU, port 7788)
- Frame assembly with reset detection
- IMU data extraction (accelerometer + gyroscope)

**cluster.rs** - Spatial Clustering
- `cluster_()` / `expand_cluster()` - DBSCAN with spatial hash
- `voxel_cluster()` - Voxel connected-component BFS
- `FlatSpatialHash` - Open-addressing hash for cache locality
- NEON SIMD distance checks on aarch64
- Bridge threshold support for both algorithms

**ground.rs** - Ground Plane Filter
- `GroundFilter` - IMU-guided PCA ground detection
- Polar grid binning (16 azimuth x 8 range sectors)
- Cardano's cubic root formula for eigenvalue computation
- EMA smoothing for stable ground height tracking

**formats.rs** - Message Serialization
- `format_points()` - PointCloud2 (13 bytes/point: XYZR)
- `format_points_clustered()` - Clustered PointCloud2 (17 bytes/point: XYZR + cluster_id)
- NEON SIMD-accelerated point formatting on aarch64

**lidar.rs** - Sensor Abstraction
- `LidarFrame` trait - Client-owned frame pattern
- `LidarFrameWriter` trait - Driver writes into client frames
- `LidarDriver` trait - Unified `process()` interface

**main.rs** - Application Orchestration
- Zenoh session initialization with QoS configuration
- Topic publishing: `{lidar_topic}/points`, `/depth`, `/reflect`, `/clusters`
- TF static frame broadcasting: `rt/tf_static`
- Async frame processing pipeline

**args.rs** - CLI Configuration
- `Args` struct with clap derive macros
- Sensor type selection (`ouster`, `robosense`)
- Clustering parameters, ground filter, topic names
- Timestamp parameters: `--time-sync`, `--time-source`, `--timestamp-offset`
- Environment variable support for all arguments

### Build and Deployment

**Local Development:**
```bash
# Build for development (with debug symbols)
cargo build

# Build optimized release
cargo build --release

# Run with Ouster sensor
./target/release/edgefirst-lidarpub --sensor-type ouster <SENSOR_IP>

# Run with Robosense E1R (broadcasts, no target needed)
./target/release/edgefirst-lidarpub --sensor-type robosense

# Enable voxel clustering with ground filter
./target/release/edgefirst-lidarpub --sensor-type robosense \
    --clustering voxel --ground-filter

# With Rerun visualization (requires feature)
cargo run --features rerun --example lidar_viewer -- --sensor-type ouster <SENSOR_IP>

# PCAP replay visualization
cargo run --features rerun --example pcap_viewer -- testdata/e1r_frames.pcap
```

**Cross-Compilation for ARM64:**
```bash
# Install cargo-zigbuild (preferred over cross)
cargo install cargo-zigbuild

# Build for Maivin/Raivin (ARM64)
cargo zigbuild --target aarch64-unknown-linux-gnu.2.35 --release

# Binary location
ls target/aarch64-unknown-linux-gnu/release/edgefirst-lidarpub

# Deploy to target device
scp target/aarch64-unknown-linux-gnu/release/edgefirst-lidarpub user@target-device:/usr/local/bin/
```

**Code Quality Checks:**
```bash
# Format code
cargo fmt

# Lint (zero warnings required)
cargo clippy -- -D warnings

# Security audit
cargo audit

# All checks (run before commit)
cargo fmt && cargo clippy -- -D warnings && cargo test
```

**Tracy Profiling:**
```bash
# Build with Tracy support (included by default)
cargo build --release

# Build with full profiling (sampling + system tracing)
cargo build --release --features profiling

# Run with Tracy enabled
./target/release/edgefirst-lidarpub --sensor-type robosense --tracy

# Tracy captures frame timing, pipeline stages, allocations
```

**Configuration File:**
```bash
# lidarpub.default - template for /etc/default/lidarpub
# Deployed as release artifact for systemd service configuration
# See lidarpub.default in repository root for all documented options
```

### Performance Targets

**Target Platform Performance (aarch64 Cortex-A, E1R ~25k points):**
- Frame processing: <10ms @ 10Hz (without clustering)
- Voxel clustering: ~13ms per frame
- DBSCAN clustering: ~70ms per frame
- Ground filter: ~5-9ms per frame
- Full pipeline (ground + voxel): ~29ms total, within 100ms (10Hz) budget
- Memory footprint: <50MB resident

**Pipeline Timing (logged every 100 frames):**
```
pipeline avg over 100 frames (24967 pts): valid=0.2ms ground=9.3ms cluster=13.2ms relabel=0.5ms format=3.4ms publish=2.6ms total=29.1ms
```

**Memory:**
- Pre-allocated buffers for frame assembly (no per-frame allocation)
- Zero-copy point cloud processing via fused write-into-buffer
- Reuse Vec allocations across frames
- Kanal bounded channels prevent unbounded queue growth

### Sensor Specifics

**Ouster OS1-64 LiDAR:**
- Firmware: 2.5.3 (tested)
- Packet format: RNG15_RFL8_NIR8 (15-bit range, 8-bit reflectivity, 8-bit NIR)
- Resolution modes: 512x10, 1024x10, 2048x10 (columns x Hz)
- UDP ports: 7502 (data), 7503 (IMU)
- Network: Gigabit Ethernet (static IP recommended: 192.168.1.x)
- HTTP API: Sensor configuration, metadata, calibration
- Time sync modes (`--time-sync`): Internal OSC, Sync pulse, PTP-1588
- Timestamp source (`--time-source`): `host` (CLOCK_REALTIME) or `sensor` (packet timestamp with validation)
- Timestamp offset (`--timestamp-offset`): Nanosecond correction for capture-to-arrival latency
- Packets are IP-fragmented on standard MTU networks

**Robosense E1R LiDAR:**
- Solid-state LiDAR with 120 deg H x 90 deg V FOV
- ~20k-26k points per frame at 10Hz
- MSOP port: 6699 (point cloud data), DIFOP port: 7788 (device info + IMU)
- Broadcasts UDP to 255.255.255.255 (no target IP needed)
- Built-in IMU (accelerometer + gyroscope) for ground plane detection
- IMU axis remap: LiDAR = (imu_z, -imu_x, -imu_y)

**Maivin Platform:**
- Processor: NXP i.MX 8M Plus
- Network: Gigabit Ethernet
- OS: Embedded Linux (Torizon)

**Raivin Platform:**
- Processor: NXP i.MX 8M Plus
- Radar: SmartMicro DRVEGRD-169
- Network: Gigabit Ethernet
- OS: Embedded Linux (Torizon)

### Zenoh Best Practices

**QoS Configuration:**
- Point cloud topics: Priority `DataHigh`, CongestionControl `Drop`
- TF static frames: Priority `Background`, CongestionControl `Drop`

**Topic Naming:**
- Sensor data: `{lidar_topic}/points`, `{lidar_topic}/depth`, etc. (default prefix: `rt/lidar`)
- Transforms: `rt/tf_static` (ROS2 convention)
- IMU: `rt/lidar/imu` (Robosense E1R only)
- Use CLI argument `--lidar-topic` to customize prefix

**Message Serialization:**
- Use `edgefirst_schemas::serde_cdr` for ROS2 Common Data Representation
- Schemas from `edgefirst-schemas` crate (sensor_msgs, geometry_msgs)
- PointCloud2: 13 bytes/point unclustered, 17 bytes/point clustered

**Error Handling:**
- Log Zenoh errors, continue processing
- Reconnect on session failure
- Graceful degradation if subscriber count drops

---

## Documentation Standards

### Code Documentation

**Required Documentation:**
- All public APIs: `///` doc comments with examples
- Complex algorithms: Inline comments explaining "why" not "what"
- Performance-critical sections: Note optimizations and tradeoffs
- SIMD code: Document scalar fallback and vectorization strategy
- Unsafe code: Justify safety invariants

### Project Documentation

**Existing Files:**
- README.md - Features, quick start, architecture overview, troubleshooting
- ARCHITECTURE.md - Detailed system architecture with Mermaid diagrams
- CONTRIBUTING.md - Development setup, contribution process
- SECURITY.md - Vulnerability reporting
- CHANGELOG.md - All releases documented (Keep a Changelog format)
- TESTING.md - Test strategy, unit tests, manual hardware testing
- LICENSE - Apache-2.0 full text
- NOTICE - Third-party attributions
- lidarpub.default - Configuration template for systemd deployments

**API Documentation:**
- Generate with `cargo doc --no-deps --open`
- 100% coverage for public APIs

---

## AI Assistant Practices

**Verify Before Using:**
- API exists in current Rust stable and crate versions
- License compatible (check Cargo.toml and crates.io)
- Code matches project patterns (async, Result types, NEON SIMD on aarch64)
- Tests included for new functionality
- Documentation comments for public APIs

**Common Pitfalls to Avoid:**
- Hallucinated Rust APIs (verify on docs.rs before suggesting)
- GPL/AGPL dependencies (Rust crates are usually MIT/Apache-2.0 dual-licensed)
- Using `cd` commands (stay in project root)
- Blocking I/O in async context (use tokio::fs, tokio::net)
- Panic in production code (use Result types)
- Hardcoded IP addresses (use CLI arguments or environment variables)
- Over-engineering (keep it simple, profile before optimizing)
- Assuming nightly Rust (project uses stable; nightly only for portable_simd on non-aarch64)
- Using old binary name `lidarpub` (correct name: `edgefirst-lidarpub`)
- Using `cross` for cross-compilation (use `cargo-zigbuild` instead)
- Referencing `cdr` crate directly (use `edgefirst_schemas::serde_cdr`)

**Review Checklist:**
- [ ] Code compiles: `cargo check`
- [ ] No warnings: `cargo clippy -- -D warnings`
- [ ] Formatted: `cargo fmt --check`
- [ ] Tests pass: `cargo test`
- [ ] Documentation complete for public APIs
- [ ] No secrets or hardcoded config
- [ ] Matches existing code patterns

**You are the author** - AI is a tool. Review all generated code thoroughly, test on target hardware when possible.

---

## Quick Reference

**Binary:** `edgefirst-lidarpub`
**Branch:** `feature/EDGEAI-###-description`
**Commit:** `EDGEAI-###: Brief description`
**PR:** 2 approvals (main), 1 (develop), all CI checks pass
**Licenses:** MIT/Apache/BSD allowed | GPL/AGPL blocked
**Tests:** 70% min, 90%+ critical paths
**Benchmarks:** `cargo bench`
**Security:** support@au-zone.com
**Platforms:** Maivin (NXP i.MX 8M Plus), Raivin (Maivin + SmartMicro DRVEGRD-169 radar), generic ARM64
**Sensors:** Ouster OS1-64 (firmware 2.5.3), Robosense E1R (solid-state)
**Cross-compile:** `cargo zigbuild --target aarch64-unknown-linux-gnu.2.35 --release`

---

**SPS Documentation:** See Au-Zone Software Process Specification repository
**EdgeFirst Docs:** https://doc.edgefirst.ai/perception/lidar
**v3.0** | 2026-03-12 | sebastien@au-zone.com

*This document helps AI assistants contribute effectively to EdgeFirst LiDAR Publisher while maintaining quality, performance, and Au-Zone standards.*
