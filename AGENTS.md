# AGENTS.md - AI Assistant Development Guidelines

**Purpose:** Project-specific instructions for AI coding assistants (GitHub Copilot, Claude, Cursor, etc.)

**Organization Standards:** See [Au-Zone SPS 05-copilot-instructions.md](https://github.com/au-zone/sps/blob/main/05-copilot-instructions.md) for universal rules

**Version:** 2.0
**Last Updated:** 2025-11-22
**Project:** EdgeFirst LiDAR Publisher

---

## Overview

This file provides **project-specific** guidelines for the EdgeFirst LiDAR Publisher. ALL contributors must also follow:

- **Organization-wide:** Au-Zone SPS 05-copilot-instructions.md - License policy, security, Git/JIRA
- **Process docs:** Au-Zone Software Process Specification (SPS) repository
- **This file:** LiDAR Publisher conventions, module structure, naming patterns, domain specifics

**Hierarchy:** Org standards (mandatory) → SPS processes (required) → This file (project-specific)

**EdgeFirst LiDAR Publisher** is a high-performance Rust-based point cloud publisher connecting Ouster LiDAR sensors to the Zenoh messaging framework with ROS2-compatible serialization. Part of the EdgeFirst Perception Middleware — a collection of highly-optimized services for spatial perception systems on edge hardware (Maivin, Raivin platforms).

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

## ⚠️ CRITICAL RULES

### #1: NEVER Use cd Commands

Modern build tools work from project root. Changing directories causes AI assistants to lose context and creates non-reproducible workflows.

```bash
# ✅ CORRECT: Modern Rust workflow (stay in project root)
cargo build --release
cargo test --workspace
cargo clippy -- -D warnings
cargo doc --no-deps --open

# ✅ CORRECT: Cross-compilation from root
cross build --target aarch64-unknown-linux-gnu

# ❌ WRONG: Changing directories
cd target && ./debug/lidarpub  # AI loses context!
cd src && cargo build          # Breaks workspace structure!
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

**Note:** This project is pure Rust and doesn't use Python. This rule applies if you add Python tooling (e.g., code generation, analysis scripts).

```bash
# ✅ If adding Python scripts, create venv first
python3 -m venv venv
venv/bin/pip install -r requirements-dev.txt

# ✅ Direct invocation (no activation needed)
venv/bin/python scripts/generate_bindings.py
venv/bin/pytest tests/

# ❌ WRONG: System Python pollution
python scripts/generate_bindings.py  # Which Python?
pip install package                  # Pollutes system!
```

**requirements.txt - Semver ranges:**
```txt
# ✅ Allow patches/minors, block breaking changes
pytest>=7.4.0,<8.0.0
mypy>=1.5.0,<2.0.0

# ❌ Exact pins block security patches
pytest==7.4.3
```

**Current Status:** No Python dependencies. Add venv if introducing Python tooling.

---

### #3: env.sh for Integration Tests (Optional)

Integration tests MAY require external configuration (sensor IP addresses, Zenoh endpoints). Manage through optional `env.sh`.

```bash
# env.sh - LOCAL ONLY (.gitignore, NEVER commit!)
export OUSTER_IP="192.168.1.100"         # Test LiDAR sensor
export ZENOH_ENDPOINT="tcp/127.0.0.1:7447"  # Local Zenoh router
export TEST_TIMEOUT="30"                 # Test timeout seconds
```

**Usage in tests:**
```bash
# Source env.sh if present (tests skip if missing)
[[ -f env.sh ]] && source env.sh

# Run integration tests with sensor communication
cargo test --test integration -- --nocapture
```

**Security:**
- ✅ Ephemeral config, no secrets
- ✅ MUST be in `.gitignore` (already added)
- ❌ NEVER commit sensor IPs or network config
- ❌ NO passwords or API keys (use test sensors only)

**Tests without env.sh:**
- Unit tests: Always run (no external dependencies)
- Integration tests: Skip tests requiring sensor if env vars missing
- CI/CD: Uses GitHub Secrets for sensor configuration

**Current Status:** Not yet implemented. Add if integration tests need sensor configuration.

---

## Code Quality Standards

### Edge-First Development

This project targets **resource-constrained edge platforms**:

- **Memory:** Typical 512MB-2GB RAM, minimize allocations
- **CPU:** ARM Cortex-A series, optimize hot paths with SIMD
- **Latency:** <10ms frame processing target
- **Lifespan:** 5-10 year deployment lifecycle

**Platform Targets:**
- **Maivin** - Vision module (NXP i.MX 8M Plus)
- **Raivin** - Vision + radar module (NXP i.MX 8M Plus, SmartMicro DRVEGRD-169 radar)
- **Generic ARM64** - Jetson, Raspberry Pi, embedded Linux

### Rust Standards

**Toolchain:**
- Rust edition: 2024 (see `rust-toolchain.toml`)
- Minimum supported: Latest stable -2 versions
- Features: `portable_simd` (nightly for SIMD acceleration)

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
- **Zero-copy:** Minimize memory copies in hot paths
- **SIMD:** Use portable_simd for point transformations (see `src/ouster.rs`)
- **Arena allocators:** Consider for frame buffers
- **Profile on target:** Use Tracy profiler on actual hardware (Maivin/Raivin)

**Async Runtime:**
- Tokio with `rt-multi-thread` feature
- Separate tasks for UDP receiver, frame processor, publisher
- Non-blocking I/O for network and sensor communication


---

## Testing Standards

### Coverage Requirements

- **Minimum:** 70% line coverage (enforced in CI/CD)
- **Critical paths:** 90%+ coverage (packet parsing, point transforms, clustering)
- **Edge cases:** Explicit tests (null, bounds, concurrency)
- **Error paths:** Validate error handling and recovery

**Current Status:** ~5% coverage (2 tests in cluster.rs). **Target: 70% minimum.**

### Test Organization

**Unit Tests:**
- Co-located in `#[cfg(test)] mod tests` at end of implementation files
- Test naming: `test_<function>_<scenario>`
- Fast execution (< 1s per test suite)
- No external dependencies (mock sensors, Zenoh)

**Integration Tests:**
- Separate `tests/` directory (to be created)
- Test sensor communication (mocked UDP)
- Test Zenoh publishing pipeline
- Test ROS2 message format compliance
- End-to-end workflows

**Benchmark Tests:**
- `benches/` directory for Criterion.rs benchmarks
- Performance regression detection
- Hot path optimization validation

### Test Examples

**Unit Test Pattern:**
```rust
// src/ouster.rs
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_lidar_packet_rng15_rfl8_nir8() {
        let packet = create_test_packet();
        let result = parse_packet(&packet);
        assert!(result.is_ok());
        assert_eq!(result.unwrap().points.len(), 64);
    }

    #[test]
    fn test_point_transform_simd_vs_scalar() {
        let input = vec![/* test data */];
        let simd_result = transform_points_simd(&input);
        let scalar_result = transform_points_scalar(&input);
        assert_approx_eq!(simd_result, scalar_result, 1e-6);
    }
}
```

**Integration Test Pattern:**
```rust
// tests/integration.rs
#[tokio::test]
async fn test_zenoh_publish_point_cloud() {
    let session = create_test_session().await;
    let publisher = create_lidar_publisher(&session);
    
    let frame = create_test_frame();
    publisher.publish_point_cloud(&frame).await.unwrap();
    
    // Verify message received
    let received = subscriber.recv().await.unwrap();
    assert_eq!(received.payload.len(), expected_size);
}
```

### Running Tests

```bash
# All unit tests
cargo test --workspace

# Specific module
cargo test --lib ouster::tests

# Integration tests (when created)
cargo test --test integration

# With coverage
cargo llvm-cov --html --output-dir coverage

# Benchmarks (when created)
cargo bench

# All checks (format + lint + test)
cargo fmt --check && cargo clippy -- -D warnings && cargo test
```


---

## License Policy (ZERO TOLERANCE)

**Project License:** Apache-2.0

**✅ Allowed Dependencies:**
- MIT, Apache-2.0, BSD-2-Clause, BSD-3-Clause
- ISC, 0BSD, Unlicense, Zlib, BSL-1.0 (Boost)

**⚠️ Conditional (External deps ONLY):**
- MPL-2.0, EPL-2.0 (Mozilla/Eclipse - library dependencies only)
- **LGPL FORBIDDEN in Rust** (static linking conflicts with Apache-2.0)

**❌ BLOCKED (Never Use):**
- GPL (any version), AGPL (any version)
- SSPL, Commons Clause, proprietary without approval

**Verification:**
```bash
# Check all dependencies for license compliance
cargo install cargo-license
cargo license --json | jq '.[] | select(.license | contains("GPL"))'

# SBOM generation (when CI configured)
make sbom  # Generates CycloneDX SBOM, validates licenses
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
- See [SECURITY.md](SECURITY.md) for full process
- Expected response: 48 hours acknowledgment, 7 days assessment

---

## Documentation Standards

### Code Documentation

**Required Documentation:**
- All public APIs: `///` doc comments with examples
- Complex algorithms: Inline comments explaining "why" not "what"
- Performance-critical sections: Note optimizations and tradeoffs
- SIMD code: Document scalar fallback and vectorization strategy
- Unsafe code: Justify safety invariants

**Example:**
```rust
/// Transforms raw LiDAR measurements to 3D Cartesian coordinates.
///
/// Uses SIMD acceleration when available (portable_simd feature) for 4x
/// throughput improvement on ARM Cortex-A cores.
///
/// # Arguments
/// * `measurements` - Range, azimuth, and altitude for each point
/// * `intrinsics` - Sensor beam calibration parameters
///
/// # Returns
/// Vec of (x, y, z) coordinates in sensor frame (meters)
///
/// # Performance
/// - SIMD: ~2.5ms per frame (1024 points @ 10Hz)
/// - Scalar: ~9ms per frame (fallback on non-SIMD platforms)
///
/// # Examples
/// ```
/// let points = transform_to_cartesian(&measurements, &intrinsics);
/// assert_eq!(points.len(), measurements.len());
/// ```
pub fn transform_to_cartesian(
    measurements: &[Measurement],
    intrinsics: &BeamIntrinsics,
) -> Vec<Point3D> {
    // Implementation...
}
```

### Project Documentation

**Mandatory Files (already exist):**
- README.md - Features, quick start, architecture overview
- ARCHITECTURE.md - **TO BE CREATED** (Phase 1.2)
- CONTRIBUTING.md - Development setup, contribution process
- SECURITY.md - Vulnerability reporting
- CHANGELOG.md - **TO BE CREATED** (Phase 1.4)
- LICENSE - Apache-2.0 full text
- NOTICE - Third-party attributions

**API Documentation:**
- Generate with `cargo doc --no-deps --open`
- Publish to doc.edgefirst.ai (not docs.rs)
- 100% coverage for public APIs

---

## Project-Specific Guidelines

### Technology Stack

**Language & Toolchain:**
- Rust edition: 2024 (see `rust-toolchain.toml`)
- MSRV: Latest stable (tested down to stable-2)
- Features:
  - `portable_simd` (nightly) - SIMD acceleration for point transforms
  - `rerun` (optional) - Real-time 3D visualization
  - Tracy profiling support

**Build System:**
- Cargo workspace (single crate currently)
- Cross-compilation: `cross` tool for ARM targets
- Target platforms:
  - `x86_64-unknown-linux-gnu` (development)
  - `aarch64-unknown-linux-gnu` (Maivin, Raivin, generic ARM64)

**Key Dependencies:**
- **tokio** (1.x) - Async runtime with multi-thread feature
- **zenoh** (1.6.2) - High-performance pub/sub messaging
- **cdr** - ROS2 CDR serialization
- **edgefirst-schemas** (1.3.1) - ROS2 message definitions
- **portable-simd** (nightly) - SIMD abstractions

**Target Platforms:**
- **Maivin** - Vision module (NXP i.MX 8M Plus, EdgeFirst Perception)
- **Raivin** - Vision + radar module (NXP i.MX 8M Plus, SmartMicro DRVEGRD-169 radar)
- **Generic ARM64** - Jetson, Raspberry Pi, embedded Linux

### Architecture

**Pattern:** Event-driven async pipeline with multi-threading

**Components:**
1. **UDP Receiver** - Async packet capture from Ouster sensor (7502/7503 ports)
2. **Frame Builder** - Assembles packets into complete frames
3. **Point Cloud Processor** - Applies 3D transformations (SIMD-accelerated)
4. **Clustering Engine** - Optional DBSCAN spatial clustering
5. **Zenoh Publisher** - Distributes sensor_msgs/PointCloud2, sensor_msgs/Image
6. **Transform Publisher** - Broadcasts geometry_msgs/TransformStamped for TF

**Data Flow:**
```
Ouster Sensor (UDP 7502/7503)
    ↓
UDP Receiver (tokio::net::UdpSocket)
    ↓
Frame Builder (FrameReader::read_frame)
    ↓
Point Processor (transform_to_cartesian with SIMD)
    ↓
[Optional] Clustering (DBSCAN algorithm)
    ↓
Zenoh Publisher (CDR-serialized ROS2 messages)
    ↓
Perception Middleware / ROS2 Nodes
```

**Threading Model:**
- Main thread: Argument parsing, initialization
- Tokio runtime: UDP receiver, frame assembly (async)
- Processor thread: Point transformations, clustering (dedicated thread)
- Zenoh thread pool: Message publishing (Zenoh internal)

**Error Handling:**
- Result<T, E> types throughout
- Custom error type: `ouster::Error`
- Graceful degradation: Log errors, continue processing next frame
- Network resilience: Reconnect on sensor disconnect

### Module Structure

```
src/
├── main.rs          (568 lines) - Application entry, Zenoh session, publishing
├── ouster.rs        (729 lines) - Ouster protocol, packet parsing, point transforms
├── cluster.rs       (444 lines) - DBSCAN clustering algorithm
├── args.rs          (133 lines) - CLI argument parsing with clap
├── rerun.rs         (321 lines) - Optional Rerun visualization integration
├── common.rs        (81 lines)  - Shared utilities and constants
└── lib.rs           (69 lines)  - Library exports for testing

tests/               (to be created)
└── integration/     - Sensor communication, Zenoh publishing tests

benches/             (to be created)
└── point_transform.rs - Criterion benchmarks for hot paths
```

**Module Responsibilities:**

**ouster.rs** - Ouster LiDAR Protocol
- `BeamIntrinsics` - Sensor calibration parameters
- `LidarDataFormat` - Packet format parsing (RNG15_RFL8_NIR8)
- `FrameReader` - UDP packet reception and frame assembly
- `transform_to_cartesian()` - 3D point cloud generation (SIMD-accelerated)
- `generate_range_image()` - Depth image from range measurements
- `generate_reflectivity_image()` - Grayscale image from reflectivity

**cluster.rs** - Spatial Clustering
- `cluster_points()` - DBSCAN clustering algorithm
- Density-based spatial clustering for object segmentation
- Configurable epsilon (distance threshold) and min points

**main.rs** - Application Orchestration
- Zenoh session initialization with QoS configuration
- ROS2 CDR message serialization
- Topic publishing: `/lidar/points`, `/lidar/depth`, `/lidar/reflect`, `/lidar/clusters`
- TF static frame broadcasting: `rt/tf_static`
- Async frame processing pipeline

**args.rs** - CLI Configuration
- `Args` struct with clap derive macros
- Sensor IP, lidar mode, timestamp mode configuration
- Clustering parameters, topic names
- Validation and default values

### Build and Deployment

**Local Development:**
```bash
# Build for development (with debug symbols)
cargo build

# Build optimized release
cargo build --release

# Run with default settings
./target/release/lidarpub --target 192.168.1.100

# Enable clustering
./target/release/lidarpub --target 192.168.1.100 --cluster

# With Rerun visualization (requires feature)
cargo run --features rerun --bin lidar-rerun -- --target 192.168.1.100
```

**Cross-Compilation for ARM64:**
```bash
# Install cross tool
cargo install cross

# Build for Maivin/Raivin (ARM64)
cross build --release --target aarch64-unknown-linux-gnu

# Binary location
ls target/aarch64-unknown-linux-gnu/release/lidarpub
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
# Build with Tracy support
cargo build --release --features tracy

# Run and connect Tracy profiler
./target/release/lidarpub --target 192.168.1.100

# Tracy will capture frame timing, allocations, network I/O
```

### Performance Targets

**Target Platform Performance:**
- Frame processing: <10ms @ 10Hz (1024x10 mode) on edge hardware
- Point cloud clustering: <2ms per frame
- Memory footprint: <50MB resident

**Latency Breakdown:**
- UDP packet reception: <0.5ms
- Frame assembly: <1ms
- Point transformation (SIMD): ~2.5ms (1024 points)
- Clustering (DBSCAN): ~2ms (when enabled)
- Zenoh publishing: <1ms

**Throughput:**
- 10Hz operation @ 1024x10 mode (10,240 points/sec)
- Up to 20Hz @ 512x10 mode (10,240 points/sec)
- Sustained operation without frame drops

**Memory:**
- Stack-allocated buffers for frame assembly
- Zero-copy message passing where possible
- Minimal heap allocations in hot path
- Reuse point cloud Vec across frames

### Hardware Specifics

**Ouster OS1-64 LiDAR:**
- Firmware: 2.5.3 (tested)
- Packet format: RNG15_RFL8_NIR8 (15-bit range, 8-bit reflectivity, 8-bit NIR)
- Resolution modes: 512x10, 1024x10, 2048x10 (columns × Hz)
- UDP ports: 7502 (data), 7503 (IMU)
- Network: Gigabit Ethernet (static IP recommended: 192.168.1.x)
- HTTP API: Sensor configuration, metadata, calibration
- Timestamp modes: Internal OSC, Sync pulse, PTP

**Maivin Platform:**
- Processor: NXP i.MX 8M Plus
- Network: Gigabit Ethernet
- OS: Embedded Linux

**Raivin Platform:**
- Processor: NXP i.MX 8M Plus
- Radar: SmartMicro DRVEGRD-169
- Network: Gigabit Ethernet
- OS: Embedded Linux

### Zenoh Best Practices

**QoS Configuration:**
```rust
// Point cloud publishing (high priority, drop on congestion)
let pub_config = zenoh::config::Config::default();
pub_config.priority = Priority::DataHigh;
pub_config.congestion_control = CongestionControl::Drop;

// TF static frames (background priority, block on congestion)
let tf_config = zenoh::config::Config::default();
tf_config.priority = Priority::Background;
tf_config.congestion_control = CongestionControl::Block;
```

**Topic Naming:**
- Sensor data: `/{sensor_name}/{message_type}` (e.g., `/lidar/points`)
- Transforms: `rt/tf_static` (ROS2 convention)
- Use CLI argument `--lidar-topic` to customize prefix

**Message Serialization:**
- Use `cdr` crate for ROS2 Common Data Representation
- Schemas from `edgefirst-schemas` crate (sensor_msgs, geometry_msgs)
- Serialize to `Vec<u8>` before Zenoh put()

**Error Handling:**
- Log Zenoh errors, continue processing
- Reconnect on session failure
- Graceful degradation if subscriber count drops

---

## Testing Conventions

**Unit Tests:**
- Co-located at end of source files: `#[cfg(test)] mod tests`
- Test naming: `test_<function>_<scenario>`
- Example: `test_parse_packet_rng15_rfl8_nir8()`
- Mock sensor data with known expected outputs

**Integration Tests:**
- `tests/` directory (to be created)
- Test real Zenoh publishing (local session)
- Validate ROS2 message format compliance
- End-to-end: UDP → parse → transform → publish

**Benchmark Tests:**
- `benches/` directory (to be created)
- Criterion.rs framework for micro-benchmarks
- Hot path functions: `transform_to_cartesian()`, `cluster_points()`
- Performance regression detection

**Test Fixtures:**
- `tests/fixtures/` for sample LiDAR packets
- Real capture from Ouster sensor (small files)
- Edge cases: empty frames, malformed packets, max range

**Coverage Target:** 70% minimum, 90%+ for critical paths (packet parsing, point transforms, clustering)

---

## AI Assistant Practices

**Verify Before Using:**
- ✅ API exists in current Rust version and crate versions
- ✅ License compatible (check Cargo.toml and crates.io)
- ✅ Code matches project patterns (async, Result types, SIMD where applicable)
- ✅ Tests included for new functionality
- ✅ Documentation comments for public APIs

**Common Pitfalls to Avoid:**
- ❌ Hallucinated Rust APIs (verify on docs.rs before suggesting)
- ❌ GPL/AGPL dependencies (Rust crates are usually MIT/Apache-2.0 dual-licensed)
- ❌ Using `cd` commands (stay in project root)
- ❌ Blocking I/O in async context (use tokio::fs, tokio::net)
- ❌ Panic in production code (use Result types)
- ❌ Hardcoded IP addresses (use CLI arguments)
- ❌ Over-engineering (keep it simple, profile before optimizing)

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

**Branch:** `feature/EDGEAI-###-description`
**Commit:** `EDGEAI-###: Brief description`
**PR:** 2 approvals (main), 1 (develop), all CI checks pass
**Licenses:** ✅ MIT/Apache/BSD | ❌ GPL/AGPL
**Tests:** 70% min, 90%+ critical paths
**Coverage:** `cargo llvm-cov --html`
**Security:** support@au-zone.com
**Platforms:** Maivin (NXP i.MX 8M Plus), Raivin (Maivin + SmartMicro DRVEGRD-169 radar), generic ARM64
**Sensor:** Ouster OS1-64 (firmware 2.5.3, RNG15_RFL8_NIR8)

---

**SPS Documentation:** See Au-Zone Software Process Specification repository
**EdgeFirst Docs:** https://doc.edgefirst.ai/perception/lidar
**v2.0** | 2025-11-22 | sebastien@au-zone.com

*This document helps AI assistants contribute effectively to EdgeFirst LiDAR Publisher while maintaining quality, performance, and Au-Zone standards.*
