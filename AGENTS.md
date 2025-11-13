# AGENTS.md - AI Assistant Development Guidelines

This document provides instructions for AI coding assistants (GitHub Copilot, Cursor, Claude Code, etc.) working on the EdgeFirst LiDAR Publisher project. These guidelines ensure consistent code quality, proper workflow adherence, and maintainable contributions.

**Version:** 1.0  
**Last Updated:** November 2025  
**Project:** EdgeFirst LiDAR Publisher  
**Organization:** Au-Zone Technologies

---

## Table of Contents

1. [Project Overview](#project-overview)
2. [Git Workflow](#git-workflow)
3. [Code Quality Standards](#code-quality-standards)
4. [Testing Requirements](#testing-requirements)
5. [Documentation Expectations](#documentation-expectations)
6. [License Policy](#license-policy)
7. [Security Practices](#security-practices)
8. [Performance & Edge Considerations](#performance--edge-considerations)

---

## Project Overview

A high-performance Rust application that ingests Ouster LiDAR UDP packets, constructs point cloud frames, applies optional clustering, and publishes data over Zenoh (and optionally ROS2-compatible formats) with visualization via Rerun.

**Part of EdgeFirst Perception Middleware**: This is one component of Au-Zone Technologies' EdgeFirst Perception stack — a modular collection of highly-optimized Rust services that deliver the building blocks for spatial perception systems on edge hardware. The middleware provides ROS2-compatible messaging over Zenoh for cameras, LiDAR, radar, IMU, GPS, and AI inference services.

**Technology Stack:**
- **Language**: Rust 2024 edition (see `rust-toolchain.toml`)
- **Build System**: Cargo with workspace configuration
- **Key Dependencies**: tokio (async), zenoh (messaging), cdr (ROS2 serialization), rerun (visualization)
- **Serialization**: CDR (Common Data Representation) for ROS2 compatibility
- **Message Schemas**: ROS2 common_interfaces (sensor_msgs, geometry_msgs)
- **Target Platforms**: Edge AI modules (Maivin, Raivin), x86_64, ARM64
- **Cross-Platform**: Linux primary; macOS/Windows for development

**Sensor Support:**
- **Current**: Ouster LiDAR sensors (tested with OS1-64 RevD, firmware 2.5.3)
- **Packet Format**: RNG15_RFL8_NIR8 (15-bit range, 8-bit reflectivity, 8-bit NIR)
- **Future**: Additional packet formats, newer firmware, Velodyne, Livox, and other vendors
- **References**:
  - [Ouster HTTP API v1](https://static.ouster.dev/sensor-docs/image_route1/image_route2/common_sections/API/http-api-v1.html)
  - [Ouster Sensor Data](https://static.ouster.dev/sensor-docs/image_route1/image_route2/sensor_data/sensor-data.html)
  - [Lidar Packet Format](https://static.ouster.dev/sensor-docs/image_route1/image_route2/sensor_data/sensor-data.html#lidar-data-packet-format)

**Architecture:**
- **Pattern**: Event-driven pipeline with async processing
- **Data Flow**: UDP Receiver → Frame Builder → Point Cloud Processor → Zenoh Publisher
- **Serialization**: CDR (Common Data Representation) for ROS2-compatible messages
- **Message Format**: Publishes `sensor_msgs/msg/PointCloud2`, `sensor_msgs/msg/Image`, `geometry_msgs/msg/TransformStamped`
- **Transport**: Zenoh pub/sub with configurable QoS (priority, congestion control)
- **Components**:
  - `ouster.rs` – Sensor data handling & UDP packet parsing
  - `cluster.rs` – Point cloud clustering algorithms
  - `args.rs` – CLI argument parsing
  - `rerun.rs` – Optional visualization integration
  - `main.rs` / `lib.rs` – Entrypoint and core orchestration logic

**Performance Context:**
- Real-time LiDAR processing at 10-20Hz frame rates
- Low-latency requirements (<10ms processing per frame)
- Edge deployment on resource-constrained platforms
- Maivin/Raivin module integration for NPU/GPU acceleration

**Related Middleware Services:**

The EdgeFirst Perception Middleware is a modular software stack where services communicate over Zenoh using ROS2 CDR message encoding. Other services in the stack include:

- **Camera Service** - ISP integration, H.265 encoding for streaming/recording
- **Radar Publisher** - Radar sensor integration and processing  
- **Vision Model Service** - AI inference for object detection and segmentation
- **Fusion Model Service** - Multi-sensor fusion for 3D perception
- **IMU Publisher** - Inertial measurement unit data
- **GPS/NavSat Publisher** - GNSS positioning data
- **Recorder Service** - MCAP recording for data capture and replay
- **Studio Client** - EdgeFirst Studio integration and telemetry

Contributors should understand how this LiDAR publisher fits into the broader perception ecosystem and consider compatibility with other services when making architectural decisions.

**Learn More:** [EdgeFirst Perception Documentation](https://doc.edgefirst.ai/test/perception/)

---

## Git Workflow

### Branch Naming Convention

**REQUIRED FORMAT**: `<type>/<PROJECTKEY-###>[-optional-description]`

**Branch Types:**
- `feature/` - New features and enhancements
- `bugfix/` - Non-critical bug fixes
- `hotfix/` - Critical production issues requiring immediate fix

**Examples:**
```bash
feature/LIDAR-123-add-velodyne-support
bugfix/LIDAR-456-fix-clustering-crash
hotfix/LIDAR-789-packet-parsing-overflow
```

**Rules:**
- JIRA key is REQUIRED (format: `PROJECTKEY-###`)
- Description is OPTIONAL but recommended for clarity
- Use kebab-case for descriptions (lowercase with hyphens)
- Branch from `develop` for features/bugfixes, from `main` for hotfixes

**Note**: External contributors without JIRA access can use GitHub issue references: `feature/issue-123-description`

### Commit Message Format

**REQUIRED FORMAT**: `PROJECTKEY-###: Brief description of what was done`

**Rules:**
- Subject line: 50-72 characters ideal
- Focus on WHAT changed, not HOW (implementation details belong in code)
- No type prefixes (`feat:`, `fix:`, etc.) - JIRA provides context
- Optional body: Use bullet points for additional detail

**Examples of Good Commits:**
```bash
LIDAR-123: Add Velodyne VLP-16 sensor support

LIDAR-456: Fix clustering crash on empty point clouds

LIDAR-789: Optimize SIMD point transformations
- Implemented vectorized distance calculations
- Reduced frame processing latency by 30%
- Added benchmarks to verify improvements
```

**Conventional Commits Alternative:**

For external contributors or pre-JIRA work, use conventional commits:
```bash
feat: add Velodyne VLP-16 support
fix: resolve clustering crash on empty frames
perf: optimize SIMD transformations
docs: improve installation instructions
test: add integration tests for Zenoh publisher
```

Types: `feat`, `fix`, `docs`, `style`, `refactor`, `perf`, `test`, `chore`

---

## Code Quality Standards

### Rust-Specific Best Practices

### Rust-Specific Best Practices

- **Code Style**: Rust 2024 edition; follow `cargo fmt` defaults
- **Error Handling**: Define custom error enums with `From` implementations for underlying library errors; avoid `unwrap()` in non-test code
- **Logging**: Use `tracing` crate for structured logging and diagnostics
- **Performance**: Minimize allocations in hot loops (packet processing & clustering)
- **Zero-Copy**: Use iterator adaptors & slices for zero-copy where possible
- **Ownership**: Minimize clones, use references where possible
- **Async**: Use Tokio runtime properly, avoid blocking in async contexts
- **Safety**: Minimize `unsafe` code, document why it's needed

### Code Review Checklist

Before submitting code, verify:
- [ ] Code follows Rust conventions (`cargo fmt` and `cargo clippy` pass)
- [ ] No `unwrap()` or `expect()` in library code (tests are OK)
- [ ] Complex logic has explanatory comments
- [ ] Public APIs have rustdoc documentation
- [ ] No hardcoded values that should be CLI arguments or configuration
- [ ] Resource cleanup (memory, network sockets) is proper
- [ ] No obvious panics or integer overflow risks
- [ ] Performance-critical paths are optimized (profile if unsure)

### SonarQube Integration

This project uses SonarQube for code quality. Before submitting:
- Check `sonar-project.properties` for quality gate thresholds
- Use VSCode SonarLint plugin for real-time feedback
- Address critical and high-severity issues
- Maintain or improve project quality scores

---

## Testing Requirements

### Coverage Standards

- **Minimum coverage**: 70% (enforced by CI)
- **Critical paths**: 90%+ coverage for packet parsing, clustering, publishing
- **Edge cases**: Explicit tests for boundary conditions
- **Error paths**: Validate error handling and recovery

### Test Types

**Unit Tests:**
- Test individual functions in isolation
- Mock external dependencies (sensors, network)
- Fast execution (< 1 second per test suite)
- Focus on algorithmic correctness (clustering logic, transformations)

**Integration Tests:**
- Test component interactions (UDP → frame → publish pipeline)
- Use mock packet streams (avoid requiring physical sensors in CI)
- Validate Zenoh publishing and ROS2 message compatibility
- Test configuration and initialization

**Edge Case Tests:**
- Empty point clouds
- Boundary values (min/max ranges, cluster thresholds)
- Malformed UDP packets
- Network disconnections and reconnection
- Sensor mode changes mid-stream

### Test Organization

**Rust Pattern:**
```rust
// Unit tests at end of implementation file
// src/cluster.rs
pub fn cluster_points(points: &Points, threshold: f32) -> Vec<Cluster> {
    // implementation
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cluster_empty_points() {
        let points = Points::new();
        let clusters = cluster_points(&points, 0.5);
        assert!(clusters.is_empty());
    }
    
    #[test]
    fn test_cluster_threshold_boundary() {
        // Test edge cases around threshold values
    }
}
```

```
# Integration tests in separate directory
tests/
├── integration_test.rs
└── common/
    └── mod.rs
```

### Running Tests

```bash
# Run all tests
cargo test --workspace

# Run with output
cargo test -- --nocapture

# Run specific test
cargo test test_cluster

# Run with all features
cargo test --all-features

# Check test coverage (requires cargo-tarpaulin)
cargo tarpaulin --out Html --output-dir coverage/
```

---

## Documentation Expectations

### Code Documentation

**When to document:**
- Public APIs, functions, and structs (ALWAYS)
- Complex algorithms (clustering, transformations)
- Performance considerations or optimization rationale
- Thread safety and concurrency requirements
- Hardware-specific code or platform dependencies

**Rustdoc Style:**
```rust
/// Processes a LiDAR point cloud frame and applies DBSCAN clustering.
///
/// Clusters are formed by grouping points within `threshold` distance of each other.
/// Uses spatial hashing for O(n log n) performance on typical point clouds.
///
/// # Arguments
///
/// * `points` - Raw point cloud data from the sensor (XYZ coordinates)
/// * `threshold` - Distance threshold for clustering in meters
///
/// # Returns
///
/// Vector of clustered point groups, sorted by cluster size (largest first)
///
/// # Examples
///
/// ```
/// let points = parse_lidar_frame(&packet_buffer)?;
/// let clusters = cluster_points(&points, 0.5);
/// for cluster in clusters {
///     println!("Cluster has {} points", cluster.len());
/// }
/// ```
///
/// # Performance
///
/// Typical performance on Maivin platform:
/// - 1024x10 frame (10k points): ~2ms
/// - 2048x10 frame (20k points): ~5ms
pub fn cluster_points(points: &Points, threshold: f32) -> Vec<Cluster> {
    // Implementation
}
```

### Documentation Updates

When modifying code, update corresponding documentation:
- `README.md` if CLI flags or features change
- Rustdoc comments if function signatures or semantics change
- `CONTRIBUTING.md` if development workflow changes
- CLI help text in `args.rs` for new options

---

## License Policy

**CRITICAL**: Au-Zone has strict license policy for all dependencies.

### Allowed Licenses

✅ **Permissive licenses (APPROVED)**:
- MIT
- Apache-2.0
- BSD-2-Clause, BSD-3-Clause
- ISC
- Unlicense

### Strictly Disallowed

❌ **NEVER USE THESE LICENSES**:
- GPL (any version)
- AGPL (any version)
- SSPL (Server Side Public License)
- Creative Commons with NC (Non-Commercial) or ND (No Derivatives)

### Verification Process

**Before adding dependencies:**
1. Check `Cargo.toml` license field: `cargo tree -e normal --prefix none --format "{p} {l}"`
2. Verify no GPL/AGPL in dependency tree
3. Use `cargo-deny` for automated license checking (if configured)
4. Document third-party licenses in `NOTICE` file

**If you need a library with incompatible license:**
- Search for alternatives with permissive licenses (crates.io has good filtering)
- Consider implementing functionality yourself
- Escalate to technical leadership for approval (rare exceptions)

---

## Security Practices

### Vulnerability Reporting

For security issues, follow `SECURITY.md` process:
- Email: `support@au-zone.com` with subject "Security Vulnerability"
- Expected acknowledgment: 48 hours
- Expected assessment: 7 days

### Secure Coding Guidelines

**Input Validation (Critical for LiDAR data):**
- Validate UDP packet sizes before parsing
- Check array indices to prevent out-of-bounds access
- Enforce reasonable limits on point cloud sizes
- Sanitize sensor IP addresses and network configuration

**Network Security:**
- Validate source IP for UDP packets (prevent spoofing)
- Rate-limit incoming packets to prevent DoS
- Use Zenoh's security features for production deployments
- No hardcoded credentials or API keys

**Common Vulnerabilities to Avoid:**
- **Buffer Overflows**: Use safe Rust slicing; check packet lengths
- **Integer Overflow**: Use checked arithmetic for size calculations
- **Denial of Service**: Implement backpressure and rate limiting
- **Path Traversal**: Validate file paths for recordings/playback

### Dependencies

- Keep dependencies up to date: `cargo update`
- Monitor for security advisories: `cargo audit`
- Review new dependencies before adding
- Prefer well-maintained crates with active communities

---

## Performance & Edge Considerations

---

## Performance & Edge Considerations

### Performance Targets

- **Frame Processing Latency**: < 10ms per frame (critical path)
- **Clustering Latency**: < 2ms per frame
- **Memory Footprint**: < 512MB for typical operation
- **CPU Usage**: < 15% single core on Maivin platform
- **Network Throughput**: Handle 10-20Hz LiDAR streams without drops

### Critical Performance Paths

- UDP packet parsing → frame assembly → point transform → publish
- Clustering must be O(n log n) or better; review with benchmarks before merging large changes
- Zero-copy data paths where possible (use slices, avoid cloning large buffers)
- SIMD operations for point transformations on supported platforms

### Optimization Guidelines

- **Memory**: Reuse buffers across frames; pre-allocate with `Vec::with_capacity`
- **CPU**: Profile hot loops with `cargo flamegraph` or Tracy profiler
- **Async**: Don't block Tokio threads; use `spawn_blocking` for CPU-intensive work
- **Features**: Ensure optional features (`rerun`, `profiling`) are gated with `#[cfg(feature = ...)]`

### Hardware Platform Notes

**EdgeFirst Modules:**
- **Maivin** - Vision-only module (4K/8MP, Renesas RZ/V2H, EdgeFirst Perception Engine)
- **Raivin** - Maivin + integrated 4D radar with sensor fusion for enhanced spatial perception in harsh environments (dust, low light, adverse weather)
- Both modules: IP67-rated, -40°C to +65°C operation, designed for rugged off-road and industrial use
- NPU/GPU acceleration available for point cloud processing
- SIMD support for ARM NEON instructions
- Consider power constraints (< 5W average)
- Test on target hardware before claiming performance improvements

**Platform Resources:**
- [EdgeFirst Modules](https://www.edgefirst.ai/edgefirstmodules)
- Standard configurations, development kits, and custom options available

**Benchmarking:**
- Use `criterion` for micro-benchmarks (future addition)
- Include benchmark results in PRs for performance-sensitive changes
- Document test platform (CPU, memory, OS) with benchmark results

---

## Error Handling Strategy

### Error Categories

**Recoverable Errors (log warning, retry):**
- Sensor network timeout or packet loss
- Temporary Zenoh connection issues
- Malformed packets (log and skip to next frame)

**Irrecoverable Errors (log error, exit cleanly):**
- Invalid CLI arguments or configuration
- Cannot bind to UDP port
- Critical resource allocation failures

### Error Handling Pattern

```rust
// Define custom error type in library module (e.g., ouster.rs)
pub enum Error {
    IoError(std::io::Error),
    SystemTimeError(std::time::SystemTimeError),
    ShapeError(ndarray::ShapeError),
    UnsupportedDataFormat(String),
    UnexpectedEndOfSlice(usize),
    UnknownPacketType(u16),
}

impl std::error::Error for Error {}

// Implement From traits for interop with underlying libraries
impl From<std::io::Error> for Error {
    fn from(err: std::io::Error) -> Error {
        Error::IoError(err)
    }
}

impl From<ndarray::ShapeError> for Error {
    fn from(err: ndarray::ShapeError) -> Error {
        Error::ShapeError(err)
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter) -> std::fmt::Result {
        match self {
            Error::IoError(err) => write!(f, "io error: {}", err),
            Error::UnsupportedDataFormat(format) => {
                write!(f, "unsupported lidar data format: {}", format)
            }
            // ... other variants
        }
    }
}

// Use in function signatures
pub fn process_lidar_stream(config: &Config) -> Result<(), Error> {
    let socket = bind_udp_socket(&config.bind_addr)?; // auto-converts via From trait
    // ... processing logic
    Ok(())
}
```

**Use custom error types** with `From` trait implementations to enable the `?` operator for automatic error conversion.

---

## Testing Guidance

### Unit Tests

Focus on edge cases:
- **Empty frames**: Clustering with zero points
- **Dense clusters**: Large number of closely-spaced points
- **Threshold boundaries**: Points exactly at cluster distance threshold
- **Numeric edge cases**: Min/max float values, NaN handling

### Integration Tests

- Mock packet streams (avoid requiring live hardware in CI)
- Test full pipeline: packet → frame → cluster → publish
- Validate Zenoh message formats match ROS2 schemas (sensor_msgs::PointCloud2, sensor_msgs::Image)
- Verify CDR serialization produces ROS2-compatible binary output
- Test graceful shutdown and cleanup

### Benchmarks (Future)

- Packet → frame throughput
- Clustering performance vs. point count
- Memory allocation patterns over time

---

## Suggested AI Tasks

Assistants may:
- Propose micro-optimizations with benchmarks
- Generate additional rustdoc examples
- Suggest safe concurrency patterns for packet ingestion
- Draft hardware-specific tuning notes (under review before merge)
- Add edge case tests with property-based testing
- Improve error messages and diagnostics

---

## Non-Goals for AI

- Introducing heavy dependencies unless justified by profiling
- Adding unsafe code without clear, measurable benefit
- Refactoring purely for stylistic changes without performance/readability gains
- Changing cluster algorithms without benchmark comparisons
- Breaking CLI compatibility without migration guide

---

## Future Enhancements (Hints)

- **Additional Ouster packet formats**: Support RNG19_RFL8_SIG16_NIR16, LEGACY, and other profiles
- **Newer firmware support**: Ouster firmware 3.x compatibility
- **Multi-vendor support**: Velodyne (VLP-16, VLS-128), Livox (Mid-70, Avia), and others
- Multi-sensor time synchronization module
- Streaming compression for low-bandwidth links
- Telemetry export to EdgeFirst Studio (metrics + events)
- GPU/NPU acceleration for transforms on Maivin/Raivin modules
- Point cloud recording and playback for testing

---

## Getting Help

**For development questions:**
- Check `CONTRIBUTING.md` for setup instructions
- Review existing code for patterns
- Search GitHub Issues for similar problems
- Open a discussion for architectural questions

**For security concerns:**
- Email `support@au-zone.com` with subject "Security Vulnerability"
- Do not disclose vulnerabilities publicly

**For license questions:**
- Review [License Policy](#license-policy) section above
- Check `LICENSE` file and dependency licenses
- Contact technical leadership if unclear

---

## Workflow Example

**Implementing a new feature:**

```bash
# 1. Create branch from develop
git checkout develop
git pull origin develop
git checkout -b feature/LIDAR-123-add-velodyne-support

# 2. Implement feature with tests
# - Write unit tests first (TDD)
# - Implement functionality
# - Add integration tests
# - Update documentation

# 3. Verify quality
cargo fmt           # Auto-format code
cargo clippy        # Run linter
cargo test          # Run all tests
cargo build         # Ensure it builds

# 4. Commit with proper message
git add .
git commit -m "LIDAR-123: Add Velodyne VLP-16 sensor support

- Implemented packet parser for VLP-16 format
- Added sensor detection and auto-configuration
- Comprehensive unit and integration tests
- Updated CLI arguments and documentation"

# 5. Push and create PR
git push -u origin feature/LIDAR-123-add-velodyne-support
# Create PR via Bitbucket/GitHub UI

# 6. Address review feedback
# - Make requested changes
# - Push additional commits
# - Respond to comments

# 7. Merge after approvals
# Maintainer merges via PR interface
```

---

## Working with AI Assistants

### Best Practices

**Provide Context:**
- Share relevant files, error messages, requirements
- Reference existing patterns in codebase
- Mention performance constraints and target platforms

**Verify Outputs:**
- Review generated code critically before committing
- Check that APIs actually exist (no hallucinated functions)
- Ensure suggestions match Rust 2024 edition best practices
- Run tests and benchmarks to verify correctness

**Iterate:**
- Refine solutions through follow-up questions
- Ask for alternative approaches
- Request explanations for complex generated code

### Common AI Pitfalls to Avoid

- **Hallucinated APIs**: Verify Zenoh/Rerun APIs exist in current versions
- **Outdated patterns**: Check suggestions match Rust 2024 idioms
- **Over-engineering**: Prefer simple solutions over complex ones
- **Missing edge cases**: Explicitly test boundary conditions
- **License violations**: AI may suggest code with incompatible licenses

---

*Maintain this file only if it adds clarity for AI assistance. Remove if it becomes outdated.*

