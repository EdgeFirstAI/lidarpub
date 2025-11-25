# Contributing to EdgeFirst LiDAR Publisher

Thank you for your interest in contributing! This project is part of the EdgeFirst Perception stack, advancing edge AI and LiDAR-based perception capabilities for autonomous systems.

## Code of Conduct

Please read our [Code of Conduct](CODE_OF_CONDUCT.md) before contributing.

## Ways to Contribute

- **Code** - Features, bug fixes, performance improvements
- **Documentation** - Improvements, examples, tutorials, sensor guides
- **Testing** - Bug reports, test coverage, hardware platform validation
- **Community** - Answer questions, write blog posts, create demos

## Before You Start

1. Check existing [issues](https://github.com/EdgeFirstAI/lidarpub/issues) and [pull requests](https://github.com/EdgeFirstAI/lidarpub/pulls)
2. For significant changes, open an issue for discussion first
3. Review our roadmap to understand project direction
4. Consider how changes might affect EdgeFirst Studio integration

## Development Setup

### Prerequisites

- **Rust**: Version specified in `rust-toolchain.toml` (currently 2024 edition)
- **Ouster LiDAR**: Physical sensor or simulated data for testing
- **Zenoh**: Router for multi-node testing (optional)
- **EdgeFirst Studio**: Account for integration testing (free tier available at [edgefirst.studio](https://edgefirst.studio))
- **Hardware**: Maivin platform recommended for performance testing (optional)

### Initial Setup

```bash
# Clone the repository
git clone https://github.com/EdgeFirstAI/lidarpub.git
cd lidarpub

# Build the project
cargo build

# Run tests
cargo test

# Check formatting
cargo fmt --check

# Run linter
cargo clippy -- -D warnings
```

### Building with Features

```bash
# Build with Rerun visualization support
cargo build --features rerun

# Build with profiling support
cargo build --features profiling

# Build with all features
cargo build --all-features

# Release build
cargo build --release
```

### Running Locally

```bash
# Basic run (requires Ouster sensor)
cargo run -- --target <SENSOR_IP>

# With clustering enabled
cargo run -- --target <SENSOR_IP> --cluster

# With Rerun visualization
cargo run --features rerun --bin lidar-rerun -- --target <SENSOR_IP>

# With Tracy profiling
cargo run --features profiling -- --target <SENSOR_IP> --tracy
```

### Testing

```bash
# Run all tests
cargo test

# Run tests with output
cargo test -- --nocapture

# Run specific test
cargo test test_name

# Run integration tests (requires sensor or mock data)
cargo test --test integration_tests

# Test with all features
cargo test --all-features
```

### Test Coverage Requirements

- Unit test coverage minimum: **70%**
- All public APIs must have tests
- Integration tests for Zenoh publishing
- Hardware tests when applicable (document platform used)

## Contribution Process

### 1. Fork and Clone

```bash
# Fork the repository on GitHub, then:
git clone https://github.com/YOUR_USERNAME/lidarpub.git
cd lidarpub
git remote add upstream https://github.com/EdgeFirstAI/lidarpub.git
```

### 2. Create Feature Branch

Use descriptive branch names:
```bash
git checkout -b feature/add-velodyne-support
git checkout -b bugfix/issue-123-clustering-crash
git checkout -b docs/improve-setup-guide
```

### 3. Make Changes

#### Code Style

We follow Rust standard conventions:
- Run `cargo fmt` before committing
- Run `cargo clippy` and fix all warnings
- Use meaningful variable names
- Add comments for complex logic
- Document all public APIs with rustdoc

Example:
```rust
/// Processes a LiDAR point cloud frame and applies clustering.
///
/// # Arguments
///
/// * `points` - Raw point cloud data from the sensor
/// * `threshold` - Distance threshold for clustering in meters
///
/// # Returns
///
/// Vector of clustered point groups
///
/// # Examples
///
/// ```
/// let clusters = cluster_points(&points, 0.5);
/// ```
pub fn cluster_points(points: &Points, threshold: f32) -> Vec<Cluster> {
    // Implementation
}
```

#### Documentation

- Update README.md if adding features
- Add rustdoc comments for public APIs
- Include examples in documentation
- Update CHANGELOG.md (follow Keep a Changelog format)

#### Testing

Add tests for new functionality:

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cluster_empty_points() {
        let points = Points::new();
        let clusters = cluster_points(&points, 0.5);
        assert!(clusters.is_empty());
    }
}
```

### 4. Commit Changes

Use conventional commit messages:

```bash
git commit -m "feat: add Velodyne VLP-16 support"
git commit -m "fix: resolve clustering crash on empty frames"
git commit -m "docs: improve installation instructions"
git commit -m "perf: optimize SIMD transformations"
git commit -m "test: add integration tests for Zenoh publisher"
```

Types: `feat`, `fix`, `docs`, `style`, `refactor`, `perf`, `test`, `chore`

### 5. Push and Create Pull Request

```bash
git push origin feature/your-feature-name
```

Then create a pull request on GitHub with:
- Clear title and description
- Reference related issues (e.g., "Closes #123")
- Description of changes and motivation
- Testing performed (include platform details)
- Screenshots/videos for visual changes

### 6. Code Review

- Respond to feedback promptly
- Make requested changes in new commits
- Keep discussions professional and constructive
- Request re-review when ready

## Development Guidelines

### Rust-Specific Best Practices

- **Error Handling**: Use `Result` and `?` operator, avoid `unwrap()` in library code
- **Ownership**: Minimize clones, use references where possible
- **Async**: Use Tokio runtime properly, avoid blocking in async contexts
- **Safety**: Minimize `unsafe` code, document why it's needed
- **Performance**: Profile before optimizing, document performance-critical sections

### Performance Considerations

This is a real-time system. Keep in mind:
- Target <10ms frame processing latency
- Minimize memory allocations in hot paths
- Use zero-copy techniques where possible
- SIMD operations for point transformations
- Benchmark on target hardware (Maivin platforms)

### Zenoh Integration

- Follow Zenoh best practices for publishers
- Use appropriate QoS settings
- Handle network disconnections gracefully
- Document data formats and topics

### EdgeFirst Studio Integration

When adding features that affect Studio integration:
- Test with Studio deployment workflow
- Update integration documentation
- Consider monitoring and telemetry needs
- Maintain compatibility with existing deployments

## Sensor Support

### Adding New LiDAR Sensors

When adding support for new sensors:

1. Create sensor-specific module in `src/`
2. Implement common traits for data handling
3. Add configuration options to `args.rs`
4. Document sensor-specific requirements
5. Add integration tests with real or simulated data
6. Update README with supported sensors list

### Testing with Sensors

- Document which sensor models were tested
- Include sensor firmware versions
- Note any calibration requirements
- Provide example configurations

## Documentation Guidelines

### Rustdoc Comments

All public items require documentation:

```rust
/// Brief description (one line)
///
/// Longer description explaining purpose, behavior, and usage.
///
/// # Arguments
///
/// * `arg` - Description
///
/// # Returns
///
/// Description of return value
///
/// # Errors
///
/// When this function returns an error
///
/// # Examples
///
/// ```
/// // Example code
/// ```
pub fn function(arg: Type) -> Result<ReturnType, Error> {
    // Implementation
}
```

### README Updates

When adding features, update README:
- Add to Features section
- Update Quick Start if needed
- Add usage examples
- Update configuration options

## Release Process

Maintainers follow this release process:

1. Update version in `Cargo.toml`
2. Update `CHANGELOG.md`
3. Create release tag
4. Build release binaries
5. Update EdgeFirst Studio integration
6. Announce release

## Getting Help

- **Questions**: Use [GitHub Discussions](https://github.com/EdgeFirstAI/lidarpub/discussions)
- **Bugs**: Open an [issue](https://github.com/EdgeFirstAI/lidarpub/issues)
- **Chat**: Join our community (link TBD)
- **Commercial Support**: Contact support@au-zone.com

## License

By contributing, you agree that your contributions will be licensed under Apache License 2.0. No additional contributor agreement required.

---

## Quick Reference

```bash
# Check your changes
cargo fmt --check
cargo clippy -- -D warnings
cargo test

# Before submitting
cargo fmt
cargo build --release
cargo test --all-features

# Create PR with good commit message
git commit -m "feat: clear description of what you added"
```

Thank you for contributing to the EdgeFirst Perception stack! 🚀
