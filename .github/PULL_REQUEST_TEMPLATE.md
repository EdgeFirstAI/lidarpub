## Description

<!-- Provide a clear and concise description of your changes -->

## Type of Change

<!-- Mark the relevant option with an 'x' -->

- [ ] Bug fix (non-breaking change that fixes an issue)
- [ ] New feature (non-breaking change that adds functionality)
- [ ] Breaking change (fix or feature that would cause existing functionality to not work as expected)
- [ ] Documentation update
- [ ] Performance improvement
- [ ] Code refactoring
- [ ] Dependency update

## Related Issues

<!-- Link related issues. Use "Fixes #123" to auto-close issues when PR is merged -->

Fixes #(issue number)
Related to #(issue number)

## Testing

<!-- Describe the tests you ran and how to reproduce them -->

- [ ] Tests pass locally (`cargo test --workspace`)
- [ ] Code coverage maintained or improved
- [ ] Tested on relevant platforms (specify below)
- [ ] Integration tests added/updated (if applicable)
- [ ] Performance benchmarks run (if applicable)

**Platforms Tested:**
- [ ] x86_64 Linux
- [ ] ARM64 Linux (target platforms)
- [ ] Cross-compilation verified

**Sensor Testing:**
- [ ] Tested with physical Ouster sensor
- [ ] Tested with simulated/recorded data
- [ ] N/A (code changes don't affect sensor interaction)

## Code Quality Checklist

- [ ] Code follows project style (`cargo fmt` applied)
- [ ] Self-reviewed code for clarity and correctness
- [ ] No new compiler warnings (`cargo clippy -- -D warnings` passes)
- [ ] No new security vulnerabilities (`cargo audit` passes)
- [ ] Comments added for complex logic
- [ ] Public APIs documented with rustdoc comments

## Documentation

- [ ] README.md updated (if needed)
- [ ] CHANGELOG.md updated with changes
- [ ] ARCHITECTURE.md updated (if architecture changed)
- [ ] API documentation added/updated for public functions
- [ ] Examples added/updated (if new features)

## Performance Impact

<!-- If this PR affects performance, describe the impact -->

- [ ] No performance impact expected
- [ ] Performance improved (provide benchmarks)
- [ ] Performance may be affected (explain why it's acceptable)

**Benchmark Results (if applicable):**
```
Before: ...
After: ...
```

## Breaking Changes

<!-- If this introduces breaking changes, describe them and provide migration guidance -->

- [ ] No breaking changes
- [ ] Breaking changes described below

**Migration Guide (if breaking changes):**
```
<!-- How should users adapt to this change? -->
```

## Screenshots/Videos

<!-- If applicable, add screenshots or videos demonstrating the changes -->

## Additional Notes

<!-- Any additional information, context, or concerns -->

## Contributor Checklist

- [ ] I have read the [CONTRIBUTING.md](../CONTRIBUTING.md) guidelines
- [ ] I have followed the [Code of Conduct](../CODE_OF_CONDUCT.md)
- [ ] My commits follow the conventional commit format
- [ ] I have rebased my branch on the latest `develop` branch
- [ ] I am willing to address review feedback

---

**For Maintainers:**
- [ ] Labels applied
- [ ] Milestone assigned (if applicable)
- [ ] Reviewers assigned
- [ ] Ready for merge
