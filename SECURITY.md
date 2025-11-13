# Security Policy

## Supported Versions

| Version | Support Status |
|---------|----------------|
| main    | ✅ Full support |
| 0.x     | 🔒 Security fixes only |
| < 0.1   | ❌ End of life |

This project is pre-1.0; expect rapid iteration. The `main` branch receives full support and fixes. Once 1.0 is released, versions will follow semantic versioning with defined support tiers.

## Reporting a Vulnerability

Au-Zone Technologies takes security seriously across the EdgeFirst ecosystem.

### How to Report

Email: **support@au-zone.com** (Subject: "Security Vulnerability")

If the issue may impact other EdgeFirst components or Studio integrations, note that in your report.

Please include:
- Vulnerability description
- Steps to reproduce (commands, configuration, environment)
- Affected commit or version (git SHA, tag, or branch)
- Potential impact (confidentiality, integrity, availability)
- Suggested fixes or mitigations (if any)

### Alternative Private Reporting

You may also create a private advisory (GitHub Security Advisories if mirrored) or request an encrypted channel; mention this in your email.

### What to Expect

1. **Acknowledgment** within 48 hours
2. **Initial assessment** within 7 days
3. **Fix timeline** (target goals):
   - Critical: 7 days
   - High: 30 days
   - Medium: Next minor release
   - Low: Next major release or scheduled refactor

Severity uses CVSS plus real-world impact on edge deployments.

### Responsible Disclosure

We ask that you:
- Allow reasonable time for remediation before public disclosure
- Avoid exploitation beyond proof-of-concept
- Do not publicly share details until a fix is released

### Recognition

With permission, we credit reporters in:
- Release notes
- Security advisories
- Annual security summary

## Hardening & Recommendations

Operators should:
- Run latest `main` or tagged release
- Restrict network exposure of LiDAR and Zenoh ports
- Use secure time synchronization sources
- Monitor resource usage for anomalies
- Update dependencies regularly (tracked via `cargo audit`)

## Security Update Channels

We announce fixes through:
- Repository release notes
- (Future) GitHub Security Advisories
- EdgeFirst Studio notifications (for integrated components)
- Optional mailing list (coming soon at edgefirst.ai/security)

## Additional Services

For production deployments requiring enhanced security:
- Security audits & threat modeling
- Hardening guides for Maivin platforms
- Priority patch delivery & SLAs

Contact **support@au-zone.com** for enterprise security services.

## Reporting Code of Conduct Violations

For behavior concerns, see `CODE_OF_CONDUCT.md` and email with subject "Code of Conduct".

---
*Last updated: November 2025*
