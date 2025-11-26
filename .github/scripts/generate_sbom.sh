#!/bin/bash
# Generate complete SBOM using scancode-toolkit
#
# This script implements SBOM generation per the Au-Zone Software Process Specification.
# Last synchronized with policy version: 2.0 (2025-11-22)
#
# This script generates a comprehensive Software Bill of Materials (SBOM)
# in CycloneDX format by:
#   1. Scanning source code directories with scancode
#   2. Scanning package manifests for dependencies
#   3. Merging all SBOMs into a single file
#   4. Validating license policy compliance
#   5. Validating NOTICE file (if present)
#
# CUSTOMIZE FOR YOUR PROJECT:
# - Update PROJECT_NAME and PROJECT_TYPE below
# - Update SOURCE_DIRS list for your source directories
# - Update VERSION_FILE location (single source of truth for version)
# - Update MANIFEST_FILES for your package managers
# - For C/C++ projects: Add system dependencies in Step 3
# - For multi-language: Adjust language-specific sections

set -e  # Exit on error

# ===========================================================================
# PROJECT CONFIGURATION - CUSTOMIZE THESE
# ===========================================================================

PROJECT_NAME="lidarpub"
PROJECT_TYPE="application"  # Options: library, application, framework
VERSION_FILE="Cargo.toml"  # Extract version from Cargo.toml

# Source directories to scan (space-separated)
SOURCE_DIRS="src examples"

# Package manifest files (for dependency parsing)
MANIFEST_FILES="Cargo.toml Cargo.lock"

# ===========================================================================
# SBOM GENERATION
# ===========================================================================

echo "=================================================="
echo "Generating Complete SBOM for $PROJECT_NAME"
echo "=================================================="
echo

# Extract version from Cargo.toml
if [ -f "$VERSION_FILE" ]; then
    VERSION=$(grep '^version = ' "$VERSION_FILE" | head -1 | sed 's/version = "\(.*\)"/\1/')
    echo "Detected version: $VERSION"
else
    VERSION="unknown"
    echo "Warning: Cargo.toml not found, using version: $VERSION"
fi
echo

# Step 1: Generate source code SBOM with scancode
echo "[1/6] Generating source code SBOM with scancode..."

# Check for scancode in venv first, then fall back to system
if [ -f "venv/bin/scancode" ]; then
    SCANCODE="venv/bin/scancode"
    echo "  Using scancode from venv"
elif command -v scancode &> /dev/null; then
    SCANCODE="scancode"
    echo "  Using system scancode"
else
    echo "Error: scancode not found. Please install:"
    echo "  Option 1 (recommended): python3 -m venv venv && venv/bin/pip install scancode-toolkit"
    echo "  Option 2 (system): pip install scancode-toolkit"
    exit 1
fi

# Scan each source directory separately (MUCH faster than scanning all at once)
SBOM_FILES=""
for dir in $SOURCE_DIRS; do
    if [ -d "$dir" ]; then
        echo "  Scanning $dir/..."
        OUTPUT_FILE="source-sbom-$(basename $dir).json"
        $SCANCODE -clpieu \
            --cyclonedx "$OUTPUT_FILE" \
            --only-findings \
            --timeout 300 \
            "$dir/"
        SBOM_FILES="$SBOM_FILES $OUTPUT_FILE"
    fi
done

# Scan manifest files
for file in $MANIFEST_FILES; do
    if [ -f "$file" ]; then
        echo "  Scanning $file..."
        OUTPUT_FILE="source-sbom-$(basename $file .txt | tr '.' '-').json"
        $SCANCODE -clpieu \
            --cyclonedx "$OUTPUT_FILE" \
            --only-findings \
            --timeout 300 \
            "$file"
        SBOM_FILES="$SBOM_FILES $OUTPUT_FILE"
    fi
done

echo "✓ Generated individual SBOM files"
echo

# Step 2: Merge and clean source SBOMs
echo "[2/6] Merging and cleaning source SBOMs..."

python3 << EOF
import json
import sys
import os

VERSION = "$VERSION"
PROJECT_NAME = "$PROJECT_NAME"
PROJECT_TYPE = "$PROJECT_TYPE"

def load_sbom(filename):
    """Load an SBOM file if it exists"""
    if not os.path.exists(filename):
        return None
    with open(filename, 'r') as f:
        return json.load(f)

def clean_sbom_properties(sbom):
    """Remove problematic metadata that violates CycloneDX spec"""
    if 'metadata' in sbom and 'properties' in sbom['metadata']:
        sbom['metadata']['properties'] = [
            p for p in sbom['metadata']['properties']
            if isinstance(p.get('value'), str)
        ]
    return sbom

# Load all individual SBOMs
sbom_files = """$SBOM_FILES""".split()
all_components = []

for filename in sbom_files:
    sbom = load_sbom(filename)
    if not sbom:
        continue

    # Clean the SBOM
    sbom = clean_sbom_properties(sbom)

    # Extract components, filtering out the main project component
    if 'components' in sbom:
        for component in sbom['components']:
            # Skip main project component from scancode - we define it in metadata
            if component.get('name') == PROJECT_NAME.lower():
                continue
            all_components.append(component)

# Create merged source SBOM
merged_sbom = {
    'bomFormat': 'CycloneDX',
    'specVersion': '1.6',
    'version': 1,
    'metadata': {
        'component': {
            'type': PROJECT_TYPE,
            'name': PROJECT_NAME,
            'version': VERSION,
            'licenses': [
                {'license': {'id': 'Apache-2.0'}}
            ]
        }
    },
    'components': all_components
}

# Save merged version
with open('source-sbom.json', 'w') as f:
    json.dump(merged_sbom, f, indent=2)

print(f"Merged {len(sbom_files)} source SBOMs into source-sbom.json")
print(f"Total components: {len(all_components)}")
sys.exit(0)
EOF

echo "✓ Generated source-sbom.json (merged and cleaned)"
echo

# Step 3: Generate dependency SBOM (language-specific)
echo "[3/6] Generating dependency SBOM..."

# For Rust projects with cargo cyclonedx
if [ -f "Cargo.toml" ] && cargo cyclonedx --version &> /dev/null; then
    echo "  Generating Rust dependencies with cargo cyclonedx..."
    cargo cyclonedx --format json
    # cargo cyclonedx creates PROJECT_NAME.cdx.json
    if [ -f "$PROJECT_NAME.cdx.json" ]; then
        mv "$PROJECT_NAME.cdx.json" deps-sbom.json
    fi
fi

# For Python projects (scancode already parsed requirements.txt/pyproject.toml)
# Dependencies are included in source-sbom.json

# For C/C++ projects - manually define system dependencies
if [ ! -f "deps-sbom.json" ]; then
    echo "  Creating empty deps-sbom.json (customize for system dependencies)..."
    cat > deps-sbom.json << 'EOFPYTHON'
{
  "bomFormat": "CycloneDX",
  "specVersion": "1.6",
  "version": 1,
  "components": []
}
EOFPYTHON
fi

echo "✓ Generated deps-sbom.json"
echo

# Step 4: Merge SBOMs using cyclonedx-cli
echo "[4/6] Merging source and dependency SBOMs..."
if ! command -v cyclonedx &> /dev/null; then
    if ! command -v ~/.local/bin/cyclonedx &> /dev/null; then
        echo "Error: cyclonedx CLI not found. Please install from https://github.com/CycloneDX/cyclonedx-cli"
        exit 1
    fi
    CYCLONEDX=~/.local/bin/cyclonedx
else
    CYCLONEDX=cyclonedx
fi

$CYCLONEDX merge \
    --input-files source-sbom.json deps-sbom.json \
    --output-file sbom-temp.json

# Remove duplicate project component and restore dependency graph
python3 << EOF
import json

PROJECT_NAME = "$PROJECT_NAME"

# Load merged SBOM
with open('sbom-temp.json', 'r') as f:
    sbom = json.load(f)

# Load deps SBOM to get dependency graph
with open('deps-sbom.json', 'r') as f:
    deps_sbom = json.load(f)

# Filter out project name from components (it's defined in metadata, not a dependency)
if 'components' in sbom:
    sbom['components'] = [
        c for c in sbom['components']
        if c.get('name') != PROJECT_NAME.lower()
    ]

# Restore dependency graph from deps-sbom (which cargo-cyclonedx generates correctly)
if 'dependencies' in deps_sbom:
    sbom['dependencies'] = deps_sbom['dependencies']
    
# Ensure metadata component has the correct bom-ref from deps-sbom
if 'metadata' in deps_sbom and 'component' in deps_sbom['metadata']:
    deps_bom_ref = deps_sbom['metadata']['component'].get('bom-ref')
    if deps_bom_ref and 'metadata' in sbom and 'component' in sbom['metadata']:
        sbom['metadata']['component']['bom-ref'] = deps_bom_ref

with open('sbom.json', 'w') as f:
    json.dump(sbom, f, indent=2)
EOF

rm -f sbom-temp.json
echo "✓ Generated sbom.json (merged: source + dependencies)"
echo

# Step 5: Check license policy
echo "[5/6] Checking license policy compliance..."
if [ -f ".github/scripts/check_license_policy.py" ]; then
    python3 .github/scripts/check_license_policy.py sbom.json
    POLICY_EXIT=$?
else
    echo "Warning: License policy checker not found, skipping..."
    POLICY_EXIT=0
fi
echo

# Step 6: Validate NOTICE file
echo "[6/6] Validating NOTICE file..."
if [ -f "NOTICE" ] && [ -f ".github/scripts/validate_notice.py" ]; then
    python3 .github/scripts/validate_notice.py NOTICE sbom.json
    NOTICE_EXIT=$?
    if [ $NOTICE_EXIT -ne 0 ]; then
        echo "⚠️  NOTICE file validation warnings - review output above"
    else
        echo "✓ NOTICE file validated (matches first-level dependencies)"
    fi
else
    echo "Skipping NOTICE validation (file or validator not found)"
    NOTICE_EXIT=0
fi
echo

# Cleanup temporary files
rm -f $SBOM_FILES source-sbom.json deps-sbom.json

echo "=================================================="
echo "SBOM Generation Complete"
echo "=================================================="
echo "Files generated:"
echo "  - sbom.json (merged SBOM)"
echo
echo "Files validated:"
echo "  - NOTICE (third-party attributions)"
echo

# Exit with error if either check failed
if [ $POLICY_EXIT -ne 0 ] || [ $NOTICE_EXIT -ne 0 ]; then
    exit 1
fi
exit 0
