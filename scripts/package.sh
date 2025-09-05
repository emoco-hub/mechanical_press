#!/bin/bash
set -e

# Package mechanical press using standard ROS bloom workflow
# Usage: ./scripts/package.sh [version] [ros_distro] [ubuntu_codename]

VERSION=${1:-"1.0.0"}
ROS_DISTRO=${2:-"jazzy"}
UBUNTU_CODENAME=${3:-"noble"}  # Default to Ubuntu 24.04 Noble for Jazzy

echo "Packaging mechanical_press version $VERSION for ROS $ROS_DISTRO (Ubuntu $UBUNTU_CODENAME)"
echo "Using standard ROS bloom packaging workflow"

# Check dependencies
if ! command -v bloom-generate &> /dev/null; then
    echo "Error: bloom not found. Install with:"
    echo "  sudo apt install python3-bloom"
    echo "  # or"
    echo "  pip3 install bloom"
    exit 1
fi

if ! command -v fakeroot &> /dev/null; then
    echo "Error: fakeroot not found. Install with:"
    echo "  sudo apt install fakeroot"
    exit 1
fi

# Clean previous packaging artifacts
echo "Cleaning previous packaging artifacts..."
rm -rf debian/
rm -f ../*.deb ../*.tar.gz ../*.dsc ../*.changes

# Update package.xml version if specified
if [ "$VERSION" != "1.0.0" ]; then
    echo "Updating package.xml version to $VERSION"
    sed -i.bak "s|<version>.*</version>|<version>$VERSION</version>|" package.xml
fi

# Generate Debian packaging files using bloom
echo "Generating Debian packaging files..."
bloom-generate rosdebian \
    --os-name ubuntu \
    --os-version "$UBUNTU_CODENAME" \
    --ros-distro "$ROS_DISTRO"

# Update changelog with proper version
echo "Updating changelog..."
# bloom-generate creates a changelog, but we might want to customize it
sed -i "s/^ros-$ROS_DISTRO-mechanical-press (.*)/ros-$ROS_DISTRO-mechanical-press ($VERSION-1$UBUNTU_CODENAME)/" debian/changelog

# Build the Debian package
echo "Building Debian package..."
fakeroot debian/rules binary

# Find and report the generated package
DEB_FILE=$(find .. -name "ros-$ROS_DISTRO-mechanical-press_*.deb" -type f | head -n 1)

if [ -n "$DEB_FILE" ]; then
    # Move package to current directory for convenience
    mv "$DEB_FILE" "./$(basename "$DEB_FILE")"
    echo ""
    echo "✓ Package created successfully!"
    echo "Package: $(basename "$DEB_FILE")"
    echo "Size: $(du -h "$(basename "$DEB_FILE")" | cut -f1)"
    echo ""
    echo "To install:"
    echo "  sudo dpkg -i $(basename "$DEB_FILE")"
    echo ""
    echo "Package contents:"
    dpkg-deb --contents "$(basename "$DEB_FILE")" | head -10
    if [ $(dpkg-deb --contents "$(basename "$DEB_FILE")" | wc -l) -gt 10 ]; then
        echo "  ... ($(dpkg-deb --contents "$(basename "$DEB_FILE")" | wc -l) total files)"
    fi
else
    echo "Error: Package creation failed - no .deb file found"
    exit 1
fi

# Clean up packaging artifacts
echo ""
echo "Cleaning up packaging artifacts..."
rm -rf debian/

# Restore original package.xml if we modified it
if [ -f package.xml.bak ]; then
    mv package.xml.bak package.xml
fi

echo ""
echo "Packaging complete!"