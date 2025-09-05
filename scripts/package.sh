#!/bin/bash
set -e

# Package mechanical press using standard ROS bloom workflow
# Usage: ./scripts/package.sh [version] [ros_distro] [ubuntu_codename]
#
# Prerequisites:
#   sudo apt install python3-bloom fakeroot debhelper

VERSION=${1:-"1.0.0"}
ROS_DISTRO=${2:-"jazzy"}
# Default to Ubuntu version officially supported by the ROS distro
if [ "$ROS_DISTRO" = "jazzy" ]; then
    UBUNTU_CODENAME=${3:-"noble"}
elif [ "$ROS_DISTRO" = "humble" ]; then
    UBUNTU_CODENAME=${3:-"jammy"}
else
    UBUNTU_CODENAME=${3:-"jammy"}  # Safe fallback
fi

echo "Packaging mechanical_press version $VERSION for ROS $ROS_DISTRO (Ubuntu $UBUNTU_CODENAME)"
echo "Using standard ROS bloom packaging workflow"

# Warn about potentially unsupported combinations
if [ "$ROS_DISTRO" = "humble" ] && [ "$UBUNTU_CODENAME" = "noble" ]; then
    echo "⚠ Warning: ROS Humble with Ubuntu Noble may not be fully supported in rosdep"
    echo "  Consider using: ./scripts/package.sh $VERSION humble jammy"
elif [ "$ROS_DISTRO" = "jazzy" ] && [ "$UBUNTU_CODENAME" = "jammy" ]; then
    echo "⚠ Warning: ROS Jazzy with Ubuntu Jammy may have limited support"
    echo "  Consider using: ./scripts/package.sh $VERSION jazzy noble"
fi
echo ""

# Check dependencies
MISSING_DEPS=""
if ! command -v bloom-generate &> /dev/null; then
    MISSING_DEPS="$MISSING_DEPS python3-bloom"
fi

if ! command -v fakeroot &> /dev/null; then
    MISSING_DEPS="$MISSING_DEPS fakeroot"
fi

if ! command -v dh &> /dev/null; then
    MISSING_DEPS="$MISSING_DEPS debhelper"
fi

if [ -n "$MISSING_DEPS" ]; then
    echo "Error: Missing required dependencies:$MISSING_DEPS"
    echo ""
    echo "Install all prerequisites with:"
    echo "  sudo apt install python3-bloom fakeroot debhelper"
    echo ""
    echo "Or install individually:"
    if [[ $MISSING_DEPS == *"python3-bloom"* ]]; then
        echo "  sudo apt install python3-bloom  # or: pip3 install bloom"
    fi
    if [[ $MISSING_DEPS == *"fakeroot"* ]]; then
        echo "  sudo apt install fakeroot"
    fi
    if [[ $MISSING_DEPS == *"debhelper"* ]]; then
        echo "  sudo apt install debhelper dh-python"
    fi
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