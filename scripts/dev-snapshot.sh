#!/bin/bash
set -e

# Update a running service instance with latest code
# Usage: ./scripts/dev-snapshot.sh INSTANCE_NAME

INSTANCE_NAME=${1}
APP_NAME="mechanical-press"
LAUNCH_PKG="mechanical_press"
LAUNCH_FILE="mechanical_press.launch.py"
ROS_DISTRO=${ROS_DISTRO:-"jazzy"}

if [ -z "$INSTANCE_NAME" ]; then
    echo "Usage: $0 INSTANCE_NAME"
    echo ""
    echo "Example: $0 my-press"
    echo ""
    echo "This updates the specified instance with your latest code changes."
    exit 1
fi

INSTANCE_DIR="/opt/rosapps/mechanical-press-instances/$INSTANCE_NAME"
SERVICE_NAME="mechanical-press-$INSTANCE_NAME"

# Check if instance exists
if [ ! -d "$INSTANCE_DIR" ]; then
    echo "Error: Instance '$INSTANCE_NAME' not found at $INSTANCE_DIR"
    echo "Create it first with:"
    echo "  ./scripts/create-instance.sh --name $INSTANCE_NAME --namespace /$INSTANCE_NAME --config config/examples/development.yaml"
    exit 1
fi

echo "Updating instance '$INSTANCE_NAME' with latest code..."

# Build to temporary location first (fail early if build fails)
echo "Building workspace..."
INSTALL_BASE="/tmp/${APP_NAME}-update-$(date +%s)"
source "/opt/ros/$ROS_DISTRO/setup.bash"

if ! colcon build --merge-install --packages-up-to "$LAUNCH_PKG" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  --install-base "$INSTALL_BASE"; then
    echo "Error: Build failed! Service not updated."
    rm -rf "$INSTALL_BASE" 2>/dev/null || true
    exit 1
fi

# Only stop service after successful build
echo "Build successful! Stopping service for update..."
sudo systemctl stop "$SERVICE_NAME.service" 2>/dev/null || echo "Service not running"

# Update instance installation
echo "Updating instance installation..."
sudo rm -rf "$INSTANCE_DIR/install"
sudo mkdir -p "$INSTANCE_DIR/install"
sudo cp -r "$INSTALL_BASE"/* "$INSTANCE_DIR/install/"
sudo chown -R emoco:emoco "$INSTANCE_DIR/install"

# Clean up temporary build
rm -rf "$INSTALL_BASE"

# Start the service
echo "Starting service..."
sudo systemctl start "$SERVICE_NAME.service"

echo ""
echo "✓ Instance '$INSTANCE_NAME' updated successfully!"
echo ""
echo "Service: $SERVICE_NAME.service"
echo "Installation: $INSTANCE_DIR/install/"
echo ""
echo "Check status:"
echo "  sudo systemctl status $SERVICE_NAME.service"
echo ""
echo "View logs:"
echo "  journalctl -u $SERVICE_NAME -f"