#!/bin/bash
set -e

# Update an existing mechanical press instance with new workspace build
# Usage: ./scripts/update-instance.sh INSTANCE_NAME [WORKSPACE_PATH]

INSTANCE_NAME=${1}
WORKSPACE_PATH=${2:-"~/ros2_ws"}

if [ -z "$INSTANCE_NAME" ]; then
    echo "Usage: $0 INSTANCE_NAME [WORKSPACE_PATH]"
    echo ""
    echo "Examples:"
    echo "  $0 dev-press"
    echo "  $0 dev-press ~/my_workspace"
    exit 1
fi

SERVICE_NAME="mechanical-press-$INSTANCE_NAME"
INSTANCE_DIR="/opt/rosapps/mechanical-press-instances/$INSTANCE_NAME"

echo "Updating instance: $INSTANCE_NAME"
echo "Workspace: $WORKSPACE_PATH"
echo "Service: $SERVICE_NAME"

# Check if instance exists
if [ ! -d "$INSTANCE_DIR" ]; then
    echo "Error: Instance '$INSTANCE_NAME' not found at $INSTANCE_DIR"
    echo "Create it first with: ./scripts/create-instance.sh"
    exit 1
fi

# Check if workspace exists
if [ ! -d "$WORKSPACE_PATH" ]; then
    echo "Error: Workspace not found at $WORKSPACE_PATH"
    exit 1
fi

# Stop service
echo "Stopping service..."
sudo systemctl stop "$SERVICE_NAME.service" 2>/dev/null || echo "Service not running"

# Build and update installation (clean build)
echo "Building and installing clean mechanical_press..."
TEMP_INSTALL="/tmp/mechanical_press_install_$(date +%s)"

cd "$WORKSPACE_PATH"
colcon build --packages-select mechanical_press \
    --install-base "$TEMP_INSTALL" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

# Replace service installation
sudo rm -rf "$INSTANCE_DIR/install"
sudo mkdir -p "$INSTANCE_DIR/install"
sudo cp -r "$TEMP_INSTALL"/* "$INSTANCE_DIR/install/"
sudo chown -R emoco:emoco "$INSTANCE_DIR/install"

# Cleanup
rm -rf "$TEMP_INSTALL"

# Restart service
echo "Starting service..."
sudo systemctl start "$SERVICE_NAME.service"

echo ""
echo "✓ Instance '$INSTANCE_NAME' updated successfully!"
echo ""
echo "Check status:"
echo "  sudo systemctl status $SERVICE_NAME.service"
echo ""
echo "View logs:"
echo "  journalctl -u $SERVICE_NAME -f"