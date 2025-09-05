#!/bin/bash
set -e

# Create development snapshot for rapid iteration
# Usage: ./scripts/dev-snapshot.sh [tag]

TAG=${1:-"dev-$(date +%m%d-%H%M)"}
APP_NAME="mechanical-press"
LAUNCH_PKG="mechanical_press"
LAUNCH_FILE="mechanical_press.launch.py"
ROS_DISTRO=${ROS_DISTRO:-"jazzy"}

echo "Creating development snapshot: $TAG"

# Check if directories exist, create if needed
if [ ! -d "/opt/rosapps/$APP_NAME" ]; then
    echo "Setting up directory structure..."
    sudo mkdir -p "/opt/rosapps/$APP_NAME"/{current,releases}
    sudo mkdir -p "/etc/rosapps/$APP_NAME"
    sudo mkdir -p "/var/lib/rosapps-$APP_NAME"
    sudo mkdir -p "/var/log/rosapps-$APP_NAME"
    
    # Create emoco user if needed
    sudo useradd -r -s /bin/false emoco 2>/dev/null || true
    
    # Set ownership
    sudo chown -R emoco:emoco "/opt/rosapps/$APP_NAME"
    sudo chown -R emoco:emoco "/var/lib/rosapps-$APP_NAME" 
    sudo chown -R emoco:emoco "/var/log/rosapps-$APP_NAME"
fi

# Build to temporary location
echo "Building workspace..."
INSTALL_BASE="/tmp/${APP_NAME}-dev"
source "/opt/ros/$ROS_DISTRO/setup.bash"
colcon build --merge-install --packages-up-to "$LAUNCH_PKG" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  --install-base "$INSTALL_BASE"

# Create release directory
RELEASE_DIR="/opt/rosapps/$APP_NAME/releases/snapshot-$TAG"
echo "Installing to: $RELEASE_DIR"
sudo mkdir -p "$RELEASE_DIR"
sudo cp -r "$INSTALL_BASE"/* "$RELEASE_DIR/"

# Create runtime script
sudo mkdir -p "$RELEASE_DIR/bin"
sudo tee "$RELEASE_DIR/bin/run.sh" > /dev/null << EOF
#!/usr/bin/env bash
set -eo pipefail
: "\${ROS_DISTRO:=$ROS_DISTRO}"
: "\${INSTALL_BASE:=/opt/rosapps/$APP_NAME/current}"
: "\${LAUNCH_PKG:=$LAUNCH_PKG}"
: "\${LAUNCH_FILE:=$LAUNCH_FILE}"
: "\${PARAM_FILE:=}"
: "\${NAMESPACE:=}"
: "\${EXTRA_ARGS:=}"
source "/opt/ros/\${ROS_DISTRO}/setup.bash"
source "\${INSTALL_BASE}/setup.bash"
if [[ -n "\${NAMESPACE:-}" ]]; then
  NAMESPACE="\${NAMESPACE//-/_}"
fi
exec ros2 launch "\${LAUNCH_PKG}" "\${LAUNCH_FILE}" \
  \${PARAM_FILE:+ "param_file:=\${PARAM_FILE}"} \
  \${NAMESPACE:+ "namespace:=\${NAMESPACE}"} \
  \${EXTRA_ARGS}
EOF
sudo chmod +x "$RELEASE_DIR/bin/run.sh"
sudo chown -R emoco:emoco "$RELEASE_DIR"

# Update current symlink
sudo ln -sfn "$RELEASE_DIR" "/opt/rosapps/$APP_NAME/current"

# Create/update configuration files if they don't exist
if [ ! -f "/etc/rosapps/$APP_NAME.env" ]; then
    echo "Creating environment configuration..."
    sudo tee "/etc/rosapps/$APP_NAME.env" > /dev/null << EOF
APP_NAME=$APP_NAME
ROS_DISTRO=$ROS_DISTRO
INSTALL_BASE=/opt/rosapps/$APP_NAME/current
LAUNCH_PKG=$LAUNCH_PKG
LAUNCH_FILE=$LAUNCH_FILE
PARAM_FILE=/etc/rosapps/$APP_NAME/params.yaml
NAMESPACE=/${APP_NAME//-/_}
ROS_LOCALHOST_ONLY=1
EOF
fi

if [ ! -f "/etc/rosapps/$APP_NAME/params.yaml" ]; then
    echo "Creating default parameters..."
    sudo tee "/etc/rosapps/$APP_NAME/params.yaml" > /dev/null << 'EOF'
/**:
  ros__parameters:
    friendly_name: "mechanical-press"
EOF
fi

# Clean up temporary build
rm -rf "$INSTALL_BASE"

echo "Snapshot created: $TAG"
echo "Current symlink points to: $(readlink /opt/rosapps/$APP_NAME/current)"
echo ""
echo "To restart service (if running):"
echo "  sudo systemctl restart ros2_app@$APP_NAME.service"
echo ""
echo "To create service (if not exists):"
echo "  ./scripts/create-service.sh"