#!/bin/bash
set -e

# Create deployment snapshot for mechanical press
# Usage: ./scripts/package.sh [version] [ros_distro]

VERSION=${1:-"1.0.0"}
ROS_DISTRO=${2:-"jazzy"}
APP_NAME="mechanical-press"
LAUNCH_PKG="mechanical_press"
LAUNCH_FILE="mechanical_press.launch.py"

echo "Creating deployment package: $APP_NAME version $VERSION for ROS $ROS_DISTRO"
echo "Using simple snapshot approach for transparency"

# Build to temporary location
echo "Building workspace..."
INSTALL_BASE="/tmp/${APP_NAME}-${VERSION}"
source "/opt/ros/$ROS_DISTRO/setup.bash"
colcon build --merge-install --packages-up-to "$LAUNCH_PKG" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  --install-base "$INSTALL_BASE"

# Create package structure  
echo "Creating package structure..."
PKG_DIR="/tmp/${APP_NAME}-pkg"
rm -rf "$PKG_DIR"
mkdir -p "$PKG_DIR/opt/rosapps/${APP_NAME}/releases/${VERSION}"
mkdir -p "$PKG_DIR/DEBIAN"

# Copy application files
cp -r "$INSTALL_BASE"/* "$PKG_DIR/opt/rosapps/${APP_NAME}/releases/${VERSION}/"

# Create runtime script
echo "Creating runtime script..."
mkdir -p "$PKG_DIR/opt/rosapps/${APP_NAME}/releases/${VERSION}/bin"
cat > "$PKG_DIR/opt/rosapps/${APP_NAME}/releases/${VERSION}/bin/run.sh" << EOF
#!/usr/bin/env bash
set -eo pipefail
: "\${ROS_DISTRO:=$ROS_DISTRO}"
: "\${INSTALL_BASE:=/opt/rosapps/${APP_NAME}/current}"
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
chmod +x "$PKG_DIR/opt/rosapps/${APP_NAME}/releases/${VERSION}/bin/run.sh"

# Create control file
echo "Creating package metadata..."
ARCH=$(dpkg --print-architecture)
cat > "$PKG_DIR/DEBIAN/control" << EOF
Package: $APP_NAME
Version: $VERSION
Section: utils
Priority: optional
Architecture: $ARCH
Maintainer: Emoco Labs <ops@example.com>
Description: Mechanical Press ROS 2 Application
 ROS 2 application for controlling mechanical press operations.
 Built for ROS $ROS_DISTRO distribution.
EOF

# Create post-installation script
cat > "$PKG_DIR/DEBIAN/postinst" << 'EOF'
#!/bin/sh
set -e

APP="mechanical-press"
VERSION="$VERSION"
BASE="/opt/rosapps/$APP"
CUR="$BASE/current"
REL="$BASE/releases/$VERSION"
ETC="/etc/rosapps/$APP"
ENV="/etc/rosapps/$APP.env"

# Create user
useradd -r -s /bin/false emoco 2>/dev/null || true

# Create directories
install -d -m 0755 -o emoco -g emoco "/var/lib/rosapps-$APP" "/var/log/rosapps-$APP"
mkdir -p "$ETC"

# Create environment file if it doesn't exist
if [ ! -f "$ENV" ]; then
  cat > "$ENV" << ENVEOF
APP_NAME=$APP
ROS_DISTRO=$ROS_DISTRO
INSTALL_BASE=$CUR
LAUNCH_PKG=$LAUNCH_PKG
LAUNCH_FILE=$LAUNCH_FILE
PARAM_FILE=/etc/rosapps/$APP/params.yaml
NAMESPACE=/${APP//-/_}
ROS_LOCALHOST_ONLY=1
ENVEOF
fi

# Create default params if they don't exist
if [ ! -f "$ETC/params.yaml" ]; then
  cat > "$ETC/params.yaml" << 'YAML'
/**:
  ros__parameters:
    friendly_name: "mechanical-press"
YAML
fi

# Update current symlink
ln -sfn "$REL" "$CUR"

# Set ownership
chown -R emoco:emoco "$BASE"

# Reload systemd
command -v systemctl >/dev/null 2>&1 && systemctl daemon-reload || true
exit 0
EOF

# Fix version in postinst script
sed -i "s/\$VERSION/$VERSION/g" "$PKG_DIR/DEBIAN/postinst"
sed -i "s/\$ROS_DISTRO/$ROS_DISTRO/g" "$PKG_DIR/DEBIAN/postinst"
sed -i "s/\$LAUNCH_PKG/$LAUNCH_PKG/g" "$PKG_DIR/DEBIAN/postinst"
sed -i "s/\$LAUNCH_FILE/$LAUNCH_FILE/g" "$PKG_DIR/DEBIAN/postinst"
sed -i "s/\$APP/$APP_NAME/g" "$PKG_DIR/DEBIAN/postinst"
chmod +x "$PKG_DIR/DEBIAN/postinst"

# Build package
echo "Building .deb package..."
DEB_FILE="${APP_NAME}_${VERSION}_${ARCH}.deb"
dpkg-deb --build "$PKG_DIR" "./$DEB_FILE"

# Cleanup
rm -rf "$PKG_DIR"
rm -rf "$INSTALL_BASE"

echo ""
echo "✓ Package created: $DEB_FILE"
echo ""
echo "To install:"
echo "  sudo dpkg -i $DEB_FILE"
echo ""
echo "To create instance:"
echo "  ./scripts/create-instance.sh --name my-press --namespace /my_press --config config/examples/development.yaml"