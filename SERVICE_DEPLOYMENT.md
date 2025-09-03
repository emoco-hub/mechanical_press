# Service Deployment

This section describes how to deploy the mechanical press node as a systemd service using `emoco-app-runtime`.

## Prerequisites

1. Install the emoco-app-runtime package:
   ```bash
   # The runtime package is typically pre-installed on Emoco servers
   # If not available, install from our package repository:
   sudo apt update && sudo apt install emoco-app-runtime
   ```

2. Ensure your ROS 2 workspace is built:
   ```bash
   cd ~/ros2_ws
   # Replace 'jazzy' with your ROS 2 distribution (humble, iron, jazzy, etc.)
   source /opt/ros/jazzy/setup.bash
   colcon build --packages-select mechanical_press
   ```

## Development Workflow (Snapshots)

Use snapshots for rapid development and testing without version bumps.

### 1. Create a Development Snapshot

```bash
# Build workspace to a temporary location
cd ~/ros2_ws
# Replace 'jazzy' with your ROS 2 distribution
source /opt/ros/jazzy/setup.bash
colcon build --merge-install --packages-up-to mechanical_press \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  --install-base /tmp/mechanical-press-dev

# Create and install snapshot
sudo emoco-appctl snapshot \
  --app mechanical-press \
  --install-base /tmp/mechanical-press-dev \
  --ros-distro jazzy \
  --launch-pkg mechanical_press \
  --launch-file mechanical_press.launch.py \
  --tag "dev-$(date +%m%d-%H%M)"
```

### 2. Start the Service

```bash
# Enable and start the service
sudo systemctl enable --now ros2_app@mechanical-press.service

# Check status
sudo systemctl status ros2_app@mechanical-press.service
```

### 3. Monitor Logs

```bash
# Follow logs in real-time
journalctl -u ros2_app@mechanical-press -f

# View recent logs
journalctl -u ros2_app@mechanical-press -n 50

# View logs from specific time
journalctl -u ros2_app@mechanical-press --since "10 minutes ago"
```

### 4. Update Snapshot (Development Iteration)

```bash
# Rebuild after code changes
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash  # Replace with your ROS 2 distribution
colcon build --merge-install --packages-up-to mechanical_press \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  --install-base /tmp/mechanical-press-dev

# Create new snapshot (automatically updates current symlink)
sudo emoco-appctl snapshot \
  --app mechanical-press \
  --install-base /tmp/mechanical-press-dev \
  --ros-distro jazzy \
  --launch-pkg mechanical_press \
  --launch-file mechanical_press.launch.py \
  --tag "bugfix-$(date +%m%d-%H%M)"

# Restart service to pick up changes
sudo systemctl restart ros2_app@mechanical-press.service
```

## Production Deployment

For production deployments, create versioned packages instead of snapshots.

### 1. Build Versioned Package

```bash
# Build workspace to versioned location
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash  # Replace with your ROS 2 distribution
VER=1.0.0~ros-jazzy
INSTALL_BASE=/tmp/mechanical-press-$VER
colcon build --merge-install --packages-up-to mechanical_press \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  --install-base "$INSTALL_BASE"

# Create versioned .deb package
ARCH=$(dpkg --print-architecture)
sudo emoco-appctl package \
  --app mechanical-press \
  --version "$VER" \
  --arch "$ARCH" \
  --install-base "$INSTALL_BASE" \
  --ros-distro jazzy \
  --launch-pkg mechanical_press \
  --launch-file mechanical_press.launch.py

# Install the package
sudo dpkg -i ./mechanical-press_${VER}_${ARCH}.deb
```

### 2. Start Production Service

```bash
sudo systemctl enable --now ros2_app@mechanical-press.service
journalctl -u ros2_app@mechanical-press -f
```

## Service Management

### Basic Commands

```bash
# Start/stop/restart
sudo systemctl start ros2_app@mechanical-press.service
sudo systemctl stop ros2_app@mechanical-press.service
sudo systemctl restart ros2_app@mechanical-press.service

# Enable/disable auto-start at boot
sudo systemctl enable ros2_app@mechanical-press.service
sudo systemctl disable ros2_app@mechanical-press.service

# Check status
sudo systemctl status ros2_app@mechanical-press.service
```

### Configuration

The service configuration is stored in:
- **Environment**: `/etc/rosapps/mechanical-press.env`
- **Parameters**: `/etc/rosapps/mechanical-press/params.yaml`
- **Logs**: `/var/log/rosapps-mechanical-press/`
- **State**: `/var/lib/rosapps-mechanical-press/`

Example environment file (`/etc/rosapps/mechanical-press.env`):
```bash
APP_NAME=mechanical-press
ROS_DISTRO=jazzy
INSTALL_BASE=/opt/rosapps/mechanical-press/current
LAUNCH_PKG=mechanical_press
LAUNCH_FILE=mechanical_press.launch.py
PARAM_FILE=/etc/rosapps/mechanical-press/params.yaml
NAMESPACE=/mechanical_press
ROS_LOCALHOST_ONLY=1
```

### Troubleshooting

```bash
# Check if files exist
ls -la /opt/rosapps/mechanical-press/current/
ls -la /etc/rosapps/mechanical-press*

# Check service environment
sudo systemctl show ros2_app@mechanical-press.service --property=Environment

# Test launch manually
sudo -u emoco bash
cd /opt/rosapps/mechanical-press/current
source /opt/ros/jazzy/setup.bash  # Replace with your ROS 2 distribution
source setup.bash
ros2 launch mechanical_press mechanical_press.launch.py \
  param_file:=/etc/rosapps/mechanical-press/params.yaml \
  namespace:=/mechanical_press
```

## Uninstallation

```bash
# Stop and remove service, keep config
sudo emoco-appctl uninstall --app mechanical-press

# Remove service and config
sudo emoco-appctl uninstall --app mechanical-press --purge

# Complete removal (service, config, logs, state)
sudo emoco-appctl uninstall --app mechanical-press --purge --zap
```

## Distribution

The versioned `.deb` packages can be distributed to other machines:

```bash
# Copy to target machine
scp mechanical-press_1.0.0~ros-jazzy_amd64.deb user@target-machine:

# Install on target
ssh user@target-machine
sudo dpkg -i mechanical-press_1.0.0~ros-jazzy_amd64.deb
sudo systemctl enable --now ros2_app@mechanical-press.service
```