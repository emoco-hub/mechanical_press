# Mechanical Press Node (ROS 2)

This ROS 2 Python node simulates an industrial press with manual jog controls, parameterized motion phases (approach, press, return), and position tracking. It includes scripts for deploying as system services.

The project demonstrates practical deployment patterns for ROS applications in industrial environments.

---

## Features

- **"Run as Service" Deployment**: One-command deployment to reliable systemd services
- **Simple & Transparent**: See exactly what gets deployed and where (no hidden complexity)
- **Multi-Instance Support**: Deploy multiple press instances with different configurations  
- **Manual jog operation**: Dead-man's-grip logic (`manual/up` and `manual/down`)
- **Parameterized motion phases**: `manual`, `approach`, `press`, `return`
- **Position tracking**: Simulated position feedback and markers
- **Configuration flexibility**: Environment-specific parameter sets (dev/production/maintenance)
- **Service-based control**: Trigger motion phases and save positions/parameters

---

## Project Structure

```
mechanical_press/
├── mechanical_press/           # Core functionality
│   └── mechanical_press.py    # Main press control logic
├── launch/
│   └── mechanical_press.launch.py  # Flexible instance launcher
├── config/                     # Instance configurations
│   ├── default.yaml           # Generic base configuration
│   └── examples/              # Production configuration examples
│       ├── development.yaml   # Safe development settings
│       ├── factory_line1_press1.yaml  # Production line settings
│       └── maintenance_press.yaml     # Maintenance/testing settings
├── scripts/                    # Simple deployment tools
│   ├── create-instance.sh     # Deploy as system service  
│   └── dev-snapshot.sh        # Update running services
└── PACKAGING_FOR_DISTRIBUTION.md # Guide for multi-server deployment packaging
```

---

## Development Setup

### 1. Clone and Build

```bash
cd ~/ros2_ws/src
git clone git@github.com:emoco-hub/mechanical_press.git mechanical_press
cd ~/ros2_ws
colcon build --packages-select mechanical_press --symlink-install
source install/setup.bash
```

**Note**: The `--symlink-install` flag creates symbolic links to source files. During development, edit Python files and restart the node to see changes - no rebuild required.

### 2. Quick Dev Mode Test

```bash
# Run directly in development with example configuration
ros2 launch mechanical_press mechanical_press.launch.py \
  namespace:=/dev \
  param_file:=config/examples/development.yaml \
  instance_name:=my-press
```

---

## Service Deployment

Create and manage the project as `systemd` service instances on the local server:

```bash
# Create and run a service instance with smart defaults
./scripts/create-instance.sh \
  --name my-press \
  --namespace /my_press \
  --config config/examples/development.yaml

# Start the service
sudo systemctl enable --now mechanical-press-my-press.service

# Monitor logs
journalctl -u mechanical-press-my-press -f

# Update after code changes (development workflow)
./scripts/dev-snapshot.sh my-press
```

**That's it!** This creates a reliable systemd service that:
- Automatically starts on boot
- Restarts on failure  
- Has isolated logs and configuration
- Runs from an isolated copy of your current development version

### Basic Service Management

```bash
# Check service status
sudo systemctl status mechanical-press-my-press.service

# Start/stop the service
sudo systemctl start mechanical-press-my-press.service
sudo systemctl stop mechanical-press-my-press.service

# View logs in real-time
journalctl -u mechanical-press-my-press -f

# Update after code changes
./scripts/dev-snapshot.sh my-press
```

### Stopping and Removing a Service Instance

To safely stop and remove a service instance:

```bash
# Stop the service
sudo systemctl stop mechanical-press-my-press.service

# Disable auto-start at boot
sudo systemctl disable mechanical-press-my-press.service

# Remove the service file
sudo rm /etc/systemd/system/mechanical-press-my-press.service

# Reload systemd to forget about the service
sudo systemctl daemon-reload

# Clean up instance files (optional - keeps logs and config for reference)
sudo rm -rf /opt/rosapps/mechanical-press-instances/my-press
sudo rm -rf /etc/rosapps/mechanical-press-instances/my-press
sudo rm -rf /var/lib/rosapps-mechanical-press-my-press

# Remove logs (optional - you may want to keep these for troubleshooting)  
sudo rm -rf /var/log/rosapps-mechanical-press-my-press
```

**Note**: The logs and configuration directories contain useful debugging information. Consider keeping them until you're sure you won't need to reference them.

---

## What Instance Creation Does

The `create-instance.sh` script creates isolated system services for each mechanical press instance:

- **Service File**: Creates a systemd service that starts/stops automatically
- **Configuration**: YAML settings are copied to `/etc/rosapps/mechanical-press-instances/INSTANCE_NAME/`
- **Installation**: Clean build installed to `/opt/rosapps/mechanical-press-instances/INSTANCE_NAME/`
- **Logs**: Instance logs stored in `/var/log/rosapps-mechanical-press-INSTANCE_NAME/`
- **State**: Runtime state in `/var/lib/rosapps-mechanical-press-INSTANCE_NAME/`

Each instance runs independently with its own configuration, logs, and installation directory.

---

## Testing Examples

### Basic Operation

```bash
# Test manual controls (hold for dead-man's grip)
ros2 topic pub /dev/manual/up std_msgs/Bool "data: true"
ros2 topic pub /dev/manual/down std_msgs/Bool "data: true"

# Monitor position
ros2 topic echo /dev/position

# Execute motion sequence
ros2 service call /dev/home std_srvs/srv/Trigger
ros2 service call /dev/approach std_srvs/srv/Trigger  
ros2 service call /dev/press std_srvs/srv/Trigger
ros2 service call /dev/return std_srvs/srv/Trigger
```

### Multi-Instance Testing

```bash
# Test factory press instance
ros2 topic pub /factory/line1/press1/manual/up std_msgs/Bool "data: true"
ros2 service call /factory/line1/press1/save_home std_srvs/srv/Trigger

# Test maintenance press instance
ros2 topic echo /maintenance/test_station/position
ros2 service call /maintenance/test_station/stop std_srvs/srv/Trigger
```

### Parameter Management

```bash
# View current parameters
ros2 param list /dev/mechanical_press
ros2 param get /dev/mechanical_press manual.force

# Modify parameters at runtime
ros2 param set /dev/mechanical_press manual.velocity 150.0

# Save parameters to file
ros2 service call /dev/save_params std_srvs/srv/Trigger
```

---

## ROS Interface

### Topics

| Topic                      | Type           | Description                               |
|---------------------------|----------------|-------------------------------------------|
| `manual/up`               | `std_msgs/Bool` | Hold to move up (dead-man grip)           |
| `manual/down`             | `std_msgs/Bool` | Hold to move down                         |
| `position`                | `std_msgs/Float64` | Current position feedback               |

### Services

| Service                     | Type               | Description                                     |
|----------------------------|--------------------|-------------------------------------------------|
| `save_home`                | `std_srvs/Trigger` | Save current position as home marker        |
| `save_start`               | `std_srvs/Trigger` | Save current position as start marker       |
| `save_end`                 | `std_srvs/Trigger` | Save current position as end marker         |
| `save_params`              | `std_srvs/Trigger` | Save all parameters to file      |
| `home`, `approach`, `press`| `std_srvs/Trigger` | Execute motion phases        |
| `stop`                     | `std_srvs/Trigger` | Emergency stop                                  |

### Parameters

Parameters are organized by motion phase:

```yaml
manual:         # Manual jog operation
  force: 5.0       # Force limit (N)
  velocity: 300.0  # Speed (mm/min)
approach:       # Automated approach phase  
  force: 3.0
  velocity: 200.0
press:          # Pressing operation
  force: 10.0
  velocity: 100.0
return:         # Return to start
  force: 4.0
  velocity: 400.0
home:           # Position markers
  position: 0.0
start:
  position: 0.0
end:
  position: 0.0
```

---

## Configuration Examples

### Development Configuration
```yaml
# config/examples/development.yaml
/dev/mechanical_press:
  ros__parameters:
    instance_name: "development"
    friendly_name: "Development Press"
    manual:
      force: 3.0      # Safe for testing
      velocity: 200.0
    log_level: "INFO"  # Detailed logging
```

### Production Configuration
```yaml
# config/examples/factory_line1_press1.yaml
/factory/line1/press1/mechanical_press:
  ros__parameters:
    instance_name: "factory_line1_press1"
    friendly_name: "Factory Line 1 - Press 1"
    manual:
      force: 8.0      # Production force
      velocity: 400.0 # Fast operations
    press:
      force: 25.0     # Heavy-duty pressing
    log_level: "WARN" # Minimal logging
```

### Maintenance Configuration
```yaml
# config/examples/maintenance_press.yaml
/maintenance/test_station/mechanical_press:
  ros__parameters:
    instance_name: "maintenance_press"
    friendly_name: "Maintenance Test Station"
    manual:
      force: 2.0      # Ultra-safe
      velocity: 100.0 # Very slow
    log_level: "DEBUG" # Full diagnostics
```

### Usig different Configurations

Test different configurations for different run modes:

```bash
# Test development configuration
ros2 launch mechanical_press mechanical_press.launch.py \
  namespace:=/test \
  param_file:=config/examples/development.yaml \
  instance_name:=test_development

# Test production configuration  
ros2 launch mechanical_press mechanical_press.launch.py \
  namespace:=/test \
  param_file:=config/examples/factory_line1_press1.yaml \
  instance_name:=test_production

# Test maintenance configuration
ros2 launch mechanical_press mechanical_press.launch.py \
  namespace:=/test \
  param_file:=config/examples/maintenance_press.yaml \
  instance_name:=test_maintenance
```

---

## Updates and Changes

### Update Service After Code Changes

```bash
# Development workflow - update running service with latest code
./scripts/dev-snapshot.sh my-press
# (automatically stops, rebuilds, updates, and restarts the service)
```

### Update Instance Configuration

```bash
# Modify instance-specific settings
sudo nano /etc/rosapps/mechanical-press-instances/my-press/params.yaml

# Restart to apply changes
sudo systemctl restart mechanical-press-my-press.service
```

### Create Additional Instances

```bash
# Add more instances for different robots/configurations
./scripts/create-instance.sh \
  --name robot2 \
  --namespace /robot2 \
  --config config/examples/maintenance_press.yaml
```

---

## Cleanup

### Remove Specific Instance

```bash
# Stop and remove instance
sudo systemctl stop line1-press1.service
sudo systemctl disable line1-press1.service
sudo rm /etc/systemd/system/line1-press1.service
sudo systemctl daemon-reload

# Clean up instance files
sudo rm -rf /etc/rosapps/mechanical-press-instances/line1-press1
sudo rm -rf /var/lib/rosapps-mechanical-press-line1-press1  
sudo rm -rf /var/log/rosapps-mechanical-press-line1-press1
```

### Remove Everything

```bash
# Stop all instances
sudo systemctl stop mechanical-press-*

# Remove core package
sudo dpkg -r mechanical-press

# Clean up all files
sudo rm -rf /etc/rosapps/mechanical-press-instances
sudo rm -rf /var/lib/rosapps-mechanical-press-*
sudo rm -rf /var/log/rosapps-mechanical-press-*
sudo rm /etc/systemd/system/mechanical-press-*.service
sudo systemctl daemon-reload
```

---

## Advanced: Package-Based Deployment

For environments that need versioned packages or multi-server deployment:

```bash
# 1. Create a deployment package (for version control)
./scripts/package.sh 1.2.0

# 2. Install the package
sudo dpkg -i mechanical-press_1.2.0_amd64.deb

# 3. Create instances (will use the package instead of building from source)
./scripts/create-instance.sh \
  --name production-press \
  --namespace /production/press1 \
  --config config/examples/factory_line1_press1.yaml

# 4. For multi-server deployment, transfer packages
scp mechanical-press_1.2.0_amd64.deb production-server:
ssh production-server "sudo dpkg -i mechanical-press_1.2.0_amd64.deb"
```

**When to use:** Multi-server environments, version control requirements, or when you need distributable packages (see `PACKAGING_FOR_DISTRIBUTION.md`).

---

## License

Apache-2.0