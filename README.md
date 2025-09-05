# Mechanical Press Node (ROS 2)

This ROS 2 Python node demonstrates **production-ready deployment patterns** for industrial automation systems. It simulates the control of an industrial press with parameterized motion phases, position tracking, and multi-instance deployment capabilities.

**Educational Purpose**: This project teaches the **binary functionality + multi-instance pattern** - how to package core functionality once and deploy multiple configured instances, matching real-world industrial deployments.

---

## Features

- **Multi-Instance Architecture**: One binary package, multiple configured instances
- **Production-Ready Deployment**: Demonstrates dev-to-production workflows
- **Manual jog operation**: Dead-man's-grip logic (`manual/up` and `manual/down`)
- **Parameterized motion phases**: `manual`, `approach`, `press`, `return`
- **Position tracking**: Simulated position feedback and markers
- **Configuration flexibility**: Environment-specific parameter sets
- **Service-based control**: Trigger motion phases and save positions/parameters
- **Namespace support**: Multiple press instances with isolated communication

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
├── scripts/                    # Deployment automation
│   ├── package.sh             # Build production packages
│   ├── create-instance.sh     # Create configured instances
│   └── dev-snapshot.sh        # Development iteration
└── SERVICE_DEPLOYMENT.md       # Alternative emoco-app-runtime guide
```

---

## Development Setup

### 1. Clone and Build

```bash
cd ~/ros2_ws/src
git clone <repository-url> mechanical_press
cd ~/ros2_ws
colcon build --packages-select mechanical_press
source install/setup.bash
```

### 2. Development Testing

Test with different configurations during development:

```bash
# Development configuration (safe settings)
ros2 launch mechanical_press mechanical_press.launch.py \
  namespace:=/dev \
  param_file:=config/examples/development.yaml \
  instance_name:=dev_press
```

### 3. Run as Service

Deploy your mechanical press as a system service:

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
sudo systemctl restart mechanical-press-my-press.service
```

**That's it!** This creates a reliable systemd service that:
- Automatically starts on boot
- Restarts on failure  
- Has isolated logs and configuration
- Uses your development build directly

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
sudo systemctl restart mechanical-press-my-press.service
```

---

## What Instance Creation Does

The `create-instance.sh` script creates **isolated system services** for each mechanical press:

- **Service File**: Creates a Linux service that starts/stops your press automatically
- **Configuration**: Your YAML settings are copied to a dedicated location  
- **Isolated Logs**: Each press instance has its own log files
- **Clean Installation**: Uses a clean build isolated from your development workspace

**Benefits**: Services auto-restart on failure, survive system reboots, and can be managed with standard Linux tools.

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

### Testing Different Configurations

```bash
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

## Educational Benefits

This project demonstrates:

1. **Binary vs. Instance Separation**: Core functionality packaged independently from configuration
2. **Production Deployment Patterns**: How industrial systems are actually deployed at scale  
3. **Configuration Management**: Environment-specific parameter sets (dev/production/maintenance)
4. **Service Architecture**: Independent instance lifecycle management
5. **ROS Best Practices**: Namespace usage, parameter organization, service-based control

The multi-instance pattern matches real industrial deployments where:
- One tested software version serves multiple physical devices
- Each device has specific configuration (force limits, positions, safety settings)
- Operations team manages instances independently
- Software updates affect all instances simultaneously

---

## Updates and Changes

### Update Service After Code Changes

```bash
# Development workflow - update running service with latest code
./scripts/dev-snapshot.sh my-press
sudo systemctl restart mechanical-press-my-press.service
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

**When to use:** Multi-server environments, version control requirements, or when you need standard ROS bloom packaging (see `BLOOM_PACKAGING.md`).

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

## License

Apache-2.0