# Packaging for Distribution

This guide covers creating distributable packages for deploying the mechanical press system to multiple servers or sharing with other teams. While the main README focuses on local deployment as services, this covers building packages for wider distribution.

**When to use this:** Multi-server deployments, version control, team sharing, or integration with package management systems.

---

## Prerequisites

```bash
sudo apt install python3-bloom fakeroot debhelper

# Initialize rosdep database (required for bloom)
sudo rosdep init  # Only needed once per system
rosdep update
```

---

## Creating Distribution Packages

### 1. Package Configuration

The `package.xml` is configured for packaging with proper metadata:

```xml
<package format="3">
  <name>mechanical_press</name>
  <version>1.0.0</version>
  <description>Industrial mechanical press control system for ROS 2...</description>
  
  <maintainer email="ops@example.com">Emoco Labs</maintainer>
  <license>Apache-2.0</license>
  
  <!-- URLs for package metadata -->
  <url type="website">https://github.com/example/mechanical_press</url>
  <url type="bugtracker">https://github.com/example/mechanical_press/issues</url>
  <url type="repository">https://github.com/example/mechanical_press</url>
  
  <!-- Runtime dependencies -->
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>std_srvs</depend>
  <exec_depend>python3-yaml</exec_depend>
</package>
```

### 2. Generate Distribution Package

```bash
# Generate packaging files for target platform
bloom-generate rosdebian \
    --os-name ubuntu \
    --os-version noble \
    --ros-distro jazzy

# Build the distribution package
fakeroot debian/rules binary

# Find the created package
ls ../ros-jazzy-mechanical-press_*.deb
```

**Note for Yocto Linux targets**: The Ubuntu Noble settings work for most Yocto-based systems since they typically use compatible glibc versions and package formats. The generated .deb packages should install on Yocto systems with dpkg support.

### 3. Install and Test Package

```bash
# Install the package
sudo dpkg -i ../ros-jazzy-mechanical-press_1.0.0_amd64.deb

# Verify installation
dpkg -l ros-jazzy-mechanical-press

# Test the package
ros2 pkg list | grep mechanical_press
ros2 launch mechanical_press mechanical_press.launch.py
```

---

## Multi-Server Distribution

### Transfer and Install on Remote Servers

```bash
# Transfer package to production servers
scp ../ros-jazzy-mechanical-press_*.deb production-server1:
scp ../ros-jazzy-mechanical-press_*.deb production-server2:

# Install on each server
ssh production-server1 "sudo dpkg -i ros-jazzy-mechanical-press_*.deb"
ssh production-server2 "sudo dpkg -i ros-jazzy-mechanical-press_*.deb"
```

### Create Service Instances on Remote Servers

After package installation, create service instances:

```bash
# On each production server
ssh production-server1
# Package installation provides ROS package, but service creation requires workspace approach
# Copy the create-instance.sh script or build from workspace for service deployment
```

---

## Package Repository Integration

### Internal Package Repository

For organizations with many servers:

```bash
# 1. Build package
bloom-generate rosdebian --os-name ubuntu --os-version noble --ros-distro jazzy
fakeroot debian/rules binary

# 2. Upload to internal package repository
curl -u user:pass -T ../ros-jazzy-mechanical-press_*.deb \
  https://packages.yourcompany.com/ros/jazzy/

# 3. On target servers, install from repository
sudo apt update
sudo apt install ros-jazzy-mechanical-press=1.0.0
```

### Version Management

```bash
# Build specific versions
# Update version in package.xml first, then:
bloom-generate rosdebian --os-name ubuntu --os-version noble --ros-distro jazzy
fakeroot debian/rules binary

# Install specific version
sudo apt install ros-jazzy-mechanical-press=1.2.0

# List available versions
apt-cache policy ros-jazzy-mechanical-press
```

---

## Installation Paths

Distribution packages install to standard ROS locations:

| Component | Installation Path |
|-----------|------------------|
| **Python modules** | `/opt/ros/jazzy/local/lib/python3.x/site-packages/mechanical_press/` |
| **Entry points** | `/opt/ros/jazzy/lib/mechanical_press/` |
| **Launch files** | `/opt/ros/jazzy/share/mechanical_press/launch/` |
| **Config files** | `/opt/ros/jazzy/share/mechanical_press/config/` |
| **Package metadata** | `/opt/ros/jazzy/share/mechanical_press/package.xml` |

---

## Integration with Service Deployment

The service deployment approach documented in the main README uses workspace builds. Package-based distribution provides the ROS package files but requires different service deployment methods.

---

## Troubleshooting

**Dependency resolution errors:**
```bash
# Update package databases
rosdep update
sudo apt update

# Install missing ROS dependencies
sudo apt install ros-jazzy-ament-cmake
```

**Build tool errors:**
```bash
# Install all required packaging tools
sudo apt install python3-bloom fakeroot debhelper dh-python
```

**Rosdep database errors:**
```bash
# Error: "rosdep database does not have any sources"
sudo rosdep init
rosdep update

# If rosdep init fails with "already exists", just run:
rosdep update
```

---

## Deployment Workflow Integration

**Recommended workflow for teams:**

1. **Development**: Use direct service deployment for rapid iteration
2. **Testing**: Create packages for integration testing across environments
3. **Production**: Use packages for reliable, versioned deployment

**Example development cycle:**
```bash
# 1. Develop with direct deployment (main README approach)
./scripts/create-instance.sh --name dev-press --config config/examples/development.yaml

# 2. Test with standard ROS packages
bloom-generate rosdebian --os-name ubuntu --os-version noble --ros-distro jazzy
fakeroot debian/rules binary
sudo dpkg -i ../ros-jazzy-mechanical-press_*.deb

# 3. Deploy to production servers
scp ../ros-jazzy-mechanical-press_*.deb production-servers:
ssh production-server "sudo dpkg -i ros-jazzy-mechanical-press_*.deb"
```

This approach provides different deployment methods for development, testing, and production stages.