# Standard ROS Bloom Packaging

This document explains how to use the standard ROS bloom packaging workflow instead of the simplified snapshot approach used in the main README.

**When to use this:** When you need standard ROS package distribution, dependency management, or integration with ROS build infrastructure.

---

## Prerequisites

Install the required bloom packaging tools:

```bash
sudo apt install python3-bloom fakeroot debhelper
```

---

## Package Configuration

The `package.xml` must be properly configured for bloom. The mechanical_press project includes the necessary metadata:

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
  
  <!-- Test dependencies -->
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

---

## Packaging Workflow

### 1. Generate Debian Packaging Files

```bash
# Generate bloom packaging files
bloom-generate rosdebian \
    --os-name ubuntu \
    --os-version jammy \
    --ros-distro humble

# Or for ROS Jazzy with Noble
bloom-generate rosdebian \
    --os-name ubuntu \
    --os-version noble \
    --ros-distro jazzy
```

### 2. Build the Package

```bash
# Build the Debian package
fakeroot debian/rules binary
```

### 3. Find and Install Package

```bash
# Package will be created in parent directory
ls ../ros-humble-mechanical-press_*.deb

# Install the package
sudo dpkg -i ../ros-humble-mechanical-press_1.0.0_amd64.deb
```

---

## Integration with Instance Creation

The `create-instance.sh` script can detect and use bloom-generated packages:

```bash
# Check if bloom package is installed
dpkg -l ros-humble-mechanical-press

# Create instance (will use the bloom package if installed)
./scripts/create-instance.sh \
  --name production-press \
  --namespace /production/press1 \
  --config config/examples/factory_line1_press1.yaml
```

---

## Standard ROS Installation Paths

Bloom packages install to standard ROS locations:

| Component | Installation Path |
|-----------|------------------|
| **Python modules** | `/opt/ros/humble/local/lib/python3.x/site-packages/mechanical_press/` |
| **Entry points** | `/opt/ros/humble/lib/mechanical_press/` |
| **Launch files** | `/opt/ros/humble/share/mechanical_press/launch/` |
| **Config files** | `/opt/ros/humble/share/mechanical_press/config/` |
| **Package metadata** | `/opt/ros/humble/share/mechanical_press/package.xml` |

---

## Package Distribution

### Manual Distribution

```bash
# Build package
bloom-generate rosdebian --os-name ubuntu --os-version jammy --ros-distro humble
fakeroot debian/rules binary

# Transfer to production servers
scp ../ros-humble-mechanical-press_*.deb production-server:

# Install on production servers
ssh production-server
sudo dpkg -i ros-humble-mechanical-press_*.deb
```

### Package Repository Integration

For large-scale deployment, integrate with package repositories:

```bash
# Upload to internal package repository
curl -u user:pass -T ../ros-humble-mechanical-press_*.deb \
  https://packages.company.com/ros/humble/

# On target servers, install from repository
sudo apt update
sudo apt install ros-humble-mechanical-press=1.0.0
```

---

## Comparison: Bloom vs Snapshot

| Aspect | Bloom Packaging | Snapshot Approach |
|--------|----------------|------------------|
| **Complexity** | High (bloom, rosdep, debian tools) | Low (just colcon build) |
| **Standards** | ROS standard workflow | Custom, transparent |
| **Dependencies** | Automatic resolution | Manual management |
| **Distribution** | APT repository ready | Simple file transfer |
| **Debugging** | Multiple abstraction layers | Direct, visible process |
| **Learning Curve** | Requires bloom knowledge | Familiar build process |

---

## When to Use Each Approach

### Use Bloom Packaging When:
- Distributing packages to other ROS developers
- Integration with ROS build farm or CI systems
- Need automatic dependency resolution
- Publishing to ROS package repositories
- Following enterprise ROS development practices

### Use Snapshot Approach When:
- Educational/learning projects
- Internal deployment only
- Want transparency in packaging process
- Rapid development iteration
- Simple "run as service" deployment needs

---

## Troubleshooting

### Common Issues

**rosdep resolution errors:**
```bash
# Update rosdep database
rosdep update

# Make sure ROS repositories are configured
sudo apt update
sudo apt install ros-humble-ament-cmake
```

**Missing build tools:**
```bash
# Install all required tools
sudo apt install python3-bloom fakeroot debhelper dh-python
```

**Ubuntu version compatibility:**
```bash
# Use correct Ubuntu codename for ROS distro
# Humble → jammy (22.04)
# Jazzy → noble (24.04)
bloom-generate rosdebian --os-name ubuntu --os-version jammy --ros-distro humble
```

---

## Integration with Main Workflow

The bloom packaging approach can complement the snapshot workflow:

1. **Development**: Use snapshot approach for rapid iteration
2. **Testing**: Create bloom packages for integration testing
3. **Production**: Use bloom packages for standardized deployment

Both approaches work with the same `create-instance.sh` script - it automatically detects which type of package is installed and adapts accordingly.