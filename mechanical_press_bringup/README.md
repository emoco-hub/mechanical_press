# Mechanical Press Bringup

This package contains launch files and configuration for the mechanical press system.

## Quick Start

Launch with default configuration:

```bash
ros2 launch mechanical_press_bringup mechanical_press.launch.py
```

Launch with custom namespace and config:

```bash
ros2 launch mechanical_press_bringup mechanical_press.launch.py namespace:=my_press param_file:=/path/to/config.yaml
```

## Configuration Files

- `config/default_params.yaml` - Default parameters for any namespace
- `config/example_params.yaml` - Example with specific namespace

## Launch Arguments

- `namespace` (default: "press1") - ROS namespace for the node
- `param_file` (default: config/default_params.yaml) - Path to parameter file