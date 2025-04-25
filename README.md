# Mechanical Press Node (ROS 2)

This ROS 2 Python node simulates and controls an industrial press using a CANopen-controlled motor. It is designed to work with both a physical motor and a GUI interface, supporting manual jog controls (up/down), parameterized motion phases (approach, press, return), position tracking, and persistent configuration.

---

## Features

- Manual jog operation with **dead-man's-grip** logic (`manual/up` and `manual/down` topics)
- Parameterized phases: `manual`, `approach`, `press`, `return`
- Position feedback via `position` topic (simulated for now)
- Persistent position markers:
  - `home.position`, `start.position`, `end.position`
- Trigger services:
  - `home`, `approach`, `press`, `return`, `stop`
  - Save current position to named markers
  - Save all parameters to file
- Namespaced support (e.g. `/my_press/manual/up`)
- Parameters saved to YAML, reloadable on startup

---

## Installation

```bash
cd ~/ros2_ws/src
git clone git@github.com:emoco-hub/mechanical_press.git
```

```bash
cd ~/ros2_ws
colcon build --packages-select mechanical_press
. install/setup.bash
```

---

## Launch

```bash
ros2 launch mechanical_press press_node_launch.py namespace:=P1
```

This will load parameters from `default_params.yaml` and start the node under `/P1/`.

---

## Topics

| Topic                      | Type           | Description                               |
|---------------------------|----------------|-------------------------------------------|
| `manual/up`               | `std_msgs/Bool` | Hold to move up (dead-man grip)           |
| `manual/down`             | `std_msgs/Bool` | Hold to move down                         |
| `position`                | `std_msgs/Float64` | Simulated current position               |

---

## Services

| Service                     | Type               | Description                                     |
|----------------------------|--------------------|-------------------------------------------------|
| `save_home`                | `std_srvs/Trigger` | Save current position as `home.position`        |
| `save_start`               | `std_srvs/Trigger` | Save current position as `start.position`       |
| `save_end`                 | `std_srvs/Trigger` | Save current position as `end.position`         |
| `save_params`              | `std_srvs/Trigger` | Save all current parameters to a YAML file      |
| `home`, `approach`, `press`| `std_srvs/Trigger` | Execute motion phase (simulated for now)        |
| `stop`                     | `std_srvs/Trigger` | Stop the motor                                  |

---

## ð  Parameters

Parameters are grouped by motion phase:

```yaml
manual:
  force: 5.0       # 0.1 â 10.0 N
  velocity: 300.0  # 60 â 600 mm/min
approach:
  ...
press:
  ...
return:
  ...
home:
  position: 0.0
start:
  position: 0.0
end:
  position: 0.0
```

To view parameters:

```bash
ros2 param list /P1
ros2 param get /P1 manual.velocity
```

---

## Saving Parameters

Call this service to persist all current parameters to a YAML file:

```bash
ros2 service call /my_press/save_params std_srvs/srv/Trigger
```

By default, the YAML file is written to `/tmp/industrial_press_params.yaml`.

---

## Simulated Motion

The `manual/up` and `manual/down` topics simulate motion by updating `current_position` based on `manual.velocity`. This is reflected in the `position` topic, which publishes at 20 Hz.
