# Piper ROS 2 Control Package 

This package provides a simple and practical ROS 2 interface for controlling the Piper robot arm.
It includes controller nodes, launch configurations, and demo scripts to help you quickly start interacting with the robot.

This package is designed to be used together with the official
[AgileX Piper ROS driver](https://github.com/agilexrobotics/piper_ros).

---

## Package Structure

```
piper_ros_control/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── piper_ros_control
└── piper_ros_control/
    ├── nodes/
    │   ├── piper_controller.py
    │   └── piper_ctrl_single_custom_node.py
    ├── launch/
    │   ├── start_single_piper.launch.py
    │   └── start_dual_piper.launch.py
    └── demos/
        └── keyboard_control.py
```

---

## Requirements

* Ubuntu 22.04
* ROS 2 Humble
* [piper_ros (AgileX driver)](https://github.com/agilexrobotics/piper_ros)

---

## Usage

### 1. Build

```bash
colcon build
source install/setup.bash
```

---

### 2. Launch the Robot

Before launching, make sure the CAN interface is activated:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
bash /path/to/piper_ros/piper_ros/can_activate.sh can-piper 1000000
```

Then launch the robot:

```bash
ros2 launch piper_ros_control start_single_piper.launch.py can_port:=can-piper
```

---

### 3. Keyboard Control Demo

Control the robot interactively using your keyboard on a different terminal:

```bash
ros2 run piper_ros_control keyboard_control
```

This demo is useful for:

* Testing basic motion
* Understanding how commands are sent to the controller
* Quick manual control without writing additional code

---

## Nodes

### `piper_controller.py`

Main controller node for the Piper robot arm.
Handles communication between ROS 2 and the robot hardware.
⭐ Understanding this code will help catching the concept of controlling a piper arm.

---

### `piper_ctrl_single_custom_node.py`

A custom control node for a single robot setup to add speed control.

---

## Launch Files

### `start_single_piper.launch.py`

Launch configuration for controlling a single Piper robot and visualizing it's current status on RViz.

### `start_dual_piper.launch.py`

Launch configuration for controlling two Piper robots simultaneously (TBD).

---

## 🎮 Demos

### `keyboard_control.py`

A simple interactive demo for controlling the robot via keyboard input.
⭐ This code will help you understand how the `PiperController` node works in practice.

---

## 📄 License

Apache License 2.0
