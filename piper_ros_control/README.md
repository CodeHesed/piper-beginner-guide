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
* Conda environment forged with `pinocchio=3.6.0` and `casadi` (For details, checkout **section 3.1**)

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

`CAUTION! TO USE THE KEYBOARD DEMO, A CONDA ENVIRONMENT IS REQUIRED DUE TO DEPENDENCIES`

## 3.1. Keyboard Control Demo Set Up (One-time)

Create a conda environment for the control demo.

```bash
conda create -n piper_beginner_guide python=3.10
conda activate piper_beginner_guide
conda install pinocchio=3.6.0 casadi -c conda-forge
conda deactivate
```

## 3.2. Keyboard Control Demo Run

Control the robot interactively using your keyboard on a different terminal:

```bash
source install/setup.bash
conda activate piper_controller
python3 piper_ros_control/demos/keyboard_control.py
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
⭐⭐ Currently uses an IK solver for position command due to issues caused by recent firmware updates.

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

## Demos

### `keyboard_control.py`

A simple interactive demo for controlling the robot via keyboard input.
⭐ This code will help you understand how the `PiperController` node works in practice.

---

## Utils

### `ik_solver.py`

An IK solver to convert pose commands into joint states commands.
⭐ The threshold to check errors of the IK solver can be set. 
`PLEASE NOTICE THAT THE DEFAULT THRESHOLD IS QUITE HIGH (Position: 0.01m, Orientation: 0.15rad/8.6deg)`

---

## 📄 License

Apache License 2.0
