
# Piper Beginner Guide

This guide was made based on the experience of having a hard time starting a project with a [AgileX Robotics](https://github.com/agilexrobotics) Piper arm. I hope this guide helps others easily understand how to use a Piper arm.

## Repository Structure

```
piper_beginner_guide/
├── piper_kinematics/       # Standalone FK/IK library (no ROS required)
└── piper_ros_control/      # Custom ROS 2 Humble package for robot control
```

## How to use

Build the package and source the install file. This may be done only once.

```bash
colcon build
```

### Launching the robot arm

On a bash terminal, activate the CAN interface first (requires the [piper_ros](https://github.com/agilexrobotics/piper_ros) driver to be installed):

```bash
source /opt/ros/humble/setup.bash
source ./install/setup.bash
cd /{Your destination to piper_ros-humble}/piper_ros-humble/piper_ros-humble
bash can_activate.sh
ros2 launch piper_ros_control start_single_piper.launch.py
```

### Keyboard control demo

```bash
source /opt/ros/humble/setup.bash
source ./install/setup.bash
python3 ./piper_ros_control/piper_ros_control/demos/keyboard_control.py
```
