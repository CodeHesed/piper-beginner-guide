# Piper Beginner Guide 👶

This guide is based on my experience getting started with the [AgileX Robotics](https://github.com/agilexrobotics) Piper arm.  
It aims to help others quickly understand and start using the robot without the usual setup difficulties.

Suggestions and contributions are always welcome!

## Repository Structure

```
piper_beginner_guide/
├── piper_kinematics/       # Standalone FK/IK library (no ROS required)
└── piper_ros_control/      # Custom ROS 2 Humble package for robot control
```

## How to use

### Build and requirement

Build the package and source the install file. This may be done only once.
I recommend placing the [piper_ros2-humble](https://github.com/agilexrobotics/piper_ros/tree/humble) package in this workspace.

```bash
colcon build
```

### Launching the robot arm

On a bash terminal, activate the CAN interface first (requires the `piper_ros` driver):

```bash
source /opt/ros/humble/setup.bash
source ./install/setup.bash
bash /path/to/piper_ros/can_activate.sh can-piper 1000000
ros2 launch piper_ros_control start_single_piper.launch.py  can_port:=can-piper
```

### Keyboard control demo

This demo allows you to control the robot arm using keyboard inputs.
It is useful for understanding how the `PiperController` node works in practice.

```bash
source /opt/ros/humble/setup.bash
source ./install/setup.bash
ros2 run piper_ros_control keyboard_control
```

### Kinematics visualization tool

You can visualize the Piper arm’s kinematic chain in 3D using the standalone visualizer (no ROS required):

This tool helps with:
- Visualizing the robot’s kinematic structure
- Verifying forward and inverse kinematics behavior

```bash
python3 ./piper_kinematics/piper_kinematics_visualizer.py
```

