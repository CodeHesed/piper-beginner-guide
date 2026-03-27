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

## Piper Kinematics

### Kinematics visualization tool

You can visualize the Piper arm’s kinematic chain in 3D using the standalone visualizer (no ROS required):

This tool helps with:
- Visualizing the robot’s kinematic structure
- Verifying forward and inverse kinematics behavior

```bash
python3 ./piper_kinematics/piper_kinematics_visualizer.py
```

## Piper ROS Control

### Set up

Build the package and source the install file. This may be done only once.
**⚠️ I highly recommend placing the [piper_ros2-humble](https://github.com/agilexrobotics/piper_ros/tree/humble) package in this workspace. The examples below are all written assuming this.**

```bash
conda deactivate
colcon build
```

Create a conda environment to actually use the piper controller. This may be also done only once.

```bash
conda create -n piper_beginner_guide python=3.10
conda activate piper_beginner_guide
conda install pinocchio=3.6.0 casadi -c conda-forge
conda deactivate
```

### Launching the robot arm

On a bash terminal, activate the CAN interface first (requires the `piper_ros` driver):

```bash
source scripts/launch_piper_arm.bash
```

### Keyboard control demo

This demo allows you to control the robot arm using keyboard inputs.
It is useful for understanding how the `PiperController` node works in practice.

```bash
source scripts/demo_piper_arm.bash
```
