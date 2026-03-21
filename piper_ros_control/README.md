# Piper ROS Control Package

This is a ROS 2 package for controlling the Piper robot arm. It provides nodes for robot control, launch files for different configurations, and demo scripts.

## Package Structure

```
piper_ros_control/
├── package.xml              # ROS package metadata
├── setup.py                # Python package configuration
├── resource/
│   └── piper_ros_control    # Package resource file
└── piper_ros_control/      # Python package module
    ├── __init__.py
    ├── nodes/
    │   ├── piper_controller.py               # Main controller node
    │   └── piper_ctrl_single_custom_node.py  # Single custom control node
    ├── launch/
    │   ├── start_single_piper.launch.py      # Launch single robot
    │   └── start_dual_piper.launch.py        # Launch dual robots
    └── demos/
        └── keyboard_control.py                # Keyboard control demo
```

## Dependencies

- ROS 2 Foxy or newer
- rclpy
- ros2_control
- ros2_controllers
- robot_state_publisher
- joint_state_publisher

## Usage

### Building the Package

```bash
colcon build --packages-select piper_ros_control
```

### Running Nodes

```bash
# Controller node
ros2 run piper_ros_control piper_controller

# Single custom control node
ros2 run piper_ros_control piper_ctrl_single
```

### Launch Files

```bash
# Launch single Piper robot
ros2 launch piper_ros_control start_single_piper.launch.py

# Launch dual Piper robots
ros2 launch piper_ros_control start_dual_piper.launch.py
```

### Demos

```bash
# Run keyboard control demo
ros2 run piper_ros_control keyboard_control
```

## Nodes

- `piper_controller.py`: Main controller node for Piper robot arm
- `piper_ctrl_single_custom_node.py`: Single robot custom control node that is made to control robot speed

## Launch Files

- `start_single_piper.launch.py`: Launch configuration for single Piper robot
- `start_dual_piper.launch.py`: Launch configuration for dual Piper robots

## Demos

- `keyboard_control.py`: Interactive keyboard control demo

## License

Apache License 2.0