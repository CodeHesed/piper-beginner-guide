#!/usr/bin/env python3
"""
Piper Robot Arm & Gripper Controller

Unified ROS2-based controller for the Piper 6-DOF robot arm and its gripper.
Combines arm trajectory control with intelligent gripper state monitoring.

Usage:
    from manipulation import PiperController

    bot = PiperController()
    bot.enable()
    bot.move_to_pose(x=0.3, y=0.0, z=0.2)
    bot.move_relative(dz=-0.05) # Move down 5cm
    bot.gripper_close()
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Bool, Int32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from piper_msgs.msg import PosCmd, PiperStatusMsg
from piper_msgs.srv import Enable

import numpy as np
import math
import time
import threading
from typing import Optional, List, Callable
from dataclasses import dataclass, field
from enum import Enum
import time

from scipy.spatial.transform import Rotation
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped

from pathlib import Path
import sys

# --- Import Setup ---
project_root = Path(__file__).parent.parent.parent
sys.path.append(str(project_root / 'src'))

# ==========================================
#          Piper Physical Traits
# ==========================================

# Currently set_gripper function automatically turns target width into required input
MAX_OPENING = 0.05   # Nearly fully open (50mm)
MIN_OPENING = 0.000    # Nearly fully closed (0mm)
CLOSED_THRESHOLD = 0.005  # < 5mm is considered closed
OPEN_THRESHOLD = 0.045    # > 45mm is considered open

# ==========================================
#              Data Structures
# ==========================================

class ArmStatus(Enum):
    """Arm operational status"""
    UNKNOWN = 0
    STANDBY = 1
    MOTION = 2
    TEACHING = 3
    ERROR = 4

class GripperState(Enum):
    """Gripper physical state"""
    UNKNOWN = 0
    OPEN = 1
    CLOSED = 2
    MOVING = 3
    HOLDING = 4  # Closed with object detected (stall)

@dataclass
class EndEffectorPose:
    """End effector pose data"""
    x: float = 0.0          # meters
    y: float = 0.0          # meters
    z: float = 0.0          # meters
    roll: float = 0.0       # degrees
    pitch: float = 0.0      # degrees
    yaw: float = 0.0        # degrees
    timestamp: float = 0.0

@dataclass
class GripperStatus():
    """Gripper specific status data"""
    position: float = MAX_OPENING        # Current position (m)
    target_position: float = MAX_OPENING  # Target position (m)
    state: GripperState = GripperState.UNKNOWN
    is_holding_object: bool = False
    timestamp: float = 0.0

@dataclass
class JointStates:
    """Joint states data"""
    positions: List[float] = field(default_factory=lambda: [0.0]*6)
    velocities: List[float] = field(default_factory=lambda: [0.0]*6)
    efforts: List[float] = field(default_factory=lambda: [0.0]*6)
    gripper: float = 0.0
    timestamp: float = 0.0


# ==========================================
#              Main Controller
# ==========================================

class PiperController(Node):
    """
    Piper Robot Controller
    
    Features:
    - Cartesian and Joint space control
    - Integrated Gripper control with holding detection
    - Relative motion support
    - Status monitoring
    """

    def __init__(self, node_name: str = 'piper_controller', arm_name = 'right', debug = True, dual_arm = False):
        super().__init__(node_name)

        # Arm defining left/right
        self.arm_name = arm_name
        self.dual_arm = dual_arm

        # Debug option for printing
        self.debug = debug

        # Gripper Constants
        self.MAX_OPENING = MAX_OPENING   # Fully open (m)
        self.MIN_OPENING = MIN_OPENING    # Closed/Grasping limit (m)

        # Move Speed Constants
        self.MAX_SPEED = 80
        self.MIN_SPEED = 1

        # Initialize TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_timeout = 0.1
        
        # Logic Thresholds
        self.CLOSED_THRESHOLD = CLOSED_THRESHOLD  # < 5mm is considered closed
        self.OPEN_THRESHOLD = OPEN_THRESHOLD      # > 65mm is considered open
        
        # Threading
        self._cb_group = ReentrantCallbackGroup()
        self._state_lock = threading.Lock()

        # State Storage
        self._current_joints = JointStates()
        self._current_pose = EndEffectorPose()
        self._gripper_status = GripperStatus()
        self._arm_status = ArmStatus.UNKNOWN
        self._is_enabled = False
        self._last_status_msg: Optional[PiperStatusMsg] = None

        # Gripper Event Callbacks
        self._on_grasp_callback: Optional[Callable] = None
        self._on_release_callback: Optional[Callable] = None

        # Define topic names
        _pos_cmd_topic = '/pos_cmd_' + arm_name if dual_arm else '/pos_cmd'
        _joint_cmd_topic = '/joint_ctrl_cmd_' + arm_name if dual_arm else '/joint_ctrl_single'
        _enable_topic = '/enable_flag_' + arm_name if dual_arm else '/enable_flag'
        _move_speed_topic = '/move_speed_' + arm_name if dual_arm else '/move_speed'
        _joint_states_topic = '/joint_states_' + arm_name if dual_arm else '/joint_states'
        _end_pose_topic = '/end_pose_' + arm_name if dual_arm else '/end_pose'
        _arm_status_topic = '/arm_status_' + arm_name if dual_arm else '/arm_status'

        # --- Publishers ---
        self._pos_cmd_pub = self.create_publisher(PosCmd, _pos_cmd_topic, 10)
        self._joint_cmd_pub = self.create_publisher(JointState, _joint_cmd_topic, 10)
        self._enable_pub = self.create_publisher(Bool, _enable_topic, 10)
        self._move_speed_pub = self.create_publisher(Int32, _move_speed_topic, 10)

        # --- Subscribers ---
        self._joint_sub = self.create_subscription(
            JointState, _joint_states_topic,
            self._joint_callback, 10,
            callback_group=self._cb_group
        )
        self._pose_sub = self.create_subscription(
            Pose, _end_pose_topic,
            self._pose_callback, 10,
            callback_group=self._cb_group
        )
        self._status_sub = self.create_subscription(
            PiperStatusMsg, _arm_status_topic,
            self._status_callback, 10,
            callback_group=self._cb_group
        )

        # --- Services ---
        self._enable_client = self.create_client(Enable, 'enable_srv')

        self.get_logger().info('PiperController initialized')

    # ==========================================
    #                Callbacks
    # ==========================================

    def _joint_callback(self, msg: JointState):
        """Handle joint state updates for arm and gripper logic"""
        with self._state_lock:
            # 1. Update Arm Joints
            if len(msg.position) >= 6:
                self._current_joints.positions = list(msg.position[:6])
            if len(msg.velocity) >= 6:
                self._current_joints.velocities = list(msg.velocity[:6])
            if len(msg.effort) >= 6:
                self._current_joints.efforts = list(msg.effort[:6])

            # 2. Update Gripper Data
            if len(msg.position) >= 7:
                gripper_pos = msg.position[6]
                self._current_joints.gripper = gripper_pos
                
                # Update detailed gripper status
                old_position = self._gripper_status.position
                self._gripper_status.position = gripper_pos
                self._gripper_status.timestamp = time.time()

                # Gripper Logic State Machine
                if self._gripper_status.position < self.CLOSED_THRESHOLD:
                    self._gripper_status.state = GripperState.CLOSED
                    self._gripper_status.is_holding_object = False
                elif self._gripper_status.position > self.OPEN_THRESHOLD:
                    self._gripper_status.state = GripperState.OPEN
                    self._gripper_status.is_holding_object = False
                else:
                    # Check motion
                    if abs(self._gripper_status.position - old_position) > 0.001:
                        self._gripper_status.state = GripperState.MOVING
                    else:
                        # Stationary at intermediate position -> Holding
                        self._gripper_status.state = GripperState.HOLDING
                        self._gripper_status.is_holding_object = True

            self._current_joints.timestamp = time.time()

    def _pose_callback(self, msg: Pose):
        """Handle end-effector pose updates"""
        with self._state_lock:
            self._current_pose.x = msg.position.x
            self._current_pose.y = msg.position.y
            self._current_pose.z = msg.position.z

            # Simplified Quaternion to RPY (ZYX sequence)
            q = msg.orientation
            self._current_pose.roll = np.degrees(np.arctan2(2*(q.w*q.x + q.y*q.z), 1 - 2*(q.x*q.x + q.y*q.y)))

            # Clamp arcsin input to [-1, 1] to avoid NaN
            pitch_input = 2*(q.w*q.y - q.z*q.x)
            pitch_input = np.clip(pitch_input, -1.0, 1.0)
            self._current_pose.pitch = np.degrees(np.arcsin(pitch_input))

            self._current_pose.yaw = np.degrees(np.arctan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z)))
            
            self._current_pose.timestamp = time.time()

    def _status_callback(self, msg: PiperStatusMsg):
        """Handle arm firmware status updates"""
        with self._state_lock:
            self._last_status_msg = msg
            if msg.arm_status == 1:
                self._arm_status = ArmStatus.STANDBY
            elif msg.arm_status == 2:
                self._arm_status = ArmStatus.MOTION
            elif msg.arm_status == 3:
                self._arm_status = ArmStatus.TEACHING
            elif msg.err_code != 0:
                self._arm_status = ArmStatus.ERROR
            else:
                self._arm_status = ArmStatus.UNKNOWN

    # ==========================================
    #              Getters
    # ==========================================

    def get_joint_states(self) -> JointStates:
        with self._state_lock:
            # Return copy to prevent external mutation issues
            return JointStates(
                positions=self._current_joints.positions.copy(),
                velocities=self._current_joints.velocities.copy(),
                efforts=self._current_joints.efforts.copy(),
                gripper=self._current_joints.gripper,
                timestamp=self._current_joints.timestamp
            )

    def get_end_effector_pose(self) -> EndEffectorPose:
        with self._state_lock:
            return EndEffectorPose(
                x=self._current_pose.x,
                y=self._current_pose.y,
                z=self._current_pose.z,
                roll=self._current_pose.roll,
                pitch=self._current_pose.pitch,
                yaw=self._current_pose.yaw,
                timestamp=self._current_pose.timestamp
            )

    def get_gripper_status(self) -> GripperStatus:
        with self._state_lock:
            return GripperStatus(
                position=self._gripper_status.position,
                target_position=self._gripper_status.target_position,
                state=self._gripper_status.state,
                is_holding_object=self._gripper_status.is_holding_object,
                timestamp=self._gripper_status.timestamp
            )

    def is_holding(self) -> bool:
        with self._state_lock:
            return self._gripper_status.is_holding_object

    def is_enabled(self) -> bool:
        with self._state_lock:
            return self._is_enabled

    # ==========================================
    #              Arm Control
    # ==========================================

    def enable(self) -> bool:
        msg = Bool()
        msg.data = True
        self._enable_pub.publish(msg)
        self._is_enabled = True
        self.get_logger().info('Arm enabled')
        return True

    def disable(self) -> bool:
        msg = Bool()
        msg.data = False
        self._enable_pub.publish(msg)
        self._is_enabled = False
        self.get_logger().info('Arm disabled')
        return True
    
    def set_speed(self, _move_speed):
        msg = Int32()
        msg.data = _move_speed

        # Clamp value        
        move_speed = max(self.MIN_SPEED, min(self.MAX_SPEED, _move_speed))
        if (_move_speed != move_speed):
            self.get_logger().warn(f"Fitting to moving speed range: {move_speed}")

        if (self.debug):
            self.get_logger().info(f"Setting arm speed to {move_speed}...")
        self._move_speed_pub.publish(msg)
        time.sleep(1.0) # A 1.0 second sleep for speed set up
        if (self.debug):
            self.get_logger().info(f"Arm speed set up finshed.")

        return True
    
    def instant_stop(self):
        if (self.debug):
            self.get_logger().info(f"Instantly stopping the arm in current pose.")

        current_joints = self.get_joint_states()
        self.move_joints(current_joints.positions)

        return True
    
    def move_to_pose(self, x: float, y: float, z: float,
                     roll: float = 0.0, pitch: float = 180.0, yaw: float = 0.0,
                     gripper: Optional[float] = None) -> bool:
        """
        Move end-effector to Cartesian pose.

        Args:
            x, y, z: Position (meters)
            roll, pitch, yaw: Orientation (degrees)
            gripper: Optional gripper position override
        """

        msg = PosCmd()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        msg.roll = float(np.radians(roll))
        msg.pitch = float(np.radians(pitch))
        msg.yaw = float(np.radians(yaw))
        
        # If gripper not specified, use current status
        if gripper is not None:
            msg.gripper = float(gripper)
            self._gripper_status.target_position = float(gripper)
        else:
            with self._state_lock:
                msg.gripper = self._gripper_status.target_position

        msg.mode1 = 0
        msg.mode2 = 0

        self._pos_cmd_pub.publish(msg)

        if (self.debug):
            self.get_logger().info(f"Moving gripper base to x={x:.4f}, y={y:.4f}, z={z:.4f},"
                                    f" roll={roll:.1f}, pitch={pitch:.1f}, yaw={yaw:.1f}")
        
        return True

    def get_link_offset(self, from_link, to_link):
        """Gets the static translation vector from one link to another."""
        try:
            # We use lookup_transform for static transforms between links
            trans = self.tf_buffer.lookup_transform(from_link, to_link, 
                                                        rclpy.time.Time(),
                                                        timeout=rclpy.duration.Duration(seconds=self.tf_timeout))
            offset = trans.transform.translation
            return np.array([offset.x, offset.y, offset.z])
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().info(f"Failed to get transform from {from_link} to {to_link}: {e}")
            return None
    
    def get_base_pose(self, base_frame: str) -> np.ndarray | None:
        """
        Get transform from base_frame to this arm's base_link as a 4x4 matrix
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                base_frame,                     # target frame
                f"{self.arm_name}_base_link",   # source frame
                rclpy.time.Time()               # latest available
            )

            return self.transform_to_matrix(transform)

        except TransformException as e:
            self.get_logger().warn(
                f"Failed to get transform from {base_frame} "
                f"to {self.arm_name}_base_link: {e}"
            )
            return None

    def move_relative(self,
                      dx: float = 0.0, dy: float = 0.0, dz: float = 0.0,
                      droll: float = 0.0, dpitch: float = 0.0, dyaw: float = 0.0) -> bool:
        """
        Move relative to current position

        Args:
            dx, dy, dz: Position offset in meters
            droll, dpitch, dyaw: Orientation offset in degrees
        """
        pose = self.get_end_effector_pose()
        return self.move_to_pose(
            pose.x + dx,
            pose.y + dy,
            pose.z + dz,
            pose.roll + droll,
            pose.pitch + dpitch,
            pose.yaw + dyaw
        )

    def move_joints(self, positions: List[float], gripper: Optional[float] = None) -> bool:
        """Move joints directly (rads)"""
        if len(positions) < 6:
            self.get_logger().error(f'Need 6 joint positions, got {len(positions)}')
            return False
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']

        pos_with_gripper = list(positions[:6])
        if gripper is not None:
            pos_with_gripper.append(gripper)
            self._gripper_status.target_position = float(gripper)
        else:
            with self._state_lock:
                pos_with_gripper.append(self._gripper_status.target_position)

        msg.position = pos_with_gripper
        self._joint_cmd_pub.publish(msg)
        return True
    
    def move_to_home(self) -> bool:
        """Move to all-zeros joint position"""
        return self.move_joints([0.0]*6)

    # ==========================================
    #              Gripper Control
    # ==========================================

    def set_gripper(self, _width: float) -> bool:
        """
        Set gripper to specific width in meters.
        Safely maintains current arm joint positions.
        """

        self.get_logger().info(f"Setting gripper to {_width}m")

        # Clamp value
        width = max(self.MIN_OPENING, min(self.MAX_OPENING, _width))
        if (_width != width):
            self.get_logger().warn(f"Fitting to grippers operating range: {width}m")

        with self._state_lock:
            self._gripper_status.target_position = width
            current_joints = self._current_joints.positions.copy()

        # Send command using current joint positions + new gripper value
        # This prevents the arm from moving
        return self.move_joints(current_joints, gripper=width)

    def gripper_open(self, width: Optional[float] = None) -> bool:
        """Open gripper to max or specified width"""
        target = width if width is not None else self.MAX_OPENING
        self.get_logger().info(f"Opening gripper to {target}m")
        return self.set_gripper(target)

    def gripper_close(self) -> bool:
        """Close gripper to MIN_OPENING"""
        self.get_logger().info("Closing gripper")
        return self.set_gripper(self.MIN_OPENING)

    def gripper_grasp(self, target_width: float = 0.0) -> bool:
        """
        Execute grasp logic.
        Closes gripper and triggers callback if holding detected.
        """
        success = self.set_gripper(target_width)

        # Simple non-blocking check mechanism (can be improved with timers)
        if success and self._on_grasp_callback:
            def check_grasp():
                time.sleep(1.0) # Wait for motion
                if self.is_holding():
                    self._on_grasp_callback()
            
            threading.Thread(target=check_grasp, daemon=True).start()
            
        return success

    def gripper_release(self) -> bool:
        """Release object"""
        with self._state_lock:
            self._gripper_status.is_holding_object = False
        
        success = self.gripper_open()
        
        if success and self._on_release_callback:
            self._on_release_callback()
            
        return success

    def on_grasp(self, callback: Callable):
        self._on_grasp_callback = callback

    def on_release(self, callback: Callable):
        self._on_release_callback = callback

    # ==========================================
    #              Utilities
    # ==========================================
    
    def wait_for_motion_complete(self, timeout: float = 10.0) -> bool:
        """
        Wait for arm to return to STANDBY status.
        Args:
            timeout: Max wait time in seconds
        Returns:
            True if motion completed successfully
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            with self._state_lock:
                if self._arm_status == ArmStatus.STANDBY:
                    return True
            time.sleep(0.1)
        return False
    
    @staticmethod
    def transform_to_matrix(transform: TransformStamped) -> np.ndarray:
        """
        Convert geometry_msgs/TransformStamped to 4x4 transformation matrix
        """
        t = transform.transform.translation
        q = transform.transform.rotation  # x, y, z, w

        # Translation vector
        p = np.array([t.x, t.y, t.z])

        # Quaternion components
        x, y, z, w = q.x, q.y, q.z, q.w

        # Quaternion → rotation matrix
        R = np.array([
            [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
            [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
            [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)]
        ])

        # Homogeneous transform
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = p

        return T


# ==========================================
#              Standalone Test
# ==========================================

def main(args=None):
    rclpy.init(args=args)
    controller = PiperController()

    print("\n" + "=" * 60)
    print("Piper Combined Controller - Interactive Mode")
    print("=" * 60)
    print("Commands:")
    print("  e - Enable arm | d - Disable arm | h - Home")
    print("  o - Open grip  | c - Close grip  | g - Grasp | r - Release")
    print("  s - Status     | p - Pose input  | q - Quit")
    print("  x - Relative Move (Test)")
    print("=" * 60)

    executor = MultiThreadedExecutor()
    executor.add_node(controller)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    time.sleep(2.0) # Wait for connections

    try:
        while rclpy.ok():
            cmd = input("\nCommand: ").strip().lower()

            if cmd == 'e':
                controller.enable()
            elif cmd == 'd':
                controller.disable()
            elif cmd == 'h':
                controller.move_to_home()
            elif cmd == 'o':
                controller.gripper_open()
            elif cmd == 'c':
                controller.gripper_close()
            elif cmd == 'g':
                controller.gripper_grasp()
            elif cmd == 'r':
                controller.gripper_release()
            elif cmd == 's':
                joints = controller.get_joint_states()
                pose = controller.get_end_effector_pose()
                grip = controller.get_gripper_status()
                
                print(f"\n--- Status ---")
                print(f"Arm Enabled: {controller.is_enabled()}")
                print(f"End Effector: ({pose.x:.3f}, {pose.y:.3f}, {pose.z:.3f})")
                print(f"Gripper: {grip.position*1000:.1f}mm | State: {grip.state.name} | Holding: {grip.is_holding_object}")
            
            elif cmd == 'p':
                try:
                    x = float(input("X: ") or "0.2")
                    y = float(input("Y: ") or "0.0")
                    z = float(input("Z: ") or "0.2")
                    controller.move_to_pose(x, y, z)
                except ValueError:
                    print("Invalid input")
            
            elif cmd == 'x':
                print("Moving Down 2cm...")
                controller.move_relative(dz=-0.02)
                
            elif cmd == 'q':
                break

    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
