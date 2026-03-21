#!/usr/bin/env python3
"""
Keyboard Control - Safe Teleoperation Mode
SAFETY: You must click on the 'Teleop View' window for keys to work.

Controls:
  W / S      : +/- X (Forward/Back)
  A / D      : +/- Y (Left/Right)
  I / K      : +/- Z (Up/Down)
  Q / E      : +/- Roll
  U / O      : +/- Pitch
  J / L      : +/- Yaw
  R          : Return to Home Position
  0 / 9      : +/- Move Speed
  Space      : Toggle Grasp
  ESC        : Quit
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor
import time
import threading
import numpy as np
import cv2

from piper_ros_control.nodes.piper_controller import PiperController

class KeyboardControl:
    def __init__(self):
        # Initialize ROS nodes
        self.arm = PiperController('piper_arm_controller')
        # Configuration
        self.HOME_POSE = {'x': 0.2, 'y': 0.0, 'z': 0.2, 'roll': 0, 'pitch': 90, 'yaw': 0}
        # State Variables
        self.cur_x = self.HOME_POSE['x']
        self.cur_y = self.HOME_POSE['y']
        self.cur_z = self.HOME_POSE['z']
        self.cur_roll = self.HOME_POSE['roll']
        self.cur_pitch = self.HOME_POSE['pitch']
        self.cur_yaw = self.HOME_POSE['yaw']
        self.gripper_is_closed = False
        self.move_speed = 50
        
        # Steps configuration
        self.step_xyz = 0.01   # 1cm per keypress (finer control)
        self.step_angle = 5.0  # 5 degrees per keypress

    def print_status(self):
        """Prints current target state."""
        print(f"\r[STATUS] X:{self.cur_x:.3f} Y:{self.cur_y:.3f} Z:{self.cur_z:.3f} R:{self.cur_roll:.1f} P:{self.cur_pitch:.1f} Y:{self.cur_yaw:.1f} Grip:{'CLOSE' if self.gripper_is_closed else 'OPEN'}   ", end="")

    def move_robot(self):
        """Sends the command to the arm controller."""
        self.arm.move_to_pose(
            x=self.cur_x,
            y=self.cur_y,
            z=self.cur_z,
            roll=self.cur_roll,
            pitch=self.cur_pitch,
            yaw=self.cur_yaw
        )
    
    def run_safe_teleop(self):
        print("\n" + "="*50)
        print("SAFE TELEOP MODE STARTED")
        print("INSTRUCTIONS: Click the 'Teleop View' window to control.")
        print("-" * 50)
        print("  W/S      : +/- X")
        print("  A/D      : +/- Y")
        print("  I/K      : +/- Z (Up/Down)")
        print("  Q/E      : +/- Roll")
        print("  U/O      : +/- Pitch")
        print("  J/L      : +/- Yaw")
        print("  R        : Return to Home")
        print("  Space    : Toggle Grasp")
        print("  0 / 9    : +/- Move Speed")
        print("  Esc      : Quit")
        print("="*50)

        # 1. Initialize Arm
        print("[INIT] Enabling arm...")
        self.arm.enable()
        self.arm.gripper_open()
        time.sleep(1.0)
        # Move to Home initially
        print("[INIT] Moving to Home...")
        self.move_robot()
        # 2. Main Loop (Input)
        while rclpy.ok():
            # A. Get Image & Draw UI Overlay
            window = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(window, "Keyboard Control", (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            # Add Text Overlay to Image for Feedback
            status_text1 = f"Arm Move Speed: {self.move_speed}%"
            status_text2 = f"  X:{self.cur_x:.3f} Y:{self.cur_y:.3f} Z:{self.cur_z:.3f}"
            status_text3 = f"  R:{self.cur_roll:.0f} P:{self.cur_pitch:.0f} Y:{self.cur_yaw:.0f}"
            line_height = 30
            cv2.putText(window, status_text1, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(window, status_text2, (10, 30 + line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(window, status_text3, (10, 30 + 2*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("Teleop View", window)

            # B. Wait for Key (This is the Safe Input Method)
            # The script waits 50ms for a key. If no key, it loops again (updating video)
            key_code = cv2.waitKey(50) & 0xFF

            # If no key pressed, key_code is 255
            if key_code == 255:
                continue

            # C. Handle Inputs
            moved = False

            # --- ESC (Quit) ---
            if key_code == 27:
                print("\n[STOP] Exiting...")
                break

            # --- WASD (XY) ---
            elif key_code == ord('w'):
                self.cur_x += self.step_xyz
                moved = True
            elif key_code == ord('s'):
                self.cur_x -= self.step_xyz
                moved = True
            elif key_code == ord('a'):
                self.cur_y += self.step_xyz
                moved = True
            elif key_code == ord('d'):
                self.cur_y -= self.step_xyz
                moved = True

            # --- I/K (Z) ---
            elif key_code == ord('i'):
                self.cur_z += self.step_xyz
                moved = True
            elif key_code == ord('k'):
                self.cur_z -= self.step_xyz
                moved = True

            # --- Q/E (Roll) ---
            elif key_code == ord('q'):
                self.cur_roll += self.step_angle
                moved = True
            elif key_code == ord('e'):
                self.cur_roll -= self.step_angle
                moved = True
                
            # --- U/O (Pitch) ---
            elif key_code == ord('u'):
                self.cur_pitch += self.step_angle
                moved = True
            elif key_code == ord('o'):
                self.cur_pitch -= self.step_angle
                moved = True
                
            # --- J/L (Yaw) ---
            elif key_code == ord('j'):
                self.cur_yaw += self.step_angle
                moved = True
            elif key_code == ord('l'):
                self.cur_yaw -= self.step_angle
                moved = True

            # --- R (Home) ---
            elif key_code == ord('r'):
                print("\n[CMD] Returning Home...")
                self.cur_x = self.HOME_POSE['x']
                self.cur_y = self.HOME_POSE['y']
                self.cur_z = self.HOME_POSE['z']
                self.cur_roll = self.HOME_POSE['roll']
                self.cur_pitch = self.HOME_POSE['pitch']
                self.cur_yaw = self.HOME_POSE['yaw']
                self.move_state = "gripper_base"
                moved = True

            # --- Space (Grasp) ---
            elif key_code == ord(' '):
                self.gripper_is_closed = not self.gripper_is_closed
                if self.gripper_is_closed:
                    print("\n[CMD] Grasping...")
                    self.arm.gripper_grasp(0.006)
                else:
                    print("\n[CMD] Releasing...")
                    self.arm.gripper_release()

            #  ---  0 / 9 (Move Speed) ---
            elif key_code == ord('0'):
                self.move_speed += 10
                self.arm.set_speed(self.move_speed)
            elif key_code == ord('9'):
                self.move_speed -= 10
                self.arm.set_speed(self.move_speed)

            # Execute Move if coordinates changed
            if moved:
                self.move_robot()
                self.print_status()

def main():
    rclpy.init()
    task = KeyboardControl()
    # Run ROS nodes in background
    executor = MultiThreadedExecutor()
    executor.add_node(task.arm)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    print("Waiting for ROS2 nodes to connect...")
    time.sleep(2.0)
    try:
        task.run_safe_teleop()
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        cv2.destroyAllWindows()
        task.arm.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()