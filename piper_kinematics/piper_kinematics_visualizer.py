"""
Piper Robot Arm Visualizer
3D Plot on top + Joint sliders at bottom
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D

from piper_dh_params import JOINT_LIMITS, DH_PARAMS, GRIPPER_OFFSET
from piper_kinematics import forward_kinematics_gripper_center, dh_transform


class PiperVisualizer:
    def __init__(self, base_transform=None):

        self.base_transform = np.eye(4) if base_transform is None else base_transform
        self.joint_angles = np.zeros(6)

        # Create figure
        self.fig = plt.figure(figsize=(12, 9))

        # 3D plot (top area)
        self.ax_3d = self.fig.add_axes([0.05, 0.30, 0.90, 0.65], projection='3d')

        # Sliders container
        self.sliders = []
        slider_height = 0.03
        slider_spacing = 0.045
        slider_width = 0.85

        for i in range(6):
            lower, upper = JOINT_LIMITS[i]

            ax_slider = self.fig.add_axes(
                [0.075, 0.22 - i * slider_spacing, slider_width, slider_height]
            )

            slider = Slider(
                ax_slider,
                f'J{i+1}',
                lower,
                upper,
                valinit=0.0,
                valstep=0.001
            )

            slider.on_changed(self.update_plot)
            self.sliders.append(slider)

        self.update_plot(None)

    @staticmethod
    def get_link_positions(joints, base_transform):
        T = base_transform.copy()
        positions = [T[:3, 3]]

        for joint_angle, dh in zip(joints, DH_PARAMS):
            theta = joint_angle + dh.theta_offset
            T_i = dh_transform(dh.a, dh.alpha, dh.d, theta)
            T = T @ T_i
            positions.append(T[:3, 3])

        # Add gripper
        gripper_T = np.eye(4)
        gripper_T[2, 3] = GRIPPER_OFFSET
        T = T @ gripper_T
        positions.append(T[:3, 3])

        return np.array(positions)

    @staticmethod
    def compute_joint_transforms(joints, base_transform):
        """Return list of 4x4 transforms for each joint frame (after applying DH)."""
        T = base_transform.copy()
        transforms = []

        for joint_angle, dh in zip(joints, DH_PARAMS):
            theta = joint_angle + dh.theta_offset
            T_i = dh_transform(dh.a, dh.alpha, dh.d, theta)
            T = T @ T_i
            transforms.append(T.copy())

        return transforms

    def draw_frame(self, T, scale=0.08):
        """Draw RGB (X, Y, Z) coordinate frame axes at transform T."""
        origin = T[:3, 3]
        axes = [
            ('r', T[:3, 0]),   # X - red
            ('g', T[:3, 1]),   # Y - green
            ('b', T[:3, 2]),   # Z - blue
        ]
        for color, direction in axes:
            end = origin + direction * scale
            self.ax_3d.plot(
                [origin[0], end[0]],
                [origin[1], end[1]],
                [origin[2], end[2]],
                color=color,
                linewidth=2
            )

    def update_plot(self, val):

        self.joint_angles = np.array([s.val for s in self.sliders])

        self.ax_3d.clear()

        positions = self.get_link_positions(self.joint_angles, self.base_transform)

        T = forward_kinematics_gripper_center(
            self.joint_angles, self.base_transform
        )
        ee_pos = T[:3, 3]

        # Plot links
        self.ax_3d.plot(
            positions[:, 0],
            positions[:, 1],
            positions[:, 2],
            marker='o',
            color='black',
            linewidth=3,
            markersize=6
        )

        # Plot gripper_base
        self.ax_3d.scatter(
            positions[-2, 0],
            positions[-2, 1],
            positions[-2, 2],
            marker='o',
            s=50,
            color='b',
            label='Gripper Base'
        )

        # Plot gripper center
        self.ax_3d.scatter(
            ee_pos[0],
            ee_pos[1],
            ee_pos[2],
            s=150,
            marker='*',
            color='r',
            label='Gripper Center'
        )

        # Plot base
        self.ax_3d.scatter(
            self.base_transform[0, 3],
            self.base_transform[1, 3],
            self.base_transform[2, 3],
            s=120,
            marker='^',
            color='g',
            label='Base'
        )

        # ------------------------
        # Draw frames: Base and Gripper Center
        # ------------------------
        joint_transforms = self.compute_joint_transforms(
            self.joint_angles, self.base_transform
        )

        self.draw_frame(self.base_transform, scale=0.08)   # Base
        self.draw_frame(joint_transforms[5], scale=0.08)   # Gripper Center

        # Formatting
        self.ax_3d.set_title("Piper Robot Arm Visualizer", fontsize=14, fontweight='bold')
        self.ax_3d.set_xlabel("X (m)")
        self.ax_3d.set_ylabel("Y (m)")
        self.ax_3d.set_zlabel("Z (m)")

        self.ax_3d.set_xlim([-0.5, 0.5])
        self.ax_3d.set_ylim([-0.5, 0.5])
        self.ax_3d.set_zlim([0.0, 0.8])

        self.ax_3d.set_box_aspect([1, 1, 1.2])
        self.ax_3d.view_init(elev=30, azim=140)

        self.ax_3d.legend(loc='upper right')
        self.fig.canvas.draw_idle()

    def show(self):
        plt.show()


def main():

    print("=" * 60)
    print("PIPER ARM VISUALIZER")
    print("Use sliders at bottom to adjust joint angles.")
    print("=" * 60)

    arm_base = np.eye(4)

    visualizer = PiperVisualizer(base_transform=arm_base)
    visualizer.show()


if __name__ == "__main__":
    main()