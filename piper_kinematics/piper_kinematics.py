"""
DH parameters for the Piper robot arm
Parameter values based on document version: V1.1.1 (2025.04.02)
"""

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from typing import Optional

from piper_dh_params import (
    JOINT_LIMITS, DH_PARAMS, GRIPPER_OFFSET
)

def dh_transform(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    """Compute DH transformation matrix"""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    
    return np.array([
        [ct, -st*ca, st*sa, a*ct],
        [st, ct*ca, -ct*sa, a*st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ])

# ========= Gripper base kinematics - default method for pose command of piper arms =========

def forward_kinematics_gripper_base(joints: np.ndarray, base_transform: np.ndarray = None) -> np.ndarray:
    """Compute forward kinematics for Piper robot"""
    if base_transform is None:
        T = np.eye(4)
    else:
        T = base_transform.copy()
    
    for i, (joint_angle, dh) in enumerate(zip(joints, DH_PARAMS)):
        theta = joint_angle + dh.theta_offset
        T_i = dh_transform(dh.a, dh.alpha, dh.d, theta)
        T = T @ T_i
    
    return T

def inverse_kinematics_gripper_base(target_pose: np.ndarray, 
                                    base_transform: np.ndarray = None,
                                    seed_joints: np.ndarray = None,
                                    max_iter: int = 200,
                                    tol: float = 1e-5) -> Optional[np.ndarray]:
    """Numerical inverse kinematics solver using least squares"""
    
    if seed_joints is None:
        seed_joints = np.mean(JOINT_LIMITS, axis=1)

    target_pos = target_pose[:3, 3]
    target_rot = target_pose[:3, :3]
    
    def residual(joints):
        T = forward_kinematics_gripper_base(joints, base_transform)
        pos_err = target_pos - T[:3, 3]
        R_err = T[:3, :3].T @ target_rot
        rot_err = Rotation.from_matrix(R_err).as_rotvec()
        # Weight position error more heavily (optional)
        return np.concatenate([pos_err * 1000, rot_err])
    
    # Add bounds to keep solver within joint limits
    result = least_squares(
        residual, 
        seed_joints, 
        bounds=(JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1]),
        max_nfev=max_iter,
        ftol=tol,
        xtol=1e-7
    )
    
    joints = result.x
    
    # Check if solution is good enough
    T = forward_kinematics_gripper_base(joints, base_transform)
    pos_error = np.linalg.norm(T[:3, 3] - target_pos)
    
    # Extract rotation error
    R_err = T[:3, :3].T @ target_rot
    rot_error = np.linalg.norm(Rotation.from_matrix(R_err).as_rotvec())
    
    # More lenient thresholds
    if pos_error < 5e-4 and rot_error < 1e-3:
        return joints

# ========= Gripper center kinematics =========

def forward_kinematics_gripper_center(joints: np.ndarray, base_transform: np.ndarray = None) -> np.ndarray:
    """Compute forward kinematics for Piper robot"""
    
    # Compute forward kinematics for gripper base
    T = forward_kinematics_gripper_base(joints, base_transform)

    # Add gripper transformation
    gripper_T = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, GRIPPER_OFFSET],
        [0, 0, 0, 1]
    ])
    T = T @ gripper_T
    
    return T

def inverse_kinematics_gripper_center(target_pose: np.ndarray, 
                                    base_transform: np.ndarray = None,
                                    seed_joints: np.ndarray = None,
                                    max_iter: int = 200,
                                    tol: float = 1e-5) -> Optional[np.ndarray]:
    """Numerical inverse kinematics solver using least squares"""
    
    if seed_joints is None:
        seed_joints = np.mean(JOINT_LIMITS, axis=1)

    target_pos = target_pose[:3, 3]
    target_rot = target_pose[:3, :3]
    
    def residual(joints):
        T = forward_kinematics_gripper_base(joints, base_transform)
        pos_err = target_pos - T[:3, 3]
        R_err = T[:3, :3].T @ target_rot
        rot_err = Rotation.from_matrix(R_err).as_rotvec()
        # Weight position error more heavily (optional)
        return np.concatenate([pos_err * 1000, rot_err])
    
    # Add bounds to keep solver within joint limits
    result = least_squares(
        residual, 
        seed_joints, 
        bounds=(JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1]),
        max_nfev=max_iter,
        ftol=tol,
        xtol=1e-7
    )
    
    joints = result.x
    
    # Check if solution is good enough
    T = forward_kinematics_gripper_base(joints, base_transform)
    pos_error = np.linalg.norm(T[:3, 3] - target_pos)
    
    # Extract rotation error
    R_err = T[:3, :3].T @ target_rot
    rot_error = np.linalg.norm(Rotation.from_matrix(R_err).as_rotvec())
    
    # More lenient thresholds
    if pos_error < 5e-4 and rot_error < 1e-3:
        return joints