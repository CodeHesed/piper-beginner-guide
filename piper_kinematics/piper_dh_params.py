"""
DH parameters for Piper robot arms
Parameter values based on document version: V1.1.1 (2025.04.02)
"""

import numpy as np
from dataclasses import dataclass

@dataclass
class DHParameter:
    """Denavit-Hartenberg parameters"""
    a: float            # m 
    alpha: float        # rad
    d: float            # m
    theta_offset: float # rad

GRIPPER_OFFSET = 0.1358 

# Piper official manual
DH_PARAMS = [
    DHParameter(a=0.0, alpha=-np.pi/2, d=0.123, theta_offset=0.0),
    DHParameter(a=0.28503, alpha=0.0, d=0.0, theta_offset=-np.pi + 0.1359),
    DHParameter(a=-0.021984, alpha=np.pi/2, d=0.0, theta_offset=-np.pi/2 - 0.2231),
    DHParameter(a=0.0, alpha=-np.pi/2, d=0.25075, theta_offset=0.0),
    DHParameter(a=0.0, alpha=np.pi/2, d=0.0, theta_offset=0.0),
    DHParameter(a=0.0, alpha=0.0, d=0.091, theta_offset=0.0)
]

JOINT_LIMITS = np.array([
    [-154 * np.pi/180, 154 * np.pi/180],
    [0 * np.pi/180, 195 * np.pi/180],
    [-175 * np.pi/180, 0 * np.pi/180],
    [-102 * np.pi/180, 102 * np.pi/180],
    [-75 * np.pi/180, 75 * np.pi/180],
    [-170  * np.pi/180, 170 * np.pi/180]
])