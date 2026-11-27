"""Pinhole camera intrinsics and the camera's pose on the robot.

Frames used everywhere in this code:

* robot frame : x forward, y left, z up, origin at the robot centre on the floor
                plane (the point between the wheels).
* optical frame : the OpenCV / AprilTag convention, x right, y down, z out of the lens.
"""

from dataclasses import dataclass

import numpy as np

# Rotation that turns optical-frame vectors into robot-frame vectors for a camera that
# looks straight ahead: optical z (forward) -> robot x, optical x (right) -> robot -y,
# optical y (down) -> robot -z.
_OPTICAL_TO_FORWARD = np.array([[0.0, 0.0, 1.0],
                                [-1.0, 0.0, 0.0],
                                [0.0, -1.0, 0.0]])


@dataclass
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    @classmethod
    def from_fov(cls, width, height, fovy_deg):
        """Square pixels and a centred principal point, from the vertical field of view."""
        f = (height / 2) / np.tan(np.radians(fovy_deg) / 2)
        return cls(f, f, width / 2, height / 2, width, height)

    @property
    def params(self):
        """(fx, fy, cx, cy), the tuple pupil-apriltags expects."""
        return (self.fx, self.fy, self.cx, self.cy)


@dataclass
class Mount:
    """Where the camera sits on the robot: optical-frame vectors -> robot-frame vectors."""
    rotation: np.ndarray     # 3x3, robot_from_optical
    position: np.ndarray     # 3, lens position in the robot frame [m]

    @classmethod
    def from_angles(cls, x, y, z, yaw=0.0, pitch=0.0):
        """Camera at (x, y, z) in the robot frame, turned ``yaw`` to the left and tilted ``pitch`` up [rad].

        The rear C270 has yaw = pi.
        """
        cy, sy = np.cos(yaw), np.sin(yaw)
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        # Tilting up is a rotation of -pitch about the robot's y (left) axis.
        cp, sp = np.cos(-pitch), np.sin(-pitch)
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        return cls(Rz @ Ry @ _OPTICAL_TO_FORWARD, np.array([x, y, z], dtype=float))

    def to_robot(self, p_optical):
        """Points (N x 3 or 3) in the optical frame -> robot frame."""
        return np.asarray(p_optical) @ self.rotation.T + self.position
