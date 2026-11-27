"""Kalman filter that tracks the guide person from their tag detections.

State: position and velocity of the person in the odometry frame
(``[x, y, vx, vy]``, constant-velocity model, Welch & Bishop 2006).  Working in
the odometry frame instead of the robot frame means the robot's own turning does
not look like the person moving.

The filter
* smooths the noisy tag poses,
* predicts where the person is during short occlusions (a walker crossing in
  between, the tag turning away), and
* rejects detections that are too far from the prediction (gating), which keeps
  a wrong detection from dragging the estimate away.
"""

import numpy as np


def to_odom(pose, x, y):
    """Point in the robot frame -> odometry frame, with the robot at ``pose = (x, y, yaw)``."""
    px, py, yaw = pose
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([px + c * x - s * y, py + s * x + c * y])


def to_robot(pose, p):
    """Point in the odometry frame -> robot frame."""
    px, py, yaw = pose
    dx, dy = p[0] - px, p[1] - py
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([c * dx + s * dy, -s * dx + c * dy])


class PersonTracker:
    """
    Parameters
    ----------
    accel_std  : how fast the person may change velocity [m/s^2] (process noise).
    meas_std   : tag position noise at 1 m [m]; it grows linearly with distance.
    gate       : squared Mahalanobis distance above which a detection is rejected
                 (13.8 = 99.9 % for 2 degrees of freedom).
    reset_time : after this long without a detection the next one starts a new track [s].
    max_speed  : the predicted velocity is clipped to this [m/s].
    """

    def __init__(self, accel_std=1.0, meas_std=0.05, gate=13.8, reset_time=3.0, max_speed=1.2):
        self.accel_std = accel_std
        self.meas_std = meas_std
        self.gate = gate
        self.reset_time = reset_time
        self.max_speed = max_speed
        self.x = None                 # state, None until the first detection
        self.P = None
        self.age = np.inf             # time since the last accepted detection [s]
        self.last_camera = None
        self.rejected = 0             # detections rejected by the gate (for debugging)

    @property
    def initialised(self):
        return self.x is not None

    def predict(self, dt):
        self.age += dt
        if self.x is None:
            return
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        q = self.accel_std ** 2
        G = np.array([[dt ** 2 / 2, 0], [0, dt ** 2 / 2], [dt, 0], [0, dt]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + G @ G.T * q
        v = np.linalg.norm(self.x[2:])
        if v > self.max_speed:
            self.x[2:] *= self.max_speed / v
        if self.age > self.reset_time:
            # Lost for long: stop extrapolating, keep the last position with a growing uncertainty.
            self.x[2:] = 0.0

    def update(self, pose, obs):
        """Fuse one TagObservation, taken with the robot at odometry ``pose``.  Returns True if accepted."""
        z = to_odom(pose, obs.x, obs.y)
        r = (self.meas_std * max(obs.distance, 0.3)) ** 2
        R = np.eye(2) * r
        if self.x is None or self.age > self.reset_time:
            self.x = np.array([z[0], z[1], 0.0, 0.0])
            self.P = np.diag([r, r, 0.5, 0.5])
        else:
            H = np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0]])
            innov = z - H @ self.x
            S = H @ self.P @ H.T + R
            if innov @ np.linalg.solve(S, innov) > self.gate:
                self.rejected += 1
                return False
            K = self.P @ H.T @ np.linalg.inv(S)
            self.x = self.x + K @ innov
            self.P = (np.eye(4) - K @ H) @ self.P
        self.age = 0.0
        self.last_camera = obs.camera
        return True

    def position(self):
        """Estimated position in the odometry frame, or None."""
        return None if self.x is None else self.x[:2].copy()

    def speed(self):
        return 0.0 if self.x is None else float(np.linalg.norm(self.x[2:]))

    def radial_speed(self, pose):
        """Speed of the person away from the robot [m/s] (negative when approaching)."""
        if self.x is None:
            return 0.0
        d = self.x[:2] - np.asarray(pose[:2])
        n = np.linalg.norm(d)
        return 0.0 if n < 1e-6 else float(self.x[2:] @ d / n)

    def relative(self, pose):
        """(distance [m], bearing [rad]) of the person from the robot, or (None, None)."""
        if self.x is None:
            return None, None
        p = to_robot(pose, self.x[:2])
        return float(np.hypot(*p)), float(np.arctan2(p[1], p[0]))
