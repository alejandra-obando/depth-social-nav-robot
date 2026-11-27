"""Differential-drive kinematics (Siegwart, Nourbakhsh & Scaramuzza 2011, ch. 3).

    v     = r (w_r + w_l) / 2          w_l = (v - omega b / 2) / r
    omega = r (w_r - w_l) / b          w_r = (v + omega b / 2) / r

r is the wheel radius, b the track (distance between the wheel contact points)
and w_l, w_r the wheel speeds, positive when that wheel moves the robot forward.
The signs of each motor (one of the two is mounted mirrored) are handled by the
motor driver, not here.
"""

import numpy as np


class DiffDrive:
    def __init__(self, wheel_radius=0.0375, track=0.29, max_wheel_speed=8.0):
        self.r = wheel_radius
        self.b = track
        self.max_wheel_speed = max_wheel_speed      # rad/s

    def to_wheels(self, v, omega):
        """(v [m/s], omega [rad/s]) -> (w_left, w_right) [rad/s].

        If a wheel would exceed its limit both are scaled down by the same factor,
        which keeps the curvature of the path (the robot turns the same, only slower).
        """
        wl = (v - omega * self.b / 2) / self.r
        wr = (v + omega * self.b / 2) / self.r
        scale = max(1.0, abs(wl) / self.max_wheel_speed, abs(wr) / self.max_wheel_speed)
        return wl / scale, wr / scale

    def from_wheels(self, wl, wr):
        """(w_left, w_right) [rad/s] -> (v, omega)."""
        return self.r * (wr + wl) / 2, self.r * (wr - wl) / self.b

    def integrate(self, pose, v, omega, dt):
        """Dead reckoning: new (x, y, yaw) after moving with (v, omega) for dt."""
        x, y, th = pose
        if abs(omega) < 1e-6:
            return x + v * dt * np.cos(th), y + v * dt * np.sin(th), th
        th2 = th + omega * dt
        R = v / omega
        return x + R * (np.sin(th2) - np.sin(th)), y - R * (np.cos(th2) - np.cos(th)), wrap(th2)


def wrap(a):
    """Angle to (-pi, pi]."""
    return float(np.arctan2(np.sin(a), np.cos(a)))
