"""The two motion controllers: following the person and driving to the start corner.

Both return a *desired direction* in the robot frame and a speed.  The direction
then goes through the VFH obstacle avoidance (``vfh.py``), which may change it,
before it becomes the angular speed.
"""

import numpy as np


class FollowController:
    """Phase 1: keep ``d_ref`` behind the person, heading towards them.

    Direction-following (Gockley, Forlizzi & Simmons 2007): the robot drives
    towards where the person *is*, not along the exact path they walked, which
    people judged more natural.  The linear speed is the person's own speed away
    from the robot (feed-forward, from the Kalman filter) plus a PD law on the
    distance error; without the feed-forward term a walking person would always
    be ``v_person / kp`` farther than ``d_ref``.  The speed is reduced while the
    robot is still turning towards the person.
    """

    def __init__(self, d_ref=0.9, kp=0.8, kd=0.1, v_max=0.22):
        self.d_ref = d_ref
        self.kp = kp
        self.kd = kd
        self.v_max = v_max
        self._prev = None

    def reset(self):
        self._prev = None

    def __call__(self, distance, bearing, dt, v_person=0.0):
        """``v_person``: the person's speed away from the robot [m/s].

        Returns (v [m/s], desired direction [rad])."""
        err = distance - self.d_ref
        d_err = 0.0 if self._prev is None or dt <= 0 else (err - self._prev) / dt
        self._prev = err
        v = max(v_person, 0.0) + self.kp * err + self.kd * d_err
        v *= max(0.0, np.cos(bearing))            # do not drive forward while facing away
        return float(np.clip(v, 0.0, self.v_max)), float(bearing)


class GoToGoal:
    """Phase 2: drive to a point (the start corner) in the odometry frame.

    Closed-loop steering of a unicycle in polar coordinates (Aicardi, Casalino,
    Bicchi & Balestrino 1995).  With rho the distance to the goal and alpha the
    angle between the robot's heading and the line to the goal, the Lyapunov
    function V = (rho^2 + alpha^2) / 2 decreases with

        v     = gamma cos(alpha) rho
        omega = k alpha + gamma cos(alpha) sin(alpha)

    The final orientation is not imposed (the person only needs the robot to stop
    at the corner), which is the paper's law with h = 0.
    """

    def __init__(self, gamma=0.5, k=1.2, v_max=0.18, v_min=0.05):
        self.gamma = gamma
        self.k = k
        self.v_max = v_max
        self.v_min = v_min

    def polar(self, pose, goal):
        x, y, th = pose
        dx, dy = goal[0] - x, goal[1] - y
        rho = float(np.hypot(dx, dy))
        alpha = float(np.arctan2(np.sin(np.arctan2(dy, dx) - th), np.cos(np.arctan2(dy, dx) - th)))
        return rho, alpha

    def __call__(self, rho, alpha):
        """Returns (v, omega) for the polar error (rho, alpha)."""
        v = self.gamma * np.cos(alpha) * rho
        v = float(np.clip(v, 0.0, self.v_max))
        if 0 < v < self.v_min and abs(alpha) < np.pi / 4:
            v = self.v_min                        # do not crawl the last centimetres
        omega = self.k * alpha + self.gamma * np.cos(alpha) * np.sin(alpha)
        return v, float(omega)
