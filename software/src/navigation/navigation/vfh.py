"""Obstacle avoidance with a Vector Field Histogram.

VFH (Borenstein & Koren 1991) builds a polar histogram of obstacles around the
robot, finds the *valleys* (runs of free sectors) and steers through the one
closest to the desired direction.  This version follows the refinements of
VFH+ (Ulrich & Borenstein 1998):

* every obstacle point is enlarged by the robot radius plus a safety margin, so
  a free sector means the whole robot fits;
* a sector is blocked with hysteresis (two thresholds) to avoid flicker;
* the direction is chosen with a cost that prefers the goal direction, the
  current heading and the previous choice.
"""

import numpy as np


class VFH:
    """
    Parameters
    ----------
    n_sectors     : number of histogram sectors (72 -> 5 degrees each).
    robot_radius  : radius of the circle that contains the robot [m].
    safety        : extra clearance added to every obstacle [m].
    d_max         : obstacles farther than this are ignored [m].
    d_block/d_free: a sector becomes blocked when an enlarged obstacle is closer
                    than d_block and free again when it is beyond d_free [m].
    wide          : valleys wider than this many sectors are "wide": the robot
                    steers ``wide // 2`` sectors inside the edge, not to its centre.
    mu            : cost weights (goal, current heading, previous direction).
    stop_dist     : speed is 0 when the chosen direction is blocked closer than this [m];
    slow_dist       full speed beyond slow_dist.
    """

    def __init__(self, n_sectors=72, robot_radius=0.2, safety=0.08, d_max=1.6, d_block=0.55,
                 d_free=0.7, wide=16, mu=(5.0, 2.0, 2.0), stop_dist=0.28, slow_dist=0.8):
        self.n = n_sectors
        self.res = 2 * np.pi / n_sectors
        self.centres = -np.pi + (np.arange(n_sectors) + 0.5) * self.res
        self.r_enl = robot_radius + safety
        self.d_max = d_max
        self.d_block = d_block
        self.d_free = d_free
        self.wide = wide
        self.mu = mu
        self.stop_dist = stop_dist
        self.slow_dist = slow_dist
        self.blocked = np.zeros(n_sectors, dtype=bool)
        self.clearance = np.full(n_sectors, np.inf)
        self.previous = 0.0

    def histogram(self, points):
        """Clearance per sector: distance from the robot centre to the nearest obstacle whose
        enlarged circle covers that sector."""
        clear = np.full(self.n, np.inf)
        if len(points):
            d = np.hypot(points[:, 0], points[:, 1])
            keep = d < self.d_max
            d, p = d[keep], points[keep]
            ang = np.arctan2(p[:, 1], p[:, 0])
            half = np.arcsin(np.clip(self.r_enl / np.maximum(d, 1e-3), 0, 1))
            for a, h, e in zip(ang, half, d):
                lo = int(np.floor((a - h + np.pi) / self.res))
                hi = int(np.floor((a + h + np.pi) / self.res))
                idx = np.arange(lo, hi + 1) % self.n
                clear[idx] = np.minimum(clear[idx], e)
        return clear

    def _sector(self, a):
        return int(np.floor((np.arctan2(np.sin(a), np.cos(a)) + np.pi) / self.res)) % self.n

    def __call__(self, points, target):
        """``points``: N x 2 obstacles in the robot frame; ``target``: desired direction [rad].

        Returns (direction [rad], speed factor in [0, 1]).  The direction is None
        when every sector is blocked (the caller should turn in place).
        """
        self.clearance = clear = self.histogram(points)
        # Hysteresis.
        self.blocked = np.where(clear < self.d_block, True, np.where(clear > self.d_free, False, self.blocked))
        free = ~self.blocked
        if not free.any():
            return None, 0.0
        if free.all():
            direction = target
        else:
            direction = self._choose(free, target)
        k = self._sector(direction)
        near = min(clear[(k + o) % self.n] for o in (-1, 0, 1))
        speed = float(np.clip((near - self.stop_dist) / (self.slow_dist - self.stop_dist), 0, 1))
        self.previous = direction
        return float(direction), speed

    def _choose(self, free, target):
        # Rotate so that index 0 is a blocked sector: valleys then never wrap around.
        start = int(np.argmin(free))
        f = np.roll(free, -start)
        candidates = []
        i = 0
        while i < self.n:
            if not f[i]:
                i += 1
                continue
            j = i
            while j + 1 < self.n and f[j + 1]:
                j += 1
            lo, hi = (i + start) % self.n, (j + start) % self.n
            width = j - i + 1
            if width >= self.wide:
                half = self.wide // 2
                candidates += [self.centres[(lo + half) % self.n], self.centres[(hi - half) % self.n]]
                # The target itself, if it lies inside this valley.
                kt = (self._sector(target) - start) % self.n
                if i + half <= kt <= j - half:
                    candidates.append(target)
            else:
                candidates.append(self.centres[(lo + width // 2) % self.n])
            i = j + 1

        def diff(a, b):
            return abs(np.arctan2(np.sin(a - b), np.cos(a - b)))

        cost = [self.mu[0] * diff(c, target) + self.mu[1] * diff(c, 0.0) + self.mu[2] * diff(c, self.previous)
                for c in candidates]
        return float(candidates[int(np.argmin(cost))])
