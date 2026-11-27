"""Array of Hassenstein-Reichardt elementary motion detectors over the LiDAR scan.

Input: a proximity signal per angular sector, p_i = clip(1 - r_i / r_max, 0, 1).
Each detector correlates the delayed signal of one sector with the current
signal of its neighbour, and subtracts the mirror term:

    R_i = p'_i * p_{i+1} - p_i * p'_{i+1}        (p' = signal one scan earlier)

R_i > 0: something moves towards larger angles (counter-clockwise, to the left).
R_i < 0: it moves towards smaller angles (to the right).
A static scene gives R = 0.  The responses are smoothed by a low-pass filter.

Ego-motion: when the robot itself moves, static objects also move in the scan.
The previous scan is kept as raw points, moved into the current robot frame with
the odometry displacement and only then grouped into sectors, so only objects
that really move in the world give a response.  (Moving already-binned sector
signals does not work: per step the robot moves less than one sector and the
shift is lost to rounding.)

Known limitation: at the edges of nearby objects the moved previous points and
the current beams sample different spots, so the compensation does not yet
remove the response to the robot's own motion (see exp_motion_detector, check 4).
In the closed loop, a habituation neuron in SocialNavNet keeps this from
blocking the robot.

References: Hassenstein & Reichardt (1956); Borst & Euler (2011).
"""

import numpy as np


class ReichardtArray:
    def __init__(self, n_sectors=72, r_max=4.0, tau_out=0.3):
        self.n = n_sectors
        # Sector centres (binning uses floor, so the centre is half a sector after the edge).
        self.angles = (np.arange(n_sectors) + 0.5) * 2 * np.pi / n_sectors - np.pi
        self.r_max = r_max
        self.tau_out = tau_out
        self.prev_points = None      # previous scan as (x, y) points in its robot frame
        self.out = np.zeros(n_sectors)

    def _bin(self, angles, ranges):
        """Closest return per sector, converted to proximity."""
        idx = ((np.asarray(angles) + np.pi) / (2 * np.pi) * self.n).astype(int) % self.n
        r = np.full(self.n, np.inf)
        np.minimum.at(r, idx, np.asarray(ranges, dtype=float))
        return np.clip(1.0 - r / self.r_max, 0.0, 1.0)

    def proximity(self, scan_angles, scan_ranges):
        """Bin a scan into sectors (closest return per sector) and convert to proximity."""
        scan_ranges = np.asarray(scan_ranges, dtype=float)
        return self._bin(scan_angles, np.where(np.isfinite(scan_ranges), scan_ranges, np.inf))

    @staticmethod
    def _points(scan_angles, scan_ranges):
        a = np.asarray(scan_angles, dtype=float)
        r = np.asarray(scan_ranges, dtype=float)
        ok = np.isfinite(r)
        return np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], 1)

    def step(self, dt, scan_angles, scan_ranges, ego=(0.0, 0.0, 0.0)):
        """Advance by ``dt`` (time since the previous scan) with a new scan.

        ``ego``: robot displacement since the previous scan, (dx, dy, dyaw), in the previous robot frame.
        Returns the smoothed signed motion response per sector.
        """
        pts = self._points(scan_angles, scan_ranges)
        p = self._bin(np.arctan2(pts[:, 1], pts[:, 0]), np.hypot(pts[:, 0], pts[:, 1])) if len(pts) \
            else np.zeros(self.n)
        if self.prev_points is None:
            self.prev_points = pts
            return self.out.copy()
        dx, dy, dyaw = ego
        q = self.prev_points - [dx, dy]
        c, s = np.cos(-dyaw), np.sin(-dyaw)
        q = np.stack([c * q[:, 0] - s * q[:, 1], s * q[:, 0] + c * q[:, 1]], 1)
        p_prev = self._bin(np.arctan2(q[:, 1], q[:, 0]), np.hypot(q[:, 0], q[:, 1])) if len(q) \
            else np.zeros(self.n)
        r = p_prev * np.roll(p, -1) - p * np.roll(p_prev, -1)
        k = 1 - np.exp(-dt / self.tau_out)
        self.out += k * (r - self.out)
        self.prev_points = pts
        return self.out.copy()

    def motion_energy(self):
        """Unsigned motion per sector, spread to both neighbours of each detector."""
        e = np.abs(self.out)
        return np.maximum(e, np.roll(e, 1))
