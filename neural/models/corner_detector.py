"""Corner detector: template neurons over the LiDAR scan.

A room corner seen from the robot is a wall close on the front-left and a wall
close on the front-right at the same time.  Two "wall" neurons read each cone:
they respond sigmoidally to the closeness of the nearest return, multiplied by
a sigmoid of the coverage (the fraction of beams in the cone that hit something
not much farther than the wall threshold), because a wall fills the whole cone
and a briefcase or a person only part of it.  The corner
neuron multiplies them (an AND) and is gated by the distance travelled, so a
pair of briefcases near the start does not trigger it.  A leaky integrator
requires the pattern to persist before the corner is declared.

This is an engineered template, not a model taken from the literature.
"""

import numpy as np

from .leaky_integrator import LeakyIntegrator


def _sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


class CornerDetector:
    def __init__(self, cone=(np.radians(25), np.radians(65)), d_wall=1.4, slope=10.0,
                 min_travel=6.0, tau=0.5, min_coverage=0.85):
        self.cone = cone
        self.d_wall = d_wall
        self.slope = slope
        self.min_travel = min_travel
        self.min_coverage = min_coverage
        self.integrator = LeakyIntegrator(tau_rise=tau, tau_decay=tau, theta_on=0.8, theta_off=0.4)
        self.left = 0.0
        self.right = 0.0

    def _wall(self, angles, ranges, sign):
        lo, hi = self.cone
        a = sign * np.asarray(angles)
        m = (a > lo) & (a < hi)
        r = np.asarray(ranges)[m]
        if not r.size:
            return 0.0
        coverage = float(np.mean(np.isfinite(r) & (r < 1.6 * self.d_wall)))
        r = r[np.isfinite(r)]
        d = r.min() if r.size else np.inf
        return float(_sigmoid(self.slope * (self.d_wall - d)) * _sigmoid(30 * (coverage - self.min_coverage)))

    def step(self, dt, scan_angles, scan_ranges, travelled):
        self.left = self._wall(scan_angles, scan_ranges, +1)
        self.right = self._wall(scan_angles, scan_ranges, -1)
        gate = 1.0 if travelled >= self.min_travel else 0.0
        self.integrator.step(self.left * self.right * gate, dt)
        return self.integrator.u

    @property
    def detected(self):
        return self.integrator.active
