"""Attractor dynamics of heading direction and forward speed.

Heading (Schoner, Dose & Engels 1995; Bicho, Mallet & Schoner 2000):

    d(phi)/dt = f_target + sum_i f_obstacle,i + f_stoch

* The target (the person in phase 1, the start corner in phase 2) is an
  attractor of the heading:  f_target = lam_tar * sin(theta_tar),
  where theta_tar is the target direction relative to the current heading.
* Each obstacle is a repellor whose strength decays with distance and whose
  range covers the obstacle's angular width plus the robot's own size:
      f_obs = lam * (-theta) * exp(-theta^2 / (2 sigma^2))
      lam   = beta1 * exp(-d / beta2)
      sigma = half_width + atan(tan(dtheta / 2) + R / (R + d))
  theta is the obstacle centre direction relative to the heading and d its
  closest distance.

Obstacles are clusters of contiguous occupied sectors.  Summing one small
repellor per sector (one per 5 deg) does not work with a fine sensor: for a wide
obstacle straight ahead the left and right halves cancel, and between two
obstacles the robot is always centred in the gap, even one narrower than itself.
One repellor per cluster gives the behaviour described by Schoner, Dose & Engels
(1995): a single wide repellor in front turns the robot away, and two obstacles
whose repellors overlap act as one (bifurcation from "pass through" to "go around").

Gap closing: before clustering, free sectors between two obstacles that leave
less room than the robot's width are treated as blocked, so the two obstacles
become one cluster.

f_stoch is a small random force: it lets the heading escape a repellor that lies
exactly in front of the robot (an unstable fixed point), as in Schoner et al. (1995).

The output is the turning rate omega (rad/s).  Angles are in the robot frame:
0 = straight ahead, positive = to the left.

Forward speed: a speed neuron excited by the distance error to the person and
inhibited by close obstacles in front (see ``forward_speed``).
"""

import numpy as np


class HeadingDynamics:
    def __init__(self, lam_target=1.2, beta1=8.0, beta2=0.6, robot_radius=0.2,
                 sector_width=np.radians(5), max_range=1.5, field_of_action=np.radians(100),
                 omega_max=1.2, noise=0.05, seed=0):
        self.lam_target = lam_target
        self.beta1 = beta1
        self.beta2 = beta2
        self.robot_radius = robot_radius
        self.sector_width = sector_width
        self.max_range = max_range
        self.field_of_action = field_of_action
        self.omega_max = omega_max
        self.noise = noise
        self.rng = np.random.default_rng(seed)

    def close_gaps(self, angles, dists, max_span=np.radians(90)):
        """Fill free sectors that lie in a gap narrower than the robot."""
        angles = np.asarray(angles, dtype=float)
        dists = np.asarray(dists, dtype=float).copy()
        occ = np.isfinite(dists) & (dists < self.max_range)
        if occ.sum() < 2:
            return dists
        x, y = dists * np.cos(angles), dists * np.sin(angles)
        n = len(angles)
        width = 2 * self.robot_radius
        filled = dists.copy()
        idx = np.flatnonzero(occ)
        for a, b in zip(idx, np.roll(idx, -1)):          # consecutive obstacle sectors around the ring
            gap = (b - a) % n
            if gap <= 1 or gap * 2 * np.pi / n > max_span:
                continue
            # The sampled points sit up to one sector inside the real edges: allow for it.
            allowance = 2 * min(dists[a], dists[b]) * np.tan(self.sector_width)
            if np.hypot(x[a] - x[b], y[a] - y[b]) < width + allowance:
                fill = (a + np.arange(1, gap)) % n
                filled[fill] = min(dists[a], dists[b])
        return filled

    def target_force(self, theta_tar, strength=1.0):
        theta_tar = float(np.arctan2(np.sin(theta_tar), np.cos(theta_tar)))
        # sin() has an unstable point exactly behind the robot; behind it, turn at full rate.
        if abs(theta_tar) <= np.pi / 2:
            g = np.sin(theta_tar)
        else:
            g = np.sign(theta_tar)
        return strength * self.lam_target * g

    def clusters(self, angles, dists, split=0.4):
        """Group contiguous occupied sectors (inside the field of action) into obstacles.

        Returns a list of index arrays.  A jump in distance larger than ``split`` metres
        starts a new obstacle (e.g. a stack in front of a farther wall).
        """
        angles = np.asarray(angles, dtype=float)
        dists = np.asarray(dists, dtype=float)
        order = np.argsort(angles)
        out, cur = [], []
        for i in order:
            ok = np.isfinite(dists[i]) and dists[i] < self.max_range and abs(angles[i]) < self.field_of_action
            if ok and cur and abs(dists[i] - dists[cur[-1]]) <= split:
                cur.append(i)
            else:
                if cur:
                    out.append(np.array(cur))
                cur = [i] if ok else []
        if cur:
            out.append(np.array(cur))
        return out

    def obstacle_forces(self, angles, dists, gains=None):
        """One repulsive force per obstacle cluster; returns (forces, repellors).

        ``repellors`` is a list of (psi, sigma, lam) for plotting and debugging.
        """
        angles = np.asarray(angles, dtype=float)
        dists = np.asarray(dists, dtype=float)
        forces, reps = [], []
        for c in self.clusters(angles, dists):
            a = angles[c]
            d = max(float(dists[c].min()), 0.05)
            psi = 0.5 * (a.min() + a.max())
            half = 0.5 * (a.max() - a.min())
            sigma = half + np.arctan(np.tan(self.sector_width / 2) + self.robot_radius / (self.robot_radius + d))
            lam = self.beta1 * np.exp(-d / self.beta2)
            if gains is not None:
                lam *= float(np.max(np.asarray(gains, dtype=float)[c]))
            forces.append(lam * (-psi) * np.exp(-psi ** 2 / (2 * sigma ** 2)))
            reps.append((psi, sigma, lam))
        return np.array(forces), reps

    def omega(self, theta_tar, angles, dists, target_strength=1.0, gains=None):
        dists = self.close_gaps(angles, dists)
        w = self.target_force(theta_tar, target_strength) + self.obstacle_forces(angles, dists, gains)[0].sum()
        w += self.noise * self.rng.standard_normal()
        return float(np.clip(w, -self.omega_max, self.omega_max))


def clearance(angles, dists, half_width=np.radians(30), d_stop=0.25, d_slow=0.7):
    """0 when something is closer than d_stop in front, 1 when nothing is closer than d_slow."""
    angles = np.asarray(angles)
    dists = np.asarray(dists)
    front = np.abs(angles) < half_width
    if not front.any():
        return 1.0
    d = np.min(np.where(np.isfinite(dists[front]), dists[front], np.inf))
    return float(np.clip((d - d_stop) / (d_slow - d_stop), 0.0, 1.0))


def forward_speed(distance, d_ref=0.9, gain=1.0, v_max=0.22, theta_tar=0.0,
                  clearance_factor=1.0, inhibition=0.0):
    """Speed neuron: rectified excitation by the distance error, shunted by inhibition.

    v = v_max * clip(gain * (distance - d_ref) / v_max, 0, 1) * cos+(theta) * clearance * (1 - inhibition)
    """
    excitation = np.clip(gain * (distance - d_ref), 0.0, v_max)
    alignment = max(0.0, np.cos(theta_tar))
    return float(excitation * alignment * clearance_factor * (1.0 - np.clip(inhibition, 0.0, 1.0)))
