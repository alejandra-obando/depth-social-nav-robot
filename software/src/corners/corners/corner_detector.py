"""Room corners from wall segments, and their confirmation over several scans.

A room corner, seen from inside the room, is two long wall segments that are
nearly perpendicular and whose lines meet close to their ends; the robot lies
inside the angle they form.  Box-shaped obstacles (briefcases) give convex
corners, with the robot outside the angle, and short sides, so they are rejected.
The person standing in the corner hides its tip, which is why the segments are
only required to end *near* the intersection, not at it.
"""

from dataclasses import dataclass

import numpy as np

from .line_extraction import Segment, extract_segments


@dataclass
class Corner:
    x: float              # intersection of the two walls, robot frame [m]
    y: float
    angle: float          # angle between the walls [rad]
    walls: tuple          # the two Segment objects

    @property
    def distance(self):
        return float(np.hypot(self.x, self.y))

    @property
    def bearing(self):
        return float(np.arctan2(self.y, self.x))


def _intersection(s1: Segment, s2: Segment):
    d1, d2 = s1.direction, s2.direction
    A = np.array([d1, -d2]).T
    if abs(np.linalg.det(A)) < 1e-6:
        return None
    t = np.linalg.solve(A, s2.p0 - s1.p0)
    return s1.p0 + d1 * t[0]


def _away(seg, p):
    """Unit vector from ``p`` towards the far end of ``seg``, and the gap from ``p`` to its near end."""
    d0, d1 = np.linalg.norm(seg.p0 - p), np.linalg.norm(seg.p1 - p)
    far, gap = (seg.p1, d0) if d0 < d1 else (seg.p0, d1)
    v = far - p
    return v / max(np.linalg.norm(v), 1e-9), gap


def find_corners(segments, angle_tol=np.radians(20), max_gap=0.8, min_wall=0.5):
    """Inner room corners among ``segments`` (robot frame)."""
    out = []
    for i in range(len(segments)):
        for j in range(i + 1, len(segments)):
            s1, s2 = segments[i], segments[j]
            if s1.length < min_wall or s2.length < min_wall:
                continue
            ang = np.arccos(np.clip(abs(s1.direction @ s2.direction), 0, 1))
            if abs(ang - np.pi / 2) > angle_tol:
                continue
            p = _intersection(s1, s2)
            if p is None:
                continue
            u1, g1 = _away(s1, p)
            u2, g2 = _away(s2, p)
            if g1 > max_gap or g2 > max_gap:
                continue
            # Inner corner: the robot (origin) lies inside the angle between u1 and u2.
            w = -p
            c12 = u1[0] * u2[1] - u1[1] * u2[0]
            c1w = u1[0] * w[1] - u1[1] * w[0]
            cw2 = w[0] * u2[1] - w[1] * u2[0]
            if not (np.sign(c1w) == np.sign(c12) and np.sign(cw2) == np.sign(c12)):
                continue
            out.append(Corner(float(p[0]), float(p[1]), float(ang), (s1, s2)))
    return out


class CornerConfirmer:
    """Declares a corner recognised when it is seen in ``n_confirm`` consecutive scans.

    Parameters
    ----------
    max_distance : the corner must be closer than this to the robot [m].
    max_bearing  : and within this angle of the robot's forward axis [rad]
                   (use pi to accept corners all around).
    expected     : optional (x, y) where the corner should be in the odometry
                   frame; corners farther than ``expected_radius`` from it are
                   ignored.  This keeps a pair of obstacles in the middle of the
                   room from being taken as the corner, with a tolerance large
                   enough for the odometry drift.
    """

    def __init__(self, max_distance=1.8, max_bearing=np.radians(50), n_confirm=5, expected=None,
                 expected_radius=2.5, **find_kw):
        self.max_distance = max_distance
        self.max_bearing = max_bearing
        self.n_confirm = n_confirm
        self.expected = None if expected is None else np.asarray(expected, dtype=float)
        self.expected_radius = expected_radius
        self.find_kw = find_kw
        self.count = 0
        self.last = None          # last accepted Corner
        self.last_odom = None     # its position in the odometry frame

    def reset(self):
        self.count = 0
        self.last = self.last_odom = None

    def update(self, angles, ranges, pose):
        """Process one scan taken at odometry ``pose``.  Returns True once the corner is confirmed."""
        best = None
        for c in find_corners(extract_segments(angles, ranges), **self.find_kw):
            if c.distance > self.max_distance or abs(c.bearing) > self.max_bearing:
                continue
            px, py, yaw = pose
            co, si = np.cos(yaw), np.sin(yaw)
            odom = np.array([px + co * c.x - si * c.y, py + si * c.x + co * c.y])
            if self.expected is not None and np.linalg.norm(odom - self.expected) > self.expected_radius:
                continue
            if best is None or c.distance < best[0].distance:
                best = (c, odom)
        if best is None:
            self.count = 0
        else:
            self.count += 1
            self.last, self.last_odom = best
        return self.confirmed

    @property
    def confirmed(self):
        return self.count >= self.n_confirm
