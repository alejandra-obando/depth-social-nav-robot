"""Straight wall segments from a 2-D LiDAR scan with split-and-merge.

Split-and-merge was the fastest and one of the most accurate line extractors in
the comparison of Nguyen, Gächter, Martinelli, Tomatis & Siegwart (2007):

1. break the ordered scan where two consecutive points are far apart;
2. **split**: join the first and last point of a group with a line; if the
   farthest point is more than ``split_dist`` away, cut the group there and repeat;
3. **merge**: join consecutive segments that are collinear;
4. fit each segment by total least squares and keep the long ones.
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class Segment:
    p0: np.ndarray        # end points in the robot frame [m]
    p1: np.ndarray
    n_points: int
    rms: float            # fitting error [m]

    @property
    def length(self):
        return float(np.linalg.norm(self.p1 - self.p0))

    @property
    def direction(self):
        d = self.p1 - self.p0
        return d / max(np.linalg.norm(d), 1e-9)


def scan_to_points(angles, ranges, max_range=8.0):
    """Ordered (by angle) N x 2 array of the valid scan points."""
    a, r = np.asarray(angles, dtype=float), np.asarray(ranges, dtype=float)
    ok = np.isfinite(r) & (r > 0.05) & (r < max_range)
    a, r = a[ok], r[ok]
    order = np.argsort(a)
    a, r = a[order], r[order]
    return np.stack([r * np.cos(a), r * np.sin(a)], 1)


def _fit(points):
    """Total least squares line: (centroid, unit direction, rms distance)."""
    c = points.mean(0)
    _, s, vt = np.linalg.svd(points - c, full_matrices=False)
    rms = s[-1] / np.sqrt(len(points)) if len(s) > 1 else 0.0
    return c, vt[0], float(rms)


def _segment_from(points):
    c, d, rms = _fit(points)
    t = (points - c) @ d
    return Segment(c + d * t.min(), c + d * t.max(), len(points), rms)


def _split(points, split_dist, min_points):
    stack, out = [points], []
    while stack:
        p = stack.pop()
        if len(p) < min_points:
            continue
        a, b = p[0], p[-1]
        ab = b - a
        n = np.linalg.norm(ab)
        if n < 1e-9:
            continue
        dist = np.abs(ab[0] * (p[:, 1] - a[1]) - ab[1] * (p[:, 0] - a[0])) / n
        k = int(np.argmax(dist))
        if dist[k] > split_dist and min_points <= k <= len(p) - min_points:
            stack += [p[k:], p[:k + 1]]
        elif dist[k] > split_dist:
            # The farthest point is near one end: drop that end point and try again.
            stack.append(p[1:] if k < len(p) / 2 else p[:-1])
        else:
            out.append(p)
    return out


def _collinear(s1, s2, max_angle, max_dist):
    if abs(s1.direction @ s2.direction) < np.cos(max_angle):
        return False
    n = np.array([-s1.direction[1], s1.direction[0]])
    return abs((s2.p0 - s1.p0) @ n) < max_dist and abs((s2.p1 - s1.p0) @ n) < max_dist


def extract_segments(angles, ranges, split_dist=0.03, break_dist=0.15, min_points=6, min_length=0.4,
                     merge_angle=np.radians(6), merge_dist=0.05, max_range=8.0):
    """Wall segments of a scan, as a list of Segment (robot frame)."""
    pts = scan_to_points(angles, ranges, max_range)
    if len(pts) < min_points:
        return []
    # 1. Break the scan into groups of contiguous points.
    gaps = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    # The gap allowed between neighbours grows with range (beams diverge).
    rng = np.linalg.norm(pts[1:], axis=1)
    cut = np.where(gaps > np.maximum(break_dist, rng * 0.05))[0]
    groups = np.split(pts, cut + 1)
    # The scan wraps around at +-pi: join the last and the first group if they touch.
    if len(groups) > 1 and np.linalg.norm(groups[0][0] - groups[-1][-1]) < break_dist:
        groups[0] = np.concatenate([groups[-1], groups[0]])
        groups.pop()
    # 2. Split.
    pieces = []
    for g in groups:
        pieces += _split(g, split_dist, min_points)
    segs = [_segment_from(p) for p in pieces]
    # 3. Merge consecutive collinear segments (refit on the joined points).
    merged, merged_pts = [], []
    for s, p in zip(segs, pieces):
        if merged and _collinear(merged[-1], s, merge_angle, merge_dist):
            merged_pts[-1] = np.concatenate([merged_pts[-1], p])
            merged[-1] = _segment_from(merged_pts[-1])
        else:
            merged.append(s)
            merged_pts.append(p)
    # The scan starts and ends at -pi/+pi: the last and first segments may be one wall.
    if len(merged) > 1 and _collinear(merged[-1], merged[0], merge_angle, merge_dist) and \
            np.linalg.norm(merged_pts[-1][-1] - merged_pts[0][0]) < break_dist:
        merged_pts[0] = np.concatenate([merged_pts.pop(), merged_pts[0]])
        merged.pop()
        merged[0] = _segment_from(merged_pts[0])
    # 4. Keep the long ones.
    return [s for s in merged if s.length >= min_length]
