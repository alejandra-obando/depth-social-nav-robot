import numpy as np

from corners.corner_detector import CornerConfirmer, find_corners
from corners.line_extraction import extract_segments


def room_scan(robot, yaw, room=7.0, boxes=(), n=360, noise=0.0, seed=0):
    """Ray-cast a square room (and axis-aligned boxes) from ``robot`` with heading ``yaw``."""
    rng = np.random.default_rng(seed)
    angles = -np.pi + np.arange(n) * 2 * np.pi / n
    ranges = np.full(n, np.inf)
    walls = [((0, 0), (room, 0)), ((room, 0), (room, room)), ((room, room), (0, room)), ((0, room), (0, 0))]
    for (cx, cy, hx, hy) in boxes:
        c = [(cx - hx, cy - hy), (cx + hx, cy - hy), (cx + hx, cy + hy), (cx - hx, cy + hy)]
        walls += [(c[i], c[(i + 1) % 4]) for i in range(4)]
    o = np.asarray(robot, float)
    for i, a in enumerate(angles):
        d = np.array([np.cos(a + yaw), np.sin(a + yaw)])
        for p, q in walls:
            p, q = np.asarray(p, float), np.asarray(q, float)
            e = q - p
            A = np.array([d, -e]).T
            if abs(np.linalg.det(A)) < 1e-9:
                continue
            t, s = np.linalg.solve(A, p - o)
            if t > 0 and 0 <= s <= 1:
                ranges[i] = min(ranges[i], t)
    return angles, ranges + rng.normal(0, noise, n)


def test_segments_of_a_room():
    a, r = room_scan((3.5, 3.5), 0.0)
    segs = extract_segments(a, r)
    assert len(segs) == 4
    assert all(abs(s.length - 7.0) < 0.3 for s in segs)


def test_corner_ahead_is_found():
    # Robot near the (7, 7) corner, facing it.
    a, r = room_scan((5.8, 5.8), np.pi / 4, noise=0.01)
    corners = find_corners(extract_segments(a, r))
    near = min(corners, key=lambda c: c.distance)
    assert abs(near.distance - 1.2 * np.sqrt(2)) < 0.1
    assert abs(near.bearing) < np.radians(5)


def test_briefcase_is_not_a_corner():
    # A briefcase (45 x 15 cm) in front of the robot, in the middle of the room.
    a, r = room_scan((3.5, 3.5), 0.0, boxes=[(4.3, 3.5, 0.075, 0.225)])
    corners = [c for c in find_corners(extract_segments(a, r)) if c.distance < 2.0]
    assert corners == []


def test_confirmer_needs_consecutive_scans():
    conf = CornerConfirmer(max_distance=1.8, n_confirm=3)
    far = room_scan((3.5, 3.5), np.pi / 4)
    near = room_scan((6.0, 6.0), np.pi / 4)
    assert not conf.update(*far, (0, 0, 0))
    assert not conf.update(*near, (0, 0, 0))
    assert not conf.update(*near, (0, 0, 0))
    assert conf.update(*near, (0, 0, 0))
    assert not conf.update(*far, (0, 0, 0))        # a scan without it resets the count
