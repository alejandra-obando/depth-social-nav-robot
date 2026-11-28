import numpy as np

from bringup.checks import describe_lidar, describe_points, describe_tags
from perception.apriltag_detector import TagObservation


def test_describe_tags_gives_distance_from_the_edge():
    t = TagObservation(1, "rear", -1.164, 0.0, 1.1, 80.0)
    line = describe_tags([t], "rear")
    assert "ID 1" in line and "1.16 m from centre" in line and "1.00 m from the rear edge" in line
    assert describe_tags([], "front") == "no tag"


def test_describe_points_and_lidar_corner():
    assert describe_points(np.array([0.1, -0.2]), np.array([0.8, 0.5])) == "2 points, nearest 0.50 m at -11 deg"
    # Robot 1 m from two walls meeting in front of it (x = 1 and y = 1): one corner at 45 deg.
    s = np.linspace(-1.0, 1.0, 81)
    pts = np.vstack([np.c_[np.ones_like(s), s], np.c_[s, np.ones_like(s)]])
    line = describe_lidar(np.arctan2(pts[:, 1], pts[:, 0]), np.hypot(pts[:, 0], pts[:, 1]))
    assert "corners: 1.41 m at +45 deg" in line
