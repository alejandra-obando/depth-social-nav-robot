import numpy as np

from bringup.frames import StartFrame, scan_points


def test_scan_points_second_term_lidar():
    # On the second-term robot an object straight ahead is measured at 90 deg by the LD19
    # (robot angle = LiDAR angle - 90 deg), and the LiDAR is 89 mm in front of the wheel axle.
    n = 360
    ranges = np.full(n, np.inf)
    ranges[90] = 1.0                     # 1 m straight ahead of the LiDAR
    ranges[180] = 0.5                    # 180 deg of the LiDAR: the robot's left
    a, r = scan_points(ranges, 0.0, 2 * np.pi / n, 0.02, 12.0)
    order = np.argsort(r)
    assert np.isclose(a[order[1]], 0.0, atol=1e-4) and np.isclose(r[order[1]], 1.089)
    left = order[0]
    assert np.isclose(a[left], np.arctan2(0.5, 0.089), atol=1e-4)


def test_scan_points_drops_invalid():
    a, r = scan_points([0.0, 0.03, np.nan, 20.0, 1.0], 0.0, 0.1, 0.02, 12.0, x=0.0, yaw=0.0)
    assert len(r) == 1 and np.isclose(r[0], 1.0)


def test_start_frame_relative_and_ekf_restart():
    f = StartFrame()
    assert f.update((2.0, 1.0, np.pi / 2)) == (0.0, 0.0, 0.0)
    for k in range(1, 11):                               # 1 m forward along the start heading, 10 cm steps
        x, y, yaw = f.update((2.0, 1.0 + 0.1 * k, np.pi / 2))
    assert np.allclose((x, y, yaw), (1.0, 0.0, 0.0)) and f.resets == 0
    # The EKF restarts at 0: the pose continues from (1, 0, 0) instead of jumping.
    assert np.allclose(f.update((0.0, 0.0, 0.0)), (1.0, 0.0, 0.0)) and f.resets == 1
    assert np.allclose(f.update((0.1, 0.0, 0.0)), (1.1, 0.0, 0.0), atol=1e-9)
