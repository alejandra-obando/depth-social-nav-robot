"""Frame conversions of the ROS node, without ROS (so they are unit tested)."""

import numpy as np

# An odometry jump bigger than this between two messages is a restart of the EKF
# (robot.launch.py respawns it, and it restarts at 0): the start frame is re-anchored.
ODOM_JUMP = 0.5


def scan_points(ranges, angle_min, increment, range_min, range_max, x=0.089, y=0.0, yaw=-1.5708,
                reversed=False, min_valid=0.06):
    """LaserScan -> (angles, ranges) from the robot centre (base_link).

    Same conversion as freno_lidar (wallens_safety): the LiDAR is at (x, y) with its zero
    turned ``yaw``; on the second-term robot an object straight ahead is read at 90 deg (yaw -90 deg).
    """
    r = np.asarray(ranges, dtype=float)
    a = angle_min + np.arange(len(r)) * increment
    if reversed:
        a = -a
    a = a + yaw
    ok = np.isfinite(r) & (r > max(range_min, min_valid)) & (r < range_max)
    px = x + r[ok] * np.cos(a[ok])
    py = y + r[ok] * np.sin(a[ok])
    return np.arctan2(py, px), np.hypot(px, py)


class StartFrame:
    """Odometry poses expressed in the frame of the first pose (the start, facing the far corner).

    If the odometry jumps (the EKF restarted at 0), the frame is re-anchored so that the
    pose continues from the last value instead of jumping.
    """

    def __init__(self, jump=ODOM_JUMP):
        self.jump = jump
        self.origin = None
        self.last_raw = None
        self.pose = None
        self.resets = 0

    @staticmethod
    def _relative(origin, raw):
        ox, oy, oyaw = origin
        c, s = np.cos(oyaw), np.sin(oyaw)
        dx, dy = raw[0] - ox, raw[1] - oy
        return (c * dx + s * dy, -s * dx + c * dy, float(np.arctan2(np.sin(raw[2] - oyaw), np.cos(raw[2] - oyaw))))

    def update(self, raw):
        if self.origin is None:
            self.origin = raw
        elif np.hypot(raw[0] - self.last_raw[0], raw[1] - self.last_raw[1]) > self.jump:
            # Choose the origin that maps the new raw pose onto the last start-frame pose.
            px, py, pyaw = self.pose
            oyaw = raw[2] - pyaw
            c, s = np.cos(oyaw), np.sin(oyaw)
            self.origin = (raw[0] - (c * px - s * py), raw[1] - (s * px + c * py), oyaw)
            self.resets += 1
        self.last_raw = raw
        self.pose = self._relative(self.origin, raw)
        return self.pose
