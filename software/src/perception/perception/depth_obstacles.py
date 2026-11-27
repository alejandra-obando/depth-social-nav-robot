"""Low obstacles from the D415 depth image.

The LiDAR scans a plane ~16 cm above the floor, so it does not see the stacks of
tablets (~10 cm).  The depth image fills that gap: every pixel becomes a 3-D
point in the robot frame, and points between ``min_height`` and ``max_height``
above the floor are obstacles.
"""

import numpy as np

from .camera import Intrinsics, Mount


class DepthObstacles:
    """
    Parameters
    ----------
    intrinsics : intrinsics of the depth image.
    mount      : pose of the depth camera on the robot.
    min_height : points lower than this are the floor [m]; keep it above the floor noise.
    max_height : points higher than this cannot hit the robot [m].
    max_range  : depth beyond this is ignored [m].
    stride     : use one pixel out of ``stride`` in each direction (speed).
    """

    def __init__(self, intrinsics: Intrinsics, mount: Mount, min_height=0.03, max_height=0.45,
                 min_range=0.15, max_range=3.0, stride=4):
        self.mount = mount
        self.min_height = min_height
        self.max_height = max_height
        self.min_range = min_range
        self.max_range = max_range
        self.stride = stride
        h, w = intrinsics.height, intrinsics.width
        v, u = np.mgrid[0:h:stride, 0:w:stride]
        # Ray directions in the optical frame for a depth of 1 m (z = 1).
        self._rays = np.stack([(u - intrinsics.cx) / intrinsics.fx,
                               (v - intrinsics.cy) / intrinsics.fy,
                               np.ones_like(u, dtype=float)], -1).reshape(-1, 3)

    def points(self, depth):
        """``depth``: H x W array of z-depth in metres (0 or nan = no data).

        Returns obstacle points ``(angles, ranges)`` around the robot centre in the
        horizontal plane (angle positive to the left).
        """
        z = np.asarray(depth, dtype=float)[::self.stride, ::self.stride].reshape(-1)
        ok = np.isfinite(z) & (z > self.min_range) & (z < self.max_range)
        p = self.mount.to_robot(self._rays[ok] * z[ok, None])
        keep = (p[:, 2] > self.min_height) & (p[:, 2] < self.max_height)
        p = p[keep]
        return np.arctan2(p[:, 1], p[:, 0]), np.hypot(p[:, 0], p[:, 1])
