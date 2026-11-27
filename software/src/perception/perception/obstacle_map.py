"""Obstacle points around the robot, from the LiDAR and the depth camera.

* LiDAR points are used as they arrive (360 degrees, walls, legs, briefcases).
* Depth points are kept for ``depth_memory`` seconds in the odometry frame: the
  D415 only sees ~65 degrees ahead, so without memory a tablet stack would be
  forgotten as soon as the robot starts turning around it.  The memory is long
  (the stacks do not move, and the robot may stand still for a while waiting for
  the person); points farther than ``memory_range`` from the robot are dropped.
  Only obstacles the LiDAR does **not** see are remembered: a depth point with a
  LiDAR point within ``lidar_explains`` is part of something tall (a person, a
  briefcase, a wall) that the LiDAR keeps tracking live.  Without this rule, the
  lower body of people who walked by would stay behind as phantom obstacles.  Remembered points
  that fall inside the camera's view again are replaced by the new image, so a
  person who walked away does not leave a trail of phantom obstacles.
* Points close to the tracked person are removed, otherwise the robot would treat
  the person it follows as an obstacle to avoid.
"""

import numpy as np


class ObstacleMap:
    def __init__(self, depth_memory=20.0, memory_range=3.0, lidar_explains=0.25, person_radius=0.35, max_range=4.0,
                 voxel=0.05,
                 depth_origin=(0.16, 0.0), depth_fov=np.radians(58), depth_near=0.35, depth_far=2.5):
        self.depth_memory = depth_memory
        self.memory_range = memory_range
        self.lidar_explains = lidar_explains
        # Region the depth camera re-observes in every image, measured from the camera
        # (``depth_origin``, robot frame): inside it, memory is replaced by the new image.
        # Keep depth_fov a little narrower than the real one.  Closer than depth_near the
        # camera cannot see the floor, so remembered points there are kept.
        self.depth_origin = np.asarray(depth_origin, dtype=float)
        self.depth_fov = depth_fov
        self.depth_near = depth_near
        self.depth_far = depth_far
        self.person_radius = person_radius
        self.max_range = max_range
        self.voxel = voxel
        self._lidar = np.zeros((0, 2))         # robot frame
        self._depth = np.zeros((0, 2))         # odometry frame
        self._depth_t = np.zeros(0)
        self.t = 0.0

    def add_scan(self, angles, ranges):
        ok = np.isfinite(ranges) & (ranges < self.max_range)
        a, r = np.asarray(angles)[ok], np.asarray(ranges)[ok]
        self._lidar = np.stack([r * np.cos(a), r * np.sin(a)], 1)

    def add_depth(self, pose, angles, ranges):
        """Add one depth image's obstacle points (robot frame, polar) taken at odometry ``pose``."""
        if len(self._depth):
            rel = _to_robot_many(pose, self._depth) - self.depth_origin
            ang, rng = np.arctan2(rel[:, 1], rel[:, 0]), np.hypot(rel[:, 0], rel[:, 1])
            visible = (np.abs(ang) < self.depth_fov / 2) & (rng > self.depth_near) & (rng < self.depth_far)
            visible |= np.hypot(rel[:, 0] + self.depth_origin[0], rel[:, 1] + self.depth_origin[1]) > self.memory_range
            self._depth, self._depth_t = self._depth[~visible], self._depth_t[~visible]
        if len(ranges) == 0:
            return
        a, r = np.asarray(angles), np.asarray(ranges)
        x, y = r * np.cos(a), r * np.sin(a)
        if len(self._lidar):
            d2 = (x[:, None] - self._lidar[None, :, 0]) ** 2 + (y[:, None] - self._lidar[None, :, 1]) ** 2
            low = d2.min(1) > self.lidar_explains ** 2
            x, y = x[low], y[low]
            if len(x) == 0:
                return
        p = _to_odom_many(pose, x, y)
        # One point per voxel keeps the memory small.
        _, keep = np.unique(np.round(p / self.voxel).astype(int), axis=0, return_index=True)
        self._depth = np.concatenate([self._depth, p[keep]])
        self._depth_t = np.concatenate([self._depth_t, np.full(len(keep), self.t)])

    def step(self, dt):
        self.t += dt
        keep = self.t - self._depth_t < self.depth_memory
        self._depth, self._depth_t = self._depth[keep], self._depth_t[keep]

    def points(self, pose, person_xy=None):
        """All obstacle points in the robot frame (N x 2), without the person."""
        pts = np.concatenate([self._lidar, _to_robot_many(pose, self._depth)])
        if person_xy is not None and len(pts):
            pts = pts[np.hypot(pts[:, 0] - person_xy[0], pts[:, 1] - person_xy[1]) > self.person_radius]
        return pts


def _to_odom_many(pose, x, y):
    px, py, yaw = pose
    c, s = np.cos(yaw), np.sin(yaw)
    return np.stack([px + c * x - s * y, py + s * x + c * y], 1)


def _to_robot_many(pose, p):
    px, py, yaw = pose
    dx, dy = p[:, 0] - px, p[:, 1] - py
    c, s = np.cos(yaw), np.sin(yaw)
    return np.stack([c * dx + s * dy, -s * dx + c * dy], 1)
