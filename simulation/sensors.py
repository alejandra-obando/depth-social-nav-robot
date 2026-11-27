"""Simulated sensors, computed with MuJoCo rays.

All outputs are expressed relative to the robot centre in the robot frame
(x forward, y left, angles positive to the left), like the real robot after its
TF transforms.

* LD19 LiDAR      : 360 beams in the horizontal plane at the LiDAR height (~16 cm), 10 Hz.
* D415 depth      : grid of rays inside its 65 x 40 deg field of view; points
                    above the floor become obstacle points (sees the 10 cm stacks).
* Sign cameras    : D415 colour (front) and C270 (rear).  The sign is "seen" when
                    its centre is inside the field of view, it is within range, it
                    faces the camera and nothing blocks the line of sight.  This is
                    a geometric model of the detector: the real QR/pattern detection
                    is tested on the robot.
* Odometry        : wheel encoders for distance, BNO055-like yaw with drift.
"""

import mujoco
import numpy as np

from .world import robot_pose

GROUP_WORLD = np.array([0, 1, 0, 0, 0, 0], dtype=np.uint8)
GROUP_WORLD_SIGNS = np.array([0, 1, 1, 0, 0, 0], dtype=np.uint8)

WHEEL_RADIUS = 0.0375
TRACK = 0.29          # effective, measured by spinning the robot in place


def _to_robot_frame(points, pose):
    x, y, yaw = pose
    dx, dy = points[:, 0] - x, points[:, 1] - y
    c, s = np.cos(-yaw), np.sin(-yaw)
    return c * dx - s * dy, s * dx + c * dy


class Lidar:
    def __init__(self, model, n_beams=360, max_range=8.0, noise=0.01, dropout=0.02, seed=0):
        self.n = n_beams
        self.max_range = max_range
        self.noise = noise
        self.dropout = dropout
        self.rng = np.random.default_rng(seed)
        mesh = model.mesh("ld19_middle").id
        self.geom = [i for i in range(model.ngeom) if model.geom_dataid[i] == mesh][0]
        self.beam = np.arange(n_beams) * 2 * np.pi / n_beams - np.pi
        self._gid = np.zeros(n_beams, dtype=np.int32)
        self._dist = np.zeros(n_beams)

    def read(self, model, data):
        pose = robot_pose(model, data)
        origin = data.geom_xpos[self.geom].copy()
        origin[2] = max(origin[2], 0.16)
        world = self.beam + pose[2]
        vec = np.stack([np.cos(world), np.sin(world), np.zeros(self.n)], 1).ravel()
        mujoco.mj_multiRay(model, data, origin, vec, GROUP_WORLD, 1, -1,
                           self._gid, self._dist, None, self.n, self.max_range)
        hit = (self._dist > 0) & (self._dist < self.max_range)
        hit &= self.rng.random(self.n) > self.dropout
        d = self._dist + self.rng.normal(0, self.noise, self.n)
        pts = origin[:2] + d[:, None] * np.stack([np.cos(world), np.sin(world)], 1)
        xr, yr = _to_robot_frame(pts, pose)
        ang = np.arctan2(yr, xr)
        rng = np.where(hit, np.hypot(xr, yr), np.inf)
        return ang, rng


class DepthCamera:
    def __init__(self, model, camera="d415_prof", fov=(65, 40), grid=(33, 11), max_range=3.0,
                 min_height=0.02, noise=0.01, seed=0):
        self.cam = model.camera(camera).id
        self.max_range = max_range
        self.min_height = min_height
        self.noise = noise
        self.rng = np.random.default_rng(seed)
        ax = np.radians(np.linspace(-fov[0] / 2, fov[0] / 2, grid[0]))
        ay = np.radians(np.linspace(-fov[1] / 2, fov[1] / 2, grid[1]))
        AX, AY = np.meshgrid(ax, ay)
        d = np.stack([np.tan(AX), np.tan(AY), -np.ones_like(AX)], -1).reshape(-1, 3)
        self.dirs = d / np.linalg.norm(d, axis=1, keepdims=True)
        self.n = len(self.dirs)
        self._gid = np.zeros(self.n, dtype=np.int32)
        self._dist = np.zeros(self.n)

    def read(self, model, data):
        pose = robot_pose(model, data)
        R = data.cam_xmat[self.cam].reshape(3, 3)
        origin = data.cam_xpos[self.cam].copy()
        vec = (self.dirs @ R.T)
        mujoco.mj_multiRay(model, data, origin, vec.ravel(), GROUP_WORLD, 1, -1,
                           self._gid, self._dist, None, self.n, self.max_range)
        hit = (self._dist > 0) & (self._dist < self.max_range)
        d = self._dist[hit] * (1 + self.rng.normal(0, self.noise, hit.sum()))
        pts = origin + d[:, None] * vec[hit]
        pts = pts[pts[:, 2] > self.min_height]
        if len(pts) == 0:
            return np.zeros(0), np.zeros(0)
        xr, yr = _to_robot_frame(pts, pose)
        return np.arctan2(yr, xr), np.hypot(xr, yr)


class SignCamera:
    def __init__(self, model, camera, sign_geom, fov=(69, 42), min_range=0.3, max_range=3.0,
                 max_view_angle=60, dropout=0.1, bearing_noise=1.5, dist_noise=0.03, seed=0):
        self.cam = model.camera(camera).id
        self.sign = model.geom(sign_geom).id
        self.half_fov = np.radians(fov) / 2
        self.min_range = min_range
        self.max_range = max_range
        self.cos_view = np.cos(np.radians(max_view_angle))
        self.dropout = dropout
        self.bearing_noise = np.radians(bearing_noise)
        self.dist_noise = dist_noise
        self.rng = np.random.default_rng(seed)
        self._gid = np.zeros(1, dtype=np.int32)
        self._dist = np.zeros(1)

    def visible(self, model, data):
        """Geometric visibility of the sign (no noise)."""
        cam_p = data.cam_xpos[self.cam]
        R = data.cam_xmat[self.cam].reshape(3, 3)
        p = data.geom_xpos[self.sign]
        c = R.T @ (p - cam_p)
        if c[2] >= 0:
            return False
        ax = np.arctan2(c[0], -c[2])
        ay = np.arctan2(c[1], -c[2])
        dist = np.linalg.norm(c)
        if abs(ax) > self.half_fov[0] or abs(ay) > self.half_fov[1]:
            return False
        if not (self.min_range <= dist <= self.max_range):
            return False
        normal = data.geom_xmat[self.sign].reshape(3, 3)[:, 0]
        to_cam = (cam_p - p) / dist
        if abs(normal @ to_cam) < self.cos_view:
            return False
        vec = (p - cam_p) / dist
        mujoco.mj_multiRay(model, data, cam_p.copy(), vec, GROUP_WORLD_SIGNS, 1, -1,
                           self._gid, self._dist, None, 1, dist + 0.05)
        return int(self._gid[0]) == self.sign

    def read(self, model, data):
        """List of detections (bearing, distance, score) relative to the robot centre."""
        if not self.visible(model, data) or self.rng.random() < self.dropout:
            return []
        pose = robot_pose(model, data)
        p = data.geom_xpos[self.sign][None, :2]
        xr, yr = _to_robot_frame(p, pose)
        b = float(np.arctan2(yr[0], xr[0]) + self.rng.normal(0, self.bearing_noise))
        d = float(np.hypot(xr[0], yr[0]) * (1 + self.rng.normal(0, self.dist_noise)))
        return [(b, d, 1.0)]


class Odometry:
    """Wheel-encoder distance + IMU yaw (with a slow drift), in the start frame."""

    def __init__(self, model, data, slip=0.02, yaw_drift=np.radians(0.5) / 60, seed=0):
        self.jl = model.joint("rueda1_wheel").dofadr[0]
        self.jr = model.joint("rueda2_wheel").dofadr[0]
        self.rng = np.random.default_rng(seed)
        self.slip = slip
        self.yaw_drift = yaw_drift
        x, y, yaw = robot_pose(model, data)
        self.origin = (x, y, yaw)
        self.x = self.y = 0.0
        self.t = 0.0

    def step(self, model, data, dt):
        wl = -data.qvel[self.jl]   # left wheel, forward positive
        wr = data.qvel[self.jr]
        v = WHEEL_RADIUS * (wl + wr) / 2 * (1 + self.rng.normal(0, self.slip))
        self.t += dt
        _, _, yaw_w = robot_pose(model, data)
        yaw = float(np.arctan2(np.sin(yaw_w - self.origin[2]), np.cos(yaw_w - self.origin[2])))
        yaw += self.yaw_drift * self.t
        self.x += v * np.cos(yaw) * dt
        self.y += v * np.sin(yaw) * dt
        return (self.x, self.y, yaw)


def wheel_commands(v, omega, wheel_limit=8.0):
    """Differential drive: (v, omega) -> (motor_izq, motor_der) in rad/s."""
    wl = (v - omega * TRACK / 2) / WHEEL_RADIUS
    wr = (v + omega * TRACK / 2) / WHEEL_RADIUS
    scale = max(1.0, abs(wl) / wheel_limit, abs(wr) / wheel_limit)
    return -wl / scale, wr / scale
