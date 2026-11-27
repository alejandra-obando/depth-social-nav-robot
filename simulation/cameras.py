"""Rendered camera images for the conventional controller.

Instead of the geometric sign model in ``sensors.py`` (used by the neural
architecture), these classes render what the real cameras would see, so the
robot's own AprilTag detector and depth-obstacle code run on them unchanged:

* ``RenderedCamera`` : grey image from the D415 colour camera or the C270.
* ``RenderedDepth``  : z-depth image [m] from the D415 depth camera.

``mount_from_model`` reads each camera's pose on the robot from the MuJoCo model,
in the robot frame the software uses (x forward, y left, z up).

Rendering needs an OpenGL context; on a machine without a display run with
``MUJOCO_GL=egl``.
"""

import mujoco
import numpy as np

from .software_path import add_software_to_path

add_software_to_path()

from perception.camera import Intrinsics, Mount     # noqa: E402

# MuJoCo cameras look along their -z axis with y up; OpenCV's optical frame looks
# along +z with y down.
_MJ_FROM_OPTICAL = np.diag([1.0, -1.0, -1.0])


def robot_axes(model, data):
    """3x3 matrix whose columns are the robot's x (forward), y (left), z (up) axes in the world."""
    R = data.xmat[model.body("base").id].reshape(3, 3)
    return np.stack([-R[:, 1], R[:, 0], R[:, 2]], 1)


def mount_from_model(model, data, camera):
    cam = model.camera(camera).id
    A = robot_axes(model, data)
    base = data.xpos[model.body("base").id]
    pos = A.T @ (data.cam_xpos[cam] - base)
    pos[2] = data.cam_xpos[cam][2]          # height above the floor, like on the robot
    rot = A.T @ data.cam_xmat[cam].reshape(3, 3) @ _MJ_FROM_OPTICAL
    return Mount(rot, pos)


class RenderedCamera:
    def __init__(self, model, camera, width=640, height=360):
        self.camera = camera
        self.renderer = mujoco.Renderer(model, height, width)
        self.intrinsics = Intrinsics.from_fov(width, height, float(model.camera(camera).fovy[0]))

    def gray(self, data):
        self.renderer.update_scene(data, camera=self.camera)
        rgb = self.renderer.render()
        return (rgb @ np.array([0.299, 0.587, 0.114])).astype(np.uint8)


class RenderedDepth:
    def __init__(self, model, camera="d415_prof", width=168, height=96, noise=0.01, seed=0):
        self.camera = camera
        self.renderer = mujoco.Renderer(model, height, width)
        self.renderer.enable_depth_rendering()
        self.intrinsics = Intrinsics.from_fov(width, height, float(model.camera(camera).fovy[0]))
        self.noise = noise
        self.rng = np.random.default_rng(seed)

    def depth(self, data):
        self.renderer.update_scene(data, camera=self.camera)
        z = self.renderer.render().astype(float)
        return z * (1 + self.rng.normal(0, self.noise, z.shape))
