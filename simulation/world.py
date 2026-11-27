"""Builds the 7 m x 7 m exam room around the robot model.

The robot comes unchanged from ``xml 2 cameras set.xml``; walls, tablet stacks,
briefcases and people are added with MjSpec, so the layout can be randomised
per seed.

Geom groups (used by the simulated sensors' rays):
    0  robot meshes, floor, signal light   (ignored by every sensor)
    1  walls, obstacles, people           (seen by the LiDAR, the depth camera and the cameras)
    2  signs worn by the guide person     (seen only by the sign cameras)
"""

from dataclasses import dataclass, field
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
ROBOT_XML = HERE / "xml 2 cameras set.xml"

ROOM = 7.0
START = np.array([0.6, 0.6])
START_YAW = np.pi / 4                 # facing the opposite corner
GOAL = np.array([6.7, 6.7])           # where the guide person stops in phase 1
PERSON_RADIUS = 0.18
SIGN_HALF = 0.10                      # signs are 20 cm x 20 cm


@dataclass
class Layout:
    stacks: list = field(default_factory=list)       # (x, y) centres of the 25 x 25 x 10 cm stacks
    briefcases: list = field(default_factory=list)   # (x, y, yaw) static briefcases
    moving_briefcase: tuple = None                   # (x0, y0, x1, y1, t_start, duration)
    walkers: list = field(default_factory=list)      # list of goal loops [(x, y), ...]
    guide_path: list = field(default_factory=list)   # waypoints of the guide person in phase 1
    sign_heights: tuple = (0.30, 1.10)               # (back sign, front sign) heights [m]


def make_layout(scenario="full", seed=0):
    """Return the obstacle layout of a scenario.

    empty   : only the walls
    static  : + 9 tablet stacks and 2 briefcases
    moving  : + one briefcase that is pushed during the run
    people  : static + 2 people walking across the diagonal
    full    : everything (closest to the exam)
    """
    rng = np.random.default_rng(seed)
    lay = Layout()
    diag = np.array([1, 1]) / np.sqrt(2)
    perp = np.array([-1, 1]) / np.sqrt(2)
    # The guide walks a gentle S around the diagonal.
    s_values = np.linspace(1.4, 8.2, 6)
    offsets = [0.0, 0.35, -0.35, 0.3, -0.2, 0.0]
    lay.guide_path = [START + diag * s + perp * o for s, o in zip(s_values, offsets)] + [GOAL]

    if scenario in ("static", "moving", "people", "full"):
        for k, s in enumerate(np.linspace(1.6, 7.6, 9)):
            side = 1 if k % 2 == 0 else -1
            off = side * rng.uniform(0.25, 0.9)
            p = START + diag * (s + rng.uniform(-0.2, 0.2)) + perp * off
            lay.stacks.append(tuple(np.clip(p, 0.4, ROOM - 0.4)))
        lay.guide_path = _detour(lay.guide_path, lay.stacks)
        lay.briefcases = [tuple(START + diag * 3.3 + perp * -1.2) + (0.3,),
                          tuple(START + diag * 6.2 + perp * 1.3) + (-0.5,)]
    if scenario in ("moving", "full"):
        a = START + diag * 4.6 + perp * 1.6
        b = START + diag * 4.6 + perp * 0.9
        lay.moving_briefcase = (a[0], a[1], b[0], b[1], 20.0, 4.0)
    if scenario in ("people", "full"):
        lay.walkers = [
            [(1.0, 5.8), (5.5, 1.2)],           # crosses the diagonal in the middle
            [(3.0, 6.4), (6.4, 3.0), (2.0, 2.6)],
        ]
    return lay


def _detour(path, stacks, clearance=0.45, step=0.1):
    """Resample the guide path and push it sideways away from stacks (people walk around them)."""
    pts = [np.asarray(path[0], dtype=float)]
    for a, b in zip(path[:-1], path[1:]):
        a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
        n = max(1, int(np.linalg.norm(b - a) / step))
        pts += [a + (b - a) * k / n for k in range(1, n + 1)]
    out = []
    for p in pts:
        q = p.copy()
        for s in stacks:
            d = q - np.asarray(s)
            n = np.linalg.norm(d)
            if n < clearance:
                q = np.asarray(s) + d / max(n, 1e-6) * clearance
        out.append(q)
    return out


def _add_box(parent, name, pos, half, rgba, yaw=0.0, group=1, **kw):
    q = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
    return parent.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX, pos=list(pos), quat=q,
                           size=list(half), rgba=list(rgba), group=group, **kw)


def _add_person(spec, name, rgba, with_signs=False, sign_heights=(0.4, 1.1)):
    body = spec.worldbody.add_body(name=name, pos=[-5, -5, 0], mocap=True)
    body.add_geom(name=f"{name}_body", type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                  fromto=[0, 0, PERSON_RADIUS, 0, 0, 1.55], size=[PERSON_RADIUS, 0, 0],
                  rgba=list(rgba), group=1)
    body.add_geom(name=f"{name}_head", type=mujoco.mjtGeom.mjGEOM_SPHERE, pos=[0, 0, 1.68],
                  size=[0.11, 0, 0], rgba=[0.9, 0.75, 0.6, 1], group=1)
    if with_signs:
        back_h, front_h = sign_heights
        off = PERSON_RADIUS + 0.01
        # Person faces local +x: the front sign is at +x, the back sign at -x.
        _add_box(body, f"{name}_sign_front", [off, 0, front_h], [0.005, SIGN_HALF, SIGN_HALF],
                 [0.05, 0.05, 0.05, 1], group=2, contype=0, conaffinity=0)
        _add_box(body, f"{name}_sign_back", [-off, 0, back_h], [0.005, SIGN_HALF, SIGN_HALF],
                 [0.05, 0.05, 0.05, 1], group=2, contype=0, conaffinity=0)
    return body


def build(layout: Layout):
    spec = mujoco.MjSpec.from_file(str(ROBOT_XML))
    wb = spec.worldbody
    wall_rgba = [0.75, 0.75, 0.7, 1]
    h = 0.3
    _add_box(wb, "wall_s", [ROOM / 2, -0.025, h], [ROOM / 2 + 0.05, 0.025, h], wall_rgba)
    _add_box(wb, "wall_n", [ROOM / 2, ROOM + 0.025, h], [ROOM / 2 + 0.05, 0.025, h], wall_rgba)
    _add_box(wb, "wall_w", [-0.025, ROOM / 2, h], [0.025, ROOM / 2, h], wall_rgba)
    _add_box(wb, "wall_e", [ROOM + 0.025, ROOM / 2, h], [0.025, ROOM / 2, h], wall_rgba)
    for i, (x, y) in enumerate(layout.stacks):
        _add_box(wb, f"stack_{i}", [x, y, 0.05], [0.125, 0.125, 0.05], [0.55, 0.35, 0.2, 1])
    for i, (x, y, yaw) in enumerate(layout.briefcases):
        _add_box(wb, f"briefcase_{i}", [x, y, 0.175], [0.225, 0.075, 0.175], [0.15, 0.15, 0.2, 1], yaw=yaw)
    if layout.moving_briefcase is not None:
        x0, y0 = layout.moving_briefcase[:2]
        mb = wb.add_body(name="moving_briefcase", pos=[x0, y0, 0.175], mocap=True)
        _add_box(mb, "moving_briefcase_geom", [0, 0, 0], [0.225, 0.075, 0.175], [0.35, 0.1, 0.1, 1])

    _add_person(spec, "guide", [0.2, 0.45, 0.85, 1], with_signs=True, sign_heights=layout.sign_heights)
    for i in range(len(layout.walkers)):
        _add_person(spec, f"walker_{i}", [0.6, 0.6, 0.6, 1])

    # Signal light on top of the robot (green when the robot gives its visual signal).
    base = spec.body("base")
    base.add_geom(name="signal_light", type=mujoco.mjtGeom.mjGEOM_SPHERE, pos=[0, 0, 0.2],
                  size=[0.03, 0, 0], rgba=[0.3, 0.3, 0.3, 1], group=0, contype=0, conaffinity=0)
    model = spec.compile()
    return spec, model


def robot_quat(yaw):
    """Body quaternion so that the robot's forward axis (body -Y) points along ``yaw``."""
    th = yaw + np.pi / 2
    return np.array([np.cos(th / 2), 0, 0, np.sin(th / 2)])


def robot_pose(model, data):
    """(x, y, yaw) of the robot centre in the world; yaw of the forward axis (body -Y)."""
    bid = model.body("base").id
    R = data.xmat[bid].reshape(3, 3)
    fwd = -R[:, 1]
    return data.xpos[bid][0], data.xpos[bid][1], float(np.arctan2(fwd[1], fwd[0]))
