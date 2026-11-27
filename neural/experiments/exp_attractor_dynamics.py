"""Experiment: heading attractor dynamics steer to a target around obstacles.

A point robot (unicycle, 0.2 m/s) with a 72-sector range sensor drives to a goal 4 m away.
Scenes:
1. free space            -> reaches the goal;
2. one obstacle in the way -> goes around it without touching it;
3. a wide gap (1.0 m)     -> passes through the gap;
4. a narrow gap (0.3 m)   -> does not squeeze in, goes around both obstacles
                             (the bifurcation described by Schoner, Dose & Engels 1995).

    python -m neural.experiments.exp_attractor_dynamics
"""

import numpy as np

from neural.experiments._common import Checks, plt
from neural.models.attractor_dynamics import HeadingDynamics

N = 72
SECTORS = (np.arange(N) + 0.5) * 2 * np.pi / N - np.pi
ROBOT_R = 0.17


def ranges(pos, yaw, obstacles, max_range=3.0):
    """Ray-cast circular obstacles (x, y, r) from the robot, one ray per sector."""
    out = np.full(N, np.inf)
    for k, a in enumerate(SECTORS):
        d = np.array([np.cos(yaw + a), np.sin(yaw + a)])
        for ox, oy, r in obstacles:
            oc = np.array([ox, oy]) - pos
            b = oc @ d
            disc = b * b - (oc @ oc - r * r)
            if disc >= 0 and b - np.sqrt(disc) > 0:
                out[k] = min(out[k], b - np.sqrt(disc))
    out[out > max_range] = np.inf
    return out


def drive(obstacles, goal=(4.0, 0.0), dt=0.05, t_max=60.0):
    hd = HeadingDynamics()
    pos, yaw = np.array([0.0, 0.0]), 0.0
    path, min_clear = [pos.copy()], np.inf
    for _ in range(int(t_max / dt)):
        g = np.asarray(goal) - pos
        if np.linalg.norm(g) < 0.15:
            return np.array(path), True, min_clear
        theta = np.arctan2(g[1], g[0]) - yaw
        w = hd.omega(theta, SECTORS, ranges(pos, yaw, obstacles))
        yaw += w * dt
        pos = pos + 0.2 * dt * np.array([np.cos(yaw), np.sin(yaw)])
        path.append(pos.copy())
        for ox, oy, r in obstacles:
            min_clear = min(min_clear, np.hypot(*(pos - [ox, oy])) - r - ROBOT_R)
    return np.array(path), False, min_clear


def main():
    c = Checks("exp_attractor_dynamics")
    scenes = {
        "free space": [],
        "one obstacle": [(2.0, 0.05, 0.25)],
        "wide gap (1.0 m)": [(2.0, 0.75, 0.25), (2.0, -0.75, 0.25)],
        "narrow gap (0.3 m)": [(2.0, 0.40, 0.25), (2.0, -0.40, 0.25)],
    }
    results = {}
    for name, obs in scenes.items():
        path, ok, clear = drive(obs)
        results[name] = (path, obs)
        c.check(f"{name}: reaches the goal", ok)
        if obs:
            c.check(f"{name}: never touches an obstacle", clear > 0, f"min clearance {clear:.2f} m")
    # Wide gap: crosses x = 2 between the obstacles; narrow gap: goes around outside them.
    for name, inside in (("wide gap (1.0 m)", True), ("narrow gap (0.3 m)", False)):
        path, obs = results[name]
        k = np.argmin(np.abs(path[:, 0] - 2.0))
        y_at = path[k, 1]
        between = abs(y_at) < abs(obs[0][1]) - obs[0][2]
        c.check(f"{name}: {'passes through' if inside else 'goes around'} the gap", between == inside,
                f"y at the obstacles = {y_at:+.2f} m")

    fig, axes = plt.subplots(1, 4, figsize=(15, 3.6))
    for ax, (name, (path, obs)) in zip(axes, results.items()):
        for ox, oy, r in obs:
            ax.add_patch(plt.Circle((ox, oy), r, color="#a0663a"))
        ax.plot(path[:, 0], path[:, 1], color="#e07b00", lw=2)
        ax.plot(4, 0, "*", color="#1a9a50", ms=14)
        ax.set_aspect("equal"); ax.set_xlim(-0.3, 4.4); ax.set_ylim(-1.6, 1.6); ax.set_title(name, fontsize=10)
    fig.suptitle("Heading attractor dynamics: target attractor + obstacle repellors")
    return c.done(fig)


if __name__ == "__main__":
    raise SystemExit(0 if main() else 1)
