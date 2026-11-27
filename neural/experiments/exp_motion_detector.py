"""Experiment: Reichardt detectors respond to moving objects, not to static ones or to ego-motion.

Synthetic LiDAR scans (72 sectors) of a room:
1. a person crosses from left to right at 1.5 m  -> strong response, negative sign (to the right);
2. the same person crossing right to left         -> positive sign;
3. a static scene                                 -> no response;
4. the robot drives forward past static obstacles -> no response with ego-motion compensation,
   a clear (false) response without it.

    python -m neural.experiments.exp_motion_detector
"""

import numpy as np

from neural.experiments._common import Checks, plt
from neural.models.motion_detector import ReichardtArray

BEAMS = np.arange(360) * 2 * np.pi / 360 - np.pi


def scan(pos, yaw, circles, room=7.0):
    """Ranges from ``pos`` to circles (x, y, r) and to the walls of a square room."""
    out = np.full(len(BEAMS), np.inf)
    for k, b in enumerate(BEAMS):
        d = np.array([np.cos(yaw + b), np.sin(yaw + b)])
        for axis in (0, 1):
            for wall in (0.0, room):
                if abs(d[axis]) > 1e-9:
                    t = (wall - pos[axis]) / d[axis]
                    if t > 0:
                        out[k] = min(out[k], t)
        for cx, cy, r in circles:
            oc = np.array([cx, cy]) - pos
            p = oc @ d
            disc = p * p - (oc @ oc - r * r)
            if disc >= 0 and p - np.sqrt(disc) > 0:
                out[k] = min(out[k], p - np.sqrt(disc))
    return out


def run(steps, scene_fn, ego_comp=True, dt=0.1):
    ra = ReichardtArray()
    prev = None
    peak, hist = [], []
    for k in range(steps):
        pos, yaw, circles = scene_fn(k * dt)
        ranges = scan(pos, yaw, circles)
        ego = (0.0, 0.0, 0.0)
        if ego_comp and prev is not None:
            dx, dy = pos - prev[0]
            c, s = np.cos(-prev[1]), np.sin(-prev[1])
            ego = (c * dx - s * dy, s * dx + c * dy, yaw - prev[1])
        prev = (pos, yaw)
        out = ra.step(dt, BEAMS, ranges, ego)
        hist.append(out)
        peak.append(out[np.argmax(np.abs(out))])
    return np.array(hist), np.array(peak), ra


def main():
    c = Checks("exp_motion_detector")
    robot = np.array([3.5, 1.5])
    static = [(2.5, 3.5, 0.2), (4.8, 3.0, 0.15)]

    def crossing(direction):
        # The person walks parallel to the x axis 1.5 m in front of the robot (robot faces +y).
        return lambda t: (robot, np.pi / 2, static + [(3.5 - direction * (1.5 - 0.6 * t), 3.0, 0.2)])

    H1, P1, _ = run(50, crossing(+1))     # left -> right as seen by the robot
    H2, P2, _ = run(50, crossing(-1))     # right -> left
    H3, P3, _ = run(50, lambda t: (robot, np.pi / 2, static))
    moving_robot = lambda t: (robot + np.array([0, 0.25 * t]), np.pi / 2 + 0.05 * t, static)
    H4, P4, _ = run(50, moving_robot, ego_comp=True)
    H5, P5, _ = run(50, moving_robot, ego_comp=False)

    a1, a2 = np.abs(P1[10:]).max(), np.abs(P2[10:]).max()
    c.check("crossing person gives a strong response", min(a1, a2) > 0.05, f"{a1:.3f}, {a2:.3f}")
    s1 = np.sign(H1[10:].sum()); s2 = np.sign(H2[10:].sum())
    c.check("direction is coded by the sign (left->right < 0, right->left > 0)", s1 < 0 < s2)
    c.check("static scene gives no response", np.abs(P3[5:]).max() < 0.01, f"{np.abs(P3[5:]).max():.4f}")
    e4, e5 = np.abs(P4[5:]).max(), np.abs(P5[5:]).max()
    c.check("robot moving: ego-motion compensation removes the false response",
            e4 < 0.3 * e5, f"with {e4:.3f} vs without {e5:.3f}")

    fig, axes = plt.subplots(1, 4, figsize=(15, 3.6), sharey=True)
    ang = np.degrees((np.arange(72) + 0.5) * 5 - 180)
    for ax, H, title in zip(axes, (H1, H2, H4, H5),
                            ("person left -> right", "person right -> left",
                             "robot moving, compensated", "robot moving, NOT compensated")):
        ax.imshow(H.T, aspect="auto", origin="lower", cmap="coolwarm", vmin=-0.15, vmax=0.15,
                  extent=[0, len(H) * 0.1, ang[0], ang[-1]])
        ax.set_title(title, fontsize=10); ax.set_xlabel("time [s]")
    axes[0].set_ylabel("sector angle [deg]")
    fig.suptitle("Hassenstein-Reichardt detector array on the LiDAR scan (red: to the left, blue: to the right)")
    return c.done(fig)


if __name__ == "__main__":
    raise SystemExit(0 if main() else 1)
