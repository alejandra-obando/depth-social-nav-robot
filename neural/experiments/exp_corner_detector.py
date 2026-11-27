"""Experiment: the corner neuron fires only near the far corner, after travelling.

Synthetic LiDAR scans of the 7 m x 7 m room.  Checks:
1. robot ~1 m from both walls of the far corner, after 7 m of travel -> corner;
2. robot in the middle of the room                                  -> no corner;
3. robot at the far corner but the odometry says it travelled 2 m   -> no corner (gate);
4. robot next to a single wall                                      -> no corner;
5. two briefcases on both sides in the middle of the room            -> no corner.

    python -m neural.experiments.exp_corner_detector
"""

import numpy as np

from neural.experiments._common import Checks, plt
from neural.experiments.exp_motion_detector import BEAMS, scan
from neural.models.corner_detector import CornerDetector


def settle(pos, yaw, travelled, circles=()):
    cd = CornerDetector()
    r = scan(np.asarray(pos, dtype=float), yaw, list(circles))
    trace = [cd.step(0.05, BEAMS, r, travelled) for _ in range(60)]
    return cd.detected, trace, cd


def main():
    c = Checks("exp_corner_detector")
    cases = [
        ("far corner, 7 m travelled", (6.0, 6.0), np.pi / 4, 7.0, (), True),
        ("middle of the room", (3.5, 3.5), np.pi / 4, 7.0, (), False),
        ("far corner, only 2 m travelled", (6.0, 6.0), np.pi / 4, 2.0, (), False),
        ("next to one wall", (6.0, 3.5), np.pi / 4, 7.0, (), False),
        ("between two briefcases", (3.5, 3.5), np.pi / 4, 7.0, ((3.5, 4.5, 0.25), (4.5, 3.5, 0.25)), False),
    ]
    traces = []
    for name, pos, yaw, travelled, circles, expected in cases:
        det, trace, cd = settle(pos, yaw, travelled, circles)
        traces.append((name, trace))
        c.check(f"{name}: {'corner' if expected else 'no corner'}", det == expected,
                f"left wall {cd.left:.2f}, right wall {cd.right:.2f}")

    fig, ax = plt.subplots(figsize=(8, 3.5))
    for name, tr in traces:
        ax.plot(np.arange(len(tr)) * 0.05, tr, label=name)
    ax.axhline(0.8, color="gray", ls=":")
    ax.set_xlabel("time [s]"); ax.set_ylabel("corner neuron"); ax.legend(fontsize=7)
    ax.set_title("Corner detector (template neurons + leaky integrator)")
    return c.done(fig)


if __name__ == "__main__":
    raise SystemExit(0 if main() else 1)
