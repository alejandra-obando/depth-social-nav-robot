"""Experiment: behaviour selection follows the exam protocol and ignores brief glitches.

Scripted inputs go through a whole run:
follow -> brief link glitch (0.1 s) -> real link loss -> recover -> corner -> signal ->
phase 2 -> person behind -> person lags -> person back -> start corner.

Checks: the expected behaviour sequence, no reaction to the 0.1 s glitch, and that the
phase neuron stays in phase 2 once it has switched.

    python -m neural.experiments.exp_behaviour_selector
"""

import numpy as np

from neural.experiments._common import Checks, plt
from neural.models.behaviour_selector import BEHAVIOURS, BehaviourSelector


def main():
    c = Checks("exp_behaviour_selector")
    sel = BehaviourSelector(signal_time=3.0)
    dt = 0.05
    # (duration, front_link, corner, rear_link, person_close, at_home)
    script = [
        (4.0, 1, 0, 0, 0, 0),
        (0.1, 0, 0, 0, 0, 0),     # glitch
        (3.0, 1, 0, 0, 0, 0),
        (3.0, 0, 0, 0, 0, 0),     # link lost
        (3.0, 1, 0, 0, 0, 0),
        (5.0, 1, 1, 0, 0, 0),     # corner -> signal -> phase 2
        (3.0, 0, 1, 0, 0, 0),     # robot turns, person not yet behind
        (5.0, 0, 0, 1, 1, 0),     # person behind -> guide
        (3.0, 0, 0, 1, 0, 0),     # person lags -> wait
        (4.0, 0, 0, 1, 1, 0),     # back -> guide
        (3.0, 0, 0, 1, 1, 1),     # start corner -> done
    ]
    t, beh, phase, U = [], [], [], []
    time = 0.0
    for dur, *inp in script:
        for _ in range(int(round(dur / dt))):
            b = sel.step(dt, *inp)
            t.append(time); beh.append(b); phase.append(sel.phase); U.append(sel.net.u.copy())
            time += dt
    seq = [b for i, b in enumerate(beh) if i == 0 or b != beh[i - 1]]
    expected = ["FOLLOW", "SEARCH", "FOLLOW", "SIGNAL", "WAIT", "GUIDE", "WAIT", "GUIDE", "DONE"]
    c.check("behaviour sequence follows the protocol", seq == expected, " -> ".join(seq))
    k0 = int(4.0 / dt)
    c.check("0.1 s glitch does not trigger SEARCH", "SEARCH" not in beh[k0:k0 + int(1.0 / dt)])
    first_p2 = phase.index(2)
    c.check("phase neuron stays in phase 2", all(p == 2 for p in phase[first_p2:]))

    fig, axes = plt.subplots(2, 1, figsize=(10, 5), sharex=True)
    U = np.array(U)
    for j, name in enumerate(BEHAVIOURS):
        axes[0].plot(t, U[:, j], label=name)
    axes[0].legend(fontsize=7, ncol=6); axes[0].set_ylabel("unit activity")
    axes[1].plot(t, [BEHAVIOURS.index(b) for b in beh], drawstyle="steps-post", color="#6a3d9a")
    axes[1].plot(t, np.array(phase) - 1, color="gray", ls="--", label="phase neuron (0 = phase 1, 1 = phase 2)")
    axes[1].set_yticks(range(len(BEHAVIOURS))); axes[1].set_yticklabels(BEHAVIOURS, fontsize=8)
    axes[1].set_xlabel("time [s]"); axes[1].legend(fontsize=7, loc="upper left")
    axes[0].set_title("Behaviour selection (competing units + bistable phase neuron)")
    return c.done(fig)


if __name__ == "__main__":
    raise SystemExit(0 if main() else 1)
