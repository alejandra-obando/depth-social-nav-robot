"""Experiment: winner-take-all with hysteresis.

Three units receive inputs that change over time.  Checks:
* in steady state only one unit is active (both can be briefly active during a switch),
* a challenger 10 % larger than the winner does not take over (hysteresis),
* a challenger clearly larger (50 %) does,
* brief noise does not make the winner flicker.

    python -m neural.experiments.exp_competitive_network
"""

import numpy as np

from neural.experiments._common import Checks, plt
from neural.models.competitive_network import CompetitiveNetwork


def main():
    c = Checks("exp_competitive_network")
    rng = np.random.default_rng(0)
    net = CompetitiveNetwork(["A", "B", "C"])
    dt = 0.05
    schedule = [  # (duration, inputs)
        (3.0, [0.8, 0.5, 0.1]),
        (3.0, [0.8, 0.88, 0.1]),   # B 10 % larger: A must stay
        (3.0, [0.6, 0.9, 0.1]),    # B 50 % larger: B must win
        (3.0, [0.6, 0.9, 0.1]),    # with noise below
    ]
    t, U, W, I = [], [], [], []
    time = 0.0
    seg_winners = []
    for k, (dur, inp) in enumerate(schedule):
        ws = []
        for _ in range(int(dur / dt)):
            x = np.array(inp, dtype=float)
            if k == 3:
                x = x + rng.normal(0, 0.15, 3)
            net.step(x, dt)
            t.append(time); U.append(net.u.copy()); W.append(net.winner); I.append(x)
            ws.append(net.winner)
            time += dt
        seg_winners.append(ws)
    U = np.array(U)
    n_active = (U > net.threshold).sum(1)
    ends = (np.cumsum([int(d / dt) for d, _ in schedule]) - 1)
    c.check("one active unit at the end of every segment (steady state)", all(n_active[k] == 1 for k in ends),
            f"active units: {[int(n_active[k]) for k in ends]}")
    c.check("A wins at the start", seg_winners[0][-1] == "A")
    c.check("B 10 % larger does not take over", all(w == "A" for w in seg_winners[1]))
    c.check("B 50 % larger takes over", seg_winners[2][-1] == "B")
    switches = sum(1 for a, b in zip(seg_winners[3][:-1], seg_winners[3][1:]) if a != b)
    c.check("no flicker with noisy inputs", switches == 0, f"{switches} switches")

    fig, axes = plt.subplots(2, 1, figsize=(9, 5), sharex=True)
    I = np.array(I)
    for j, lab in enumerate("ABC"):
        axes[0].plot(t, I[:, j], label=f"input {lab}")
        axes[1].plot(t, U[:, j], label=f"unit {lab}")
    axes[0].set_ylabel("input"); axes[0].legend(fontsize=8, ncol=3)
    axes[1].axhline(net.threshold, color="gray", ls=":")
    axes[1].set_ylabel("activity u"); axes[1].set_xlabel("time [s]"); axes[1].legend(fontsize=8, ncol=3)
    for x in np.cumsum([d for d, _ in schedule])[:-1]:
        for a in axes:
            a.axvline(x, color="gray", lw=0.5)
    axes[0].set_title("Competitive network: selection with hysteresis")
    return c.done(fig)


if __name__ == "__main__":
    raise SystemExit(0 if main() else 1)
