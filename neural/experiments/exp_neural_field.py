"""Experiment: the neural field selects the person, remembers their direction and forgets it.

Reproduces the target-representation properties of Bicho, Mallet & Schoner (2000):
1. detection: a sign at 20 deg creates a peak at 20 deg;
2. memory: the sign disappears and the peak stays for > 3 s;
3. forgetting: the peak disappears before 8 s without input;
4. selection: two simultaneous signs give a single peak;
5. stability: while tracking the person, a stronger sign elsewhere (someone crossing)
   does not steal the peak;
6. tracking: a person moving at 10 deg/s is followed with a lag < 8 deg.

    python -m neural.experiments.exp_neural_field
"""

import numpy as np

from neural.experiments._common import Checks, plt
from neural.models.neural_field import NeuralField1D

DT = 0.05


def run(field, steps, dirs_fn):
    hist = []
    for k in range(steps):
        d = dirs_fn(k)
        field.step(DT, [a for a, _ in d], [s for _, s in d])
        hist.append(field.u.copy())
    return np.array(hist)


def main():
    c = Checks("exp_neural_field")
    f = NeuralField1D()
    deg = np.radians

    # 1-3. detection, memory, forgetting.
    H1 = run(f, 40, lambda k: [(deg(20), 3.0)])
    c.check("peak at the sign direction", abs(np.degrees(f.peak()) - 20) < 3, f"{np.degrees(f.peak()):.1f} deg")
    t_forget = None
    hist_mem = []
    for k in range(int(10 / DT)):
        f.step(DT)
        hist_mem.append(f.u.copy())
        if t_forget is None and not f.active:
            t_forget = (k + 1) * DT
    H1 = np.vstack([H1, hist_mem])
    c.check("memory lasts > 3 s without the sign", t_forget is not None and t_forget > 3.0, f"{t_forget} s")
    c.check("memory is forgotten before 8 s", t_forget is not None and t_forget < 8.0)

    # 4. selection of one of two simultaneous signs.
    f.reset()
    run(f, 60, lambda k: [(0.0, 3.0), (deg(90), 3.2)])
    c.check("two simultaneous signs -> one peak", f.n_peaks() == 1, f"peak at {np.degrees(f.peak()):.0f} deg")

    # 5. a stronger distractor does not steal an established peak.
    f.reset()
    H5 = np.vstack([run(f, 40, lambda k: [(0.0, 3.0)]),
                    run(f, 60, lambda k: [(0.0, 3.0), (deg(90), 3.3)])])
    c.check("established target kept when a stronger sign appears", abs(np.degrees(f.peak())) < 5,
            f"peak at {np.degrees(f.peak()):.1f} deg")

    # 6. tracking a moving person.
    f.reset()
    H6 = run(f, 200, lambda k: [(deg(-30 + k * DT * 10), 3.0)])
    lag = (-30 + 199 * DT * 10) - np.degrees(f.peak())
    c.check("tracks a person moving at 10 deg/s with lag < 8 deg", abs(lag) < 8, f"lag {lag:.1f} deg")

    fig, axes = plt.subplots(1, 3, figsize=(13, 4))
    x = np.degrees(f.x)
    for ax, H, title in ((axes[0], H1, "detection, memory, forgetting"),
                         (axes[1], H5, "distractor at 90 deg from t = 2 s"),
                         (axes[2], H6, "tracking a moving person")):
        im = ax.imshow(H.T, aspect="auto", origin="lower", cmap="magma",
                       extent=[0, len(H) * DT, x[0], x[-1]], vmin=-5, vmax=6)
        ax.set_title(title, fontsize=10); ax.set_xlabel("time [s]")
    axes[0].axvline(2.0, color="w", ls="--", lw=1)
    axes[0].set_ylabel("direction [deg]")
    fig.colorbar(im, ax=axes, shrink=0.8, label="u(x)")
    fig.suptitle("Dynamic neural field (Amari): person direction memory")
    fig.subplots_adjust(right=0.88)
    path_ok = c.done(None)
    from neural.experiments._common import RESULTS
    fig.savefig(RESULTS / "exp_neural_field.png", dpi=110)
    plt.close(fig)
    return path_ok


if __name__ == "__main__":
    raise SystemExit(0 if main() else 1)
