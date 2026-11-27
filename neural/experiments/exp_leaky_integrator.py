"""Experiment: the link with the person survives short occlusions and breaks on long ones.

The sign is seen, then hidden for 1, 2, 4 and 6 s.  Checks:
* an occlusion of up to 2 s keeps the link,
* an occlusion of 4 s or more breaks it,
* when the sign reappears, the link is back in less than 0.5 s.

    python -m neural.experiments.exp_leaky_integrator
"""

import numpy as np

from neural.experiments._common import Checks, plt
from neural.models.leaky_integrator import LeakyIntegrator


def simulate(gaps, dt=0.05, seen=4.0):
    li = LeakyIntegrator(tau_rise=0.2, tau_decay=3.0, theta_on=0.6, theta_off=0.3)
    t, u, act, inp = [], [], [], []
    events = []
    time = 0.0
    for gap in gaps:
        for phase, dur in (("seen", seen), ("hidden", gap)):
            start = time
            lost = False
            for _ in range(int(round(dur / dt))):
                I = 1.0 if phase == "seen" else 0.0
                li.step(I, dt)
                lost |= not li.active
                t.append(time); u.append(li.u); act.append(li.active); inp.append(I)
                time += dt
            if phase == "hidden":
                events.append((gap, lost, start))
    for _ in range(int(round(2.0 / dt))):      # the sign comes back at the end
        li.step(1.0, dt)
        t.append(time); u.append(li.u); act.append(li.active); inp.append(1.0)
        time += dt
    return np.array(t), np.array(u), np.array(act), np.array(inp), events


def main():
    c = Checks("exp_leaky_integrator")
    gaps = [1.0, 2.0, 4.0, 6.0]
    t, u, act, inp, events = simulate(gaps)
    for gap, lost, _ in events:
        if gap <= 2.0:
            c.check(f"{gap:.0f} s occlusion keeps the link", not lost)
        else:
            c.check(f"{gap:.0f} s occlusion breaks the link", lost)
    # Recovery time after the last long gap.
    _, _, start = events[-1]
    k0 = np.searchsorted(t, start + gaps[-1])
    k1 = k0 + np.argmax(act[k0:])
    c.check("link recovered in < 0.5 s after reappearing", t[k1] - t[k0] < 0.5, f"{t[k1] - t[k0]:.2f} s")

    fig, ax = plt.subplots(figsize=(9, 3.5))
    ax.fill_between(t, 0, inp, step="post", color="#cfe3f5", label="sign seen")
    ax.plot(t, u, color="k", label="integrator activity u")
    ax.plot(t, act * 1.05, color="#2a9d4b", lw=1, drawstyle="steps-post", label="link on")
    ax.axhline(0.6, ls=":", color="gray"); ax.axhline(0.3, ls=":", color="gray")
    for gap, _, start in events:
        ax.text(start + gap / 2, 1.12, f"{gap:.0f} s", ha="center", fontsize=8)
    ax.set_xlabel("time [s]"); ax.set_ylim(-0.05, 1.2); ax.legend(fontsize=8, loc="lower right")
    ax.set_title("Leaky integrator: link with the person under occlusions")
    return c.done(fig)


if __name__ == "__main__":
    raise SystemExit(0 if main() else 1)
