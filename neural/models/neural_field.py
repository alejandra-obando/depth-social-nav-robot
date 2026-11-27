"""One-dimensional dynamic neural field on a ring of directions (Amari 1977).

    tau * du(x)/dt = -u(x) + h + s(x) + sum_x' w(x - x') f(u(x'))
    w(d) = w_exc * exp(-d^2 / (2 sigma^2)) - w_inh        (local excitation, global inhibition)
    f(u) = 1 / (1 + exp(-beta u))

Used as the memory of the direction of the person, as in Bicho, Mallet &
Schoner (2000):

* a detection creates a localised peak of activity at its direction,
* lateral inhibition lets only one peak survive when there are several
  detections (selection of one target),
* the peak sustains itself when the input disappears (short-term memory while
  the person is hidden),
* the resting level ``h`` slowly drops while there is no input, so the memory
  is forgotten after a characteristic delay.

Directions are in radians, in the world (odometry) frame, so the memory stays
valid while the robot turns.
"""

import numpy as np


def wrap(a):
    """Wrap an angle (or array) to (-pi, pi]."""
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


class NeuralField1D:
    def __init__(self, n=72, tau=0.1, h_rest=-2.0, h_forget=-4.0, tau_forget=8.0,
                 w_exc=2.5, sigma_exc=np.radians(10), w_inh=0.9, beta=4.0,
                 input_sigma=np.radians(8), max_substep=0.01):
        self.n = n
        self.x = np.arange(n) * 2 * np.pi / n - np.pi
        self.tau = tau
        self.h_rest = h_rest
        self.h_forget = h_forget
        self.tau_forget = tau_forget
        self.beta = beta
        self.input_sigma = input_sigma
        self.max_substep = max_substep
        d = wrap(self.x[:, None] - self.x[None, :])
        self.W = w_exc * np.exp(-d ** 2 / (2 * sigma_exc ** 2)) - w_inh
        self.h = h_rest
        self.u = np.full(n, h_rest)

    def f(self, u):
        return 1.0 / (1.0 + np.exp(-self.beta * u))

    def input_from(self, directions, amplitudes):
        """Sum of Gaussian bumps, one per detection."""
        s = np.zeros(self.n)
        for a, amp in zip(directions, amplitudes):
            s += amp * np.exp(-wrap(self.x - a) ** 2 / (2 * self.input_sigma ** 2))
        return s

    def step(self, dt, directions=(), amplitudes=()):
        """Advance by ``dt`` with the given detections; return the field activity."""
        s = self.input_from(directions, amplitudes)
        has_input = len(directions) > 0
        # Resting level: back to h_rest with input, slowly towards h_forget without it.
        h_target = self.h_rest if has_input else self.h_forget
        if has_input:
            self.h = self.h_rest
        else:
            self.h += (h_target - self.h) * (1 - np.exp(-dt / self.tau_forget))
        n_sub = max(1, int(np.ceil(dt / self.max_substep)))
        hh = dt / n_sub
        for _ in range(n_sub):
            self.u += hh / self.tau * (-self.u + self.h + s + self.W @ self.f(self.u))
        return self.u.copy()

    @property
    def active(self):
        return bool(self.u.max() > 0.0)

    def peak(self):
        """Direction of the strongest peak (population vector around it), or None."""
        if not self.active:
            return None
        i = int(np.argmax(self.u))
        near = np.abs(wrap(self.x - self.x[i])) < np.radians(30)
        a = self.f(self.u) * near
        return float(np.arctan2((a * np.sin(self.x)).sum(), (a * np.cos(self.x)).sum()))

    def n_peaks(self):
        """Number of separate supra-threshold regions on the ring."""
        on = self.u > 0
        if on.all():
            return 1
        return int(np.sum(on & ~np.roll(on, 1)))

    def reset(self):
        self.h = self.h_rest
        self.u[:] = self.h_rest
