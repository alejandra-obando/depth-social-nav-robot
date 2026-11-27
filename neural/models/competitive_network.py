"""Winner-take-all network of leaky competing accumulators.

    tau * du_i/dt = -u_i + I_i + alpha * f(u_i) - beta * sum_{j != i} f(u_j)
    f(u) = max(u, 0)

Each unit integrates its input with a leak, excites itself (``alpha`` < 1) and
inhibits the others (``beta``).  With ``beta > 1 - alpha`` only one unit stays
active: the winner, at u = I / (1 - alpha).  A challenger j takes over only when
I_j > beta / (1 - alpha) * I_winner, so the winner does not change on brief
noise (hysteresis; with the defaults the challenger must be 25 % larger).

Based on the leaky competing accumulator of Usher & McClelland (2001) and the
selection mechanism of Redgrave, Prescott & Gurney (1999).
"""

import numpy as np


def _rectify(u):
    return np.maximum(u, 0.0)


class CompetitiveNetwork:
    def __init__(self, labels, tau=0.15, alpha=0.2, beta=1.0, threshold=0.1, max_substep=0.01):
        self.labels = list(labels)
        self.n = len(self.labels)
        self.tau = tau
        self.alpha = alpha
        self.beta = beta
        self.threshold = threshold
        self.max_substep = max_substep
        self.u = np.zeros(self.n)

    def step(self, inputs, dt):
        """Advance by ``dt`` with input vector ``inputs``; return the activities."""
        I = np.asarray(inputs, dtype=float)
        n_sub = max(1, int(np.ceil(dt / self.max_substep)))
        h = dt / n_sub
        for _ in range(n_sub):
            f = _rectify(self.u)
            lateral = self.beta * (f.sum() - f)
            self.u += h / self.tau * (-self.u + I + self.alpha * f - lateral)
        return self.u.copy()

    @property
    def winner_index(self):
        i = int(np.argmax(self.u))
        return i if self.u[i] > self.threshold else None

    @property
    def winner(self):
        i = self.winner_index
        return None if i is None else self.labels[i]

    def reset(self):
        self.u[:] = 0.0
