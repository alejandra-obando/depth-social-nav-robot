"""Behaviour selection: competing behaviour units plus a bistable phase neuron.

Behaviours compete in a winner-take-all network (see ``competitive_network``):

    FOLLOW  phase 1, the person is linked       -> follow them
    SEARCH  phase 1, the link is lost           -> stop and turn towards the memory
    SIGNAL  phase 1, the corner is recognised   -> stop and give the visual signal
    GUIDE   phase 2, the person is behind       -> drive to the start corner
    WAIT    phase 2, the person is lost or far  -> stop (turn in place towards home)
    DONE    phase 2, the start corner is reached

The salience of each behaviour is a product of its conditions (all inputs in
[0, 1]).  The phase neuron is bistable: it flips to phase 2 once the signal has
lasted long enough and then holds itself by self-excitation.

Inspired by the basal ganglia selection models of Redgrave, Prescott & Gurney
(1999) and their robot implementation (Prescott et al. 2006).
"""

import numpy as np

from .competitive_network import CompetitiveNetwork
from .leaky_integrator import LeakyIntegrator

BEHAVIOURS = ("FOLLOW", "SEARCH", "SIGNAL", "GUIDE", "WAIT", "DONE")


class PhaseNeuron:
    """Bistable neuron: tau dP/dt = -P + H(w * P + I - theta)."""

    def __init__(self, w_self=1.0, theta=0.5, tau=0.2):
        self.w_self = w_self
        self.theta = theta
        self.tau = tau
        self.p = 0.0

    def step(self, I, dt):
        target = 1.0 if self.w_self * self.p + I - self.theta > 0 else 0.0
        self.p += (target - self.p) * (1 - np.exp(-dt / self.tau))
        return self.p

    @property
    def phase(self):
        return 2 if self.p > 0.5 else 1


class BehaviourSelector:
    def __init__(self, signal_time=3.0):
        self.net = CompetitiveNetwork(BEHAVIOURS)
        self.phase_neuron = PhaseNeuron()
        # Integrates while SIGNAL wins; its activity reaching 1 - e^-1 marks the end of the signal.
        self.signal_timer = LeakyIntegrator(tau_rise=signal_time, tau_decay=0.2,
                                            theta_on=1 - np.exp(-1.0), theta_off=0.05)
        self.behaviour = "FOLLOW"

    def saliences(self, front_link, corner, rear_link, person_close, at_home):
        p2 = 1.0 if self.phase_neuron.phase == 2 else 0.0
        p1 = 1.0 - p2
        return np.array([
            p1 * front_link * (1 - corner),                    # FOLLOW
            p1 * (1 - front_link) * (1 - corner) * 0.9,        # SEARCH
            p1 * corner,                                       # SIGNAL
            p2 * rear_link * person_close * (1 - at_home),     # GUIDE
            p2 * (1 - rear_link * person_close) * (1 - at_home) * 0.9,  # WAIT
            p2 * at_home,                                      # DONE
        ])

    def step(self, dt, front_link, corner, rear_link, person_close, at_home):
        s = self.saliences(front_link, corner, rear_link, person_close, at_home)
        self.net.step(s, dt)
        winner = self.net.winner
        if winner is not None:
            self.behaviour = winner
        self.signal_timer.step(1.0 if self.behaviour == "SIGNAL" else 0.0, dt)
        self.phase_neuron.step(1.0 if self.signal_timer.active else 0.0, dt)
        return self.behaviour

    @property
    def phase(self):
        return self.phase_neuron.phase
