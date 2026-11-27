"""Leaky integrator neuron.

    tau * du/dt = -u + I(t)

The time constant can differ when the input is above the state (rise) and below
it (decay): a fast attack and a slow release.  The binary readout has
hysteresis: it switches on above ``theta_on`` and off below ``theta_off``.

Used for the link with the person: while the sign is seen the activity
rises quickly; when the person is hidden it decays slowly, so a short
occlusion does not break the link (Usher & McClelland 2001, leaky accumulation).
"""

import math


class LeakyIntegrator:
    def __init__(self, tau_rise=0.2, tau_decay=None, theta_on=0.6, theta_off=0.3, u0=0.0):
        if theta_off > theta_on:
            raise ValueError("theta_off must be <= theta_on")
        self.tau_rise = tau_rise
        self.tau_decay = tau_rise if tau_decay is None else tau_decay
        self.theta_on = theta_on
        self.theta_off = theta_off
        self.u = u0
        self.active = u0 >= theta_on

    def step(self, I, dt):
        """Advance by ``dt`` seconds with constant input ``I``; return the activity."""
        tau = self.tau_rise if I >= self.u else self.tau_decay
        # Exact solution for a constant input over dt (stable for any dt).
        self.u = I + (self.u - I) * math.exp(-dt / tau)
        if not self.active and self.u >= self.theta_on:
            self.active = True
        elif self.active and self.u <= self.theta_off:
            self.active = False
        return self.u

    def reset(self, u0=0.0):
        self.u = u0
        self.active = u0 >= self.theta_on
