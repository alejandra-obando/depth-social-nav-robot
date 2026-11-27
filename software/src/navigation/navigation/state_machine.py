"""Behaviour of the robot during the two exam phases, as a finite-state machine.

    FOLLOW ──person unseen > lost_time──> LOST ──seen again──> FOLLOW
    FOLLOW ──far corner confirmed──> SIGNAL ──signal_time──> TURN_AROUND
    TURN_AROUND ──rear tag seen and facing home──> GUIDE
    GUIDE ──person too far or unseen──> WAIT_PERSON ──person close again──> GUIDE
    GUIDE ──start corner reached──> DONE

Only one state is active at a time, and every transition depends on explicit,
logged conditions, which makes a run easy to explain and debug.
"""

from dataclasses import dataclass
from enum import Enum


class State(str, Enum):
    FOLLOW = "FOLLOW"
    LOST = "LOST"
    SIGNAL = "SIGNAL"
    TURN_AROUND = "TURN_AROUND"
    GUIDE = "GUIDE"
    WAIT_PERSON = "WAIT_PERSON"
    DONE = "DONE"

    @property
    def phase(self):
        return 1 if self in (State.FOLLOW, State.LOST, State.SIGNAL) else 2


# Sound the speaker plays when a state is entered (None: silent).
ALERTS = {State.LOST: "lost", State.SIGNAL: "corner", State.WAIT_PERSON: "wait", State.DONE: "done"}


@dataclass
class Inputs:
    person_age: float        # seconds since the person's tag was last seen (by the camera of this phase)
    person_distance: float   # estimated distance to the person [m] (inf if unknown)
    far_corner: bool         # far corner confirmed by the LiDAR
    facing_home: bool        # heading within tolerance of the direction to the start corner
    home_reached: bool       # start corner reached


class StateMachine:
    def __init__(self, lost_time=1.5, found_time=0.2, signal_time=3.0, wait_lost_time=2.5,
                 wait_distance=1.2, resume_distance=1.0):
        self.lost_time = lost_time
        self.found_time = found_time
        self.signal_time = signal_time
        self.wait_lost_time = wait_lost_time
        self.wait_distance = wait_distance
        self.resume_distance = resume_distance
        self.state = State.FOLLOW
        self.time_in_state = 0.0
        self.history = []        # (time, old state, new state, reason)
        self.t = 0.0

    def _go(self, new, reason):
        self.history.append((round(self.t, 2), self.state.value, new.value, reason))
        self.state = new
        self.time_in_state = 0.0
        return ALERTS.get(new)

    def step(self, dt, i: Inputs):
        """Advance by dt.  Returns the alert to play if a state was just entered, else None."""
        self.t += dt
        self.time_in_state += dt
        s = self.state
        if s == State.FOLLOW:
            if i.far_corner:
                return self._go(State.SIGNAL, "far corner confirmed")
            # At start-up the person may still be getting in front of the camera.
            if i.person_age > self.lost_time and self.t > self.lost_time:
                return self._go(State.LOST, f"tag unseen for {min(i.person_age, self.t):.1f} s")
        elif s == State.LOST:
            if i.person_age < self.found_time:
                return self._go(State.FOLLOW, "tag seen again")
        elif s == State.SIGNAL:
            if self.time_in_state >= self.signal_time:
                return self._go(State.TURN_AROUND, "signal given")
        elif s == State.TURN_AROUND:
            if i.facing_home and i.person_age < self.found_time:
                return self._go(State.GUIDE, "rear tag seen, facing home")
        elif s == State.GUIDE:
            if i.home_reached:
                return self._go(State.DONE, "start corner reached")
            if i.person_age > self.wait_lost_time:
                return self._go(State.WAIT_PERSON, f"rear tag unseen for {i.person_age:.1f} s")
            if i.person_distance > self.wait_distance:
                return self._go(State.WAIT_PERSON, f"person at {i.person_distance:.2f} m")
        elif s == State.WAIT_PERSON:
            if i.home_reached:
                return self._go(State.DONE, "start corner reached")
            if i.person_age < self.found_time and i.person_distance < self.resume_distance:
                return self._go(State.GUIDE, "person close again")
        return None
