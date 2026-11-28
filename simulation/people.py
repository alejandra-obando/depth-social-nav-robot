"""Scripted people: the guide person and walkers driven by a social force model.

The guide person follows the exam protocol:
* phase 1: walks along its path ahead of the robot, and stops whenever the
  robot falls more than ~1.25 m behind (in the exam the person is asked to stop
  if the robot loses them);
* phase 2: once the robot has given its signal, turns around and walks behind
  the robot at about 0.8 m.  ``lag_events`` make them stop for a while, to test
  that the robot waits.

Walkers move between goals with a reduced social force model
(Helbing & Molnar 1995): a drive towards the goal plus exponential repulsion
from the robot and the other people.
"""

import numpy as np


def _yaw_quat(yaw):
    return np.array([np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)])


class GuidePerson:
    def __init__(self, path, speed=0.25, wait_dist=1.3, follow_dist=1.0, lag_events=()):
        self.path = [np.asarray(p, dtype=float) for p in path]
        self.speed = speed
        self.wait_dist = wait_dist
        self.follow_dist = follow_dist
        self.lag_events = list(lag_events)     # (t_start, duration) in phase-2 time
        self.pos = self.path[0].copy()
        self.yaw = np.arctan2(*(self.path[1] - self.path[0])[::-1])
        self.i = 1
        self.mode = "lead"
        self.t_phase2 = None

    def step(self, dt, t, robot_xy, robot_phase):
        robot_xy = np.asarray(robot_xy)
        if self.mode == "lead" and robot_phase == 2:
            self.mode = "follow"
            self.t_phase2 = t
        if self.mode == "lead":
            if self.i < len(self.path) and np.linalg.norm(self.pos - robot_xy) < self.wait_dist:
                d = self.path[self.i] - self.pos
                n = np.linalg.norm(d)
                step = min(n, self.speed * dt)
                if n > 1e-6:
                    self.pos += d / n * step
                    self.yaw = np.arctan2(d[1], d[0])
                if n < 0.05:
                    self.i += 1
        else:
            tp = t - self.t_phase2
            stopped = any(t0 <= tp < t0 + dur for t0, dur in self.lag_events)
            d = robot_xy - self.pos
            n = np.linalg.norm(d)
            self.yaw = np.arctan2(d[1], d[0])  # faces the robot: front sign towards the rear camera
            if not stopped and n > self.follow_dist:
                self.pos += d / n * min(self.speed * 1.3 * dt, n - self.follow_dist)
        return self.pos, self.yaw

    @property
    def at_goal(self):
        return self.i >= len(self.path)


class Walker:
    def __init__(self, goals, speed=0.6, A=3.0, B=0.4, tau=0.5, seed=0):
        self.goals = [np.asarray(g, dtype=float) for g in goals]
        self.k = 1
        self.pos = self.goals[0].copy()
        self.vel = np.zeros(2)
        self.speed = speed * np.random.default_rng(seed).uniform(0.85, 1.15)
        self.A, self.B, self.tau = A, B, tau

    def step(self, dt, others):
        goal = self.goals[self.k]
        d = goal - self.pos
        dist = np.linalg.norm(d)
        if dist < 0.3:
            self.k = (self.k + 1) % len(self.goals)
        f = (self.speed * d / max(dist, 1e-6) - self.vel) / self.tau
        for q, r in others:
            diff = self.pos - np.asarray(q)
            n = np.linalg.norm(diff)
            if 1e-6 < n < 2.5:
                f += self.A * np.exp((r - n) / self.B) * diff / n
        self.vel += f * dt
        sp = np.linalg.norm(self.vel)
        if sp > 1.3 * self.speed:
            self.vel *= 1.3 * self.speed / sp
        self.pos += self.vel * dt
        yaw = np.arctan2(self.vel[1], self.vel[0]) if sp > 0.05 else 0.0
        return self.pos, yaw


class Crosser:
    """Walks straight across the line between the robot and the guide, once.

    At ``t_start`` it appears ``reach`` metres to one side of the midpoint of that line
    and walks to the same distance on the other side, then leaves the room.  It does
    not avoid the guide (it passes in front of the back tag on purpose).
    """

    HIDDEN = np.array([-5.0, -5.0])

    def __init__(self, t_start, side=1, speed=0.8, reach=1.5, room=7.0):
        self.t_start, self.side, self.speed, self.reach, self.room = t_start, side, speed, reach, room
        self.pos = self.HIDDEN.copy()
        self.a = self.b = None
        self.done = False

    def step(self, dt, t, robot_xy, guide_xy, phase):
        if self.done or t < self.t_start or (self.a is None and phase != 1):
            return self.pos, 0.0
        if self.a is None:
            r, g = np.asarray(robot_xy, dtype=float), np.asarray(guide_xy, dtype=float)
            d = (g - r) / max(np.linalg.norm(g - r), 1e-6)
            mid, perp = (r + g) / 2, np.array([-d[1], d[0]]) * self.side
            self.a = np.clip(mid + perp * self.reach, 0.3, self.room - 0.3)
            self.b = np.clip(mid - perp * self.reach, 0.3, self.room - 0.3)
            self.pos = self.a.copy()
        d = self.b - self.pos
        n = np.linalg.norm(d)
        if n < self.speed * dt:
            self.done, self.pos = True, self.HIDDEN.copy()
            return self.pos, 0.0
        self.pos = self.pos + d / n * self.speed * dt
        return self.pos, float(np.arctan2(d[1], d[0]))


def set_mocap(model, data, body_name, xy, yaw, z=0.0):
    mid = model.body(body_name).mocapid[0]
    data.mocap_pos[mid] = [xy[0], xy[1], z]
    data.mocap_quat[mid] = _yaw_quat(yaw)
