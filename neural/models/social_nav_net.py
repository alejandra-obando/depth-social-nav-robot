"""Complete neural architecture for the exam: follow (phase 1) and guide (phase 2).

It does not depend on MuJoCo or ROS.  Each control step receives a
``SensorFrame`` with plain numbers (what the real robot gets from its topics)
and returns a ``Command``.  The same class runs in the simulation and, later,
inside a ROS 2 node on the robot.

Data flow (one step, ~20 Hz):

    front signs ──► NeuralField1D (person direction memory + selection) ──► front link (LeakyIntegrator)
    rear signs  ──────────────────────────────────────────────────────────► rear link  (LeakyIntegrator)
    LiDAR       ──► ReichardtArray (moving obstacles) ─┐
    LiDAR+depth ──► obstacle sectors ──────────────────┼─► HeadingDynamics ──► omega
                                                        └─► speed neuron   ──► v
    LiDAR+odom  ──► CornerDetector ─┐
    links, odom ────────────────────┴─► BehaviourSelector ──► which behaviour drives (v, omega)

Frames: robot frame x forward, y left, angles positive to the left.
Odometry frame: origin and heading where the robot started (the start corner).
"""

from dataclasses import dataclass, field
from typing import List, Tuple

import numpy as np

from .attractor_dynamics import HeadingDynamics, clearance, forward_speed
from .behaviour_selector import BehaviourSelector
from .corner_detector import CornerDetector
from .leaky_integrator import LeakyIntegrator
from .motion_detector import ReichardtArray
from .neural_field import NeuralField1D, wrap

Detection = Tuple[float, float, float]  # (bearing [rad, robot frame], distance [m], score [0..1])


@dataclass
class SensorFrame:
    scan_angles: np.ndarray            # LiDAR beam angles, robot frame [rad]
    scan_ranges: np.ndarray            # LiDAR ranges [m], inf = no return
    depth_angles: np.ndarray           # depth camera obstacle points, robot frame [rad]
    depth_ranges: np.ndarray           # horizontal distance to those points [m]
    front_signs: List[Detection]       # signs seen by the D415 (person's back in phase 1)
    rear_signs: List[Detection]        # signs seen by the C270 (person's front in phase 2)
    odom: Tuple[float, float, float]   # (x, y, yaw) in the odometry frame
    scan_new: bool = True              # False when the LiDAR scan is the same as in the last step


@dataclass
class Command:
    v: float                           # forward speed [m/s]
    omega: float                       # turning rate [rad/s], positive = left
    signal: bool                       # visual signal on (end of phase 1 / arrival)
    behaviour: str
    phase: int
    debug: dict = field(default_factory=dict)


class SocialNavNet:
    def __init__(self, d_ref=0.9, v_max=0.22, v_guide=0.18, rear_max=1.2, home_radius=0.3,
                 n_sectors=72, use_motion=True, use_memory=True):
        self.d_ref = d_ref
        self.v_max = v_max
        self.v_guide = v_guide
        self.rear_max = rear_max
        self.home_radius = home_radius
        self.n_sectors = n_sectors
        self.use_motion = use_motion
        self.use_memory = use_memory
        self.sector_angles = (np.arange(n_sectors) + 0.5) * 2 * np.pi / n_sectors - np.pi  # centres

        self.person_field = NeuralField1D()
        self.front_link = LeakyIntegrator(tau_rise=0.2, tau_decay=3.0, theta_on=0.6, theta_off=0.3)
        self.rear_link = LeakyIntegrator(tau_rise=0.2, tau_decay=2.0, theta_on=0.6, theta_off=0.3)
        self.motion = ReichardtArray(n_sectors=n_sectors)
        # Habituation of the yield response: if something keeps moving next to the robot for
        # several seconds, the robot stops yielding and goes around it with the heading dynamics.
        self.yield_adaptation = LeakyIntegrator(tau_rise=4.0, tau_decay=2.0, theta_on=0.5, theta_off=0.2)
        self.heading = HeadingDynamics(sector_width=2 * np.pi / n_sectors)
        self.corner = CornerDetector()
        # Start corner in phase 2: odometry says "close" and the LiDAR sees both walls close
        # (corrects the odometry drift accumulated over the round trip).
        self.home_corner = CornerDetector(d_wall=0.8, min_travel=0.0)
        self.selector = BehaviourSelector()

        # Short-term memory of low obstacles seen by the depth camera (odometry frame). The LiDAR
        # passes over 10 cm stacks, so once a stack leaves the camera's view while the robot turns
        # past it, this memory still repels the robot.  Each point fades after ``depth_memory_s``.
        self.depth_memory_s = 4.0
        self.depth_mem = np.zeros((0, 3))      # (x, y, time)
        self.t = 0.0

        self.person_dist = None   # last distance to the person in front [m]
        self.rear_dist = None     # last distance to the person behind [m]
        self.prev_odom = None
        self.motion_odom = None   # odometry at the last motion-detector update
        self.motion_dt = 0.0
        self.travelled = 0.0
        self.search_dir = 1.0

    # ------------------------------------------------------------------ helpers
    def _sectors(self, angles, ranges):
        """Closest obstacle per angular sector."""
        r = np.full(self.n_sectors, np.inf)
        if len(angles):
            idx = ((np.asarray(angles) + np.pi) / (2 * np.pi) * self.n_sectors).astype(int) % self.n_sectors
            np.minimum.at(r, idx, np.asarray(ranges, dtype=float))
        return r

    @staticmethod
    def _relative(odom0, odom1):
        """Displacement from pose odom0 to odom1, expressed in the odom0 robot frame."""
        x0, y0, a0 = odom0
        x1, y1, a1 = odom1
        c, s = np.cos(-a0), np.sin(-a0)
        return (c * (x1 - x0) - s * (y1 - y0), s * (x1 - x0) + c * (y1 - y0), float(wrap(a1 - a0)))

    def _update_travelled(self, odom):
        if self.prev_odom is not None:
            self.travelled += float(np.hypot(odom[0] - self.prev_odom[0], odom[1] - self.prev_odom[1]))
        self.prev_odom = odom

    def _depth_memory(self, frame, x, y, yaw):
        """Add the new depth points to the memory, forget old/far ones, return them in the robot frame."""
        a = np.asarray(frame.depth_angles, dtype=float)
        r = np.asarray(frame.depth_ranges, dtype=float)
        near = r < 1.5
        if near.any():
            wx = x + r[near] * np.cos(yaw + a[near])
            wy = y + r[near] * np.sin(yaw + a[near])
            new = np.stack([wx, wy, np.full(near.sum(), self.t)], 1)
            # Thin out: one point per 5 cm cell is enough.
            cells = np.round(new[:, :2] / 0.05)
            _, keep = np.unique(cells, axis=0, return_index=True)
            self.depth_mem = np.vstack([self.depth_mem, new[keep]])
        m = self.depth_mem
        if len(m):
            dx, dy = m[:, 0] - x, m[:, 1] - y
            m = m[(self.t - m[:, 2] < self.depth_memory_s) & (dx * dx + dy * dy < 1.5 ** 2)]
            # Keep the newest point per cell.
            cells = np.round(m[:, :2] / 0.05)
            order = np.argsort(-m[:, 2])
            _, first = np.unique(cells[order], axis=0, return_index=True)
            self.depth_mem = m[order[first]]
        m = self.depth_mem
        if not len(m):
            return np.zeros(0), np.zeros(0)
        dx, dy = m[:, 0] - x, m[:, 1] - y
        return wrap(np.arctan2(dy, dx) - yaw), np.hypot(dx, dy)

    def _lidar_person(self, frame, theta_mem, max_angle=np.radians(15), max_jump=0.35):
        """Closest LiDAR return near the remembered person direction and distance, or None."""
        a = np.asarray(frame.scan_angles, dtype=float)
        r = np.asarray(frame.scan_ranges, dtype=float)
        ok = np.isfinite(r) & (np.abs(wrap(a - theta_mem)) < max_angle) & \
            (np.abs(r - self.person_dist) < max_jump)
        if not ok.any():
            return None
        i = np.flatnonzero(ok)[np.argmin(np.abs(wrap(a[ok] - theta_mem)))]
        return float(a[i]), float(r[i])

    # --------------------------------------------------------------------- step
    def step(self, frame: SensorFrame, dt: float) -> Command:
        x, y, yaw = frame.odom
        self._update_travelled(frame.odom)
        self.t += dt

        # 1. Person in front: field over world directions, selection + memory.
        #    Signs are the strong input.  While no sign is seen, a LiDAR return at the remembered
        #    direction and distance (the person's legs) is a weaker supporting input, so the field
        #    and the link hold the person while they are briefly out of the camera's view
        #    (multi-sensor input to the field, as in Bicho, Mallet & Schoner 2000).
        dirs = [yaw + b for b, _, _ in frame.front_signs]
        amps = [3.0 * s for _, _, s in frame.front_signs]
        prev_peak = self.person_field.peak()
        lidar_hit = None
        if not frame.front_signs and prev_peak is not None and self.person_dist is not None \
                and self.selector.phase == 1:
            lidar_hit = self._lidar_person(frame, float(wrap(prev_peak - yaw)))
            if lidar_hit is not None:
                dirs, amps = [yaw + lidar_hit[0]], [1.5]
        self.person_field.step(dt, dirs, amps)
        if not self.use_memory and not frame.front_signs:
            self.person_field.reset()
        peak = self.person_field.peak()
        theta_person = None if peak is None else float(wrap(peak - yaw))
        matched = [d for b, d, _ in frame.front_signs
                   if theta_person is not None and abs(wrap(b - theta_person)) < np.radians(20)]
        if matched:
            self.person_dist = min(matched)
            link_input = 1.0
        elif lidar_hit is not None:
            self.person_dist = lidar_hit[1]
            link_input = 0.6      # holds an existing link (above theta_off) but cannot create one
        else:
            link_input = 0.0
        self.front_link.step(link_input, dt)

        # 2. Person behind (phase 2).
        if frame.rear_signs:
            self.rear_dist = min(d for _, d, _ in frame.rear_signs)
        self.rear_link.step(1.0 if frame.rear_signs else 0.0, dt)

        # 3. Obstacles: LiDAR + depth camera, the followed person masked out.
        mem_a, mem_r = self._depth_memory(frame, x, y, yaw)
        obs = np.minimum(self._sectors(frame.scan_angles, frame.scan_ranges),
                         self._sectors(mem_a, mem_r))
        if theta_person is not None and self.person_dist is not None and self.selector.phase == 1:
            near_person = (np.abs(wrap(self.sector_angles - theta_person)) < np.radians(25)) & \
                          (obs < self.person_dist + 0.5)
            obs = np.where(near_person, np.inf, obs)

        # 4. Moving obstacles (Reichardt): stronger repulsion, and yield to crossings in front.
        #    The detector runs only on new scans, with the ego-motion since the previous scan.
        gains = np.ones(self.n_sectors)
        yield_inhibition = 0.0
        if self.use_motion:
            self.motion_dt += dt
            if frame.scan_new or self.motion_odom is None:
                ego = (0.0, 0.0, 0.0) if self.motion_odom is None else self._relative(self.motion_odom, frame.odom)
                self.motion.step(self.motion_dt, frame.scan_angles, frame.scan_ranges, ego)
                self.motion_odom = frame.odom
                self.motion_dt = 0.0
            # Soft threshold over the noise floor of the detector (~0.01 for a static scene).
            moving = np.clip((self.motion.motion_energy() - 0.03) / 0.07, 0.0, 1.0)
            gains = 1.0 + 4.0 * moving
            # Yield to something moving close in front or crossing from the sides.
            danger = (np.abs(self.sector_angles) < np.radians(70)) & (obs < 1.1)
            drive = float(moving[danger].max()) if danger.any() else 0.0
            adapt = self.yield_adaptation.step(drive, dt)
            yield_inhibition = drive * (1.0 - adapt)

        # 5. Corner and home.
        self.corner.step(dt, frame.scan_angles, frame.scan_ranges, self.travelled)
        home_dist = float(np.hypot(x, y))
        theta_home = float(wrap(np.arctan2(-y, -x) - yaw))
        near_home = self.selector.phase == 2 and home_dist < 1.2
        self.home_corner.step(dt, frame.scan_angles, frame.scan_ranges, 1.0 if near_home else -1.0)
        at_home = self.selector.phase == 2 and (home_dist < self.home_radius or
                                                (near_home and self.home_corner.detected))

        # 6. Behaviour selection.
        person_close = 1.0 if (self.rear_dist is not None and self.rear_dist < self.rear_max) else 0.0
        behaviour = self.selector.step(
            dt,
            front_link=1.0 if self.front_link.active else 0.0,
            corner=1.0 if self.corner.detected else 0.0,
            rear_link=1.0 if self.rear_link.active else 0.0,
            person_close=person_close,
            at_home=1.0 if at_home else 0.0,
        )

        # 7. Motor output of the winning behaviour.
        v, omega, signal = 0.0, 0.0, False
        clear = clearance(self.sector_angles, obs)
        if behaviour == "FOLLOW" and theta_person is not None:
            omega = self.heading.omega(theta_person, self.sector_angles, obs, gains=gains)
            v = forward_speed(self.person_dist or self.d_ref, self.d_ref, v_max=self.v_max,
                              theta_tar=theta_person, clearance_factor=clear,
                              inhibition=yield_inhibition)
            self.search_dir = np.sign(theta_person) or 1.0
        elif behaviour == "SEARCH":
            # The person is asked to stop: stay in place and turn towards the remembered direction.
            target = theta_person if theta_person is not None else self.search_dir * np.pi / 2
            omega = 0.6 * self.heading.target_force(target)
        elif behaviour == "SIGNAL":
            signal = True
        elif behaviour in ("GUIDE", "WAIT"):
            omega = self.heading.omega(theta_home, self.sector_angles, obs, gains=gains)
            if behaviour == "GUIDE":
                lag = np.clip((self.rear_max - (self.rear_dist or self.rear_max)) / 0.4, 0.0, 1.0)
                v = self.v_guide * lag * clear * max(0.0, np.cos(theta_home)) * (1.0 - yield_inhibition)
                v *= np.clip(home_dist / 0.8, 0.3, 1.0)
            else:
                omega *= 0.6
        elif behaviour == "DONE":
            signal = True

        return Command(v=v, omega=omega, signal=signal, behaviour=behaviour, phase=self.selector.phase,
                       debug=dict(theta_person=theta_person, person_dist=self.person_dist,
                                  front_link=self.front_link.u, rear_link=self.rear_link.u,
                                  rear_dist=self.rear_dist, corner=self.corner.integrator.u,
                                  yield_=yield_inhibition, travelled=self.travelled,
                                  home_dist=home_dist, field_u=self.person_field.u.copy()))
