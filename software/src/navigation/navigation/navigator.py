"""The whole conventional controller in one object, without ROS.

``Navigator.step`` receives what the sensors measured in the last control period
and returns the wheel command.  The ROS 2 node (``bringup``) and the MuJoCo
simulation (``simulation/run_conventional.py``) both call it, so the logic tested
in simulation is the logic that runs on the robot.

    tags, scan, depth, odometry
        -> PersonTracker (Kalman)          person distance and bearing
        -> ObstacleMap                     LiDAR + remembered depth points
        -> CornerConfirmer x 2             far corner (phase 1), start corner (phase 2)
        -> StateMachine                    FOLLOW / LOST / SIGNAL / TURN_AROUND / GUIDE / WAIT_PERSON / DONE
        -> FollowController or GoToGoal    desired direction and speed
        -> VFH                             direction that avoids obstacles
        -> (v, omega)
"""

from dataclasses import dataclass, field

import numpy as np

from corners.corner_detector import CornerConfirmer
from perception.obstacle_map import ObstacleMap
from perception.person_tracker import PersonTracker, to_robot

from .controllers import FollowController, GoToGoal
from .diff_drive import wrap
from .state_machine import Inputs, State, StateMachine
from .vfh import VFH


@dataclass
class Observation:
    dt: float
    pose: tuple                    # odometry (x, y, yaw); the start pose is (0, 0, 0) facing the far corner
    tags: list = field(default_factory=list)      # TagObservation from both cameras
    scan: tuple = None             # (angles, ranges) when a new LiDAR scan arrived, else None
    depth: tuple = None            # (angles, ranges) of depth obstacle points, else None


@dataclass
class Command:
    v: float
    omega: float
    state: str
    phase: int
    alert: str = None              # sound to play now ("lost", "corner", "wait", "done") or None
    debug: dict = field(default_factory=dict)


@dataclass
class Params:
    # Tags (tag36h11): the back tag is seen by the front camera, the chest tag by the rear camera.
    back_tag_id: int = 0
    chest_tag_id: int = 1
    # Room geometry, used only as loose priors for the corner detector.
    room_size: float = 7.0
    start_to_corner: float = 0.85     # distance from the start pose back to the start corner [m]
    # Following.
    follow_distance: float = 0.9
    v_follow: float = 0.22
    # Guiding.
    v_guide: float = 0.18
    home_radius: float = 0.25         # odometry distance that counts as "at the start" [m]
    # Turning.
    k_omega: float = 1.5
    omega_max: float = 1.0
    # State machine.
    lost_time: float = 0.5           # tag unseen this long -> LOST: the robot stops at once [s]
    wait_lost_time: float = 2.5
    signal_time: float = 3.0
    wait_distance: float = 1.2
    resume_distance: float = 1.0
    # Search when the person is lost: turn towards the last bearing for this many seconds,
    # then sweep in place at search_speed [rad/s] until the tag is seen again.
    search_after: float = 5.0
    search_speed: float = 0.35
    # Far corner: closer than this to the robot [m].
    far_corner_distance: float = 1.8
    # "Careful with the obstacles": said when VFH slows the robot below this speed factor.
    obstacle_speed: float = 0.5


class Navigator:
    def __init__(self, params: Params = None):
        p = self.p = params or Params()
        self.tracker = PersonTracker()
        self.obstacles = ObstacleMap(person_radius=0.45)
        diag = p.room_size * np.sqrt(2) - p.start_to_corner
        self.far_corner = CornerConfirmer(max_distance=p.far_corner_distance, expected=(diag, 0.0))
        self.home_corner = CornerConfirmer(max_distance=p.start_to_corner + 0.6,
                                           expected=(-p.start_to_corner, 0.0), expected_radius=1.0)
        self.home = np.zeros(2)
        self.fsm = StateMachine(lost_time=p.lost_time, wait_lost_time=p.wait_lost_time, signal_time=p.signal_time,
                                wait_distance=p.wait_distance, resume_distance=p.resume_distance)
        self.follow = FollowController(d_ref=p.follow_distance, v_max=p.v_follow)
        self.goto = GoToGoal(v_max=p.v_guide)
        self.vfh = VFH()
        self.started = False          # "start" said once, when the back tag is first seen
        self.near_obstacle = False

    @property
    def state(self):
        return self.fsm.state

    def step(self, obs: Observation) -> Command:
        p, pose, dt = self.p, obs.pose, obs.dt
        state = self.fsm.state
        self.tracker.predict(dt)
        self.obstacles.step(dt)

        # Person: only the tag of the current phase, from the camera that faces it.
        want = ("front", p.back_tag_id) if state.phase == 1 else ("rear", p.chest_tag_id)
        for t in obs.tags:
            if (t.camera, t.tag_id) == want:
                self.tracker.update(pose, t)
        dist, bearing = self.tracker.relative(pose)
        dist = np.inf if dist is None else dist

        # Obstacles and corners.
        if obs.scan is not None:
            self.obstacles.add_scan(*obs.scan)
            if state == State.FOLLOW:
                self.far_corner.update(*obs.scan, pose)
            elif state in (State.GUIDE, State.WAIT_PERSON):
                self.home_corner.update(*obs.scan, pose)
        if obs.depth is not None:
            self.obstacles.add_depth(pose, *obs.depth)
        person_xy = None if self.tracker.position() is None else to_robot(pose, self.tracker.position())
        points = self.obstacles.points(pose, person_xy)

        rho, alpha = self.goto.polar(pose, self.home)
        # The LiDAR corrects the odometry drift at the end: the robot started
        # ``start_to_corner`` from the corner, so it stops when the corner is that close.
        corner_home = (self.home_corner.confirmed and rho < 0.8 and
                       self.home_corner.last.distance < p.start_to_corner + 0.05)
        inputs = Inputs(person_age=self.tracker.age, person_distance=dist,
                        far_corner=self.far_corner.confirmed, facing_home=abs(alpha) < 0.5,
                        home_reached=rho < p.home_radius or corner_home)
        alert = self.fsm.step(dt, inputs)
        state = self.fsm.state
        if alert == "corner":
            self.follow.reset()
        if state in (State.LOST, State.WAIT_PERSON) and self.tracker.initialised:
            # The person was just asked to wait: assume they stopped where they were last seen,
            # instead of extrapolating their last velocity (which would point the camera off them).
            self.tracker.x[2:] = 0.0
        if state == State.TURN_AROUND and self.fsm.time_in_state == 0.0:
            # From now on the person is tracked from the chest tag: start a fresh track.
            self.tracker = PersonTracker()

        v, omega, steer, speed = 0.0, 0.0, None, 0.0
        if state == State.FOLLOW and bearing is None:
            pass                                  # start-up: wait until the person's tag is seen
        elif state == State.FOLLOW:
            v, target = self.follow(dist, bearing, dt, self.tracker.radial_speed(pose))
            steer, speed = self.vfh(points, target)
            if steer is None:
                v, omega = 0.0, self._turn(target)
            else:
                v, omega = v * speed * max(0.0, np.cos(steer)), self._turn(steer)
        elif state == State.LOST:
            # Stop (the person is asked to stop) and point the front camera at them.
            omega = self._search(bearing, camera_angle=0.0)
        elif state == State.TURN_AROUND:
            omega = self._turn(alpha, limit=0.8)
        elif state == State.GUIDE:
            steer, speed = self.vfh(points, alpha)
            if steer is None:
                omega = self._turn(alpha)
            else:
                v, omega = self.goto(rho, steer)
                v *= speed
                if dist > p.resume_distance:      # slow down while the person falls behind
                    v *= float(np.clip((p.wait_distance - dist) / (p.wait_distance - p.resume_distance), 0.3, 1))
        elif state == State.WAIT_PERSON:
            # Stop and point the rear camera at the person.
            omega = self._search(bearing, camera_angle=np.pi)
        omega = float(np.clip(omega, -p.omega_max, p.omega_max))

        # Spoken alerts that are not state changes (the speaker limits how often they repeat).
        if not self.started and state == State.FOLLOW and bearing is not None:
            self.started, alert = True, alert or "start"
        near = steer is not None and speed < p.obstacle_speed and v > 0
        if near and not self.near_obstacle:
            alert = alert or "obstacle"
        self.near_obstacle = near

        return Command(v=float(v), omega=omega, state=state.value, phase=state.phase, alert=alert,
                       debug=dict(person_distance=float(dist), person_bearing=bearing,
                                  person_age=float(self.tracker.age), rho=rho, alpha=alpha,
                                  steer=steer, vfh_speed=speed,
                                  far_corner_count=self.far_corner.count,
                                  home_corner_count=self.home_corner.count))

    def _search(self, bearing, camera_angle):
        """Turning speed that brings the person back into the view of the camera at ``camera_angle``.

        While the estimate is recent the robot turns towards it; once it is older than
        ``search_after`` it is no longer trusted and the robot sweeps slowly, in the
        direction where the person was last estimated, until the tag is seen again.
        """
        if bearing is None:
            return self.p.search_speed
        err = wrap(bearing - camera_angle)
        if self.tracker.age < self.p.search_after:
            # Centre the estimate in the camera (a 20 deg dead band left the person at the edge
            # of the D415's 65 deg view, unseen, for the whole 5 s).
            return float(np.clip(0.8 * err, -0.5, 0.5)) if abs(err) > 0.1 else 0.0
        return float(np.sign(err) or 1.0) * self.p.search_speed

    def _turn(self, angle, limit=None):
        lim = self.p.omega_max if limit is None else limit
        return float(np.clip(self.p.k_omega * angle, -lim, lim))
