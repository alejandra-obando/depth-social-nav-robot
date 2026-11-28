import numpy as np

from navigation.controllers import FollowController, GoToGoal
from navigation.diff_drive import DiffDrive
from navigation.state_machine import Inputs, State, StateMachine
from navigation.vfh import VFH


def test_diff_drive_round_trip_and_saturation():
    dd = DiffDrive(max_wheel_speed=8.0)
    wl, wr = dd.to_wheels(0.15, 0.4)
    assert np.allclose(dd.from_wheels(wl, wr), (0.15, 0.4))
    wl, wr = dd.to_wheels(1.0, 1.0)                        # too fast: scaled, same curvature
    v, w = dd.from_wheels(wl, wr)
    assert max(abs(wl), abs(wr)) <= 8.0 + 1e-9 and np.isclose(w / v, 1.0)


def test_follow_controller():
    f = FollowController(d_ref=0.9, v_max=0.22)
    v, d = f(0.9, 0.0, 0.05)
    assert v == 0.0 and d == 0.0
    v, _ = f(1.5, 0.0, 0.05)
    assert v > 0
    f.reset()
    v, _ = f(1.5, np.pi * 0.6, 0.05)                       # person behind: turn first, do not drive
    assert v == 0.0
    f.reset()
    v_ff, _ = f(0.9, 0.0, 0.05, v_person=0.2)              # walking person at the right distance
    assert np.isclose(v_ff, 0.2)


def test_go_to_goal_reaches_the_goal():
    g = GoToGoal()
    dd = DiffDrive()
    pose = (3.0, 2.0, np.pi / 2)
    for _ in range(2000):
        rho, alpha = g.polar(pose, (0.0, 0.0))
        if rho < 0.05:
            break
        v, w = g(rho, alpha)
        pose = dd.integrate(pose, v, w, 0.05)
    assert np.hypot(pose[0], pose[1]) < 0.05


def test_vfh_free_space_keeps_target():
    vfh = VFH()
    d, s = vfh(np.zeros((0, 2)), 0.3)
    assert np.isclose(d, 0.3) and s == 1.0


def test_vfh_avoids_obstacle_ahead():
    vfh = VFH()
    # A row of points across the path, 0.5 m ahead, from y = -0.2 to 0.2.
    pts = np.stack([np.full(9, 0.5), np.linspace(-0.2, 0.2, 9)], 1)
    d, s = vfh(pts, 0.0)
    assert d is not None and abs(d) > np.radians(30)
    # The chosen direction itself is clear of the obstacle.
    k = vfh._sector(d)
    assert not vfh.blocked[k]


def test_vfh_all_blocked():
    vfh = VFH()
    a = np.linspace(-np.pi, np.pi, 200, endpoint=False)
    d, s = vfh(np.stack([0.3 * np.cos(a), 0.3 * np.sin(a)], 1), 0.0)
    assert d is None and s == 0.0


def _run(fsm, inputs, seconds, dt=0.05):
    alerts = []
    for _ in range(int(seconds / dt)):
        a = fsm.step(dt, inputs)
        if a:
            alerts.append(a)
    return alerts


def test_state_machine_whole_exam():
    fsm = StateMachine()
    seen = dict(person_age=0.0, person_distance=0.9, far_corner=False, facing_home=False, home_reached=False)
    _run(fsm, Inputs(**seen), 2.0)
    assert fsm.state == State.FOLLOW
    assert _run(fsm, Inputs(**{**seen, "person_age": 2.0}), 0.1) == ["lost"]
    assert fsm.state == State.LOST
    assert _run(fsm, Inputs(**seen), 0.1) == ["found"]
    assert fsm.state == State.FOLLOW
    assert _run(fsm, Inputs(**{**seen, "far_corner": True}), 0.1) == ["corner"]
    _run(fsm, Inputs(**seen), 3.1)
    assert fsm.state == State.TURN_AROUND
    assert _run(fsm, Inputs(**{**seen, "facing_home": True}), 0.1) == ["guide"]
    assert fsm.state == State.GUIDE
    assert _run(fsm, Inputs(**{**seen, "person_distance": 1.4}), 0.1) == ["wait"]
    assert fsm.state == State.WAIT_PERSON
    assert _run(fsm, Inputs(**seen), 0.1) == ["found"]
    assert fsm.state == State.GUIDE
    assert _run(fsm, Inputs(**{**seen, "home_reached": True}), 0.1) == ["done"]
    assert fsm.state == State.DONE


def test_no_lost_alert_at_start_up():
    fsm = StateMachine(lost_time=0.5)
    unseen = Inputs(person_age=np.inf, person_distance=np.inf, far_corner=False, facing_home=False,
                    home_reached=False)
    assert _run(fsm, unseen, 0.45) == []
