"""Runs both exam phases in the simulated room with the conventional controller.

    python -m simulation.run_conventional --scenario full --seed 0
    python -m simulation.run_conventional --scenario full --view     # open the MuJoCo viewer

The controller is ``software/src/navigation/navigation/navigator.py``, the same
code the ROS 2 node runs on the robot.  The simulation gives it:

* rendered D415 colour and C270 images -> the real AprilTag detector (tag36h11);
* a rendered D415 depth image -> the real depth-obstacle code;
* the simulated LD19 scan and wheel/IMU odometry from ``sensors.py``.

It measures what the rubric grades (robot-person distance <= 1.5 m, collisions,
link losses, corner recognition, arrival at the start corner, time) and writes a
plot and a JSON summary to simulation/results/.
"""

import argparse
import json
import os
import sys
import time
from pathlib import Path

if "--view" not in sys.argv:
    os.environ.setdefault("MUJOCO_GL", "egl")      # off-screen rendering without a display

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from . import world as W  # noqa: E402
from .cameras import RenderedCamera, RenderedDepth, mount_from_model  # noqa: E402
from .people import GuidePerson, Walker, set_mocap  # noqa: E402
from .sensors import Lidar, Odometry  # noqa: E402
from .software_path import add_software_to_path  # noqa: E402

add_software_to_path()

from navigation.diff_drive import DiffDrive  # noqa: E402
from navigation.navigator import Navigator, Observation, Params  # noqa: E402
from perception.apriltag_detector import TagDetector  # noqa: E402
from perception.depth_obstacles import DepthObstacles  # noqa: E402

CONTROL_DT = 0.05
CAMERA_EVERY = 2          # cameras and LiDAR at 10 Hz
RESULTS = Path(__file__).resolve().parent / "results"
STATES = ["FOLLOW", "LOST", "SIGNAL", "TURN_AROUND", "GUIDE", "WAIT_PERSON", "DONE"]


def run(scenario="full", seed=0, max_time=400.0, view=False, lag_events=((15.0, 5.0),), plot=True,
        tag=None, quiet=False, params=None):
    layout = W.make_layout(scenario, seed)
    _, model = W.build(layout, tags=True)
    data = mujoco.MjData(model)
    data.qpos[0:3] = [W.START[0], W.START[1], 0.03]
    data.qpos[3:7] = W.robot_quat(W.START_YAW)
    mujoco.mj_forward(model, data)

    guide = GuidePerson([W.START + np.array([0.9, 0.9]) / np.sqrt(2) * 1.0] + layout.guide_path,
                        lag_events=lag_events)
    walkers = [Walker(g, seed=seed + i) for i, g in enumerate(layout.walkers)]
    set_mocap(model, data, "guide", guide.pos, guide.yaw)
    for _ in range(int(1.0 / model.opt.timestep)):     # let the robot settle
        mujoco.mj_step(model, data)

    # Sensors.
    lidar = Lidar(model, seed=seed)
    odom = Odometry(model, data, seed=seed)
    front = RenderedCamera(model, "d415_color")
    rear = RenderedCamera(model, "c270")
    depth_cam = RenderedDepth(model, seed=seed)
    detectors = [(front, TagDetector("front", front.intrinsics, mount_from_model(model, data, "d415_color"),
                                     tag_size=W.TAG_SIZE)),
                 (rear, TagDetector("rear", rear.intrinsics, mount_from_model(model, data, "c270"),
                                    tag_size=W.TAG_SIZE))]
    depth = DepthObstacles(depth_cam.intrinsics, mount_from_model(model, data, "d415_prof"), stride=2)

    nav = Navigator(params or Params())
    drive = DiffDrive()

    robot_bodies = {model.body(n).id for n in ("base", "part_2", "part_2_2")}
    light = model.geom("signal_light").id
    n_sub = int(round(CONTROL_DT / model.opt.timestep))
    viewer = None
    if view:
        from mujoco import viewer as mj_viewer
        viewer = mj_viewer.launch_passive(model, data)

    log = {k: [] for k in ("t", "rx", "ry", "px", "py", "dist", "state", "phase", "seen", "v", "omega",
                           "walkers")}
    alerts, last_contact, collisions, omega_hist = [], {}, [], []
    t, step_i, t0_wall = 0.0, 0, time.time()
    corner_time = done_time = None
    detect_ms = []

    while t < max_time:
        rx, ry, _ = W.robot_pose(model, data)
        gpos, gyaw = guide.step(CONTROL_DT, t, (rx, ry), nav.state.phase)
        set_mocap(model, data, "guide", gpos, gyaw)
        others_base = [((rx, ry), 0.25), (tuple(gpos), 0.35)]
        for i, wk in enumerate(walkers):
            others = others_base + [(tuple(o.pos), 0.4) for j, o in enumerate(walkers) if j != i]
            p, yaw = wk.step(CONTROL_DT, others)
            set_mocap(model, data, f"walker_{i}", p, yaw)
        if layout.moving_briefcase is not None:
            x0, y0, x1, y1, ts, dur = layout.moving_briefcase
            a = np.clip((t - ts) / dur, 0, 1)
            set_mocap(model, data, "moving_briefcase", (x0 + a * (x1 - x0), y0 + a * (y1 - y0)), 0.4, z=0.175)
        mujoco.mj_forward(model, data)

        # Sensors -> observation.
        pose = odom.step(model, data, CONTROL_DT)
        obs = Observation(dt=CONTROL_DT, pose=pose)
        if step_i % CAMERA_EVERY == 0:
            obs.scan = lidar.read(model, data)
            t_det = time.time()
            for cam, det in detectors:
                obs.tags += det.detect(cam.gray(data))
            detect_ms.append((time.time() - t_det) * 1000)
            obs.depth = depth.points(depth_cam.depth(data))

        cmd = nav.step(obs)
        wl, wr = drive.to_wheels(cmd.v, cmd.omega)
        data.ctrl[:] = [-wl, wr]                 # motor_izq is mounted mirrored: negative is forward
        if cmd.alert:
            alerts.append((round(t, 2), cmd.alert))
        blink = cmd.state in ("SIGNAL", "DONE") and int(t * 4) % 2 == 0
        model.geom_rgba[light] = [0.1, 1.0, 0.2, 1] if blink else [0.3, 0.3, 0.3, 1]
        omega_hist = (omega_hist + [abs(cmd.omega)])[-20:]

        for _ in range(n_sub):
            mujoco.mj_step(model, data)

        # Collisions: new contacts between the robot and walls/obstacles/people.
        now = set()
        for c in data.contact[:data.ncon]:
            b1, b2 = model.geom_bodyid[c.geom1], model.geom_bodyid[c.geom2]
            for rb, og in ((b1, c.geom2), (b2, c.geom1)):
                if rb in robot_bodies and model.geom_bodyid[og] not in robot_bodies and model.geom_group[og] == 1:
                    now.add(int(og))
        for g in now:
            if t - last_contact.get(g, -10.0) > 1.0:
                manoeuvre = max(omega_hist) > 0.3
                collisions.append(dict(t=round(t, 2), obstacle=model.geom(g).name,
                                       kind="minor (during manoeuvre)" if manoeuvre else "serious"))
            last_contact[g] = t

        rx, ry, _ = W.robot_pose(model, data)
        log["t"].append(t); log["rx"].append(rx); log["ry"].append(ry)
        log["px"].append(gpos[0]); log["py"].append(gpos[1])
        log["dist"].append(float(np.hypot(gpos[0] - rx, gpos[1] - ry)))
        log["state"].append(cmd.state); log["phase"].append(cmd.phase)
        log["seen"].append(cmd.debug["person_age"] < 0.3)
        log["v"].append(cmd.v); log["omega"].append(cmd.omega)
        log["walkers"].append([tuple(w.pos) for w in walkers])
        if cmd.state == "SIGNAL" and corner_time is None:
            corner_time = t
        if cmd.state == "DONE" and done_time is None:
            done_time = t
        if viewer is not None:
            viewer.sync()
            time.sleep(max(0.0, CONTROL_DT - (time.time() - t0_wall - t)))
            if not viewer.is_running():
                break
        t += CONTROL_DT
        step_i += 1
        if done_time is not None and t - done_time > 2.0:
            break

    summary = summarise(log, collisions, corner_time, done_time)
    summary.update(scenario=scenario, seed=seed, alerts=alerts, transitions=nav.fsm.history,
                   detector_ms_per_frame=round(float(np.mean(detect_ms)), 1) if detect_ms else None,
                   wall_time_s=round(time.time() - t0_wall, 1))
    RESULTS.mkdir(exist_ok=True)
    name = tag or f"conventional_{scenario}_seed{seed}"
    (RESULTS / f"{name}.json").write_text(json.dumps(summary, indent=2))
    if plot:
        plot_run(log, layout, summary, RESULTS / f"{name}.png")
    if not quiet:
        print(json.dumps({k: v for k, v in summary.items() if k != "transitions"}, indent=2))
    return summary, log


def _episodes(flags):
    """Number of times a boolean series goes from True to False (after it first became True)."""
    n, started, prev = 0, False, True
    for f in flags:
        started |= f
        if started and prev and not f:
            n += 1
        prev = f
    return n


def summarise(log, collisions, corner_time, done_time):
    t = np.array(log["t"])
    dist = np.array(log["dist"])
    phase = np.array(log["phase"])
    state = np.array(log["state"])
    k_corner = None if corner_time is None else int(round(corner_time / CONTROL_DT))
    return dict(
        phase1_completed=corner_time is not None,
        phase1_time_s=None if corner_time is None else round(corner_time, 1),
        corner_robot_to_corner_m=None if k_corner is None else round(float(np.hypot(
            log["rx"][k_corner] - W.ROOM, log["ry"][k_corner] - W.ROOM)), 2),
        phase2_completed=done_time is not None,
        total_time_s=round(float(t[-1]), 1),
        start_corner_error_m=None if done_time is None else round(float(np.hypot(
            log["rx"][-1] - W.START[0], log["ry"][-1] - W.START[1])), 2),
        max_distance_m=round(float(dist.max()), 2),
        time_over_1_5m_s=round(float((dist > 1.5).sum() * CONTROL_DT), 1),
        link_losses_phase1=_episodes(np.array(log["seen"])[phase == 1]),
        lost_episodes=int(np.sum((state[1:] == "LOST") & (state[:-1] != "LOST"))),
        wait_episodes=int(np.sum((state[1:] == "WAIT_PERSON") & (state[:-1] != "WAIT_PERSON"))),
        collisions=collisions,
        serious_faults=sum(c["kind"] == "serious" for c in collisions),
        minor_faults=sum(c["kind"] != "serious" for c in collisions),
    )


def plot_run(log, layout, summary, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    t = np.array(log["t"])
    phase = np.array(log["phase"])
    fig = plt.figure(figsize=(13, 6.5))
    ax = fig.add_subplot(1, 2, 1)
    ax.add_patch(Rectangle((0, 0), W.ROOM, W.ROOM, fill=False, lw=2))
    for x, y in layout.stacks:
        ax.add_patch(Rectangle((x - 0.125, y - 0.125), 0.25, 0.25, color="#a0663a"))
    for x, y, _ in layout.briefcases:
        ax.add_patch(Rectangle((x - 0.225, y - 0.075), 0.45, 0.15, color="#333344"))
    if layout.moving_briefcase is not None:
        x0, y0, x1, y1 = layout.moving_briefcase[:4]
        ax.annotate("", (x1, y1), (x0, y0), arrowprops=dict(arrowstyle="->", color="#992222"))
        ax.add_patch(Rectangle((x1 - 0.225, y1 - 0.075), 0.45, 0.15, color="#992222"))
    for i in range(len(layout.walkers)):
        w = np.array([p[i] for p in log["walkers"]])
        ax.plot(w[:, 0], w[:, 1], color="0.75", lw=0.8, label="walkers" if i == 0 else None)
    rx, ry = np.array(log["rx"]), np.array(log["ry"])
    ax.plot(log["px"], log["py"], color="#3a78c8", lw=1, ls="--", label="guide person")
    ax.plot(rx[phase == 1], ry[phase == 1], color="#e07b00", lw=2, label="robot, phase 1")
    ax.plot(rx[phase == 2], ry[phase == 2], color="#1a9a50", lw=2, label="robot, phase 2")
    for c in summary["collisions"]:
        k = min(int(c["t"] / CONTROL_DT), len(rx) - 1)
        ax.plot(rx[k], ry[k], "x", color="red", ms=10, mew=2)
    ax.set_aspect("equal"); ax.set_xlim(-0.2, 7.2); ax.set_ylim(-0.2, 7.2)
    ax.legend(loc="upper left", fontsize=8); ax.set_title("Room (x = collision)")

    ax2 = fig.add_subplot(3, 2, 2)
    ax2.plot(t, log["dist"], color="k", lw=1)
    ax2.axhline(1.5, color="red", ls="--", lw=1)
    ax2.set_ylabel("robot-person [m]"); ax2.set_ylim(0, 3)
    ax3 = fig.add_subplot(3, 2, 4, sharex=ax2)
    ax3.plot(t, log["seen"], drawstyle="steps-post", color="#3a78c8")
    ax3.set_ylabel("tag seen"); ax3.set_yticks([0, 1]); ax3.set_yticklabels(["no", "yes"])
    ax4 = fig.add_subplot(3, 2, 6, sharex=ax2)
    ax4.plot(t, [STATES.index(s) for s in log["state"]], drawstyle="steps-post", color="#6a3d9a")
    ax4.set_yticks(range(len(STATES))); ax4.set_yticklabels(STATES, fontsize=7)
    ax4.set_xlabel("time [s]")
    fig.suptitle(f"conventional | {summary.get('scenario', '')}  |  phase 1: {summary['phase1_completed']}  "
                 f"phase 2: {summary['phase2_completed']}  |  max dist {summary['max_distance_m']} m  |  "
                 f"faults: {summary['serious_faults']} serious, {summary['minor_faults']} minor")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", default="full", choices=["empty", "static", "moving", "people", "full"])
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--max-time", type=float, default=400.0)
    ap.add_argument("--view", action="store_true")
    a = ap.parse_args()
    run(a.scenario, a.seed, a.max_time, a.view)


if __name__ == "__main__":
    main()
