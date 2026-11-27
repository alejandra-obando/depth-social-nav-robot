"""Runs both exam phases in the simulated room with the neural architecture.

    python -m simulation.run_exam --scenario full --seed 0
    python -m simulation.run_exam --scenario full --view          # open the MuJoCo viewer
    python -m simulation.run_exam --scenario full --no-memory     # ablation: no field memory

Measures what the rubric grades: robot-person distance (<= 1.5 m), collisions,
link losses, corner recognition, arrival at the start corner and time.
Writes a plot and a JSON summary to simulation/results/.
"""

import argparse
import json
import time
from pathlib import Path

import mujoco
import numpy as np

from neural.models.social_nav_net import SensorFrame, SocialNavNet

from . import world as W
from .people import GuidePerson, Walker, set_mocap
from .sensors import DepthCamera, Lidar, Odometry, SignCamera, wheel_commands

CONTROL_DT = 0.05
RESULTS = Path(__file__).resolve().parent / "results"


def run(scenario="full", seed=0, max_time=400.0, view=False, use_motion=True, use_memory=True,
        lag_events=((15.0, 5.0),), plot=True, tag=None, quiet=False):
    layout = W.make_layout(scenario, seed)
    _, model = W.build(layout)
    data = mujoco.MjData(model)

    # Robot at the start corner, facing the opposite corner.
    data.qpos[0:3] = [W.START[0], W.START[1], 0.03]
    data.qpos[3:7] = W.robot_quat(W.START_YAW)
    mujoco.mj_forward(model, data)

    guide = GuidePerson([W.START + np.array([0.9, 0.9]) / np.sqrt(2) * 1.0] + layout.guide_path,
                        lag_events=lag_events)
    walkers = [Walker(g, seed=seed + i) for i, g in enumerate(layout.walkers)]
    for _ in range(int(1.0 / model.opt.timestep)):     # let the robot settle
        mujoco.mj_step(model, data)

    lidar = Lidar(model, seed=seed)
    depth = DepthCamera(model, seed=seed)
    front_cam = SignCamera(model, "d415_color", "guide_sign_back", fov=(69, 42), dropout=0.1, seed=seed)
    rear_cam = SignCamera(model, "c270", "guide_sign_front", fov=(48.8, 28.6), dropout=0.15,
                          dist_noise=0.08, seed=seed + 1)
    odom = Odometry(model, data, seed=seed)
    net = SocialNavNet(use_motion=use_motion, use_memory=use_memory)

    robot_bodies = {model.body(n).id for n in ("base", "part_2", "part_2_2")}
    light = model.geom("signal_light").id
    n_sub = int(round(CONTROL_DT / model.opt.timestep))
    viewer = None
    if view:
        from mujoco import viewer as mj_viewer
        viewer = mj_viewer.launch_passive(model, data)

    log = {k: [] for k in ("t", "rx", "ry", "px", "py", "dist", "behaviour", "phase", "front_link",
                           "rear_link", "v", "omega", "signal", "walkers")}
    last_contact = {}        # obstacle geom -> last time it touched the robot
    collisions = []
    scan = None
    t = 0.0
    corner_time = None
    done_time = None
    omega_hist = []
    step_i = 0
    t0_wall = time.time()

    while t < max_time:
        rx, ry, _ = W.robot_pose(model, data)
        # People.
        gpos, gyaw = guide.step(CONTROL_DT, t, (rx, ry), net.selector.phase)
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

        # Sensors.
        scan_new = scan is None or step_i % 2 == 0     # LiDAR at 10 Hz
        if scan_new:
            scan = lidar.read(model, data)
        d_ang, d_rng = depth.read(model, data)
        frame = SensorFrame(scan_angles=scan[0], scan_ranges=scan[1],
                            depth_angles=d_ang, depth_ranges=d_rng,
                            front_signs=front_cam.read(model, data),
                            rear_signs=rear_cam.read(model, data),
                            odom=odom.step(model, data, CONTROL_DT), scan_new=scan_new)

        # Neural architecture.
        cmd = net.step(frame, CONTROL_DT)
        data.ctrl[:] = wheel_commands(cmd.v, cmd.omega)
        model.geom_rgba[light] = [0.1, 1.0, 0.2, 1] if cmd.signal and int(t * 4) % 2 == 0 else [0.3, 0.3, 0.3, 1]
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
            if t - last_contact.get(g, -10.0) > 1.0:      # a new collision, not the same one continuing
                manoeuvre = max(omega_hist) > 0.3
                collisions.append(dict(t=round(t, 2), obstacle=model.geom(g).name,
                                       kind="minor (during manoeuvre)" if manoeuvre else "serious"))
            last_contact[g] = t

        # Log.
        rx, ry, _ = W.robot_pose(model, data)
        log["t"].append(t); log["rx"].append(rx); log["ry"].append(ry)
        log["px"].append(gpos[0]); log["py"].append(gpos[1])
        log["dist"].append(float(np.hypot(gpos[0] - rx, gpos[1] - ry)))
        log["behaviour"].append(cmd.behaviour); log["phase"].append(cmd.phase)
        log["front_link"].append(cmd.debug["front_link"]); log["rear_link"].append(cmd.debug["rear_link"])
        log["v"].append(cmd.v); log["omega"].append(cmd.omega); log["signal"].append(cmd.signal)
        log["walkers"].append([tuple(w.pos) for w in walkers])
        if cmd.behaviour == "SIGNAL" and corner_time is None:
            corner_time = t
        if cmd.behaviour == "DONE" and done_time is None:
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

    summary = summarise(log, collisions, corner_time, done_time, layout)
    summary.update(scenario=scenario, seed=seed, use_motion=use_motion, use_memory=use_memory,
                   wall_time_s=round(time.time() - t0_wall, 1))
    RESULTS.mkdir(exist_ok=True)
    name = tag or f"{scenario}_seed{seed}" + ("" if use_motion else "_nomotion") + ("" if use_memory else "_nomemory")
    (RESULTS / f"{name}.json").write_text(json.dumps(summary, indent=2))
    if plot:
        plot_run(log, layout, summary, RESULTS / f"{name}.png")
    if not quiet:
        print(json.dumps(summary, indent=2))
    return summary, log


def _episodes(flags):
    """Number of False runs inside a boolean series (after it first became True)."""
    n, started, prev = 0, False, True
    for f in flags:
        started |= f
        if started and prev and not f:
            n += 1
        prev = f
    return n


def summarise(log, collisions, corner_time, done_time, layout):
    t = np.array(log["t"])
    dist = np.array(log["dist"])
    phase = np.array(log["phase"])
    beh = np.array(log["behaviour"])
    over = dist > 1.5
    p1 = phase == 1
    end_err = float(np.hypot(log["rx"][-1] - W.START[0], log["ry"][-1] - W.START[1]))
    return dict(
        phase1_completed=corner_time is not None,
        phase1_time_s=None if corner_time is None else round(corner_time, 1),
        corner_robot_to_corner_m=None if corner_time is None else round(float(np.hypot(
            *(np.array([log["rx"][int(corner_time / CONTROL_DT)], log["ry"][int(corner_time / CONTROL_DT)]])
              - W.ROOM))), 2),
        phase2_completed=done_time is not None,
        total_time_s=round(float(t[-1]), 1),
        start_corner_error_m=round(end_err, 2) if done_time is not None else None,
        max_distance_m=round(float(dist.max()), 2),
        time_over_1_5m_s=round(float(over.sum() * CONTROL_DT), 1),
        link_losses_phase1=_episodes(np.array(log["front_link"])[p1] > 0.3),
        search_episodes=int(np.sum((beh[1:] == "SEARCH") & (beh[:-1] != "SEARCH"))),
        wait_episodes=int(np.sum((beh[1:] == "WAIT") & (beh[:-1] != "WAIT"))),
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
        k = int(c["t"] / CONTROL_DT)
        ax.plot(rx[k], ry[k], "x", color="red", ms=10, mew=2)
    ax.set_aspect("equal"); ax.set_xlim(-0.2, 7.2); ax.set_ylim(-0.2, 7.2)
    ax.legend(loc="upper left", fontsize=8); ax.set_title("Room (x = collision)")

    ax2 = fig.add_subplot(3, 2, 2)
    ax2.plot(t, log["dist"], color="k", lw=1)
    ax2.axhline(1.5, color="red", ls="--", lw=1)
    ax2.set_ylabel("robot-person [m]"); ax2.set_ylim(0, 3)
    ax3 = fig.add_subplot(3, 2, 4, sharex=ax2)
    ax3.plot(t, log["front_link"], label="front link (D415)")
    ax3.plot(t, log["rear_link"], label="rear link (C270)")
    ax3.set_ylabel("link activity"); ax3.legend(fontsize=7, loc="upper right")
    ax4 = fig.add_subplot(3, 2, 6, sharex=ax2)
    names = ["FOLLOW", "SEARCH", "SIGNAL", "GUIDE", "WAIT", "DONE"]
    ax4.plot(t, [names.index(b) for b in log["behaviour"]], drawstyle="steps-post", color="#6a3d9a")
    ax4.set_yticks(range(len(names))); ax4.set_yticklabels(names, fontsize=7)
    ax4.set_xlabel("time [s]")
    fig.suptitle(f"{summary.get('scenario', '')}  |  phase 1: {summary['phase1_completed']}  "
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
    ap.add_argument("--no-motion", action="store_true", help="ablation: switch off the Reichardt detector")
    ap.add_argument("--no-memory", action="store_true", help="ablation: no memory of the person's direction")
    a = ap.parse_args()
    run(a.scenario, a.seed, a.max_time, a.view, not a.no_motion, not a.no_memory)


if __name__ == "__main__":
    main()
