"""Runs the conventional controller over many seeds and scenarios.

    python -m simulation.batch_conventional --seeds 6
    python -m simulation.batch_conventional --seeds 6 --scenarios static,full --workers 4

Each seed changes the obstacle layout, the walkers and the sensor noise.  Prints
a Markdown table per scenario with the rubric results and writes it to
simulation/results/batch_conventional.md.  Each run takes about a minute of CPU
(it renders the cameras and runs the AprilTag detector on every frame).
"""

import argparse
import multiprocessing as mp
import os
from concurrent.futures import ProcessPoolExecutor

os.environ.setdefault("MUJOCO_GL", "egl")

import numpy as np  # noqa: E402

from .run_conventional import RESULTS, run  # noqa: E402


def _one(args):
    scenario, seed = args
    s, _ = run(scenario, seed, plot=True, quiet=True)
    return scenario, s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=6)
    ap.add_argument("--scenarios", default="empty,static,moving,people,full")
    ap.add_argument("--workers", type=int, default=os.cpu_count())
    a = ap.parse_args()
    scenarios = a.scenarios.split(",")
    jobs = [(sc, s) for sc in scenarios for s in range(a.seeds)]
    # "spawn": each worker starts clean (OpenGL contexts do not survive a fork), and a
    # worker that crashes raises an error instead of hanging the whole batch.
    with ProcessPoolExecutor(a.workers, mp_context=mp.get_context("spawn")) as pool:
        out = list(pool.map(_one, jobs))
    lines = [f"Conventional controller, {a.seeds} seeds per scenario.", "",
             "| Scenario | Both phases done | Link losses (phase 1) | Waits (phase 2) | Serious faults | "
             "Minor faults | Max distance [m] | Time > 1.5 m [s] | Start error [m] | Mean time [s] |",
             "|---|---|---|---|---|---|---|---|---|---|"]
    for sc in scenarios:
        rs = [s for ss, s in out if ss == sc]
        done = sum(s["phase1_completed"] and s["phase2_completed"] for s in rs)
        errs = [s["start_corner_error_m"] for s in rs if s["start_corner_error_m"] is not None]
        lines.append(
            f"| {sc} | {done}/{len(rs)} | {sum(s['link_losses_phase1'] for s in rs)} | "
            f"{sum(s['wait_episodes'] for s in rs)} | {sum(s['serious_faults'] for s in rs)} | "
            f"{sum(s['minor_faults'] for s in rs)} | {max(s['max_distance_m'] for s in rs):.2f} | "
            f"{sum(s['time_over_1_5m_s'] for s in rs):.1f} | "
            f"{(max(errs) if errs else float('nan')):.2f} | {np.mean([s['total_time_s'] for s in rs]):.0f} |")
    collisions = [f"- `{sc}` seed {s['seed']}: t = {c['t']} s, {c['obstacle']} ({c['kind']})"
                  for sc, s in out for c in s["collisions"]]
    table = "\n".join(lines + ["", "Collisions:" if collisions else "No collisions."] + collisions)
    print(table)
    RESULTS.mkdir(exist_ok=True)
    (RESULTS / "batch_conventional.md").write_text(table + "\n")


if __name__ == "__main__":
    main()
