"""Runs many seeds (random layouts, people paths and sensor noise) and the ablations.

    python -m simulation.batch --seeds 8          # full architecture + ablations, 8 seeds each

Prints a Markdown table with the success rate and the rubric faults of each
variant, and writes it to simulation/results/batch.md.
"""

import argparse
from multiprocessing import Pool

import numpy as np

from .run_exam import RESULTS, run

VARIANTS = {
    "full architecture": dict(),
    "without field memory": dict(use_memory=False),
    "without Reichardt detector": dict(use_motion=False),
}


def _one(args):
    variant, seed, scenario = args
    s, _ = run(scenario, seed, plot=False, quiet=True, tag=f"batch_{scenario}_{seed}_{variant.replace(' ', '_')}",
               **VARIANTS[variant])
    return variant, s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=8)
    ap.add_argument("--scenario", default="full")
    ap.add_argument("--workers", type=int, default=8)
    a = ap.parse_args()
    jobs = [(v, s, a.scenario) for v in VARIANTS for s in range(a.seeds)]
    with Pool(a.workers) as pool:
        out = pool.map(_one, jobs)
    lines = [f"Scenario `{a.scenario}`, {a.seeds} seeds per variant.", "",
             "| Variant | Both phases done | Link losses (phase 1) | Serious faults | Minor faults | "
             "Max distance [m] | Time > 1.5 m [s] | Mean time [s] |",
             "|---|---|---|---|---|---|---|---|"]
    for v in VARIANTS:
        rs = [s for vv, s in out if vv == v]
        done = sum(s["phase1_completed"] and s["phase2_completed"] for s in rs)
        lines.append(
            f"| {v} | {done}/{len(rs)} | {sum(s['link_losses_phase1'] for s in rs)} | "
            f"{sum(s['serious_faults'] for s in rs)} | {sum(s['minor_faults'] for s in rs)} | "
            f"{max(s['max_distance_m'] for s in rs):.2f} | {sum(s['time_over_1_5m_s'] for s in rs):.1f} | "
            f"{np.mean([s['total_time_s'] for s in rs]):.0f} |")
    table = "\n".join(lines)
    print(table)
    RESULTS.mkdir(exist_ok=True)
    (RESULTS / "batch.md").write_text(table + "\n")


if __name__ == "__main__":
    main()
