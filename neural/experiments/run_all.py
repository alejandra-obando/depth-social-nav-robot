"""Runs every experiment and reports which ones pass.

    python -m neural.experiments.run_all
"""

import importlib

EXPERIMENTS = ["exp_leaky_integrator", "exp_competitive_network", "exp_neural_field",
               "exp_attractor_dynamics", "exp_motion_detector", "exp_corner_detector",
               "exp_behaviour_selector"]


def main():
    ok = {}
    for name in EXPERIMENTS:
        print(f"\n=== {name}")
        ok[name] = importlib.import_module(f"neural.experiments.{name}").main()
    print("\nSummary:")
    for name, passed in ok.items():
        print(f"  {'PASS' if passed else 'FAIL'}  {name}")
    return all(ok.values())


if __name__ == "__main__":
    raise SystemExit(0 if main() else 1)
