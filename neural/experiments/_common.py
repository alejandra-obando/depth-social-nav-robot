"""Helpers shared by the experiments: output folder and pass/fail checks."""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

RESULTS = Path(__file__).resolve().parent / "results"
RESULTS.mkdir(exist_ok=True)


class Checks:
    def __init__(self, name):
        self.name = name
        self.results = []

    def check(self, label, ok, detail=""):
        self.results.append((label, bool(ok), detail))
        print(f"  [{'PASS' if ok else 'FAIL'}] {label}" + (f"  ({detail})" if detail else ""))

    def done(self, fig=None):
        if fig is not None:
            path = RESULTS / f"{self.name}.png"
            fig.tight_layout()
            fig.savefig(path, dpi=110)
            plt.close(fig)
            print(f"  plot: {path.relative_to(Path.cwd()) if path.is_relative_to(Path.cwd()) else path}")
        failed = [r for r in self.results if not r[1]]
        print(f"{self.name}: {len(self.results) - len(failed)}/{len(self.results)} checks passed")
        return not failed
