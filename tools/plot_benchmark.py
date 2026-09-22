#!/usr/bin/env python3
"""Create a benchmark figure from distributed_stress JSON."""
import argparse
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import labstyle as ls


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("report")
    parser.add_argument("--output", default="artifacts/benchmark.png")
    args = parser.parse_args()
    report = json.loads(Path(args.report).read_text())
    cases = report["cases"]
    names = [c["name"].replace("_", " ") for c in cases]
    indices = np.arange(len(cases))

    ls.use()
    fig, (latency, clearance) = plt.subplots(1, 2, figsize=(12, 6.4), sharey=True)

    latency.barh(indices - .18, [c["p95_ms"] for c in cases], .28,
                 label="p95", color=ls.SERIES[0])
    latency.barh(indices + .18, [c["max_ms"] for c in cases], .28,
                 label="maximum", color=ls.SERIES[1])
    budget = report["deadline_ms"]
    latency.axvline(budget, color=ls.RULE_2, linestyle=(0, (4, 3)), linewidth=1)
    # y is inverted, so -0.45 is the top of the panel, clear of the legend.
    latency.text(budget - 2, -.45, f"{budget:g} ms budget", ha="right",
                 va="top", fontsize=8, color=ls.MUTED, family=ls.MONO)
    latency.set_yticks(indices, names)
    latency.invert_yaxis()
    latency.set_xlabel("measured decision latency [ms]")
    latency.set_title("Decision latency")
    latency.legend(loc="lower right", ncols=2)

    values = [c["min_clearance_m"] or 0 for c in cases]
    states = ["complete" if c["success"] else "safety stop" if c["safety_stop"] else "incomplete"
              for c in cases]
    colors = [ls.MUTED if c["success"] else ls.SERIES[1] for c in cases]
    clearance.barh(indices, values, .56, color=colors)
    for i, (case, state) in enumerate(zip(cases, states)):
        notes = [] if case["success"] else [state]
        if case["fallback_steps"]:
            notes.append(f"{case['fallback_steps']} fallback")
        if notes:
            clearance.text(values[i] + .02, i, " · ".join(notes), va="center",
                           fontsize=8, color=ls.MUTED, family=ls.MONO)
    clearance.set_xlim(0, max(values) + .45)
    clearance.set_xlabel("minimum sampled inter-vehicle body clearance [m]")
    clearance.set_title("Clearance")

    for ax in (latency, clearance):
        ax.grid(axis="x")
        ax.tick_params(axis="y", length=0)
        ax.spines["left"].set_color(ls.RULE)

    s = report["summary"]
    fig.text(.045, .955, "Parallel parking with overtaking traffic", fontsize=13, color=ls.INK)
    fig.text(.045, .918, f"{s['completed']}/{s['cases']} completed · {s['collisions']} collisions · "
             f"{s['deadline_misses']} deadline misses", fontsize=9.5, color=ls.MUTED)
    fig.text(.045, .035, f"Seed {report['seed']} · {report['dt_s'] * 1000:g} ms control ticks · "
             f"{report['dt_s'] / report['substeps'] * 1000:g} ms collision sampling. "
             "Kinematic simulation; measured software timing, not a continuous safety "
             "or hard real-time guarantee.", fontsize=8, color=ls.FAINT)
    fig.tight_layout(rect=(.02, .07, .99, .89))
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=150)
    plt.close(fig)
    print(output)


if __name__ == "__main__":
    main()
