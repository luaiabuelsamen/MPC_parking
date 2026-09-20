#!/usr/bin/env python3
"""Create a shareable benchmark figure from distributed_stress JSON."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("report")
    parser.add_argument("--output", default="artifacts/benchmark.png")
    args = parser.parse_args()
    report = json.loads(Path(args.report).read_text())
    cases = report["cases"]
    names = [c["name"].replace("_", " ") for c in cases]
    indices = np.arange(len(cases))
    fig, (latency, clearance) = plt.subplots(1, 2, figsize=(13, 8), sharey=True)
    fig.patch.set_facecolor("#f6f8fa")
    latency.barh(indices - .17, [c["p95_ms"] for c in cases], .32,
                 label="p95 decision", color="#3578ba")
    latency.barh(indices + .17, [c["max_ms"] for c in cases], .32,
                 label="maximum decision", color="#7fb3dd")
    latency.axvline(report["deadline_ms"], color="#d84a4a", linestyle="--", label="budget")
    latency.set_yticks(indices, names)
    latency.invert_yaxis()
    latency.set_xlabel("Measured decision latency [ms]")
    latency.legend(loc="lower right", fontsize=9)
    values = [c["min_clearance_m"] or 0 for c in cases]
    clearance.barh(indices, values, .6,
                   color=["#228b68" if c["success"] else "#d68a30" for c in cases])
    for i, c in enumerate(cases):
        state = "complete" if c["success"] else "stopped" if c["safety_stop"] else "incomplete"
        clearance.text(values[i] + .01, i, f"{state} · {c['fallback_steps']} fallback", va="center", fontsize=9)
    clearance.set_xlim(0, max(values) + .6)
    clearance.set_xlabel("Minimum sampled inter-vehicle body clearance [m]")
    for ax in (latency, clearance):
        ax.set_axisbelow(True)
        ax.grid(axis="x", alpha=.2)
        ax.spines[["top", "right"]].set_visible(False)
    s = report["summary"]
    fig.suptitle("Parallel parking with overtaking traffic\n"
                 f"{s['completed']}/{s['cases']} completed · {s['collisions']} collisions · "
                 f"{s['deadline_misses']} deadline misses", fontsize=17, weight="bold")
    fig.text(.5, .02, f"Seed {report['seed']} · {report['dt_s'] * 1000:g} ms control ticks · "
             f"{report['dt_s'] / report['substeps'] * 1000:g} ms collision sampling\n"
             "Kinematic simulation; measured software timing, not a continuous safety or hard real-time guarantee.",
             ha="center", fontsize=9, color="#52616b")
    fig.tight_layout(rect=(0, .075, 1, .92))
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=150)
    plt.close(fig)
    print(output)


if __name__ == "__main__":
    main()
