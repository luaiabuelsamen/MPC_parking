#!/usr/bin/env python3
"""Make a self-contained interactive replay and optional GIF from encounter telemetry."""
import argparse
import json
import math
from pathlib import Path


def render_gif(data, path):
    import sys

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.patches import Polygon

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import labstyle as ls

    samples, agents = data["samples"], data["agents"]
    colors = ls.SERIES[:3]
    ls.use()
    fig = plt.figure(figsize=(12, 5.4))
    ax = fig.add_axes((.03, .30, .94, .50))
    ax.set(xlim=(-27, 27), ylim=(-2.6, 10.5), aspect="equal")
    ax.set_facecolor(ls.FIELD)
    ax.set_xticks([]); ax.set_yticks([])
    for spine in ax.spines.values(): spine.set_visible(False)

    # Road furniture is greyscale throughout; colour only identifies an agent.
    ax.axhspan(-1.4, 8.0, color=ls.ROAD, zorder=0)
    ax.axhspan(-2.6, -1.4, color=ls.CURB, zorder=0)
    ax.axhspan(8.0, 10.5, color=ls.CURB, zorder=0)
    ax.axhline(4.8, color=ls.RULE_2, linestyle=(0, (5, 5)), lw=1, zorder=1)
    ax.axhline(8, color=ls.RULE_2, lw=1, zorder=1)
    ax.axhline(1.3, color=ls.RULE, linestyle=(0, (1.5, 4)), lw=1, zorder=1)
    # Kept clear of the lane itself: the oncoming vehicle sweeps its full length.
    ax.text(-26, 8.7, "opposing lane  \u2190", color=ls.FAINT, fontsize=8, family=ls.MONO)
    ax.text(-26, 1.8, "travel lane  \u2192", color=ls.FAINT, fontsize=8, family=ls.MONO)
    ax.text(-2.4, -2.3, "parking bay", color=ls.FAINT, fontsize=8, family=ls.MONO)

    def corners(x, y, yaw, length, width, rear):
        c, s = math.cos(yaw), math.sin(yaw)
        return [(x+c*px-s*py, y+s*px+c*py) for px, py in
                [(-rear, -width/2), (length-rear, -width/2),
                 (length-rear, width/2), (-rear, width/2)]]

    for x, y, length, width, yaw in data["obstacles"][:2]:
        ax.add_patch(Polygon(corners(x, y, yaw, length, width, length/2),
                             facecolor=ls.OBSTACLE, edgecolor=ls.OBSTACLE_LINE,
                             lw=1.0, zorder=2))
    ax.add_patch(Polygon(corners(-1.25, 0, 0, 4.5, 1.8, 1), fill=False,
                         edgecolor=ls.RULE_2, linestyle=(0, (4, 3)), lw=1.2, zorder=2))

    fig.text(.03, .945, "Shared-street encounter", color=ls.INK, fontsize=13)
    phase = fig.text(.03, .895, "", color=ls.MUTED, fontsize=10)
    clock = fig.text(.97, .94, "", color=ls.INK, fontsize=13, ha="right", family=ls.MONO)
    cars, trails, predictions, labels, values = [], [], [], [], []
    for i, a in enumerate(agents):
        car = Polygon(corners(0, 0, 0, a["length"], a["width"], a["rear_overhang"]),
                      facecolor=ls.fill(colors[i]), edgecolor=colors[i], lw=1.4, zorder=5)
        ax.add_patch(car); cars.append(car)
        trail, = ax.plot([], [], color=colors[i], lw=1.5)
        pred, = ax.plot([], [], color=colors[i], linestyle=(0, (1.5, 3)), lw=1.4)
        trails.append(trail); predictions.append(pred)
        # A colour chip carries identity so the text can stay in ink tokens.
        x = .03 + i * .32
        ls.swatch(fig, x, .205, colors[i])
        fig.text(x + .017, .20, a["name"], color=ls.INK, fontsize=10)
        values.append(fig.text(x + .017, .145, "", color=ls.MUTED, fontsize=9, family=ls.MONO))
        labels.append(fig.text(x + .017, .095, "", color=ls.MUTED, fontsize=9))
    status = fig.text(.03, .035, "", color=ls.FAINT, fontsize=8, family=ls.MONO)
    fig.text(.97, .035, "Kinematic simulation \u00b7 dotted: proposed MPC horizons "
             "\u00b7 sampled collision checks", color=ls.FAINT, fontsize=8, ha="right")
    indices = list(range(0, len(samples), max(1, len(samples)//160)))
    if indices[-1] != len(samples)-1: indices.append(len(samples)-1)

    def update(frame):
        k = indices[frame]; sample = samples[k]
        clock.set_text(f"{sample['t']:5.1f} s")
        mode = sample["modes"][1]
        titles = {"WAIT_FOR_GAP": "Oncoming traffic has priority; the follower holds its lane",
                  "PASS": "Pass granted \u2014 the parking vehicle holds before reversing",
                  "RETURN": "The follower clears \u2014 the parking vehicle resumes",
                  "LANE_RESTORED": "The follower is back in lane; parking continues",
                  "DONE": "Through traffic complete; finishing the parking manoeuvre",
                  "END": "Encounter complete" if data["summary"]["success"] else "Encounter incomplete"}
        phase.set_text(titles.get(mode, mode.replace("_", " ").lower()))
        for i, a in enumerate(agents):
            x = sample["states"][i]
            cars[i].set_xy(corners(x[0], x[1], x[2], a["length"], a["width"], a["rear_overhang"]))
            trails[i].set_data([s["states"][i][0] for s in samples[:k+1]], [s["states"][i][1] for s in samples[:k+1]])
            pred = sample["predictions"][i] if sample["predictions"] else []
            predictions[i].set_data([p[0] for p in pred], [p[1] for p in pred])
            values[i].set_text(f"{x[3]:+.2f} m/s")
            labels[i].set_text(sample["modes"][i].replace("_", " ").lower())
        status.set_text(f"clearance {sample['clearance']:.2f} m     decision {sample['ms']:.1f} ms     "
                        f"{'braking fallback' if sample['fallback'] else 'controller in the loop'}")
    animation = FuncAnimation(fig, update, frames=len(indices), interval=1000/12)
    path.parent.mkdir(parents=True, exist_ok=True)
    animation.save(path, writer=PillowWriter(fps=12))
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("report", type=Path)
    parser.add_argument("--html", type=Path, default=Path("artifacts/traffic_replay.html"))
    parser.add_argument("--gif", type=Path)
    args = parser.parse_args()
    data = json.loads(args.report.read_text())
    template = Path(__file__).with_name("traffic_replay.html").read_text()
    payload = json.dumps(data, separators=(",", ":"), allow_nan=False).replace("<", "\\u003c")
    args.html.parent.mkdir(parents=True, exist_ok=True)
    args.html.write_text(template.replace("__TRAFFIC_DATA__", payload))
    print(args.html, flush=True)
    if args.gif:
        render_gif(data, args.gif)
        print(args.gif)


if __name__ == "__main__":
    main()
