#!/usr/bin/env python3
"""Make a self-contained interactive replay and optional GIF from encounter telemetry."""
import argparse
import json
import math
from pathlib import Path


def render_gif(data, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.patches import Polygon

    samples, agents = data["samples"], data["agents"]
    colors = ["#56caff", "#ff7899", "#ffc66a"]
    bg, text, muted = "#0b121b", "#edf3f9", "#9cacc0"
    fig = plt.figure(figsize=(12, 5.8), facecolor=bg)
    ax = fig.add_axes((.025, .24, .95, .52))
    ax.set(xlim=(-27, 27), ylim=(-2.6, 10.5), aspect="equal")
    ax.set_facecolor("#263849")
    ax.set_xticks([]); ax.set_yticks([])
    for spine in ax.spines.values(): spine.set_visible(False)
    ax.axhline(4.8, color="#c8b870", linestyle="--", lw=1)
    ax.axhline(8, color=muted, lw=1)
    ax.axhspan(-2.6, -1.4, color="#34485a")
    ax.axhspan(8, 10.5, color="#142231")
    ax.text(-25, 8.7, "ONCOMING  ←", color=muted, fontsize=9)
    ax.text(16, 1.7, "TRAVEL  →", color=muted, fontsize=9)

    def corners(x, y, yaw, length, width, rear):
        c, s = math.cos(yaw), math.sin(yaw)
        return [(x+c*px-s*py, y+s*px+c*py) for px, py in
                [(-rear, -width/2), (length-rear, -width/2),
                 (length-rear, width/2), (-rear, width/2)]]

    for x, y, length, width, yaw in data["obstacles"][:2]:
        ax.add_patch(Polygon(corners(x, y, yaw, length, width, length/2),
                             facecolor="#455261", edgecolor=muted))
    ax.add_patch(Polygon(corners(-1.25, 0, 0, 4.5, 1.8, 1), fill=False,
                         edgecolor="#6fe1bb", linestyle="--", lw=1.5))
    fig.text(.04, .92, "SHARED STREET", color="#6fe1bb", fontsize=10, weight="bold")
    fig.text(.04, .86, "Parallel parking meets oncoming traffic", color=text, fontsize=23, weight="bold")
    phase = fig.text(.04, .79, "", color=text, fontsize=12)
    clock = fig.text(.95, .91, "", color=text, fontsize=14, ha="right", family="monospace")
    cars, trails, predictions, labels = [], [], [], []
    for i, a in enumerate(agents):
        car = Polygon(corners(0, 0, 0, a["length"], a["width"], a["rear_overhang"]),
                      facecolor=colors[i], edgecolor=bg, lw=1.5, zorder=5)
        ax.add_patch(car); cars.append(car)
        trail, = ax.plot([], [], color=colors[i], alpha=.55, lw=1.5)
        pred, = ax.plot([], [], color=colors[i], linestyle=":", lw=1.5)
        trails.append(trail); predictions.append(pred)
        labels.append(fig.text(.04+i*.32, .17, "", color=colors[i], fontsize=11, linespacing=1.6))
    status = fig.text(.04, .065, "", color=muted, fontsize=10)
    fig.text(.04, .025, "Kinematic simulation · dotted lines: proposed MPC horizons · sampled collision checks", color=muted, fontsize=8)
    indices = list(range(0, len(samples), max(1, len(samples)//160)))
    if indices[-1] != len(samples)-1: indices.append(len(samples)-1)
    def update(frame):
        k = indices[frame]; sample = samples[k]
        clock.set_text(f"{sample['t']:5.1f} s")
        mode = sample["modes"][1]
        titles = {"WAIT_FOR_GAP": "01 / Wait for oncoming traffic and the parking car’s stop",
                  "PASS": "02 / Pass granted — parking car holds before reversing",
                  "RETURN": "03 / Follower clears — parking car resumes its manoeuvre",
                  "LANE_RESTORED": "04 / Follower back in lane — parking continues",
                  "DONE": "05 / Through traffic complete — finishing the parking manoeuvre",
                  "END": "Encounter complete" if data["summary"]["success"] else "Encounter incomplete"}
        phase.set_text(titles.get(mode, mode))
        for i, a in enumerate(agents):
            x = sample["states"][i]
            cars[i].set_xy(corners(x[0], x[1], x[2], a["length"], a["width"], a["rear_overhang"]))
            trails[i].set_data([s["states"][i][0] for s in samples[:k+1]], [s["states"][i][1] for s in samples[:k+1]])
            pred = sample["predictions"][i] if sample["predictions"] else []
            predictions[i].set_data([p[0] for p in pred], [p[1] for p in pred])
            labels[i].set_text(f"{a['name'].upper()}  /  {x[3]:+.2f} m/s\n{sample['modes'][i].replace('_', ' ').lower()}")
        status.set_text(f"Body clearance  {sample['clearance']:.2f} m     |     Decision  {sample['ms']:.1f} ms     |     "
                        f"{'BRAKING FALLBACK' if sample['fallback'] else 'Controller in the loop'}")
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
