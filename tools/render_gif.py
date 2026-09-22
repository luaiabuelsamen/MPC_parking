#!/usr/bin/env python3
"""Render a closed-loop mpc_simulator CSV as an animated GIF."""

import argparse
import csv
import math
from pathlib import Path

import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Polygon

sys.path.insert(0, str(Path(__file__).resolve().parent))
import labstyle as ls


SCENES = {
    "parallel": {
        "bounds": (-14.0, 14.0, -2.6, 9.5),
        "goal": (-1.25, 0.0, 0.0),
        "obstacles": [
            (-5.0, 0.0, 4.5, 1.8, 0.0, "parked car"),
            (5.0, 0.0, 4.5, 1.8, 0.0, "parked car"),
            (0.0, -1.9, 40.0, 1.0, 0.0, "curb"),
            (0.0, 8.0, 40.0, 2.0, 0.0, "wall"),
        ],
    },
    "perpendicular": {
        "bounds": (-14.0, 14.0, -7.5, 8.5),
        "goal": (0.0, -4.5, math.pi / 2),
        "obstacles": [
            (-2.4, -3.3, 4.5, 1.8, math.pi / 2, "parked car"),
            (2.4, -3.3, 4.5, 1.8, math.pi / 2, "parked car"),
            (-7.2, -3.3, 4.5, 1.8, math.pi / 2, "parked car"),
            (0.0, -6.2, 40.0, 1.2, 0.0, "wall"),
            (0.0, 7.0, 40.0, 2.0, 0.0, "wall"),
        ],
    },
    "garage": {
        "bounds": (-13.0, 10.0, -7.5, 10.5),
        "goal": (0.0, -3.7, math.pi / 2),
        "obstacles": [
            (-1.8, -3.0, 6.0, 0.6, math.pi / 2, "garage"),
            (1.8, -3.0, 6.0, 0.6, math.pi / 2, "garage"),
            (0.0, -6.3, 5.0, 0.6, 0.0, "garage"),
            (7.5, 3.0, 1.0, 8.0, 0.0, "fence"),
            (0.0, 9.0, 40.0, 2.0, 0.0, "wall"),
        ],
    },
}


def rectangle(cx, cy, length, width, yaw, rear_axle=False):
    if rear_axle:
        back, front = -1.0, 3.5
    else:
        back, front = -length / 2, length / 2
    half_width = width / 2
    points = [(back, -half_width), (front, -half_width),
              (front, half_width), (back, half_width)]
    c, s = math.cos(yaw), math.sin(yaw)
    return [(cx + c * x - s * y, cy + s * x + c * y) for x, y in points]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", choices=SCENES, default="parallel")
    parser.add_argument("--trajectory", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--fps", type=int, default=12)
    args = parser.parse_args()

    with open(args.trajectory, newline="") as source:
        samples = list(csv.DictReader(source))
    if not samples:
        raise SystemExit("trajectory CSV is empty")
    scene = SCENES[args.scenario]
    stride = max(1, len(samples) // 100)
    frame_indices = list(range(0, len(samples), stride))
    if frame_indices[-1] != len(samples) - 1:
        frame_indices.append(len(samples) - 1)

    ls.use()
    fig, ax = plt.subplots(figsize=(9.6, 5.4))
    fig.subplots_adjust(left=.07, right=.98, top=.86, bottom=.17)
    xmin, xmax, ymin, ymax = scene["bounds"]
    ax.set(xlim=(xmin, xmax), ylim=(ymin, ymax), aspect="equal",
           xlabel="x [m]", ylabel="y [m]")
    ax.grid(color=ls.GRID, linewidth=0.6)
    for cx, cy, length, width, yaw, _ in scene["obstacles"]:
        ax.add_patch(Polygon(rectangle(cx, cy, length, width, yaw), closed=True,
                             facecolor=ls.OBSTACLE, edgecolor=ls.OBSTACLE_LINE, linewidth=1.0))
    gx, gy, gyaw = scene["goal"]
    ax.add_patch(Polygon(rectangle(gx, gy, 4.5, 1.8, gyaw, rear_axle=True),
                         closed=True, fill=False, edgecolor=ls.RULE_2,
                         linewidth=1.2, linestyle=(0, (4, 3)), label="goal"))
    trail, = ax.plot([], [], color=ls.SERIES[0], linewidth=1.6, label="driven path")
    first = samples[0]
    car = Polygon(rectangle(float(first["x"]), float(first["y"]), 4.5, 1.8,
                            float(first["yaw"]), rear_axle=True),
                  closed=True, facecolor=ls.fill(ls.SERIES[0]), edgecolor=ls.SERIES[0],
                  linewidth=1.4, zorder=5)
    ax.add_patch(car)
    ax.legend(loc="lower right", bbox_to_anchor=(1, 1.01), ncols=2)

    fig.text(.07, .945, f"Closed-loop MPC · {args.scenario} parking", fontsize=12, color=ls.INK)
    status = fig.text(.07, .90, "", fontsize=9, color=ls.MUTED, family=ls.MONO)
    flag = fig.text(.98, .90, "", fontsize=9, color=ls.SERIES[7], family=ls.MONO, ha="right")
    fig.text(.07, .045, "Kinematic simulation · measured software timing, not a hard "
             "real-time guarantee.", fontsize=8, color=ls.FAINT)

    xs = [float(row["x"]) for row in samples]
    ys = [float(row["y"]) for row in samples]

    def update(frame):
        i = frame_indices[frame]
        row = samples[i]
        trail.set_data(xs[:i + 1], ys[:i + 1])
        car.set_xy(rectangle(xs[i], ys[i], 4.5, 1.8, float(row["yaw"]), rear_axle=True))
        collision = int(row["collision"])
        car.set_edgecolor(ls.SERIES[7] if collision else ls.SERIES[0])
        car.set_facecolor(ls.fill(ls.SERIES[7] if collision else ls.SERIES[0]))
        flag.set_text("COLLISION" if collision else "")
        status.set_text(
            f"t {float(row['t']):5.1f} s     "
            f"v {float(row['v']):+5.2f} m/s     "
            f"steer {math.degrees(float(row['steer'])):+5.1f}°     "
            f"solve {float(row['solve_ms']):5.1f} ms"
        )
        return trail, car, status, flag

    animation = FuncAnimation(fig, update, frames=len(frame_indices),
                              interval=1000 / args.fps)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    animation.save(output, writer=PillowWriter(fps=args.fps))
    plt.close(fig)
    print(output)


if __name__ == "__main__":
    main()
