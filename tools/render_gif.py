#!/usr/bin/env python3
"""Render a closed-loop mpc_simulator CSV as an animated GIF."""

import argparse
import csv
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Polygon


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

    fig, ax = plt.subplots(figsize=(9.6, 5.4), dpi=100)
    xmin, xmax, ymin, ymax = scene["bounds"]
    ax.set(xlim=(xmin, xmax), ylim=(ymin, ymax), aspect="equal",
           xlabel="x [m]", ylabel="y [m]", title=f"Closed-loop MPC: {args.scenario}")
    ax.set_facecolor("#edf1f4")
    ax.grid(color="white", linewidth=0.8)
    for cx, cy, length, width, yaw, _ in scene["obstacles"]:
        ax.add_patch(Polygon(rectangle(cx, cy, length, width, yaw), closed=True,
                             facecolor="#4f5964", edgecolor="#252a30", linewidth=1.2))
    gx, gy, gyaw = scene["goal"]
    ax.add_patch(Polygon(rectangle(gx, gy, 4.5, 1.8, gyaw, rear_axle=True),
                         closed=True, fill=False, edgecolor="#1b9e77",
                         linewidth=2.0, linestyle="--", label="goal"))
    trail, = ax.plot([], [], color="#377eb8", linewidth=2.0, label="driven path")
    first = samples[0]
    car = Polygon(rectangle(float(first["x"]), float(first["y"]), 4.5, 1.8,
                            float(first["yaw"]), rear_axle=True),
                  closed=True, facecolor="#ffb000", edgecolor="#6b4800",
                  linewidth=1.5, zorder=5)
    ax.add_patch(car)
    status = ax.text(0.02, 0.97, "", transform=ax.transAxes, va="top",
                     family="monospace", fontsize=9,
                     bbox=dict(boxstyle="round", facecolor="white", alpha=0.88))
    ax.legend(loc="upper right")

    xs = [float(row["x"]) for row in samples]
    ys = [float(row["y"]) for row in samples]

    def update(frame):
        i = frame_indices[frame]
        row = samples[i]
        trail.set_data(xs[:i + 1], ys[:i + 1])
        car.set_xy(rectangle(xs[i], ys[i], 4.5, 1.8, float(row["yaw"]), rear_axle=True))
        collision = int(row["collision"])
        car.set_facecolor("#d62728" if collision else "#ffb000")
        status.set_text(
            f"t = {float(row['t']):5.1f} s\n"
            f"v = {float(row['v']):+5.2f} m/s\n"
            f"steer = {math.degrees(float(row['steer'])):+5.1f} deg\n"
            f"MPC = {float(row['solve_ms']):5.1f} ms"
        )
        return trail, car, status

    animation = FuncAnimation(fig, update, frames=len(frame_indices),
                              interval=1000 / args.fps, blit=True)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    animation.save(output, writer=PillowWriter(fps=args.fps))
    plt.close(fig)
    print(output)


if __name__ == "__main__":
    main()
