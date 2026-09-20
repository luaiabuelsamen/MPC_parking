#!/usr/bin/env python3
"""Render the distributed iLQR passing experiment."""

import argparse
import csv
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Polygon


def rectangle(x, y, yaw, rear_axle=True):
    back, front = (-1.0, 3.5) if rear_axle else (-2.25, 2.25)
    points = [(back, -0.9), (front, -0.9), (front, 0.9), (back, 0.9)]
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * px - s * py, y + s * px + c * py)
            for px, py in points]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--fps", type=int, default=12)
    args = parser.parse_args()
    with open(args.trajectory, newline="") as source:
        samples = list(csv.DictReader(source))
    if not samples:
        raise SystemExit("trajectory CSV is empty")

    stride = max(1, len(samples) // 110)
    indices = list(range(0, len(samples), stride))
    if indices[-1] != len(samples) - 1:
        indices.append(len(samples) - 1)

    fig, ax = plt.subplots(figsize=(10, 4.8), dpi=100)
    ax.set(xlim=(-14, 14), ylim=(-5.6, 5.6), aspect="equal",
           xlabel="road position x [m]", ylabel="lateral position y [m]",
           title="Distributed iLQR: negotiated opposing-lane pass")
    ax.set_facecolor("#555b61")
    ax.axhline(0, color="#f6d55c", linewidth=1.5, linestyle="--")
    ax.axhline(-3.25, color="white", linewidth=1.0)
    ax.axhline(3.25, color="white", linewidth=1.0)
    stopped = Polygon(rectangle(0, -1.55, 0, rear_axle=False), closed=True,
                      facecolor="#2f3439", edgecolor="white", linewidth=1.2)
    ax.add_patch(stopped)
    ax.text(0, -1.55, "STOPPED", color="white", ha="center", va="center",
            fontsize=8, weight="bold")

    colors = ["#36c2f0", "#ef476f"]
    names = ["passing agent", "oncoming agent"]
    prefixes = ["passing", "oncoming"]
    cars, trails = [], []
    for color, name, prefix in zip(colors, names, prefixes):
        row = samples[0]
        car = Polygon(rectangle(float(row[prefix + "_x"]),
                                float(row[prefix + "_y"]),
                                float(row[prefix + "_yaw"])),
                      closed=True, facecolor=color, edgecolor="#17202a",
                      linewidth=1.4, zorder=5)
        ax.add_patch(car)
        trail, = ax.plot([], [], color=color, linewidth=2.0, label=name)
        cars.append(car); trails.append(trail)
    passing_x = [float(r["passing_x"]) for r in samples]
    passing_y = [float(r["passing_y"]) for r in samples]
    oncoming_x = [float(r["oncoming_x"]) for r in samples]
    oncoming_y = [float(r["oncoming_y"]) for r in samples]
    status = ax.text(0.015, 0.965, "", transform=ax.transAxes, va="top",
                     color="#17202a", family="monospace", fontsize=9,
                     bbox=dict(boxstyle="round", facecolor="white", alpha=0.9))
    ax.legend(loc="upper right")

    def update(frame):
        i = indices[frame]
        row = samples[i]
        trails[0].set_data(passing_x[:i + 1], passing_y[:i + 1])
        trails[1].set_data(oncoming_x[:i + 1], oncoming_y[:i + 1])
        for car, prefix in zip(cars, prefixes):
            car.set_xy(rectangle(float(row[prefix + "_x"]),
                                 float(row[prefix + "_y"]),
                                 float(row[prefix + "_yaw"])))
        collision = int(row["collision"])
        status.set_text(f"t = {float(row['t']):5.1f} s\n"
                        f"coordination = {float(row['solve_ms']):6.1f} ms\n"
                        f"collision = {'YES' if collision else 'no'}")
        return tuple(trails + cars + [status])

    animation = FuncAnimation(fig, update, frames=len(indices),
                              interval=1000 / args.fps, blit=True)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    animation.save(output, writer=PillowWriter(fps=args.fps))
    plt.close(fig)
    print(output)


if __name__ == "__main__":
    main()
