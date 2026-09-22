#!/usr/bin/env python3
"""Render the distributed iLQR passing experiment."""

import argparse
import csv
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Polygon

sys.path.insert(0, str(Path(__file__).resolve().parent))
import labstyle as ls


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

    ls.use()
    fig, ax = plt.subplots(figsize=(10, 4.8))
    fig.subplots_adjust(left=.07, right=.98, top=.83, bottom=.19)
    ax.set(xlim=(-17, 17), ylim=(-2.7, 10.5), aspect="equal",
           xlabel="road position x [m]", ylabel="lateral position y [m]")
    ax.grid(False)

    # Road furniture, all greyscale: colour only ever identifies an agent.
    ax.axhspan(-1.4, 8.0, color=ls.ROAD, zorder=0)
    ax.axhspan(-2.4, -1.4, color=ls.CURB, zorder=0)
    ax.axhspan(8.0, 10.5, color=ls.CURB, zorder=0)
    ax.axhline(4.8, color=ls.RULE_2, linewidth=1, linestyle=(0, (5, 5)), zorder=1)
    ax.axhline(8.0, color=ls.RULE_2, linewidth=1, zorder=1)
    for parked_x in (-5.0, 5.0):
        ax.add_patch(Polygon(rectangle(parked_x, 0.0, 0, rear_axle=False), closed=True,
                             facecolor=ls.OBSTACLE, edgecolor=ls.OBSTACLE_LINE,
                             linewidth=1.0, zorder=2))
    ax.add_patch(Polygon(rectangle(-1.25, 0.0, 0.0), closed=True, fill=False,
                         edgecolor=ls.RULE_2, linewidth=1.2, linestyle=(0, (4, 3)), zorder=2))
    ax.text(-16.4, 6.2, "opposing lane", fontsize=8, color=ls.FAINT, family=ls.MONO)
    ax.text(12.6, 1.5, "travel lane", fontsize=8, color=ls.FAINT, family=ls.MONO)

    names = ["parking", "passing"]
    prefixes = ["parking", "passing"]
    cars, trails = [], []
    for i, (name, prefix) in enumerate(zip(names, prefixes)):
        color = ls.SERIES[i]
        row = samples[0]
        car = Polygon(rectangle(float(row[prefix + "_x"]),
                                float(row[prefix + "_y"]),
                                float(row[prefix + "_yaw"])),
                      closed=True, facecolor=ls.fill(color), edgecolor=color,
                      linewidth=1.4, zorder=5)
        ax.add_patch(car)
        trail, = ax.plot([], [], color=color, linewidth=1.6, label=f"{name} agent")
        cars.append(car); trails.append(trail)
    parking_x = [float(r["parking_x"]) for r in samples]
    parking_y = [float(r["parking_y"]) for r in samples]
    passing_x = [float(r["passing_x"]) for r in samples]
    passing_y = [float(r["passing_y"]) for r in samples]
    ax.legend(loc="lower right", bbox_to_anchor=(1, 1.01), ncols=2)

    fig.text(.07, .935, "Distributed iLQR · passing a parallel-parking vehicle",
             fontsize=12, color=ls.INK)
    # Readout in the margin rather than a boxed overlay on the scene.
    status = fig.text(.07, .885, "", fontsize=9, color=ls.MUTED, family=ls.MONO)
    flag = fig.text(.98, .885, "", fontsize=9, color=ls.SERIES[7], family=ls.MONO, ha="right")
    fig.text(.07, .04, "Kinematic simulation · sampled collision checks · measured software "
             "timing, not a hard real-time guarantee.", fontsize=8, color=ls.FAINT)

    def update(frame):
        i = indices[frame]
        row = samples[i]
        trails[0].set_data(parking_x[:i + 1], parking_y[:i + 1])
        trails[1].set_data(passing_x[:i + 1], passing_y[:i + 1])
        for car, prefix in zip(cars, prefixes):
            car.set_xy(rectangle(float(row[prefix + "_x"]),
                                 float(row[prefix + "_y"]),
                                 float(row[prefix + "_yaw"])))
        collision = int(row["collision"])
        fallback = int(row.get("fallback", 0))
        status.set_text(f"t {float(row['t']):5.1f} s     "
                        f"coordination {float(row['solve_ms']):5.1f} ms     "
                        f"clearance {float(row['clearance']):4.2f} m     "
                        f"mode {'braking fallback' if fallback else 'MPC'}")
        # Exceptions are named, not signalled by colour alone.
        flag.set_text("COLLISION" if collision else "")
        return tuple(trails + cars + [status, flag])

    animation = FuncAnimation(fig, update, frames=len(indices), interval=1000 / args.fps)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    animation.save(output, writer=PillowWriter(fps=args.fps))
    plt.close(fig)
    print(output)


if __name__ == "__main__":
    main()
