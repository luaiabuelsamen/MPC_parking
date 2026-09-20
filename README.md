# MPC Parking

A dependency-free C++17 parking trajectory planner and closed-loop simulator.
Hybrid A* first finds a collision-free, kinematically feasible route;
constrained iLQR then tracks it with receding-horizon control of a kinematic
bicycle model.

Three built-in scenes exercise parallel parking, perpendicular parking, and a
narrow garage. The vehicle supports forward/reverse motion, steering-rate and
acceleration limits, exact footprint collision checks, and differentiable
obstacle constraints based on covering circles.

## Build and run

```sh
buck2 build //:mpc_parking
buck2 test //...
buck2 run //:mpc_parking -- parallel --csv parallel.csv
```

Run the controller in a receding-horizon simulation and render its driven
trajectory as a GIF:

```sh
buck2 run //:mpc_simulator -- parallel simulation.csv
python3 tools/render_gif.py --scenario parallel \
  --trajectory simulation.csv --output parking.gif
```

GIF rendering requires Python 3, Matplotlib, and Pillow. An example closed-loop
run is included at [artifacts/parallel_parking.gif](artifacts/parallel_parking.gif).

Use `perpendicular` or `garage` for the other scenes. `--plan-only` runs just
Hybrid A*, which is useful when tuning the search. CSV output contains the
state and control at every time step.

The executable prints planner timing and expansions, solver timing and
constraint violation, final goal error, collision status, and whether the
trajectory meets the parking tolerances.
