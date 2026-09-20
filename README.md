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

Requires Linux, GCC with C++17 support, Python 3, and Buck2. Validated with
GCC 11.4 on ARM64 and the Buck2 `2026-08-01` binary (bundled prelude).
The Buck targets use `-O2`, explicit exported headers, and pthread linkage.

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

## Distributed multi-agent iLQR

The passing experiment runs two independent MPC agents. A lead vehicle slows,
turns, and reverses into a parallel-parking slot while a faster vehicle
approaches from behind. The parking agent publishes its predicted manoeuvre;
the following agent optimizes around it and crosses into the opposing lane to
pass. Different safety margins preserve clearance without making the parking
slot artificially infeasible.

```sh
buck2 run //:distributed_ilqr -- artifacts/distributed_ilqr.csv
python3 tools/render_multi_agent_gif.py \
  --trajectory artifacts/distributed_ilqr.csv \
  --output artifacts/distributed_ilqr.gif
```

Run deterministic timing and safety stress cases with:

```sh
buck2 run //:distributed_stress
```

The benchmark includes seven named cases and six seeded variations of follower
speed, start distance, width, and plant wheelbase. One case discards three
successive controller results to exercise braking and recovery. It writes a
machine-readable report and returns nonzero for incomplete maneuvers, collisions,
or deadline misses; failed cases are retained in the report.

```sh
buck2 run //:distributed_stress -- --seed 20260920 --random-cases 6 \
  --output artifacts/benchmark.json
python3 tools/plot_benchmark.py artifacts/benchmark.json
buck2 test //... -c mpcpark.sanitizers=true
```

See the [benchmark figure](artifacts/benchmark.png) and
[full report](artifacts/benchmark.json). Timing depends on host load; the seed
reproduces scenario parameters, not wall-clock scheduling.

## Controller and safety checks

Each agent solves concurrently against the previous exchange's predictions
(Jacobi iteration), then publishes its new trajectory. Additional exchanges
reuse the preceding solution and stop when the largest predicted state change
falls below 0.03 or the solve budget expires. States have mixed units, so this
threshold is an engineering heuristic, not a convergence certificate.

Before applying a command, actuator bounds are enforced and both vehicles are
rolled out together at ten substeps per control tick. This checks their body
rectangles against each other and every static obstacle, including the terminal
state. The simulated plant also advances at those substeps. Clearance is exact
at each sampled pose; collisions between samples remain possible.

A missing, expired, nonfinite, or rejected result selects bounded braking while
holding steering. Braking is itself checked before application. If neither
command passes, the experiment ends with `safety_stop=true`, `success=false` at
the last measured state. This is an explicit failure to find a safe command,
not a claim that a moving physical vehicle can freeze in place. The check covers
the next control interval, not the entire stopping distance. Fallback pauses
the parking reference and resets its control warm start.

Solvers cooperatively check a common deadline between iterations and line-search
trials, reserving 10% of the 150 ms decision budget for supervision. The report
distinguishes solver timeouts from decisions exceeding 150 ms. A full derivative
pass, thread scheduling, or operating-system delay can overrun the budget;
this is not hard real-time enforcement. Simulation time does not advance while
the host computes. Model mismatch changes only the plant; the controller and
supervisor retain the nominal vehicle model.

The multi-agent experiment declares completion at 20 cm position error,
0.08 rad heading error, and 0.08 m/s speed for both cars. These are looser than
the original single-car parking tolerances. The opposing lane is assumed empty;
there is no traffic-law decision layer, sensor model, or external driving simulator.

`buck2 test //...` covers numerical derivatives, geometric planning, moving
obstacle stages, mid-tick and terminal collisions, malformed inputs, braking in
both directions, expired deadlines, and injected missing results. The longer
closed-loop suite is `//:distributed_stress`. Sanitizers run the test targets;
timing benchmarks should use the normal optimized build.

Use `perpendicular` or `garage` for the other scenes. `--plan-only` runs just
Hybrid A*, which is useful when tuning the search. CSV output contains the
state and control at every time step.

The executable prints planner timing and expansions, solver timing and
constraint violation, final goal error, collision status, and whether the
trajectory meets the parking tolerances.
