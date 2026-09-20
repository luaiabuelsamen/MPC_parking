// Closed-loop simulation: repeatedly solve a finite-horizon parking problem,
// apply one control to the plant, and advance the reference window.
#pragma once

#include <vector>

#include "mpcpark/ilqr.hpp"
#include "mpcpark/scenario.hpp"
#include "mpcpark/trajectory.hpp"

namespace mpcpark {

struct SimulatorOptions {
  int mpc_horizon = 30;
  int settle_steps = 40;
  int max_inner_iterations = 12;
  int max_outer_iterations = 2;
};

struct SimulationSample {
  double time = 0.0;
  VecX state{};
  VecU control{};
  double solve_ms = 0.0;
  double constraint_violation = 0.0;
  bool collision = false;
};

struct SimulationResult {
  std::vector<SimulationSample> samples;
  GoalError final_error;
  bool parked = false;
  bool collision = false;
  double mean_solve_ms = 0.0;
  double max_solve_ms = 0.0;
};

SimulationResult simulate_mpc(const Scenario& scenario,
                              const ReferenceTrajectory& reference,
                              const SimulatorOptions& options = {});

}  // namespace mpcpark
