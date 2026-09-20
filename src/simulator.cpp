#include "mpcpark/simulator.hpp"

#include <algorithm>
#include <stdexcept>

namespace mpcpark {

SimulationResult simulate_mpc(const Scenario& scenario,
                              const ReferenceTrajectory& reference,
                              const SimulatorOptions& options) {
  if (reference.xs.size() != reference.us.size() + 1 || reference.us.empty()) {
    throw std::invalid_argument("closed-loop simulation needs a nonempty reference");
  }
  if (options.mpc_horizon < 2) {
    throw std::invalid_argument("MPC horizon must be at least two steps");
  }

  Problem problem;
  problem.vehicle = scenario.vehicle;
  problem.obstacles = scenario.obstacles;
  problem.dt = reference.dt;
  problem.horizon = options.mpc_horizon;
  problem.xref.assign(problem.horizon + 1, reference.xs.front());
  problem.weights.pos = 8.0;
  problem.weights.yaw = 10.0;
  problem.weights.v = 0.5;
  problem.weights.delta = 0.8;
  problem.weights.accel = 0.35;
  problem.weights.steer_rate = 0.4;
  problem.weights.term_pos = 3000.0;
  problem.weights.term_yaw = 4000.0;
  problem.weights.term_v = 300.0;
  problem.options.max_inner = options.max_inner_iterations;
  problem.options.max_outer = options.max_outer_iterations;
  problem.options.mu_init = 20.0;

  ParkingSolver controller(problem);
  std::vector<VecU> warm(problem.horizon);
  VecX state = scenario.start;
  SimulationResult result;
  const int reference_steps = static_cast<int>(reference.us.size());
  const int max_steps = reference_steps + options.settle_steps;
  double total_solve_ms = 0.0;

  for (int step = 0; step < max_steps; ++step) {
    std::vector<VecX> window(problem.horizon + 1);
    for (int j = 0; j <= problem.horizon; ++j) {
      const int index = std::min(step + j, reference_steps);
      window[j] = reference.xs[index];
    }
    controller.set_reference(std::move(window));
    controller.reset_duals();

    if (step == 0) {
      for (int j = 0; j < problem.horizon; ++j) {
        warm[j] = reference.us[std::min(j, reference_steps - 1)];
      }
    }
    const Solution solution = controller.solve(state, warm);
    const VecU applied = solution.us.front();
    const bool collision = in_collision(scenario.vehicle, state,
                                        scenario.obstacles);
    result.samples.push_back(SimulationSample{
        step * reference.dt, state, applied, solution.stats.solve_ms,
        solution.stats.max_violation, collision});
    result.collision = result.collision || collision;
    total_solve_ms += solution.stats.solve_ms;
    result.max_solve_ms = std::max(result.max_solve_ms, solution.stats.solve_ms);

    state = step_rk4(scenario.vehicle, state, applied, reference.dt);
    for (int j = 0; j + 1 < problem.horizon; ++j) warm[j] = solution.us[j + 1];
    warm.back() = VecU{};

    if (step >= reference_steps && is_parked(scenario, state)) break;
  }

  const bool final_collision = in_collision(scenario.vehicle, state,
                                             scenario.obstacles);
  result.samples.push_back(SimulationSample{
      result.samples.size() * reference.dt, state, VecU{}, 0.0, 0.0,
      final_collision});
  result.collision = result.collision || final_collision;
  result.final_error = goal_error(state, scenario.goal);
  result.parked = is_parked(scenario, state) && !result.collision;
  if (result.samples.size() > 1) {
    result.mean_solve_ms = total_solve_ms / (result.samples.size() - 1);
  }
  return result;
}

}  // namespace mpcpark
