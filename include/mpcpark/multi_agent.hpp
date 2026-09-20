// Distributed MPC simulation using sequential iterative best response. Each
// vehicle owns an iLQR problem and treats the other vehicle's latest predicted
// trajectory as a sequence of moving obstacle rectangles.
#pragma once

#include <string>
#include <vector>

#include "mpcpark/ilqr.hpp"
#include "mpcpark/scenario.hpp"
#include "mpcpark/trajectory.hpp"

namespace mpcpark {

struct TrafficAgent {
  std::string name;
  VehicleParams vehicle;
  VecX start{};
  VecX goal{};
  double cruise_speed = 0.8;
};

struct PassingScenario {
  std::vector<TrafficAgent> agents;
  std::vector<Rect> static_obstacles;
  double xmin = -14.0, xmax = 14.0, ymin = -5.5, ymax = 5.5;
};

struct DistributedOptions {
  double dt = 0.15;
  int mpc_horizon = 25;
  int coordination_rounds = 1;
  int settle_steps = 120;
  int max_inner_iterations = 10;
  int max_outer_iterations = 2;
  double deadline_ms = 150.0;
};

struct MultiAgentSample {
  double time = 0.0;
  std::vector<VecX> states;
  std::vector<VecU> controls;
  double solve_ms = 0.0;
  double clearance = 0.0;
  bool deadline_miss = false;
  bool collision = false;
};

struct DistributedResult {
  std::vector<MultiAgentSample> samples;
  std::vector<GoalError> final_errors;
  bool success = false;
  bool collision = false;
  double mean_round_ms = 0.0;
  double max_round_ms = 0.0;
  double min_clearance = 0.0;
  int deadline_misses = 0;
};

PassingScenario make_parallel_parking_traffic_scenario();
std::vector<ReferenceTrajectory> plan_agent_references(
    const PassingScenario& scenario, double dt = 0.15);
DistributedResult simulate_distributed_ilqr(
    const PassingScenario& scenario,
    const std::vector<ReferenceTrajectory>& references,
    const DistributedOptions& options = {});

}  // namespace mpcpark
