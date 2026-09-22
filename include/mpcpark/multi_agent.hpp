// Distributed MPC simulation using simultaneous Jacobi best responses. Each
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
  // Three-car encounter: agents are parking, passing, and oncoming.
  bool negotiate_pass = false;
};

enum class PassPhase { Waiting, Passing, Returning, Complete };
const char* phase_name(PassPhase phase);

// A conservative right-of-way protocol over broadcast measured states.
// It admits a pass only after oncoming traffic has physically cleared and
// the parking car acknowledges a stop before its reverse manoeuvre.
struct PassNegotiation {
  PassPhase phase = PassPhase::Waiting;
  bool parking_hold = false;
  bool parking_released = false;
  void update(const PassingScenario& scenario, const std::vector<VecX>& states,
              bool parking_near_reverse);
};

struct DistributedOptions {
  double dt = 0.15;
  int mpc_horizon = 25;
  int coordination_rounds = 1;
  int settle_steps = 120;
  int max_inner_iterations = 15;
  int max_outer_iterations = 4;
  double deadline_ms = 150.0;
  int integration_substeps = 10;  // collision/state checks every dt/substeps
  int max_steps = 0;             // zero uses reference duration + settling
  // Deterministic fault injection: discard the solves in this step interval.
  int fault_start_step = -1;
  int fault_steps = 0;
  double plant_wheelbase_scale = 1.0;  // model mismatch, controller stays nominal
};

struct MotionCheck {
  std::vector<VecX> states;
  double min_clearance = 0.0;
  double min_static_clearance = 0.0;
  bool collision = false;
  bool limits_ok = true;
  bool finite = true;
  bool safe() const { return finite && limits_ok && !collision; }
};

// Checks the initial state and every RK4 substep, including the final state.
// This is sampled collision checking, not a continuous-time proof.
MotionCheck check_motion(const PassingScenario& scenario,
                         const std::vector<VecX>& states,
                         const std::vector<VecU>& controls, double dt,
                         int substeps);
VecU braking_control(const VehicleParams& vehicle, const VecX& state, double dt);

struct MultiAgentSample {
  double time = 0.0;
  std::vector<VecX> states;
  std::vector<VecU> controls;
  double solve_ms = 0.0;
  double clearance = 0.0;
  bool deadline_miss = false;
  bool collision = false;
  bool fallback = false;
  int rounds = 0;
  std::vector<std::string> modes;
  std::vector<std::vector<VecX>> predictions;
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
  int solver_timeouts = 0;
  int injected_faults = 0;
  int fallback_steps = 0;
  int safety_rejections = 0;
  bool safety_stop = false;
  double p95_round_ms = 0.0;
  double min_static_clearance = 0.0;
  double mean_rounds = 0.0;
};

PassingScenario make_parallel_parking_traffic_scenario();
PassingScenario make_oncoming_traffic_scenario();
std::vector<ReferenceTrajectory> plan_agent_references(
    const PassingScenario& scenario, double dt = 0.15);
DistributedResult simulate_distributed_ilqr(
    const PassingScenario& scenario,
    const std::vector<ReferenceTrajectory>& references,
    const DistributedOptions& options = {});

}  // namespace mpcpark
