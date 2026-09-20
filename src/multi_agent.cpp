#include "mpcpark/multi_agent.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <stdexcept>

#include "mpcpark/planner.hpp"

namespace mpcpark {
namespace {

VecX state(double x, double y, double yaw) {
  VecX value{};
  value(kPx) = x; value(kPy) = y; value(kTheta) = yaw;
  return value;
}

bool at_goal(const TrafficAgent& agent, const VecX& x) {
  const GoalError error = goal_error(x, agent.goal);
  return error.pos <= 0.20 && error.yaw <= 0.08 && error.speed <= 0.08;
}

std::vector<VecX> rollout(const TrafficAgent& agent, const VecX& initial,
                          const std::vector<VecU>& controls, double dt) {
  std::vector<VecX> states(controls.size() + 1);
  states.front() = initial;
  for (size_t k = 0; k < controls.size(); ++k) {
    states[k + 1] = step_rk4(agent.vehicle, states[k], controls[k], dt);
  }
  return states;
}

}  // namespace

PassingScenario make_parallel_parking_traffic_scenario() {
  PassingScenario scenario;
  scenario.xmin = -17.0; scenario.xmax = 17.0;
  scenario.ymin = -2.6; scenario.ymax = 10.5;
  scenario.agents = {
      TrafficAgent{"parking", VehicleParams{}, state(-9.0, 3.2, 0.0),
                   state(-1.25, 0.0, 0.0), 0.8},
      TrafficAgent{"passing", VehicleParams{}, state(-14.0, 3.2, 0.0),
                   state(13.0, 3.2, 0.0), 1.3},
  };
  scenario.static_obstacles = {
      Rect::from_size(-5.0, 0.0, 4.5, 1.8, 0.0, "parked_car_rear"),
      Rect::from_size(5.0, 0.0, 4.5, 1.8, 0.0, "parked_car_front"),
      Rect::from_size(0.0, -1.9, 40.0, 1.0, 0.0, "curb"),
      Rect::from_size(0.0, 9.0, 40.0, 2.0, 0.0, "far_wall"),
  };
  return scenario;
}

std::vector<ReferenceTrajectory> plan_agent_references(
    const PassingScenario& scenario, double dt) {
  std::vector<ReferenceTrajectory> references;
  for (const TrafficAgent& agent : scenario.agents) {
    PlanRequest request;
    request.vehicle = agent.vehicle;
    request.obstacles = scenario.static_obstacles;
    request.start = agent.start; request.goal = agent.goal;
    request.xmin = scenario.xmin; request.xmax = scenario.xmax;
    request.ymin = scenario.ymin; request.ymax = scenario.ymax;
    PlannerOptions planner_options;
    planner_options.max_expansions = 500000;
    const Plan plan = hybrid_astar(request, planner_options);
    if (!plan.success) {
      throw std::runtime_error("failed to plan reference for " + agent.name);
    }
    TrajectoryOptions trajectory_options;
    trajectory_options.dt = dt;
    trajectory_options.cruise_speed = agent.cruise_speed;
    references.push_back(make_reference(plan, agent.start, agent.goal,
                                        agent.vehicle, trajectory_options));
  }
  return references;
}

DistributedResult simulate_distributed_ilqr(
    const PassingScenario& scenario,
    const std::vector<ReferenceTrajectory>& references,
    const DistributedOptions& options) {
  using clock = std::chrono::steady_clock;
  const int count = static_cast<int>(scenario.agents.size());
  if (count < 2 || static_cast<int>(references.size()) != count) {
    throw std::invalid_argument("distributed simulation requires matching agents and references");
  }
  std::vector<ReferenceTrajectory> active_references = references;

  int reference_steps = 0;
  for (const auto& reference : active_references) {
    reference_steps = std::max(reference_steps,
                               static_cast<int>(reference.us.size()));
  }
  std::vector<VecX> states(count);
  std::vector<std::vector<VecU>> warm(
      count, std::vector<VecU>(options.mpc_horizon));
  std::vector<std::vector<VecX>> predictions(count);
  std::vector<std::vector<VecU>> candidates = warm;
  std::vector<int> cursors(count, 0);
  for (int i = 0; i < count; ++i) {
    states[i] = scenario.agents[i].start;
    for (int j = 0; j < options.mpc_horizon; ++j) {
      const int index = std::min(j, static_cast<int>(references[i].us.size()) - 1);
      warm[i][j] = references[i].us[index];
    }
    predictions[i] = rollout(scenario.agents[i], states[i], warm[i], options.dt);
  }

  DistributedResult result;
  result.min_clearance = 1e100;
  double total_round_ms = 0.0;
  int total_rounds = 0;
  const int max_steps = reference_steps + options.settle_steps + 180;
  for (int step = 0; step < max_steps; ++step) {
    std::vector<bool> done(count, false);
    int active_agents = 0;
    for (int i = 0; i < count; ++i) {
      const int last_ref = static_cast<int>(active_references[i].xs.size()) - 1;
      if (i == 0) {
        // The lead vehicle owns the parking manoeuvre and therefore advances
        // on its nominal time-indexed reference. Followers adapt around the
        // trajectory it publishes.
        cursors[i] = std::min(step, last_ref);
      } else {
        int best = cursors[i];
        double best_score = 1e100;
        for (int index = cursors[i];
             index <= std::min(last_ref, cursors[i] + 15); ++index) {
          const VecX& ref = active_references[i].xs[index];
          const double dx = states[i](kPx) - ref(kPx);
          const double dy = states[i](kPy) - ref(kPy);
          const double dyaw = wrap_pi(states[i](kTheta) - ref(kTheta));
          const double dv = states[i](kV) - ref(kV);
          const double score = dx * dx + dy * dy + 0.5 * dyaw * dyaw +
                               0.2 * dv * dv;
          if (score < best_score) {
            best_score = score;
            best = index;
          }
        }
        cursors[i] = best;
      }
      done[i] = cursors[i] >= last_ref - 1 &&
                at_goal(scenario.agents[i], states[i]);
      if (!done[i]) ++active_agents;
    }
    const auto round_start = clock::now();
    const int rounds = active_agents > 1 ? options.coordination_rounds : 1;
    for (int round = 0; round < rounds; ++round) {
      const auto prediction_snapshot = predictions;
      std::vector<std::future<Solution>> futures(count);
      std::vector<bool> launched(count, false);
      for (int i = 0; i < count; ++i) {
        if (done[i]) {
          candidates[i].assign(options.mpc_horizon, VecU{});
          predictions[i].assign(options.mpc_horizon + 1, states[i]);
          continue;
        }
        launched[i] = true;
        futures[i] = std::async(std::launch::async, [&, i]() {
          Problem problem;
          problem.vehicle = scenario.agents[i].vehicle;
          problem.obstacles = scenario.static_obstacles;
          problem.dt = options.dt;
          problem.horizon = options.mpc_horizon;
          problem.xref.resize(problem.horizon + 1);
          problem.dynamic_obstacles.resize(problem.horizon + 1);
          const int last_ref =
              static_cast<int>(active_references[i].xs.size()) - 1;
          for (int k = 0; k <= problem.horizon; ++k) {
            problem.xref[k] = active_references[i].xs[
                std::min(cursors[i] + k, last_ref)];
            for (int other = 0; other < count; ++other) {
              if (other == i) continue;
              problem.dynamic_obstacles[k].push_back(vehicle_rect(
                  scenario.agents[other].vehicle,
                  prediction_snapshot[other][k], scenario.agents[other].name));
            }
          }
          problem.weights.pos = i == 0 ? 8.0 : 7.0;
          problem.weights.yaw = i == 0 ? 10.0 : 8.0;
          problem.weights.v = 0.6;
          problem.weights.delta = 0.8;
          problem.weights.accel = 0.35;
          problem.weights.steer_rate = 0.4;
          problem.weights.term_pos = 2500.0;
          problem.weights.term_yaw = 3000.0;
          problem.weights.term_v = 300.0;
          problem.options.max_inner = options.max_inner_iterations;
          problem.options.max_outer = options.max_outer_iterations;
          problem.options.mu_init = 50.0;
          problem.options.safety_margin = i == 0 ? 0.05 : 0.70;
          ParkingSolver controller(std::move(problem));
          return controller.solve(states[i], warm[i]);
        });
      }
      for (int i = 0; i < count; ++i) {
        if (!launched[i]) continue;
        const Solution solution = futures[i].get();
        candidates[i] = solution.us;
        predictions[i] = solution.xs;
      }
    }
    const double coordination_ms = std::chrono::duration<double, std::milli>(
                                       clock::now() - round_start).count();
    total_round_ms += coordination_ms;
    result.max_round_ms = std::max(result.max_round_ms, coordination_ms);
    const bool deadline_miss = coordination_ms > options.deadline_ms;
    if (deadline_miss) ++result.deadline_misses;
    ++total_rounds;

    bool collision = false;
    double clearance = 1e100;
    for (int i = 0; i < count; ++i) {
      collision = collision || in_collision(scenario.agents[i].vehicle, states[i],
                                            scenario.static_obstacles);
      for (int j = i + 1; j < count; ++j) {
        const Rect first = vehicle_rect(scenario.agents[i].vehicle, states[i]);
        const Rect second = vehicle_rect(scenario.agents[j].vehicle, states[j]);
        collision = collision || rects_overlap(first, second);
        clearance = std::min(clearance, rect_distance(first, second));
      }
    }
    result.min_clearance = std::min(result.min_clearance, clearance);
    std::vector<VecU> applied(count);
    for (int i = 0; i < count; ++i) {
      if (!deadline_miss) {
        applied[i] = candidates[i].front();
      } else {
        applied[i](kAccel) = clampd(-states[i](kV) / options.dt,
                                    -scenario.agents[i].vehicle.a_max,
                                    scenario.agents[i].vehicle.a_max);
        applied[i](kSteerRate) = clampd(
            -states[i](kDelta) / options.dt,
            -scenario.agents[i].vehicle.steer_rate_max,
            scenario.agents[i].vehicle.steer_rate_max);
      }
    }
    result.samples.push_back(MultiAgentSample{
        step * options.dt, states, applied, coordination_ms, clearance,
        deadline_miss, collision});
    result.collision = result.collision || collision;

    for (int i = 0; i < count; ++i) {
      states[i] = step_rk4(scenario.agents[i].vehicle, states[i], applied[i],
                           options.dt);
      for (int k = 0; k + 1 < options.mpc_horizon; ++k) {
        warm[i][k] = candidates[i][k + 1];
      }
      warm[i].back() = VecU{};
      predictions[i] = rollout(scenario.agents[i], states[i], warm[i], options.dt);
    }
    bool finished = true;
    for (int i = 0; i < count; ++i) finished = finished && at_goal(scenario.agents[i], states[i]);
    if (finished) break;
  }

  result.samples.push_back(MultiAgentSample{
      result.samples.size() * options.dt, states, std::vector<VecU>(count), 0.0,
      result.min_clearance, false, false});
  for (int i = 0; i < count; ++i) {
    result.final_errors.push_back(goal_error(states[i], scenario.agents[i].goal));
  }
  result.success = !result.collision;
  for (int i = 0; i < count; ++i) result.success = result.success && at_goal(scenario.agents[i], states[i]);
  result.mean_round_ms = total_rounds > 0 ? total_round_ms / total_rounds : 0.0;
  return result;
}

}  // namespace mpcpark
