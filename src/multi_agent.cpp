#include "mpcpark/multi_agent.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
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
  if (!std::isfinite(options.dt) || options.dt <= 0 || options.mpc_horizon < 2 ||
      options.coordination_rounds < 1 || options.integration_substeps < 1 ||
      options.max_inner_iterations < 1 || options.max_outer_iterations < 1 ||
      !std::isfinite(options.deadline_ms) || options.deadline_ms <= 0 ||
      !std::isfinite(options.plant_wheelbase_scale) || options.plant_wheelbase_scale <= 0 ||
      options.fault_steps < 0 || options.max_steps < 0 || options.settle_steps < 0) {
    throw std::invalid_argument("invalid distributed controller options");
  }
  for (const auto& reference : references) {
    if (reference.us.empty() || reference.xs.size() != reference.us.size() + 1 ||
        !std::isfinite(reference.dt) || std::fabs(reference.dt - options.dt) > 1e-9) {
      throw std::invalid_argument("reference dimensions or dt do not match controller");
    }
    for (const auto& x : reference.xs)
      for (double value : x.d)
        if (!std::isfinite(value)) throw std::invalid_argument("nonfinite reference");
    for (const auto& u : reference.us)
      for (double value : u.d)
        if (!std::isfinite(value)) throw std::invalid_argument("nonfinite controls");
  }
  for (const auto& agent : scenario.agents) {
    const auto& v = agent.vehicle;
    for (double value : {v.wheelbase, v.length, v.width, v.a_max, v.steer_rate_max,
                         v.delta_max, v.v_min, v.v_max, v.rear_overhang})
      if (!std::isfinite(value)) throw std::invalid_argument("nonfinite vehicle parameter");
    for (double value : agent.start.d)
      if (!std::isfinite(value)) throw std::invalid_argument("nonfinite initial state");
    for (double value : agent.goal.d)
      if (!std::isfinite(value)) throw std::invalid_argument("nonfinite goal");
    if (!(v.wheelbase > 0 && v.length > 0 && v.width > 0 && v.a_max > 0 &&
          v.steer_rate_max > 0 && v.delta_max > 0 && v.delta_max < M_PI_2 &&
          v.v_min < 0 && v.v_max > 0 && v.n_discs > 0))
      throw std::invalid_argument("invalid vehicle parameters");
  }
  PassingScenario plant = scenario;
  for (auto& agent : plant.agents) agent.vehicle.wheelbase *= options.plant_wheelbase_scale;
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
  result.min_clearance = result.min_static_clearance =
      std::numeric_limits<double>::infinity();
  std::vector<double> latencies;
  double total_round_ms = 0.0;
  int total_rounds = 0;
  int exchanges = 0;
  int parking_tick = 0;
  const int max_steps = options.max_steps > 0 ? options.max_steps :
      reference_steps + options.settle_steps + 180;
  for (int step = 0; step < max_steps; ++step) {
    const auto round_start = clock::now();
    const auto deadline = round_start + std::chrono::duration_cast<clock::duration>(
        std::chrono::duration<double, std::milli>(options.deadline_ms * 0.9));
    std::vector<bool> done(count, false);
    int active_agents = 0;
    for (int i = 0; i < count; ++i) {
      const int last_ref = static_cast<int>(active_references[i].xs.size()) - 1;
      if (i == 0) {
        // The lead vehicle owns the parking manoeuvre and therefore advances
        // on its nominal time-indexed reference. Followers adapt around the
        // trajectory it publishes.
        cursors[i] = std::min(parking_tick, last_ref);
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
    const int rounds = active_agents > 1 ? options.coordination_rounds : 1;
    int executed_rounds = 0;
    bool timed_out = false;
    for (int round = 0; round < rounds; ++round) {
      const auto prediction_snapshot = predictions;
      std::vector<std::future<Solution>> futures(count);
      std::vector<bool> launched(count, false);
      for (int i = 0; i < count; ++i) {
        if (done[i]) {
          VecX stopped = states[i];
          for (int k = 0; k < options.mpc_horizon; ++k) {
            candidates[i][k] = braking_control(scenario.agents[i].vehicle, stopped, options.dt);
            stopped = step_rk4(scenario.agents[i].vehicle, stopped, candidates[i][k], options.dt);
          }
          predictions[i] = rollout(scenario.agents[i], states[i], candidates[i], options.dt);
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
          return controller.solve(states[i], warm[i], deadline);
        });
      }
      for (int i = 0; i < count; ++i) {
        if (!launched[i]) continue;
        const Solution solution = futures[i].get();
        timed_out = timed_out || solution.stats.timed_out;
        candidates[i] = solution.us;
        predictions[i] = solution.xs;
      }
      ++executed_rounds;
      double change = 0.0;
      for (int i = 0; i < count; ++i) {
        warm[i] = candidates[i];
        for (int k = 0; k <= options.mpc_horizon; ++k) {
          change = std::max(change, (predictions[i][k] - prediction_snapshot[i][k]).max_abs());
        }
      }
      if (timed_out || change < 0.03) break;
    }
    std::vector<VecU> applied(count);
    for (int i = 0; i < count; ++i) {
      applied[i] = candidates[i].front();
      if (!std::isfinite(applied[i](kAccel)) || !std::isfinite(applied[i](kSteerRate)))
        continue;  // Do not let clampd hide a NaN from the supervisor.
      const auto& v = scenario.agents[i].vehicle;
      // Bound the integrated actuator states as well as command magnitudes.
      applied[i](kAccel) = clampd(applied[i](kAccel),
          std::max(-v.a_max, (v.v_min - states[i](kV)) / options.dt),
          std::min(v.a_max, (v.v_max - states[i](kV)) / options.dt));
      applied[i](kSteerRate) = clampd(applied[i](kSteerRate),
          std::max(-v.steer_rate_max, (-v.delta_max - states[i](kDelta)) / options.dt),
          std::min(v.steer_rate_max, (v.delta_max - states[i](kDelta)) / options.dt));
    }
    auto checked = check_motion(scenario, states, applied, options.dt, options.integration_substeps);
    const bool budget_expired = timed_out || clock::now() >= deadline;
    result.solver_timeouts += budget_expired;
    const bool injected = options.fault_start_step >= 0 && step >= options.fault_start_step &&
                         step - options.fault_start_step < options.fault_steps;
    result.injected_faults += injected;
    const bool rejected = !checked.safe();
    result.safety_rejections += rejected;
    const bool fallback = budget_expired || injected || rejected;
    if (fallback) {
      ++result.fallback_steps;
      for (int i = 0; i < count; ++i)
        applied[i] = braking_control(scenario.agents[i].vehicle, states[i], options.dt);
      checked = check_motion(scenario, states, applied, options.dt, options.integration_substeps);
    }
    const double coordination_ms = std::chrono::duration<double, std::milli>(
        clock::now() - round_start).count();
    const bool deadline_miss = coordination_ms > options.deadline_ms;
    result.deadline_misses += deadline_miss;
    result.max_round_ms = std::max(result.max_round_ms, coordination_ms);
    total_round_ms += coordination_ms;
    latencies.push_back(coordination_ms);
    ++total_rounds;
    exchanges += executed_rounds;
    if (!checked.safe()) {
      // No safe command was found. Terminate at the measured state rather
      // than execute an unsafe fallback or silently label it collision-free.
      result.safety_stop = true;
      break;
    }
    // The supervisor knows only the nominal model. Mismatch belongs in the
    // plant, and is assessed after application rather than used as an oracle.
    checked = check_motion(plant, states, applied, options.dt, options.integration_substeps);
    result.min_clearance = std::min(result.min_clearance, checked.min_clearance);
    result.min_static_clearance = std::min(result.min_static_clearance, checked.min_static_clearance);
    result.samples.push_back(MultiAgentSample{
        step * options.dt, states, applied, coordination_ms, checked.min_clearance,
        deadline_miss, checked.collision, fallback, executed_rounds});
    result.collision = result.collision || checked.collision;
    states = std::move(checked.states);
    if (!checked.safe()) {
      result.safety_stop = true;
      break;
    }
    if (!fallback) ++parking_tick;
    for (int i = 0; i < count; ++i) {
      for (int k = 0; k + 1 < options.mpc_horizon; ++k) {
        warm[i][k] = fallback ? VecU{} : candidates[i][k + 1];
      }
      warm[i].back() = VecU{};
      predictions[i] = rollout(scenario.agents[i], states[i], warm[i], options.dt);
    }
    bool finished = true;
    for (int i = 0; i < count; ++i) finished = finished && at_goal(scenario.agents[i], states[i]);
    if (finished) break;
  }

  // Only inspect the current geometry here, not the hypothetical coast after
  // stopping the experiment. It was already checked at the last plant substep.
  double final_clearance = std::numeric_limits<double>::infinity();
  bool final_collision = false;
  for (int i = 0; i < count; ++i) {
    const Rect body = vehicle_rect(plant.agents[i].vehicle, states[i]);
    for (const auto& obstacle : plant.static_obstacles)
      result.min_static_clearance = std::min(result.min_static_clearance, rect_distance(body, obstacle));
    final_collision = final_collision || in_collision(plant.agents[i].vehicle, states[i], plant.static_obstacles);
    for (int j = 0; j < i; ++j) {
      const Rect other = vehicle_rect(plant.agents[j].vehicle, states[j]);
      final_collision = final_collision || rects_overlap(body, other);
      final_clearance = std::min(final_clearance, rect_distance(body, other));
    }
  }
  result.min_clearance = std::min(result.min_clearance, final_clearance);
  result.collision = result.collision || final_collision;
  result.samples.push_back(MultiAgentSample{
      result.samples.size() * options.dt, states, std::vector<VecU>(count), 0.0,
      final_clearance, false, final_collision});
  for (int i = 0; i < count; ++i) {
    result.final_errors.push_back(goal_error(states[i], scenario.agents[i].goal));
  }
  result.success = !result.collision && !result.safety_stop;
  for (int i = 0; i < count; ++i) result.success = result.success && at_goal(scenario.agents[i], states[i]);
  result.mean_round_ms = total_rounds > 0 ? total_round_ms / total_rounds : 0.0;
  result.mean_rounds = total_rounds > 0 ? static_cast<double>(exchanges) / total_rounds : 0.0;
  std::sort(latencies.begin(), latencies.end());
  if (!latencies.empty()) result.p95_round_ms = latencies[static_cast<size_t>(0.95 * (latencies.size() - 1))];
  return result;
}

}  // namespace mpcpark
