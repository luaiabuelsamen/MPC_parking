#include "mpcpark/world.hpp"

#include <algorithm>
#include <chrono>
#include <future>
#include <stdexcept>

namespace mpcpark {
namespace {
using Clock = std::chrono::steady_clock;
double milliseconds(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}
bool arrived(const LiveAgent& a) {
  const auto error = goal_error(a.state, a.spec.goal);
  return error.pos < 0.18 && error.yaw < 0.08 && error.speed < 0.06;
}
std::vector<VecX> predict(const LiveAgent& a, const std::vector<VecU>& controls) {
  std::vector<VecX> xs{a.state};
  for (const auto& u : controls) xs.push_back(step_rk4(a.spec.vehicle, xs.back(), u, LiveWorld::dt));
  return xs;
}
std::vector<VecU> stop_controls(const LiveAgent& a) {
  std::vector<VecU> us(LiveWorld::horizon);
  auto x = a.state;
  for (auto& u : us) {
    u = braking_control(a.spec.vehicle, x, LiveWorld::dt);
    x = step_rk4(a.spec.vehicle, x, u, LiveWorld::dt);
  }
  return us;
}
void finite_pose(const VecX& x) {
  for (double value : x.d) if (!std::isfinite(value))
    throw std::invalid_argument("State and goal values must be finite");
  if (std::fabs(x(kPx)) > 21 || std::fabs(x(kPy)) > 12)
    throw std::invalid_argument("Keep cars and goals inside the map");
}
bool can_stop(const PassingScenario& world, std::vector<VecX> states) {
  int ticks = 1;
  for (size_t i = 0; i < states.size(); ++i) {
    const double duration = std::ceil(std::fabs(states[i](kV)) /
        (world.agents[i].vehicle.a_max * LiveWorld::dt)) + 1;
    if (!std::isfinite(duration) || duration > 1000) return false;
    ticks = std::max(ticks, static_cast<int>(duration));
  }
  for (int k = 0; k < ticks; ++k) {
    std::vector<VecU> controls;
    for (size_t i = 0; i < states.size(); ++i)
      controls.push_back(braking_control(world.agents[i].vehicle, states[i], LiveWorld::dt));
    auto checked = check_motion(world, states, controls, LiveWorld::dt, 10);
    if (!checked.safe()) return false;
    states = std::move(checked.states);
  }
  return true;
}
}

LiveWorld::LiveWorld(double decision_budget_ms) : decision_budget_ms_(decision_budget_ms) {
  if (!std::isfinite(decision_budget_ms) || decision_budget_ms <= 0)
    throw std::invalid_argument("Decision budget must be positive and finite");
}

PassingScenario LiveWorld::scene() const {
  PassingScenario result;
  result.xmin = -25; result.xmax = 25; result.ymin = -15; result.ymax = 15;
  result.static_obstacles = obstacles_;
  // The map edge is a real obstacle for both the planner and supervisor.
  result.static_obstacles.push_back(Rect::from_size(0, -16, 54, 2, 0));
  result.static_obstacles.push_back(Rect::from_size(0, 16, 54, 2, 0));
  result.static_obstacles.push_back(Rect::from_size(-26, 0, 2, 34, 0));
  result.static_obstacles.push_back(Rect::from_size(26, 0, 2, 34, 0));
  for (const auto& a : agents_) {
    result.agents.push_back(a.spec);
    result.agents.back().start = a.state;
  }
  return result;
}

void LiveWorld::validate_edit(const std::vector<TrafficAgent>& agents,
                              const std::vector<Rect>& obstacles) const {
  if (agents.empty() || agents.size() > 8) throw std::invalid_argument("Use between one and eight cars");
  if (obstacles.size() > 40) throw std::invalid_argument("Use at most forty obstacles");
  for (const auto& r : obstacles) {
    for (double n : {r.cx, r.cy, r.hx, r.hy, r.yaw})
      if (!std::isfinite(n)) throw std::invalid_argument("Obstacle values must be finite");
    if (r.hx < 0.1 || r.hy < 0.1 || r.hx > 15 || r.hy > 15 ||
        std::fabs(r.cx) > 24 || std::fabs(r.cy) > 14)
      throw std::invalid_argument("Obstacle dimensions or position outside supported map");
  }
  PassingScenario proposed;
  proposed.agents = agents;
  proposed.static_obstacles = obstacles;
  auto bounds = scene().static_obstacles;
  proposed.static_obstacles.insert(proposed.static_obstacles.end(), bounds.end()-4, bounds.end());
  std::vector<VecX> states;
  for (const auto& a : agents) {
    finite_pose(a.start); finite_pose(a.goal);
    const auto& v = a.vehicle;
    for (double n : {v.wheelbase, v.length, v.width, v.rear_overhang, v.v_min,
                     v.v_max, v.a_max, v.delta_max, v.steer_rate_max})
      if (!std::isfinite(n)) throw std::invalid_argument("Vehicle parameters must be finite");
    if (v.wheelbase <= 0 || v.length <= 0 || v.width <= 0 || v.rear_overhang < 0 ||
        v.rear_overhang >= v.length || v.v_min >= 0 || v.v_max <= 0 || v.a_max <= 0 ||
        v.delta_max <= 0 || v.delta_max >= M_PI_2 || v.steer_rate_max <= 0 || v.n_discs < 1)
      throw std::invalid_argument("Invalid vehicle dimensions or actuator limits");
    if (a.cruise_speed < 0.3 || a.cruise_speed > 1.8 || !std::isfinite(a.cruise_speed))
      throw std::invalid_argument("Cruise speed must be between 0.3 and 1.8 m/s");
    states.push_back(a.start);
  }
  // Reject edits intersecting the complete nominal stopping rollout. An edit
  // must not teleport a wall into a vehicle's unavoidable braking path.
  if (!can_stop(proposed, states))
    throw std::invalid_argument("Edit intersects a car or its stopping path; place it farther away");
}

void LiveWorld::reset(std::vector<TrafficAgent> agents, std::vector<Rect> obstacles) {
  validate_edit(agents, obstacles);
  agents_.clear();
  for (auto& spec : agents) {
    LiveAgent a; a.spec = std::move(spec); a.state = a.spec.start;
    a.warm = stop_controls(a); a.prediction = predict(a, a.warm);
    agents_.push_back(std::move(a));
  }
  obstacles_ = std::move(obstacles);
  time_ = decision_ms_ = planning_ms_ = clearance_ = 0;
  replans_ = 0;
  request_replan("New world loaded; planning from measured poses");
}

void LiveWorld::request_replan(const std::string& event) {
  ++revision_; pending_plan_ = true; safety_stop_ = false;
  blocked_ticks_ = recovery_attempts_ = 0; event_ = event;
  for (auto& a : agents_) {
    a.route_valid = false; a.reference = {}; a.cursor = 0;
    a.warm = stop_controls(a); a.prediction = predict(a, a.warm);
    a.status = "BRAKING_TO_REPLAN";
  }
}

void LiveWorld::set_goal(size_t index, const VecX& goal) {
  if (index >= agents_.size()) throw std::invalid_argument("Unknown car");
  auto proposed = scene().agents;
  proposed[index].goal = goal;
  validate_edit(proposed, obstacles_);
  agents_[index].spec.goal = goal;
  request_replan("Goal changed; stopping before replanning");
}

void LiveWorld::set_obstacles(std::vector<Rect> obstacles) {
  validate_edit(scene().agents, obstacles);
  obstacles_ = std::move(obstacles);
  request_replan("Map changed; stopping before replanning");
}

void LiveWorld::add_agent(const TrafficAgent& spec) {
  auto proposed = scene().agents;
  proposed.push_back(spec);
  validate_edit(proposed, obstacles_);
  LiveAgent a; a.spec = spec; a.state = spec.start;
  agents_.push_back(std::move(a));
  request_replan("Car added; all agents update their plans");
}

void LiveWorld::plan() {
  const auto start = Clock::now();
  const auto world = scene();
  int routes = 0;
  for (size_t i = 0; i < agents_.size(); ++i) {
    auto& a = agents_[i];
    a.cursor = 0; a.route_valid = false;
    a.warm = stop_controls(a); a.prediction = predict(a, a.warm);
    if (arrived(a)) { a.status = "ARRIVED"; continue; }
    PlanRequest request;
    request.start = a.state; request.goal = a.spec.goal; request.vehicle = a.spec.vehicle;
    request.obstacles = world.static_obstacles;
    request.xmin = world.xmin; request.xmax = world.xmax;
    request.ymin = world.ymin; request.ymax = world.ymax;
    for (size_t j = 0; j < agents_.size(); ++j) if (i != j) {
      auto r = vehicle_rect(agents_[j].spec.vehicle, agents_[j].state);
      r.hx += 0.15; r.hy += 0.15;
      request.obstacles.push_back(r);
    }
    PlannerOptions planner_options;
    planner_options.max_expansions = 100000;
    const auto route = hybrid_astar(request, planner_options);
    if (!route.success) { a.status = "NO_ROUTE"; continue; }
    TrajectoryOptions trajectory_options;
    trajectory_options.dt = dt; trajectory_options.cruise_speed = a.spec.cruise_speed;
    a.reference = make_reference(route, a.state, a.spec.goal, a.spec.vehicle, trajectory_options);
    for (int k = 0; k < horizon; ++k) a.warm[k] = a.reference.us[std::min(k, static_cast<int>(a.reference.us.size()) - 1)];
    a.prediction = predict(a, a.warm);
    a.route_valid = true; a.status = "DRIVING"; ++routes;
  }
  planning_ms_ = milliseconds(start); ++replans_; pending_plan_ = false;
  event_ = std::to_string(routes) + " routes planned from current poses";
}

void LiveWorld::step() {
  if (agents_.empty() || safety_stop_) return;
  bool resting = true;
  for (const auto& a : agents_) resting = resting && std::fabs(a.state(kV)) < 0.001;
  if (pending_plan_ && resting) plan();
  const auto start = Clock::now();
  const auto deadline = start + std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double, std::milli>(decision_budget_ms_ * 0.9));
  const auto world = scene();
  std::vector<VecX> states;
  std::vector<std::vector<VecX>> broadcast;
  for (const auto& a : agents_) { states.push_back(a.state); broadcast.push_back(a.prediction); }
  std::vector<VecU> controls(agents_.size());
  std::vector<std::future<Solution>> solves(agents_.size());
  std::vector<bool> active(agents_.size(), false);
  for (size_t i = 0; i < agents_.size(); ++i) {
    auto& a = agents_[i];
    if (pending_plan_ || !a.route_valid || arrived(a)) {
      if (arrived(a)) { a.status = "ARRIVED"; a.route_valid = false; }
      a.warm = stop_controls(a); a.prediction = predict(a, a.warm);
      controls[i] = a.warm[0]; continue;
    }
    active[i] = true;
    solves[i] = std::async(std::launch::async, [&, i]() {
      const auto& agent = agents_[i];
      Problem p;
      p.vehicle = agent.spec.vehicle; p.obstacles = world.static_obstacles;
      p.horizon = horizon; p.dt = dt;
      p.xref.resize(horizon+1); p.dynamic_obstacles.resize(horizon+1);
      for (int k = 0; k <= horizon; ++k) {
        p.xref[k] = agent.reference.xs[std::min(agent.cursor + k, static_cast<int>(agent.reference.xs.size())-1)];
        for (size_t j = 0; j < agents_.size(); ++j) if (i != j)
          p.dynamic_obstacles[k].push_back(vehicle_rect(agents_[j].spec.vehicle, broadcast[j][k]));
      }
      p.weights.pos = 8; p.weights.yaw = 10; p.weights.v = 0.6;
      p.weights.delta = 0.8; p.weights.accel = 0.35; p.weights.steer_rate = 0.4;
      p.weights.term_pos = 2500; p.weights.term_yaw = 3000; p.weights.term_v = 300;
      p.options.max_inner = 15; p.options.max_outer = 4;
      p.options.mu_init = 50; p.options.safety_margin = 0.1;
      ParkingSolver solver(std::move(p));
      return solver.solve(agent.state, agent.warm, deadline);
    });
  }
  bool timeout = false;
  for (size_t i = 0; i < agents_.size(); ++i) if (active[i]) {
    auto solution = solves[i].get();
    timeout = timeout || solution.stats.timed_out;
    agents_[i].warm = std::move(solution.us);
    agents_[i].prediction = std::move(solution.xs);
    auto& u = controls[i] = agents_[i].warm[0];
    const auto& v = agents_[i].spec.vehicle;
    if (std::isfinite(u(kAccel)) && std::isfinite(u(kSteerRate))) {
      u(kAccel) = clampd(u(kAccel), std::max(-v.a_max, (v.v_min-states[i](kV))/dt), std::min(v.a_max, (v.v_max-states[i](kV))/dt));
      u(kSteerRate) = clampd(u(kSteerRate), std::max(-v.steer_rate_max, (-v.delta_max-states[i](kDelta))/dt), std::min(v.steer_rate_max, (v.delta_max-states[i](kDelta))/dt));
    }
  }
  auto checked = check_motion(world, states, controls, dt, 10);
  // Every accepted joint command must leave a sampled collision-free nominal
  // stop available. Deadline fallback then follows that same checked stop.
  const bool stoppable = checked.safe() && can_stop(world, checked.states);
  const bool fallback = timeout || Clock::now() >= deadline || !stoppable;
  if (fallback) {
    for (size_t i = 0; i < agents_.size(); ++i)
      controls[i] = braking_control(agents_[i].spec.vehicle, states[i], dt);
    checked = check_motion(world, states, controls, dt, 10);
    event_ = timeout ? "Solve budget expired; checked braking" : "Joint safety check rejected commands; checked braking";
  }
  decision_ms_ = milliseconds(start);
  if (!checked.safe()) {
    safety_stop_ = true; event_ = "No safe control found; simulation stopped at last measured state";
    for (auto& a : agents_) a.status = "SAFETY_STOP";
    return;
  }
  clearance_ = std::min(checked.min_clearance, checked.min_static_clearance);
  bool progressing = false, unfinished = false;
  for (size_t i = 0; i < agents_.size(); ++i) {
    auto& a = agents_[i]; a.state = checked.states[i];
    progressing = progressing || std::hypot(a.state(kPx)-states[i](kPx), a.state(kPy)-states[i](kPy)) > 0.005;
    unfinished = unfinished || !arrived(a);
    if (active[i]) {
      a.status = fallback ? "YIELDING" : "DRIVING";
      if (!fallback) ++a.cursor;
    }
    if (fallback) a.warm = stop_controls(a);
    else { std::rotate(a.warm.begin(), a.warm.begin()+1, a.warm.end()); a.warm.back() = {}; }
    a.prediction = predict(a, a.warm);
    if (arrived(a)) a.status = "ARRIVED";
  }
  time_ += dt;
  blocked_ticks_ = !pending_plan_ && unfinished && !progressing ? blocked_ticks_+1 : 0;
  if (blocked_ticks_ >= 20) {
    if (recovery_attempts_ < 3) {
      ++recovery_attempts_; ++revision_; blocked_ticks_ = 0; pending_plan_ = true;
      for (auto& a : agents_) { a.route_valid = false; a.status = "BRAKING_TO_REPLAN"; }
      event_ = "Progress stalled; replanning around current traffic";
    } else {
      for (auto& a : agents_) if (!arrived(a)) { a.status = "BLOCKED"; a.route_valid = false; }
      event_ = "No progress after three recovery attempts; edit the map or goals to continue";
    }
  }
}
}  // namespace mpcpark
