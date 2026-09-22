#include "mpcpark/multi_agent.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace mpcpark {
namespace {
std::pair<double, double> longitudinal_extent(const TrafficAgent& agent, const VecX& x) {
  double lo = std::numeric_limits<double>::infinity(), hi = -lo;
  for (const auto& p : footprint(agent.vehicle, x)) {
    lo = std::min(lo, p(0)); hi = std::max(hi, p(0));
  }
  return {lo, hi};
}
}

const char* phase_name(PassPhase phase) {
  switch (phase) {
    case PassPhase::Waiting: return "WAIT_FOR_GAP";
    case PassPhase::Passing: return "PASS";
    case PassPhase::Returning: return "RETURN";
    case PassPhase::Complete: return "LANE_RESTORED";
  }
  return "UNKNOWN";
}

void PassNegotiation::update(const PassingScenario& scenario,
                             const std::vector<VecX>& states,
                             bool parking_near_reverse) {
  if (scenario.agents.size() != 3 || states.size() != 3)
    throw std::invalid_argument("pass negotiation requires exactly three agents");
  for (const auto& x : states)
    for (double value : x.d)
      if (!std::isfinite(value)) throw std::invalid_argument("nonfinite traffic state");
  const auto parking = longitudinal_extent(scenario.agents[0], states[0]);
  const auto passing = longitudinal_extent(scenario.agents[1], states[1]);
  const auto oncoming = longitudinal_extent(scenario.agents[2], states[2]);
  if (parking_near_reverse && !parking_released) parking_hold = true;
  // A bumper-based clearance, not a time-scripted launch. Waiting is
  // deliberately conservative: no attempt to race an approaching vehicle.
  if (phase == PassPhase::Waiting && parking_hold &&
      std::fabs(states[0](kV)) < 0.04 && oncoming.second + 1.0 < passing.first) {
    phase = PassPhase::Passing;
  }
  if (phase == PassPhase::Passing && passing.first > parking.second + 1.5) {
    parking_released = true;
    parking_hold = false;
    phase = PassPhase::Returning;
  }
  if (phase == PassPhase::Returning) {
    double top = -std::numeric_limits<double>::infinity();
    for (const auto& p : footprint(scenario.agents[1].vehicle, states[1]))
      top = std::max(top, p(1));
    if (top < 4.8 && std::fabs(wrap_pi(states[1](kTheta))) < 0.08)
      phase = PassPhase::Complete;
  }
}

PassingScenario make_oncoming_traffic_scenario() {
  auto scenario = make_parallel_parking_traffic_scenario();
  scenario.negotiate_pass = true;
  scenario.xmin = -27; scenario.xmax = 27;
  scenario.agents[1].goal(kPx) = 20;
  scenario.agents[1].cruise_speed = 1.6;
  TrafficAgent oncoming;
  oncoming.name = "oncoming";
  oncoming.start(kPx) = 12; oncoming.start(kPy) = 6.4;
  oncoming.start(kTheta) = M_PI;
  oncoming.goal = oncoming.start;
  oncoming.goal(kPx) = -22;
  oncoming.cruise_speed = 1.8;
  scenario.agents.push_back(oncoming);
  scenario.static_obstacles[2] = Rect::from_size(0, -1.9, 60, 1, 0, "curb");
  scenario.static_obstacles[3] = Rect::from_size(0, 9, 60, 2, 0, "far_wall");
  return scenario;
}
}  // namespace mpcpark
