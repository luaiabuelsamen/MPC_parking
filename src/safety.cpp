#include "mpcpark/multi_agent.hpp"

#include <limits>
#include <stdexcept>

namespace mpcpark {

VecU braking_control(const VehicleParams& vehicle, const VecX& state, double dt) {
  if (!std::isfinite(dt) || dt <= 0.0) throw std::invalid_argument("invalid brake dt");
  VecU u{};
  u(kAccel) = clampd(-state(kV) / dt, -vehicle.a_max, vehicle.a_max);
  // Hold steering: unwinding it while braking changes the swept path.
  return u;
}

MotionCheck check_motion(const PassingScenario& scenario,
                         const std::vector<VecX>& states,
                         const std::vector<VecU>& controls, double dt,
                         int substeps) {
  if (states.size() != scenario.agents.size() || controls.size() != states.size() ||
      states.empty() || !std::isfinite(dt) || dt <= 0.0 || substeps < 1) {
    throw std::invalid_argument("invalid motion check dimensions or interval");
  }
  MotionCheck result;
  result.states = states;
  result.min_clearance = result.min_static_clearance =
      std::numeric_limits<double>::infinity();
  for (const auto& u : controls) {
    for (double value : u.d) result.finite = result.finite && std::isfinite(value);
  }
  for (int tick = 0; tick <= substeps; ++tick) {
    std::vector<Rect> bodies;
    for (size_t i = 0; i < states.size(); ++i) {
      const auto& x = result.states[i];
      for (double value : x.d) result.finite = result.finite && std::isfinite(value);
      if (!result.finite) return result;
      const auto& v = scenario.agents[i].vehicle;
      // AL state constraints use a small numerical tolerance.
      constexpr double tolerance = 0.02;
      result.limits_ok = result.limits_ok &&
          x(kV) <= v.v_max + tolerance && x(kV) >= v.v_min - tolerance &&
          std::fabs(x(kDelta)) <= v.delta_max + tolerance &&
          std::fabs(controls[i](kAccel)) <= v.a_max + 1e-9 &&
          std::fabs(controls[i](kSteerRate)) <= v.steer_rate_max + 1e-9;
      bodies.push_back(vehicle_rect(v, x));
      for (const auto& obstacle : scenario.static_obstacles) {
        result.collision = result.collision || rects_overlap(bodies.back(), obstacle);
        result.min_static_clearance = std::min(result.min_static_clearance,
                                                rect_distance(bodies.back(), obstacle));
      }
      for (size_t j = 0; j < i; ++j) {
        result.collision = result.collision || rects_overlap(bodies[i], bodies[j]);
        result.min_clearance = std::min(result.min_clearance,
                                        rect_distance(bodies[i], bodies[j]));
      }
    }
    if (tick < substeps) {
      for (size_t i = 0; i < states.size(); ++i) {
        result.states[i] = step_rk4(scenario.agents[i].vehicle, result.states[i],
                                    controls[i], dt / substeps);
      }
    }
  }
  return result;
}

}  // namespace mpcpark
