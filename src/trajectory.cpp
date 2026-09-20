#include "mpcpark/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mpcpark {
namespace {

double pose_distance(const PlanPose& a, const PlanPose& b) {
  return std::hypot(b.x - a.x, b.y - a.y);
}

PlanPose interpolate(const PlanPose& a, const PlanPose& b, double t) {
  PlanPose p;
  p.x = a.x + t * (b.x - a.x);
  p.y = a.y + t * (b.y - a.y);
  p.yaw = wrap_pi(a.yaw + t * wrap_pi(b.yaw - a.yaw));
  p.direction = b.direction;
  p.steer = a.steer + t * (b.steer - a.steer);
  return p;
}

}  // namespace

ReferenceTrajectory make_reference(const Plan& plan, const VecX& start,
                                   const VecX& goal,
                                   const VehicleParams& vehicle,
                                   const TrajectoryOptions& options) {
  if (!plan.success || plan.poses.empty()) {
    throw std::invalid_argument("cannot build a reference from a failed plan");
  }
  if (options.dt <= 0.0 || options.cruise_speed <= 0.0) {
    throw std::invalid_argument("trajectory dt and cruise speed must be positive");
  }

  // Give every path segment a duration. Direction changes receive an explicit
  // pause so acceleration can cross zero instead of switching instantaneously.
  std::vector<double> time(plan.poses.size(), 0.0);
  for (size_t i = 1; i < plan.poses.size(); ++i) {
    if (plan.poses[i].direction != plan.poses[i - 1].direction) {
      time[i] += options.cusp_pause;
    }
    time[i] += time[i - 1] +
               pose_distance(plan.poses[i - 1], plan.poses[i]) /
                   options.cruise_speed;
  }
  const int horizon = std::max(1, static_cast<int>(std::ceil(time.back() / options.dt)));

  ReferenceTrajectory out;
  out.dt = options.dt;
  out.xs.resize(horizon + 1);
  out.us.resize(horizon);
  out.xs.front() = start;

  size_t seg = 1;
  for (int k = 1; k <= horizon; ++k) {
    const double t = std::min(time.back(), k * options.dt);
    while (seg + 1 < time.size() && time[seg] < t) ++seg;
    const size_t prev = seg - 1;
    const double duration = time[seg] - time[prev];
    const double alpha = duration > 1e-9 ? clampd((t - time[prev]) / duration, 0.0, 1.0)
                                         : 1.0;
    const PlanPose p = interpolate(plan.poses[prev], plan.poses[seg], alpha);
    VecX x{};
    x(kPx) = p.x;
    x(kPy) = p.y;
    x(kTheta) = p.yaw;
    x(kV) = p.direction * options.cruise_speed;
    x(kDelta) = clampd(p.steer, -vehicle.delta_max, vehicle.delta_max);
    out.xs[k] = x;
  }

  // Rate-limit the reference itself. In particular, Hybrid A* can change gear
  // at a cusp instantaneously, while the bicycle model must brake through
  // zero first. Forward/backward passes make every speed and steering change
  // reachable from both fixed endpoints.
  out.xs.front()(kV) = start(kV);
  out.xs.front()(kDelta) = start(kDelta);
  out.xs.back() = goal;
  const double dv = vehicle.a_max * options.dt;
  const double dd = vehicle.steer_rate_max * options.dt;
  for (int pass = 0; pass < 3; ++pass) {
    for (int k = 1; k < horizon; ++k) {
      out.xs[k](kV) = clampd(out.xs[k](kV), out.xs[k - 1](kV) - dv,
                             out.xs[k - 1](kV) + dv);
      out.xs[k](kDelta) = clampd(out.xs[k](kDelta),
                                 out.xs[k - 1](kDelta) - dd,
                                 out.xs[k - 1](kDelta) + dd);
    }
    for (int k = horizon - 1; k > 0; --k) {
      out.xs[k](kV) = clampd(out.xs[k](kV), out.xs[k + 1](kV) - dv,
                             out.xs[k + 1](kV) + dv);
      out.xs[k](kDelta) = clampd(out.xs[k](kDelta),
                                 out.xs[k + 1](kDelta) - dd,
                                 out.xs[k + 1](kDelta) + dd);
    }
  }

  for (int k = 0; k < horizon; ++k) {
    out.us[k](kAccel) = clampd((out.xs[k + 1](kV) - out.xs[k](kV)) / options.dt,
                               -vehicle.a_max, vehicle.a_max);
    const double delta_step = out.xs[k + 1](kDelta) - out.xs[k](kDelta);
    out.us[k](kSteerRate) = clampd(delta_step / options.dt, -vehicle.steer_rate_max,
                                   vehicle.steer_rate_max);
  }
  return out;
}

}  // namespace mpcpark
