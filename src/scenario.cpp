#include "mpcpark/scenario.hpp"

#include <stdexcept>

namespace mpcpark {

namespace {

VecX make_state(double x, double y, double yaw, double v = 0.0,
                double delta = 0.0) {
  VecX s{};
  s(kPx) = x;
  s(kPy) = y;
  s(kTheta) = yaw;
  s(kV) = v;
  s(kDelta) = delta;
  return s;
}

// Rear-axle x/y for a car whose body centre sits at (cx, cy) with heading yaw.
VecX goal_from_body_center(const VehicleParams& vp, double cx, double cy,
                           double yaw) {
  const double back = 0.5 * vp.length - vp.rear_overhang;
  return make_state(cx - back * std::cos(yaw), cy - back * std::sin(yaw), yaw);
}

Scenario parallel_scenario() {
  Scenario s;
  s.name = "parallel";
  s.start = make_state(-9.0, 3.2, 0.0);
  // Slot is 5.5 m long for a 4.5 m car: 0.5 m of slack at each bumper.
  s.obstacles = {
      Rect::from_size(-5.0, 0.0, 4.5, 1.8, 0.0, "parked_car_rear"),
      Rect::from_size(5.0, 0.0, 4.5, 1.8, 0.0, "parked_car_front"),
      Rect::from_size(0.0, -1.9, 40.0, 1.0, 0.0, "curb"),
      Rect::from_size(0.0, 8.0, 40.0, 2.0, 0.0, "far_wall"),
  };
  s.goal = goal_from_body_center(s.vehicle, 0.0, 0.0, 0.0);
  s.xmin = -14.0; s.xmax = 14.0; s.ymin = -2.6; s.ymax = 9.5;
  return s;
}

Scenario perpendicular_scenario() {
  Scenario s;
  s.name = "perpendicular";
  s.start = make_state(-8.0, 2.5, 0.0);
  // Bay is 3.0 m wide for a 1.8 m car, entered in reverse from the aisle.
  s.obstacles = {
      Rect::from_size(-2.4, -3.3, 4.5, 1.8, M_PI_2, "parked_car_left"),
      Rect::from_size(2.4, -3.3, 4.5, 1.8, M_PI_2, "parked_car_right"),
      Rect::from_size(-7.2, -3.3, 4.5, 1.8, M_PI_2, "parked_car_far_left"),
      Rect::from_size(0.0, -6.2, 40.0, 1.2, 0.0, "bay_wall"),
      Rect::from_size(0.0, 7.0, 40.0, 2.0, 0.0, "far_wall"),
  };
  s.goal = goal_from_body_center(s.vehicle, 0.0, -3.25, M_PI_2);
  s.xmin = -14.0; s.xmax = 14.0; s.ymin = -7.5; s.ymax = 8.5;
  return s;
}

Scenario garage_scenario() {
  Scenario s;
  s.name = "garage";
  s.start = make_state(-8.0, 3.0, 0.0);
  // A 3.0 m wide garage backed into from a driveway that is fenced on the
  // right, so the car cannot simply swing wide before reversing.
  s.obstacles = {
      Rect::from_size(-1.8, -3.0, 6.0, 0.6, M_PI_2, "garage_wall_left"),
      Rect::from_size(1.8, -3.0, 6.0, 0.6, M_PI_2, "garage_wall_right"),
      Rect::from_size(0.0, -6.3, 5.0, 0.6, 0.0, "garage_back"),
      Rect::from_size(7.5, 3.0, 1.0, 8.0, 0.0, "fence"),
      Rect::from_size(0.0, 9.0, 40.0, 2.0, 0.0, "street_wall"),
  };
  s.goal = goal_from_body_center(s.vehicle, 0.0, -2.45, M_PI_2);
  s.xmin = -13.0; s.xmax = 10.0; s.ymin = -7.5; s.ymax = 10.5;
  return s;
}

}  // namespace

std::vector<std::string> scenario_names() {
  return {"parallel", "perpendicular", "garage"};
}

Scenario make_scenario(const std::string& name) {
  if (name == "parallel") return parallel_scenario();
  if (name == "perpendicular") return perpendicular_scenario();
  if (name == "garage") return garage_scenario();
  throw std::runtime_error("unknown scenario: " + name);
}

GoalError goal_error(const VecX& x, const VecX& goal) {
  GoalError e;
  const double dx = x(kPx) - goal(kPx), dy = x(kPy) - goal(kPy);
  e.pos = std::sqrt(dx * dx + dy * dy);
  e.yaw = std::fabs(wrap_pi(x(kTheta) - goal(kTheta)));
  e.speed = std::fabs(x(kV));
  return e;
}

bool is_parked(const Scenario& s, const VecX& x) {
  const GoalError e = goal_error(x, s.goal);
  return e.pos <= s.pos_tol && e.yaw <= s.yaw_tol && e.speed <= s.v_tol;
}

}  // namespace mpcpark
