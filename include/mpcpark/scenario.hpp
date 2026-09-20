// Parking scenarios: the lot geometry, where the car starts, and the pose it
// has to end up in.
#pragma once

#include <string>
#include <vector>

#include "mpcpark/geometry.hpp"
#include "mpcpark/vehicle.hpp"

namespace mpcpark {

struct Scenario {
  std::string name;
  VehicleParams vehicle;
  VecX start{};
  VecX goal{};
  std::vector<Rect> obstacles;
  // Planning / drawing extent of the lot.
  double xmin = -15.0, xmax = 15.0, ymin = -8.0, ymax = 10.0;
  // What counts as parked.
  double pos_tol = 0.15;   // m
  double yaw_tol = 0.052;  // rad, 3 deg
  double v_tol = 0.05;     // m/s
};

// Known scenarios: "parallel", "perpendicular", "garage".
Scenario make_scenario(const std::string& name);
std::vector<std::string> scenario_names();

// Distance of a state from the goal pose, split into the three terms the
// tolerances are checked against.
struct GoalError {
  double pos = 0.0;
  double yaw = 0.0;
  double speed = 0.0;
};
GoalError goal_error(const VecX& x, const VecX& goal);
bool is_parked(const Scenario& s, const VecX& x);

}  // namespace mpcpark
