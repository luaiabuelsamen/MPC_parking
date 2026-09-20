// Conversion from a geometric Hybrid A* path into the time-indexed state and
// control reference consumed by the trajectory optimiser.
#pragma once

#include <vector>

#include "mpcpark/planner.hpp"

namespace mpcpark {

struct TrajectoryOptions {
  double dt = 0.15;             // seconds per optimiser stage
  double cruise_speed = 0.8;    // magnitude in m/s
  double cusp_pause = 1.6;      // seconds allowed for a direction change
};

struct ReferenceTrajectory {
  double dt = 0.15;
  std::vector<VecX> xs;  // state reference, including both endpoints
  std::vector<VecU> us;  // dynamically feasible warm-start controls
};

// Resample a successful plan in time. The returned controls are bounded and
// are rolled through the vehicle model to form a useful optimiser warm start.
// The state reference itself follows the geometric path and ends exactly at
// goal, at rest with straight wheels.
ReferenceTrajectory make_reference(const Plan& plan, const VecX& start,
                                   const VecX& goal,
                                   const VehicleParams& vehicle,
                                   const TrajectoryOptions& options = {});

}  // namespace mpcpark
