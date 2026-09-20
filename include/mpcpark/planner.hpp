// Hybrid A* over the car's kinematically feasible motion primitives.
//
// The plan is a warm start, not the final answer: it gets the MPC into the
// right homotopy class (drive past the slot, then reverse in) which a local
// optimiser will not discover on its own. Final pose accuracy is left to the
// MPC, so the search stops at a loose goal tolerance and there is no
// Reeds-Shepp analytic expansion -- just a straight-segment shortcut, which is
// what the last leg into a slot looks like anyway.
#pragma once

#include <vector>

#include "mpcpark/geometry.hpp"
#include "mpcpark/vehicle.hpp"

namespace mpcpark {

struct PlannerOptions {
  double xy_resolution = 0.3;     // m, grid cell for the closed set
  int yaw_bins = 72;              // 5 deg per bin
  double arc_length = 0.7;        // m travelled per primitive
  int substeps = 4;               // collision checks per primitive
  int steer_levels = 2;           // steering samples per side (plus straight)
  double reverse_cost = 1.6;      // multiplier on reverse motion
  double switch_cost = 4.0;       // penalty for changing direction
  double steer_cost = 0.6;        // penalty on |steer|
  double steer_change_cost = 0.6; // penalty on changing steer
  double heuristic_weight = 1.4;  // weighted A*
  double goal_pos_tol = 0.15;     // m
  double goal_yaw_tol = 0.052;    // rad, 3 deg
  double shortcut_max_len = 8.0;  // m, longest straight shortcut attempted
  int max_expansions = 400000;
};

struct PlanPose {
  double x = 0.0, y = 0.0, yaw = 0.0;
  int direction = 1;  // +1 forward, -1 reverse
  double steer = 0.0;
};

struct Plan {
  bool success = false;
  std::vector<PlanPose> poses;  // dense, spaced by arc_length / substeps
  int expansions = 0;
  double length = 0.0;
  double plan_ms = 0.0;
};

struct PlanRequest {
  VehicleParams vehicle;
  std::vector<Rect> obstacles;
  VecX start{};
  VecX goal{};
  double xmin = -15.0, xmax = 15.0, ymin = -10.0, ymax = 10.0;
};

Plan hybrid_astar(const PlanRequest& req,
                  const PlannerOptions& opt = PlannerOptions{});

// True if the footprint at (x, y, yaw) is collision free and inside the lot.
bool pose_is_free(const PlanRequest& req, double x, double y, double yaw);

}  // namespace mpcpark
