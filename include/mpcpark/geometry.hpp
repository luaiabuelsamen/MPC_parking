// Obstacle geometry. Everything in a parking lot is a box, so obstacles are
// oriented rectangles and the collision constraint is the exact signed
// distance from a covering circle's centre to the box.
#pragma once

#include <array>
#include <string>
#include <utility>
#include <vector>

#include "mpcpark/linalg.hpp"
#include "mpcpark/vehicle.hpp"

namespace mpcpark {

struct Rect {
  double cx = 0.0, cy = 0.0;  // centre
  double hx = 1.0, hy = 1.0;  // half extents along the local axes
  double yaw = 0.0;           // rotation of the local frame
  std::string label;

  static Rect from_size(double cx, double cy, double len, double wid,
                        double yaw, std::string label = "") {
    return Rect{cx, cy, 0.5 * len, 0.5 * wid, yaw, std::move(label)};
  }

  std::array<Vec2, 4> corners() const;
};

// Signed distance from a point to the rectangle: positive outside, negative
// inside (depth of penetration).
double rect_sdf(const Rect& r, const Vec2& p);
// Same, plus the gradient with respect to the point. The gradient is the unit
// outward normal wherever it is defined; at the exact centre it is zero, which
// never happens for a feasible warm start.
double rect_sdf_grad(const Rect& r, const Vec2& p, Vec2& grad);

// True if the vehicle footprint at state x overlaps any obstacle. Uses the
// exact rectangle-rectangle separating axis test, not the circle
// approximation, so it is the honest collision verdict used by the tests and
// the benchmark.
bool in_collision(const VehicleParams& vp, const VecX& x,
                  const std::vector<Rect>& obstacles);
Rect vehicle_rect(const VehicleParams& vp, const VecX& x,
                  std::string label = "vehicle");
bool rects_overlap(const Rect& a, const Rect& b);

}  // namespace mpcpark
