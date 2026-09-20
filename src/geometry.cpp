#include "mpcpark/geometry.hpp"

#include <limits>

namespace mpcpark {

std::array<Vec2, 4> Rect::corners() const {
  const double c = std::cos(yaw), s = std::sin(yaw);
  const double local[4][2] = {{-hx, -hy}, {hx, -hy}, {hx, hy}, {-hx, hy}};
  std::array<Vec2, 4> out{};
  for (int i = 0; i < 4; ++i) {
    out[i](0) = cx + c * local[i][0] - s * local[i][1];
    out[i](1) = cy + s * local[i][0] + c * local[i][1];
  }
  return out;
}

namespace {
// Point expressed in the rectangle's local frame.
Vec2 to_local(const Rect& r, const Vec2& p) {
  const double c = std::cos(r.yaw), s = std::sin(r.yaw);
  const double dx = p(0) - r.cx, dy = p(1) - r.cy;
  Vec2 q{};
  q(0) = c * dx + s * dy;
  q(1) = -s * dx + c * dy;
  return q;
}
}  // namespace

double rect_sdf(const Rect& r, const Vec2& p) {
  const Vec2 q = to_local(r, p);
  const double dx = std::fabs(q(0)) - r.hx;
  const double dy = std::fabs(q(1)) - r.hy;
  const double ox = std::max(dx, 0.0), oy = std::max(dy, 0.0);
  return std::sqrt(ox * ox + oy * oy) + std::min(std::max(dx, dy), 0.0);
}

double rect_sdf_grad(const Rect& r, const Vec2& p, Vec2& grad) {
  const Vec2 q = to_local(r, p);
  const double sx = q(0) >= 0.0 ? 1.0 : -1.0;
  const double sy = q(1) >= 0.0 ? 1.0 : -1.0;
  const double dx = std::fabs(q(0)) - r.hx;
  const double dy = std::fabs(q(1)) - r.hy;

  Vec2 gq{};
  double sd;
  if (dx > 0.0 || dy > 0.0) {
    const double ox = std::max(dx, 0.0), oy = std::max(dy, 0.0);
    sd = std::sqrt(ox * ox + oy * oy);
    const double inv = 1.0 / std::max(sd, 1e-9);
    gq(0) = sx * ox * inv;
    gq(1) = sy * oy * inv;
  } else {
    sd = std::max(dx, dy);
    if (dx > dy) {
      gq(0) = sx;
    } else {
      gq(1) = sy;
    }
  }
  // Rotate the local gradient back into the world frame.
  const double c = std::cos(r.yaw), s = std::sin(r.yaw);
  grad(0) = c * gq(0) - s * gq(1);
  grad(1) = s * gq(0) + c * gq(1);
  return sd;
}

namespace {
void project(const std::array<Vec2, 4>& pts, const Vec2& axis, double& lo,
             double& hi) {
  lo = hi = pts[0](0) * axis(0) + pts[0](1) * axis(1);
  for (int i = 1; i < 4; ++i) {
    const double v = pts[i](0) * axis(0) + pts[i](1) * axis(1);
    lo = std::min(lo, v);
    hi = std::max(hi, v);
  }
}
}  // namespace

bool rects_overlap(const Rect& a, const Rect& b) {
  const auto ca = a.corners(), cb = b.corners();
  const double yaws[2] = {a.yaw, b.yaw};
  for (double yaw : yaws) {
    for (int k = 0; k < 2; ++k) {
      Vec2 axis{};
      axis(0) = k == 0 ? std::cos(yaw) : -std::sin(yaw);
      axis(1) = k == 0 ? std::sin(yaw) : std::cos(yaw);
      double alo, ahi, blo, bhi;
      project(ca, axis, alo, ahi);
      project(cb, axis, blo, bhi);
      if (ahi < blo || bhi < alo) return false;  // separating axis found
    }
  }
  return true;
}

namespace {
double point_segment_distance(const Vec2& p, const Vec2& a, const Vec2& b) {
  const double vx = b(0) - a(0), vy = b(1) - a(1);
  const double wx = p(0) - a(0), wy = p(1) - a(1);
  const double vv = vx * vx + vy * vy;
  const double t = vv > 1e-12 ? clampd((wx * vx + wy * vy) / vv, 0.0, 1.0)
                              : 0.0;
  return std::hypot(p(0) - (a(0) + t * vx),
                    p(1) - (a(1) + t * vy));
}
}  // namespace

double rect_distance(const Rect& a, const Rect& b) {
  if (rects_overlap(a, b)) return 0.0;
  const auto ca = a.corners(), cb = b.corners();
  double distance = std::numeric_limits<double>::infinity();
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      distance = std::min(distance,
                          point_segment_distance(ca[i], cb[j], cb[(j + 1) % 4]));
      distance = std::min(distance,
                          point_segment_distance(cb[i], ca[j], ca[(j + 1) % 4]));
    }
  }
  return distance;
}

bool in_collision(const VehicleParams& vp, const VecX& x,
                  const std::vector<Rect>& obstacles) {
  const Rect car = vehicle_rect(vp, x, "ego");
  for (const Rect& o : obstacles) {
    if (rects_overlap(car, o)) return true;
  }
  return false;
}

Rect vehicle_rect(const VehicleParams& vp, const VecX& x, std::string label) {
  return Rect::from_size(
      x(kPx) + (0.5 * vp.length - vp.rear_overhang) * std::cos(x(kTheta)),
      x(kPy) + (0.5 * vp.length - vp.rear_overhang) * std::sin(x(kTheta)),
      vp.length, vp.width, x(kTheta), std::move(label));
}

}  // namespace mpcpark
