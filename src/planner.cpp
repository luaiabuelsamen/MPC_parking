#include "mpcpark/planner.hpp"

#include <chrono>
#include <cstdint>
#include <queue>
#include <unordered_map>

namespace mpcpark {

namespace {

struct Node {
  double x = 0.0, y = 0.0, yaw = 0.0;
  double g = 0.0;
  int direction = 1;
  double steer = 0.0;
  int parent = -1;
  // Poses travelled through on the way from the parent, for reconstruction.
  std::vector<PlanPose> segment;
};

struct QueueEntry {
  double f;
  int node;
  bool operator>(const QueueEntry& o) const { return f > o.f; }
};

int64_t grid_key(double x, double y, double yaw, int direction,
                 const PlanRequest& req, const PlannerOptions& opt) {
  const int ix = static_cast<int>(std::floor((x - req.xmin) / opt.xy_resolution));
  const int iy = static_cast<int>(std::floor((y - req.ymin) / opt.xy_resolution));
  double a = wrap_pi(yaw) + M_PI;
  int iyaw = static_cast<int>(a / (2.0 * M_PI) * opt.yaw_bins);
  if (iyaw >= opt.yaw_bins) iyaw = opt.yaw_bins - 1;
  const int64_t pose_key =
      (static_cast<int64_t>(ix) * 100003 + iy) * opt.yaw_bins + iyaw;
  // The same pose reached in forward and reverse is a distinct Hybrid A*
  // state: its next primitive has a different gear-switch cost, and retaining
  // both states is essential for multi-point parking manoeuvres.
  return pose_key * 2 + (direction < 0 ? 1 : 0);
}

// Obstacle-aware 2D distance-to-goal field, ignoring the car's heading. Cells
// are blocked when a disc of the car's half-width would not fit, which keeps
// the heuristic from routing the search through gaps the car cannot use.
class DistanceField {
 public:
  DistanceField(const PlanRequest& req, double res) : res_(res) {
    nx_ = std::max(2, static_cast<int>((req.xmax - req.xmin) / res) + 1);
    ny_ = std::max(2, static_cast<int>((req.ymax - req.ymin) / res) + 1);
    x0_ = req.xmin;
    y0_ = req.ymin;
    const double clearance = 0.5 * req.vehicle.width;

    blocked_.assign(nx_ * ny_, 0);
    for (int i = 0; i < nx_; ++i) {
      for (int j = 0; j < ny_; ++j) {
        Vec2 p{};
        p(0) = x0_ + i * res_;
        p(1) = y0_ + j * res_;
        for (const Rect& o : req.obstacles) {
          if (rect_sdf(o, p) < clearance) {
            blocked_[i * ny_ + j] = 1;
            break;
          }
        }
      }
    }
    dijkstra(req.goal(kPx), req.goal(kPy));
  }

  double at(double x, double y) const {
    const int i = static_cast<int>(std::lround((x - x0_) / res_));
    const int j = static_cast<int>(std::lround((y - y0_) / res_));
    if (i < 0 || j < 0 || i >= nx_ || j >= ny_) return -1.0;
    const double d = dist_[i * ny_ + j];
    return std::isfinite(d) ? d : -1.0;
  }

 private:
  void dijkstra(double gx, double gy) {
    dist_.assign(nx_ * ny_, std::numeric_limits<double>::infinity());
    const int gi = static_cast<int>(std::lround((gx - x0_) / res_));
    const int gj = static_cast<int>(std::lround((gy - y0_) / res_));
    if (gi < 0 || gj < 0 || gi >= nx_ || gj >= ny_) return;

    using QE = std::pair<double, int>;
    std::priority_queue<QE, std::vector<QE>, std::greater<QE>> q;
    dist_[gi * ny_ + gj] = 0.0;
    q.push({0.0, gi * ny_ + gj});
    const int di[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int dj[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    while (!q.empty()) {
      auto [d, idx] = q.top();
      q.pop();
      if (d > dist_[idx]) continue;
      const int i = idx / ny_, j = idx % ny_;
      for (int k = 0; k < 8; ++k) {
        const int ni = i + di[k], nj = j + dj[k];
        if (ni < 0 || nj < 0 || ni >= nx_ || nj >= ny_) continue;
        if (blocked_[ni * ny_ + nj]) continue;
        const double step = (k < 4 ? 1.0 : M_SQRT2) * res_;
        const double nd = d + step;
        if (nd < dist_[ni * ny_ + nj]) {
          dist_[ni * ny_ + nj] = nd;
          q.push({nd, ni * ny_ + nj});
        }
      }
    }
  }

  double res_, x0_, y0_;
  int nx_ = 0, ny_ = 0;
  std::vector<char> blocked_;
  std::vector<double> dist_;
};

}  // namespace

bool pose_is_free(const PlanRequest& req, double x, double y, double yaw) {
  if (x < req.xmin || x > req.xmax || y < req.ymin || y > req.ymax) return false;
  VecX s{};
  s(kPx) = x;
  s(kPy) = y;
  s(kTheta) = yaw;
  return !in_collision(req.vehicle, s, req.obstacles);
}

namespace {

// Integrate a constant-steering arc, recording intermediate poses.
bool propagate(const PlanRequest& req, const PlannerOptions& opt, double x,
               double y, double yaw, double steer, int dir,
               std::vector<PlanPose>& out) {
  const double ds = opt.arc_length / opt.substeps * dir;
  const double kappa = std::tan(steer) / req.vehicle.wheelbase;
  out.clear();
  for (int i = 0; i < opt.substeps; ++i) {
    // Exact unicycle integration over the sub-arc.
    if (std::fabs(kappa) < 1e-9) {
      x += ds * std::cos(yaw);
      y += ds * std::sin(yaw);
    } else {
      const double yaw_next = yaw + ds * kappa;
      x += (std::sin(yaw_next) - std::sin(yaw)) / kappa;
      y += -(std::cos(yaw_next) - std::cos(yaw)) / kappa;
      yaw = wrap_pi(yaw_next);
    }
    if (!pose_is_free(req, x, y, yaw)) return false;
    out.push_back(PlanPose{x, y, wrap_pi(yaw), dir, steer});
  }
  return true;
}

// Straight shortcut to the goal, used in place of a Reeds-Shepp expansion.
bool straight_shortcut(const PlanRequest& req, const PlannerOptions& opt,
                       const Node& n, std::vector<PlanPose>& out) {
  const double dx = req.goal(kPx) - n.x, dy = req.goal(kPy) - n.y;
  const double dist = std::sqrt(dx * dx + dy * dy);
  if (dist > opt.shortcut_max_len) return false;
  if (std::fabs(wrap_pi(n.yaw - req.goal(kTheta))) > 0.06) return false;
  if (dist < 1e-6) return true;
  // The offset has to lie along the heading, otherwise a straight line will
  // not arrive at the goal pose.
  const double along = dx * std::cos(n.yaw) + dy * std::sin(n.yaw);
  const double lateral = -dx * std::sin(n.yaw) + dy * std::cos(n.yaw);
  if (std::fabs(lateral) > 0.08) return false;

  const int dir = along >= 0.0 ? 1 : -1;
  const int steps = std::max(1, static_cast<int>(dist / 0.15));
  out.clear();
  for (int i = 1; i <= steps; ++i) {
    const double t = static_cast<double>(i) / steps;
    const double x = n.x + t * along * std::cos(n.yaw);
    const double y = n.y + t * along * std::sin(n.yaw);
    if (!pose_is_free(req, x, y, n.yaw)) return false;
    out.push_back(PlanPose{x, y, n.yaw, dir, 0.0});
  }
  return true;
}

}  // namespace

Plan hybrid_astar(const PlanRequest& req, const PlannerOptions& opt) {
  using clock = std::chrono::steady_clock;
  const auto t_start = clock::now();
  Plan plan;

  if (!pose_is_free(req, req.start(kPx), req.start(kPy), req.start(kTheta))) {
    plan.plan_ms =
        std::chrono::duration<double, std::milli>(clock::now() - t_start).count();
    return plan;  // start is already in collision
  }

  const DistanceField field(req, opt.xy_resolution);
  const double h_start = field.at(req.start(kPx), req.start(kPy));
  auto heuristic = [&](double x, double y) {
    const double d = field.at(x, y);
    if (d >= 0.0) return d;
    const double dx = x - req.goal(kPx), dy = y - req.goal(kPy);
    return std::sqrt(dx * dx + dy * dy);
  };
  (void)h_start;

  std::vector<Node> nodes;
  nodes.reserve(1 << 14);
  std::unordered_map<int64_t, double> best_g;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open;

  Node root;
  root.x = req.start(kPx);
  root.y = req.start(kPy);
  root.yaw = req.start(kTheta);
  root.direction = 1;
  nodes.push_back(root);
  best_g[grid_key(root.x, root.y, root.yaw, root.direction, req, opt)] = 0.0;
  open.push({opt.heuristic_weight * heuristic(root.x, root.y), 0});

  // Steering samples: straight plus `steer_levels` per side.
  std::vector<double> steers;
  steers.push_back(0.0);
  for (int i = 1; i <= opt.steer_levels; ++i) {
    const double s = req.vehicle.delta_max * static_cast<double>(i) /
                     opt.steer_levels;
    steers.push_back(s);
    steers.push_back(-s);
  }

  int goal_node = -1;
  std::vector<PlanPose> shortcut, segment;
  while (!open.empty() && plan.expansions < opt.max_expansions) {
    const QueueEntry top = open.top();
    open.pop();
    const int ni = top.node;
    const double key_g = best_g[grid_key(nodes[ni].x, nodes[ni].y,
                                         nodes[ni].yaw, nodes[ni].direction,
                                         req, opt)];
    if (nodes[ni].g > key_g + 1e-9) continue;
    ++plan.expansions;

    // Expanding a node appends children to `nodes`; keep a value copy so a
    // vector reallocation cannot invalidate the current node mid-expansion.
    const Node cur = nodes[ni];
    const double dx = cur.x - req.goal(kPx), dy = cur.y - req.goal(kPy);
    const double dpos = std::sqrt(dx * dx + dy * dy);
    const double dyaw = std::fabs(wrap_pi(cur.yaw - req.goal(kTheta)));
    if (straight_shortcut(req, opt, cur, shortcut)) {
      Node n;
      n.x = req.goal(kPx);
      n.y = req.goal(kPy);
      n.yaw = req.goal(kTheta);
      n.g = cur.g + dpos;
      n.parent = ni;
      n.direction = shortcut.empty() ? cur.direction : shortcut.back().direction;
      n.segment = shortcut;
      nodes.push_back(n);
      goal_node = static_cast<int>(nodes.size()) - 1;
      break;
    }
    if (dpos <= opt.goal_pos_tol && dyaw <= opt.goal_yaw_tol) {
      goal_node = ni;
      break;
    }

    for (double steer : steers) {
      for (int dir : {1, -1}) {
        if (!propagate(req, opt, cur.x, cur.y, cur.yaw, steer, dir, segment)) {
          continue;
        }
        const PlanPose& end = segment.back();
        double cost = opt.arc_length * (dir > 0 ? 1.0 : opt.reverse_cost);
        if (dir != cur.direction) cost += opt.switch_cost;
        cost += opt.steer_cost * std::fabs(steer);
        cost += opt.steer_change_cost * std::fabs(steer - cur.steer);
        const double g = cur.g + cost;

        const int64_t key = grid_key(end.x, end.y, end.yaw, dir, req, opt);
        auto it = best_g.find(key);
        if (it != best_g.end() && it->second <= g - 1e-9) continue;
        best_g[key] = g;

        Node n;
        n.x = end.x;
        n.y = end.y;
        n.yaw = end.yaw;
        n.g = g;
        n.direction = dir;
        n.steer = steer;
        n.parent = ni;
        n.segment = segment;
        nodes.push_back(n);
        open.push({g + opt.heuristic_weight * heuristic(end.x, end.y),
                   static_cast<int>(nodes.size()) - 1});
      }
    }
  }

  if (goal_node >= 0) {
    std::vector<std::vector<PlanPose>> chunks;
    for (int i = goal_node; i >= 0; i = nodes[i].parent) {
      if (!nodes[i].segment.empty()) chunks.push_back(nodes[i].segment);
      if (nodes[i].parent < 0) {
        chunks.push_back({PlanPose{nodes[i].x, nodes[i].y, nodes[i].yaw,
                                   nodes[i].direction, 0.0}});
      }
    }
    for (auto it = chunks.rbegin(); it != chunks.rend(); ++it) {
      plan.poses.insert(plan.poses.end(), it->begin(), it->end());
    }
    for (size_t i = 1; i < plan.poses.size(); ++i) {
      const double sx = plan.poses[i].x - plan.poses[i - 1].x;
      const double sy = plan.poses[i].y - plan.poses[i - 1].y;
      plan.length += std::sqrt(sx * sx + sy * sy);
    }
    plan.success = true;
  }
  plan.plan_ms =
      std::chrono::duration<double, std::milli>(clock::now() - t_start).count();
  return plan;
}

}  // namespace mpcpark
