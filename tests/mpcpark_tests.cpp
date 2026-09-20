#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#include "mpcpark/ilqr.hpp"
#include "mpcpark/scenario.hpp"
#include "mpcpark/trajectory.hpp"

namespace {

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

void test_rk4_jacobian() {
  mpcpark::VehicleParams p;
  mpcpark::VecX x{};
  x(mpcpark::kPx) = 1.2; x(mpcpark::kPy) = -0.4;
  x(mpcpark::kTheta) = 0.7; x(mpcpark::kV) = -0.8;
  x(mpcpark::kDelta) = 0.2;
  mpcpark::VecU u{};
  u(mpcpark::kAccel) = 0.3; u(mpcpark::kSteerRate) = -0.15;
  mpcpark::VecX next;
  mpcpark::MatXX A;
  mpcpark::MatXU B;
  mpcpark::step_rk4_jacobian(p, x, u, 0.1, next, A, B);
  const double eps = 1e-6;
  for (int j = 0; j < mpcpark::kNx; ++j) {
    auto xp = x; auto xm = x;
    xp(j) += eps; xm(j) -= eps;
    const auto fd = (0.5 / eps) * (mpcpark::step_rk4(p, xp, u, 0.1) -
                                    mpcpark::step_rk4(p, xm, u, 0.1));
    for (int i = 0; i < mpcpark::kNx; ++i)
      require(std::fabs(fd(i) - A(i, j)) < 2e-6, "RK4 state Jacobian mismatch");
  }
  for (int j = 0; j < mpcpark::kNu; ++j) {
    auto up = u; auto um = u;
    up(j) += eps; um(j) -= eps;
    const auto fd = (0.5 / eps) * (mpcpark::step_rk4(p, x, up, 0.1) -
                                    mpcpark::step_rk4(p, x, um, 0.1));
    for (int i = 0; i < mpcpark::kNx; ++i)
      require(std::fabs(fd(i) - B(i, j)) < 2e-6, "RK4 control Jacobian mismatch");
  }
}

void test_geometry() {
  const auto box = mpcpark::Rect::from_size(0.0, 0.0, 4.0, 2.0, 0.0);
  mpcpark::Vec2 p{};
  p(0) = 3.0;
  require(std::fabs(mpcpark::rect_sdf(box, p) - 1.0) < 1e-12,
          "rectangle outside distance");
  p(0) = 0.0;
  require(std::fabs(mpcpark::rect_sdf(box, p) + 1.0) < 1e-12,
          "rectangle inside distance");
  const auto other = mpcpark::Rect::from_size(5.0, 0.0, 4.0, 2.0, 0.0);
  require(std::fabs(mpcpark::rect_distance(box, other) - 1.0) < 1e-12,
          "rectangle separation distance");
}

void test_dynamic_obstacle_constraints() {
  mpcpark::Problem problem;
  problem.horizon = 1;
  problem.xref.resize(2);
  problem.dynamic_obstacles.resize(2);
  problem.dynamic_obstacles[0].push_back(
      mpcpark::Rect::from_size(1.25, 0.0, 4.5, 1.8, 0.0));
  problem.dynamic_obstacles[1] = problem.dynamic_obstacles[0];
  mpcpark::ParkingSolver solver(problem);
  require(solver.num_constraints() == problem.vehicle.n_discs + 4,
          "dynamic obstacle constraint count");
  std::vector<double> constraints;
  std::vector<mpcpark::VecX> gradients;
  solver.constraints_at(0, mpcpark::VecX{}, constraints, gradients);
  bool violated = false;
  for (int i = 0; i < problem.vehicle.n_discs; ++i)
    violated = violated || constraints[i] > 0.0;
  require(violated, "overlapping moving vehicle was not constrained");
}

void test_planner_scenarios() {
  for (const auto& name : mpcpark::scenario_names()) {
    const auto s = mpcpark::make_scenario(name);
    mpcpark::PlanRequest req;
    req.vehicle = s.vehicle; req.obstacles = s.obstacles;
    req.start = s.start; req.goal = s.goal;
    req.xmin = s.xmin; req.xmax = s.xmax; req.ymin = s.ymin; req.ymax = s.ymax;
    const auto plan = mpcpark::hybrid_astar(req);
    require(plan.success, "planner failed for " + name);
    require(plan.poses.size() > 2, "planner returned a trivial path for " + name);
    for (const auto& pose : plan.poses)
      require(mpcpark::pose_is_free(req, pose.x, pose.y, pose.yaw),
              "planner path collides for " + name);
    const auto ref = mpcpark::make_reference(plan, s.start, s.goal, s.vehicle);
    require(ref.xs.size() == ref.us.size() + 1, "reference dimensions");
    require(mpcpark::goal_error(ref.xs.back(), s.goal).pos < 1e-12,
            "reference does not end at goal");
  }
}

}  // namespace

int main() {
  try {
    test_rk4_jacobian();
    test_geometry();
    test_dynamic_obstacle_constraints();
    test_planner_scenarios();
    std::cout << "all tests passed\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "test failure: " << e.what() << '\n';
    return 1;
  }
}
