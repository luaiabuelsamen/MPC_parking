#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include "mpcpark/ilqr.hpp"
#include "mpcpark/scenario.hpp"
#include "mpcpark/trajectory.hpp"

namespace {

void usage(const char* exe) {
  std::cerr << "Usage: " << exe
            << " [parallel|perpendicular|garage] [--plan-only] [--csv FILE]\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string scenario_name = "parallel";
  std::string csv_path;
  bool plan_only = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--plan-only") {
      plan_only = true;
    } else if (arg == "--csv" && i + 1 < argc) {
      csv_path = argv[++i];
    } else if (!arg.empty() && arg[0] != '-') {
      scenario_name = arg;
    } else {
      usage(argv[0]);
      return 2;
    }
  }

  try {
    const mpcpark::Scenario scenario = mpcpark::make_scenario(scenario_name);
    mpcpark::PlanRequest request;
    request.vehicle = scenario.vehicle;
    request.obstacles = scenario.obstacles;
    request.start = scenario.start;
    request.goal = scenario.goal;
    request.xmin = scenario.xmin;
    request.xmax = scenario.xmax;
    request.ymin = scenario.ymin;
    request.ymax = scenario.ymax;

    const mpcpark::Plan plan = mpcpark::hybrid_astar(request);
    std::cout << std::fixed << std::setprecision(3)
              << "scenario=" << scenario.name << " plan_success=" << plan.success
              << " expansions=" << plan.expansions << " length_m=" << plan.length
              << " plan_ms=" << plan.plan_ms << '\n';
    if (!plan.success) return 1;
    if (plan_only) return 0;

    const mpcpark::ReferenceTrajectory reference = mpcpark::make_reference(
        plan, scenario.start, scenario.goal, scenario.vehicle);
    mpcpark::Problem problem;
    problem.vehicle = scenario.vehicle;
    problem.obstacles = scenario.obstacles;
    problem.dt = reference.dt;
    problem.horizon = static_cast<int>(reference.us.size());
    problem.xref = reference.xs;
    mpcpark::ParkingSolver solver(std::move(problem));
    const mpcpark::Solution solution = solver.solve(scenario.start, reference.us);
    const mpcpark::GoalError error = mpcpark::goal_error(solution.xs.back(), scenario.goal);

    bool collision = false;
    for (const auto& x : solution.xs) {
      collision = collision || mpcpark::in_collision(scenario.vehicle, x,
                                                      scenario.obstacles);
    }
    std::cout << "horizon=" << problem.horizon
              << " solve_ms=" << solution.stats.solve_ms
              << " cost=" << solution.stats.cost
              << " max_violation=" << solution.stats.max_violation
              << " goal_pos_m=" << error.pos << " goal_yaw_rad=" << error.yaw
              << " goal_speed_mps=" << error.speed
              << " collision=" << collision
              << " parked=" << mpcpark::is_parked(scenario, solution.xs.back()) << '\n';

    if (!csv_path.empty()) {
      std::ofstream csv(csv_path);
      if (!csv) throw std::runtime_error("cannot open CSV output: " + csv_path);
      csv << "t,x,y,yaw,v,steer,accel,steer_rate\n";
      for (size_t k = 0; k < solution.xs.size(); ++k) {
        const auto& x = solution.xs[k];
        csv << k * problem.dt << ',' << x(mpcpark::kPx) << ',' << x(mpcpark::kPy)
            << ',' << x(mpcpark::kTheta) << ',' << x(mpcpark::kV) << ','
            << x(mpcpark::kDelta) << ',';
        if (k < solution.us.size()) {
          csv << solution.us[k](mpcpark::kAccel) << ','
              << solution.us[k](mpcpark::kSteerRate);
        } else {
          csv << "0,0";
        }
        csv << '\n';
      }
    }
    return collision ? 1 : 0;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << '\n';
    usage(argv[0]);
    return 2;
  }
}
