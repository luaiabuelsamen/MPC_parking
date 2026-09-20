#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include "mpcpark/planner.hpp"
#include "mpcpark/scenario.hpp"
#include "mpcpark/simulator.hpp"
#include "mpcpark/trajectory.hpp"

int main(int argc, char** argv) {
  const std::string scenario_name = argc > 1 ? argv[1] : "parallel";
  const std::string csv_path = argc > 2 ? argv[2] : "simulation.csv";
  try {
    const auto scenario = mpcpark::make_scenario(scenario_name);
    mpcpark::PlanRequest request;
    request.vehicle = scenario.vehicle;
    request.obstacles = scenario.obstacles;
    request.start = scenario.start;
    request.goal = scenario.goal;
    request.xmin = scenario.xmin; request.xmax = scenario.xmax;
    request.ymin = scenario.ymin; request.ymax = scenario.ymax;
    const auto plan = mpcpark::hybrid_astar(request);
    if (!plan.success) throw std::runtime_error("Hybrid A* failed");
    const auto reference = mpcpark::make_reference(
        plan, scenario.start, scenario.goal, scenario.vehicle);
    const auto result = mpcpark::simulate_mpc(scenario, reference);

    std::ofstream csv(csv_path);
    if (!csv) throw std::runtime_error("cannot open output: " + csv_path);
    csv << "t,x,y,yaw,v,steer,accel,steer_rate,solve_ms,violation,collision\n";
    for (const auto& sample : result.samples) {
      csv << sample.time << ',' << sample.state(mpcpark::kPx) << ','
          << sample.state(mpcpark::kPy) << ','
          << sample.state(mpcpark::kTheta) << ','
          << sample.state(mpcpark::kV) << ','
          << sample.state(mpcpark::kDelta) << ','
          << sample.control(mpcpark::kAccel) << ','
          << sample.control(mpcpark::kSteerRate) << ',' << sample.solve_ms << ','
          << sample.constraint_violation << ',' << sample.collision << '\n';
    }
    std::cout << std::fixed << std::setprecision(3)
              << "samples=" << result.samples.size()
              << " mean_solve_ms=" << result.mean_solve_ms
              << " max_solve_ms=" << result.max_solve_ms
              << " goal_pos_m=" << result.final_error.pos
              << " goal_yaw_rad=" << result.final_error.yaw
              << " collision=" << result.collision
              << " parked=" << result.parked << " csv=" << csv_path << '\n';
    return result.parked ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << '\n';
    return 2;
  }
}
