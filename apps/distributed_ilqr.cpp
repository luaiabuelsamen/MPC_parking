#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include "mpcpark/multi_agent.hpp"

int main(int argc, char** argv) {
  const std::string csv_path = argc > 1 ? argv[1] : "distributed_ilqr.csv";
  try {
    const auto scenario = mpcpark::make_parallel_parking_traffic_scenario();
    const auto references = mpcpark::plan_agent_references(scenario);
    const auto result =
        mpcpark::simulate_distributed_ilqr(scenario, references);

    std::ofstream csv(csv_path);
    if (!csv) throw std::runtime_error("cannot open output: " + csv_path);
    csv << "t,parking_x,parking_y,parking_yaw,parking_v,parking_steer,"
           "parking_accel,parking_steer_rate,passing_x,passing_y,"
           "passing_yaw,passing_v,passing_steer,passing_accel,"
           "passing_steer_rate,solve_ms,clearance,deadline_miss,collision,fallback,rounds\n";
    for (const auto& sample : result.samples) {
      csv << sample.time;
      for (size_t i = 0; i < sample.states.size(); ++i) {
        const auto& x = sample.states[i];
        const auto& u = sample.controls[i];
        csv << ',' << x(mpcpark::kPx) << ',' << x(mpcpark::kPy) << ','
            << x(mpcpark::kTheta) << ',' << x(mpcpark::kV) << ','
            << x(mpcpark::kDelta) << ',' << u(mpcpark::kAccel) << ','
            << u(mpcpark::kSteerRate);
      }
      csv << ',' << sample.solve_ms << ',' << sample.clearance << ','
          << sample.deadline_miss << ',' << sample.collision << ','
          << sample.fallback << ',' << sample.rounds << '\n';
    }

    std::cout << std::fixed << std::setprecision(3)
              << "samples=" << result.samples.size()
              << " mean_coordination_ms=" << result.mean_round_ms
              << " max_coordination_ms=" << result.max_round_ms
              << " p95_ms=" << result.p95_round_ms
              << " fallback_steps=" << result.fallback_steps
              << " safety_stop=" << result.safety_stop
              << " static_clearance_m=" << result.min_static_clearance
              << " deadline_misses=" << result.deadline_misses
              << " solver_timeouts=" << result.solver_timeouts
              << " min_clearance_m=" << result.min_clearance
              << " collision=" << result.collision
              << " success=" << result.success;
    for (size_t i = 0; i < result.final_errors.size(); ++i) {
      std::cout << ' ' << scenario.agents[i].name << "_pos_m="
                << result.final_errors[i].pos << ' ' << scenario.agents[i].name
                << "_yaw_rad=" << result.final_errors[i].yaw;
    }
    std::cout << " csv=" << csv_path << '\n';
    return result.success ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << '\n';
    return 2;
  }
}
