#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "mpcpark/multi_agent.hpp"

namespace {
struct Case {
  std::string name;
  double start_x;
  double speed;
};
}  // namespace

int main() {
  const std::vector<Case> cases = {
      {"baseline", -14.0, 1.3},
      {"close_follower", -13.7, 1.1},
      {"fast_follower", -16.0, 1.7},
  };
  try {
    auto base = mpcpark::make_parallel_parking_traffic_scenario();
    const auto base_references = mpcpark::plan_agent_references(base);
    bool passed = true;
    std::cout << std::fixed << std::setprecision(3);
    for (const auto& test : cases) {
      auto scenario = base;
      scenario.agents[1].start(mpcpark::kPx) = test.start_x;
      scenario.agents[1].cruise_speed = test.speed;

      std::vector<mpcpark::ReferenceTrajectory> references;
      references.push_back(base_references[0]);
      if (test.name == "baseline") {
        references.push_back(base_references[1]);
      } else {
        auto follower_only = scenario;
        follower_only.agents = {scenario.agents[1]};
        references.push_back(
            mpcpark::plan_agent_references(follower_only).front());
      }
      const auto result =
          mpcpark::simulate_distributed_ilqr(scenario, references);
      passed = passed && result.success && result.deadline_misses == 0;
      std::cout << test.name << " success=" << result.success
                << " collision=" << result.collision
                << " min_clearance_m=" << result.min_clearance
                << " mean_ms=" << result.mean_round_ms
                << " max_ms=" << result.max_round_ms
                << " deadline_misses=" << result.deadline_misses << '\n';
    }
    return passed ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << '\n';
    return 2;
  }
}
