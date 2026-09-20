#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>
#include "mpcpark/multi_agent.hpp"

struct Case {
  std::string name;
  double start_x = -14.0, speed = 1.3, width = 1.8, plant_scale = 1.0;
  int fault_start = -1, fault_steps = 0, rounds = 1;
};
void number(std::ostream& out, double v) {
  if (std::isfinite(v)) out << v; else out << "null";
}

int main(int argc, char** argv) {
  try {
    unsigned seed = 20260920;
    int random_cases = 6;
    std::string output = "artifacts/benchmark.json";
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (i + 1 == argc) throw std::invalid_argument("expected option value");
      if (arg == "--seed") seed = static_cast<unsigned>(std::stoul(argv[++i]));
      else if (arg == "--random-cases") random_cases = std::stoi(argv[++i]);
      else if (arg == "--output") output = argv[++i];
      else throw std::invalid_argument("unknown option: " + arg);
    }
    if (random_cases < 0 || random_cases > 100) throw std::invalid_argument("random cases must be 0..100");
    std::vector<Case> cases = {
      {"baseline"}, {"close_follower", -13.7, 1.1}, {"fast_follower", -16.0, 1.7},
      {"wide_follower", -15.0, 1.3, 1.95},
      {"plant_mismatch", -14.0, 1.3, 1.8, 1.03},
      {"dropped_results", -14.0, 1.3, 1.8, 1.0, 60, 3},
      {"three_exchanges", -14.0, 1.3, 1.8, 1.0, -1, 0, 3},
    };
    std::mt19937 rng(seed);
    // Raw engine output makes parameter draws reproducible across libraries.
    auto draw = [&](double lo, double hi) {
      return lo + (hi - lo) * static_cast<double>(rng()) / std::mt19937::max();
    };
    for (int i = 0; i < random_cases; ++i)
      cases.push_back({"seeded_" + std::to_string(i), draw(-16.0, -13.7),
                      draw(1.05, 1.75), draw(1.75, 1.95), draw(0.98, 1.02)});
    std::ofstream report(output);
    if (!report) throw std::runtime_error("cannot open report: " + output);
    report << std::setprecision(10) << "{\n  \"schema\": 1, \"compiler\": \"" << __VERSION__
           << "\", \"seed\": " << seed
           << ", \"dt_s\": 0.15, \"deadline_ms\": 150, \"substeps\": 10,\n  \"cases\": [\n";
    const auto base = mpcpark::make_parallel_parking_traffic_scenario();
    const auto base_refs = mpcpark::plan_agent_references(base);
    int completed = 0, collisions = 0, stopped = 0, misses = 0;
    bool all_passed = true;
    std::cout << std::fixed << std::setprecision(3);
    for (size_t index = 0; index < cases.size(); ++index) {
      const auto& test = cases[index];
      auto scenario = base;
      scenario.agents[1].start(mpcpark::kPx) = test.start_x;
      scenario.agents[1].cruise_speed = test.speed;
      scenario.agents[1].vehicle.width = test.width;
      auto follower = scenario;
      follower.agents = {scenario.agents[1]};
      auto refs = base_refs;
      refs[1] = mpcpark::plan_agent_references(follower).front();
      mpcpark::DistributedOptions options;
      options.plant_wheelbase_scale = test.plant_scale;
      options.fault_start_step = test.fault_start;
      options.fault_steps = test.fault_steps;
      options.coordination_rounds = test.rounds;
      const auto r = mpcpark::simulate_distributed_ilqr(scenario, refs, options);
      all_passed = all_passed && r.success && !r.collision && r.deadline_misses == 0 &&
                   r.injected_faults == test.fault_steps;
      completed += r.success; collisions += r.collision; stopped += r.safety_stop;
      misses += r.deadline_misses;
      std::cout << test.name << " success=" << r.success << " collision=" << r.collision
                << " safety_stop=" << r.safety_stop << " fallback=" << r.fallback_steps
                << " clearance=" << r.min_clearance << " p95_ms=" << r.p95_round_ms
                << " max_ms=" << r.max_round_ms << " misses=" << r.deadline_misses << std::endl;
      if (index) report << ",\n";
      report << "    {\"name\": \"" << test.name << "\", \"start_x\": " << test.start_x
             << ", \"speed\": " << test.speed << ", \"width\": " << test.width
             << ", \"plant_scale\": " << test.plant_scale
             << ", \"requested_rounds\": " << test.rounds
             << ", \"fault_start\": " << test.fault_start
             << ", \"fault_steps\": " << test.fault_steps
             << ", \"success\": " << (r.success ? "true" : "false")
             << ", \"collision\": " << (r.collision ? "true" : "false")
             << ", \"safety_stop\": " << (r.safety_stop ? "true" : "false")
             << ", \"fallback_steps\": " << r.fallback_steps
             << ", \"injected_faults\": " << r.injected_faults
             << ", \"safety_rejections\": " << r.safety_rejections
             << ", \"deadline_misses\": " << r.deadline_misses
             << ", \"solver_timeouts\": " << r.solver_timeouts
             << ", \"mean_ms\": " << r.mean_round_ms << ", \"p95_ms\": " << r.p95_round_ms
             << ", \"max_ms\": " << r.max_round_ms << ", \"mean_rounds\": " << r.mean_rounds
             << ", \"min_clearance_m\": "; number(report, r.min_clearance);
      report << ", \"static_clearance_m\": "; number(report, r.min_static_clearance);
      report << ", \"goal_errors\": [";
      for (size_t i = 0; i < r.final_errors.size(); ++i) {
        if (i) report << ",";
        const auto& e = r.final_errors[i];
        report << "{\"pos_m\":" << e.pos << ",\"yaw_rad\":" << e.yaw
               << ",\"speed_mps\":" << e.speed << "}";
      }
      report << "]}";
      report.flush();
    }
    report << "\n  ],\n  \"summary\": {\"cases\": " << cases.size()
           << ", \"completed\": " << completed << ", \"collisions\": " << collisions
           << ", \"safety_stops\": " << stopped << ", \"deadline_misses\": " << misses << "}\n}\n";
    if (!report) throw std::runtime_error("failed writing report");
    std::cout << "completed=" << completed << '/' << cases.size() << " report=" << output << '\n';
    return all_passed ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << '\n';
    return 2;
  }
}
