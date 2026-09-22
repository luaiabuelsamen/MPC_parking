#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include "mpcpark/multi_agent.hpp"

using namespace mpcpark;
namespace {
void vector_json(std::ostream& out, const VecX& x) {
  out << '[';
  for (int j = 0; j < kNx; ++j) out << (j ? "," : "") << x(j);
  out << ']';
}
double number(const std::string& value) {
  size_t used = 0;
  const double result = std::stod(value, &used);
  if (used != value.size() || !std::isfinite(result)) throw std::invalid_argument("invalid numeric option");
  return result;
}
}

int main(int argc, char** argv) {
  try {
    auto scenario = make_oncoming_traffic_scenario();
    DistributedOptions options;
    options.max_steps = 700;
    std::string output = "artifacts/traffic_encounter.json";
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--help") {
        std::cout << "traffic_encounter [--output file.json] [--oncoming-x m] "
                     "[--oncoming-speed m/s] [--passing-speed m/s] [--fault-step integer]\n";
        return 0;
      }
      if (i + 1 == argc) throw std::invalid_argument("missing value for " + arg);
      const std::string value = argv[++i];
      if (arg == "--output") output = value;
      else if (arg == "--oncoming-x") scenario.agents[2].start(kPx) = number(value);
      else if (arg == "--oncoming-speed") scenario.agents[2].cruise_speed = number(value);
      else if (arg == "--passing-speed") scenario.agents[1].cruise_speed = number(value);
      else if (arg == "--fault-step") {
        const double step = number(value);
        if (step < 0 || step > 690 || std::floor(step) != step)
          throw std::invalid_argument("fault step must be an integer in [0,690]");
        options.fault_start_step = static_cast<int>(step); options.fault_steps = 3;
      } else throw std::invalid_argument("unknown option " + arg);
    }
    if (scenario.agents[2].start(kPx) < -5 || scenario.agents[2].start(kPx) > 24)
      throw std::invalid_argument("oncoming start must be within [-5,24] m");
    for (const auto& agent : scenario.agents)
      if (agent.cruise_speed < 0.6 || agent.cruise_speed > agent.vehicle.v_max)
        throw std::invalid_argument("cruise speed must be within [0.6,v_max]");
    const auto references = plan_agent_references(scenario, options.dt);
    const auto result = simulate_distributed_ilqr(scenario, references, options);
    double waiting = 0, parking_yield = 0, opposing = 0;
    int priority_violations = 0;
    bool restored = false;
    for (const auto& s : result.samples) {
      if (s.modes[0] == "YIELD_TO_PASS") parking_yield += options.dt;
      if (s.modes[1] == "WAIT_FOR_GAP") {
        waiting += options.dt;
        for (const auto& p : footprint(scenario.agents[1].vehicle, s.states[1]))
          if (p(1) > 4.8) { ++priority_violations; break; }
      }
      if (s.states[1](kPy) > 5.6) opposing += options.dt;
      restored = restored || s.modes[1] == "LANE_RESTORED";
    }
    const bool interaction_ok = waiting > 0 && parking_yield > 0 && opposing > 1 && restored && !priority_violations;
    std::ofstream out(output);
    if (!out) throw std::runtime_error("cannot write " + output);
    out << std::setprecision(7) << std::boolalpha;
    out << "{\n\"schema\":1,\"dt\":" << options.dt << ",\"deadline_ms\":" << options.deadline_ms
        << ",\"bounds\":[-27,27,-2.6,10.5],\"agents\":[";
    for (size_t i = 0; i < scenario.agents.size(); ++i) {
      const auto& a = scenario.agents[i];
      out << (i ? "," : "") << "{\"name\":\"" << a.name << "\",\"length\":" << a.vehicle.length
          << ",\"width\":" << a.vehicle.width << ",\"rear_overhang\":" << a.vehicle.rear_overhang
          << ",\"cruise_speed\":" << a.cruise_speed << ",\"goal\":";
      vector_json(out, a.goal); out << ",\"reference\":[";
      for (size_t k = 0; k < references[i].xs.size(); ++k) {
        if (k) out << ',';
        vector_json(out, references[i].xs[k]);
      }
      out << "]}";
    }
    out << "],\"obstacles\":[";
    for (size_t i = 0; i < scenario.static_obstacles.size(); ++i) {
      const auto& r = scenario.static_obstacles[i];
      out << (i ? "," : "") << '[' << r.cx << ',' << r.cy << ',' << r.hx*2 << ',' << r.hy*2 << ',' << r.yaw << ']';
    }
    out << "],\"summary\":{\"success\":" << result.success << ",\"interaction_ok\":" << interaction_ok
        << ",\"collision\":" << result.collision << ",\"safety_stop\":" << result.safety_stop
        << ",\"priority_violations\":" << priority_violations
        << ",\"waiting_s\":" << waiting << ",\"parking_yield_s\":" << parking_yield
        << ",\"opposing_lane_s\":" << opposing << ",\"lane_restored\":" << restored
        << ",\"min_clearance_m\":" << result.min_clearance
        << ",\"min_static_clearance_m\":" << result.min_static_clearance
        << ",\"p95_ms\":" << result.p95_round_ms << ",\"max_ms\":" << result.max_round_ms
        << ",\"deadline_misses\":" << result.deadline_misses
        << ",\"fallback_steps\":" << result.fallback_steps << ",\"solver_timeouts\":" << result.solver_timeouts
        << ",\"goal_errors\":[";
    for (size_t i = 0; i < result.final_errors.size(); ++i) {
      const auto& e = result.final_errors[i];
      out << (i ? "," : "") << "{\"position_m\":" << e.pos << ",\"yaw_rad\":" << e.yaw << ",\"speed_mps\":" << e.speed << '}';
    }
    out << "]},\"samples\":[\n";
    for (size_t k = 0; k < result.samples.size(); ++k) {
      const auto& s = result.samples[k];
      out << (k ? ",\n" : "") << "{\"t\":" << s.time << ",\"ms\":" << s.solve_ms
          << ",\"clearance\":" << s.clearance << ",\"fallback\":" << s.fallback << ",\"states\":[";
      for (size_t i = 0; i < s.states.size(); ++i) {
        if (i) out << ',';
        vector_json(out, s.states[i]);
      }
      out << "],\"modes\":[";
      for (size_t i = 0; i < s.modes.size(); ++i) out << (i ? "," : "") << '"' << s.modes[i] << '"';
      out << "],\"predictions\":[";
      for (size_t i = 0; i < s.predictions.size(); ++i) {
        out << (i ? "," : "") << '[';
        for (size_t j = 0; j < s.predictions[i].size(); j += 2) {
          if (j) out << ',';
          vector_json(out, s.predictions[i][j]);
        }
        out << ']';
      }
      out << "]}";
    }
    out << "\n]}\n";
    out.close();
    if (!out) throw std::runtime_error("failed to finish " + output);
    std::cout << std::fixed << std::setprecision(3)
              << "success=" << result.success << " interaction=" << interaction_ok
              << " collision=" << result.collision << " safety_stop=" << result.safety_stop
              << " waiting_s=" << waiting << " parking_yield_s=" << parking_yield
              << " opposing_lane_s=" << opposing << " clearance=" << result.min_clearance
              << " p95_ms=" << result.p95_round_ms << " max_ms=" << result.max_round_ms
              << " fallback=" << result.fallback_steps << " report=" << output << '\n';
    return result.success && interaction_ok ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << '\n'; return 2;
  }
}
