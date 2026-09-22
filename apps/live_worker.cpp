#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include "mpcpark/world.hpp"

using namespace mpcpark;
namespace {
void quoted(std::ostream& out, const std::string& text) {
  out << '"';
  for (char c : text) {
    if (c == '"' || c == '\\') out << '\\' << c;
    else if (c == '\n') out << "\\n";
    else if (static_cast<unsigned char>(c) >= 32) out << c;
  }
  out << '"';
}
double value(std::istream& in) {
  double x;
  if (!(in >> x) || !std::isfinite(x)) throw std::invalid_argument("Expected finite numeric value");
  return x;
}
int count(std::istream& in, int maximum) {
  const double n = value(in);
  if (n < 0 || n > maximum || std::floor(n) != n) throw std::invalid_argument("Invalid item count or car index");
  return static_cast<int>(n);
}
VecX pose(std::istream& in) {
  VecX x; x(kPx) = value(in); x(kPy) = value(in); x(kTheta) = wrap_pi(value(in)); return x;
}
TrafficAgent agent(std::istream& in) {
  TrafficAgent a; a.start = pose(in); a.goal = pose(in); a.cruise_speed = value(in); return a;
}
std::vector<Rect> obstacles(std::istream& in, int n) {
  std::vector<Rect> result;
  for (int i = 0; i < n; ++i) {
    const double x = value(in), y = value(in), l = value(in), w = value(in), yaw = value(in);
    result.push_back(Rect::from_size(x, y, l, w, yaw));
  }
  return result;
}
void end(std::istream& in) {
  std::string extra;
  if (in >> extra) throw std::invalid_argument("Unexpected command arguments");
}
void state(std::ostream& out, const VecX& x) {
  out << '[';
  for (int j = 0; j < kNx; ++j) out << (j ? "," : "") << x(j);
  out << ']';
}
void reply(const LiveWorld& world, const std::string& error) {
  std::cout << std::setprecision(7) << "{\"ok\":" << (error.empty() ? "true" : "false") << ",\"error\":";
  quoted(std::cout, error);
  std::cout << ",\"time\":" << world.time() << ",\"revision\":" << world.revision()
            << ",\"decision_ms\":" << world.decision_ms() << ",\"planning_ms\":" << world.planning_ms()
            << ",\"clearance\":" << world.clearance() << ",\"replans\":" << world.replans()
            << ",\"stopped\":" << (world.stopped() ? "true" : "false") << ",\"event\":";
  quoted(std::cout, world.event());
  std::cout << ",\"agents\":[";
  for (size_t i = 0; i < world.agents().size(); ++i) {
    const auto& a = world.agents()[i];
    std::cout << (i ? "," : "") << "{\"id\":" << i << ",\"status\":";
    quoted(std::cout, a.status); std::cout << ",\"state\":"; state(std::cout, a.state);
    std::cout << ",\"goal\":"; state(std::cout, a.spec.goal);
    std::cout << ",\"speed\":" << a.spec.cruise_speed << ",\"prediction\":[";
    for (size_t k = 0; k < a.prediction.size(); k += 2) {
      if (k) std::cout << ',';
      state(std::cout, a.prediction[k]);
    }
    std::cout << "],\"path\":[";
    for (size_t k = 0; k < a.reference.xs.size(); k += 3) {
      if (k) std::cout << ',';
      std::cout << '[' << a.reference.xs[k](kPx) << ',' << a.reference.xs[k](kPy) << ']';
    }
    std::cout << "]}";
  }
  std::cout << "],\"obstacles\":[";
  for (size_t i = 0; i < world.obstacles().size(); ++i) {
    const auto& r = world.obstacles()[i];
    std::cout << (i ? "," : "") << '[' << r.cx << ',' << r.cy << ',' << 2*r.hx << ',' << 2*r.hy << ',' << r.yaw << ']';
  }
  std::cout << "]}\n" << std::flush;
}
}

int main() {
  LiveWorld world;
  std::string line;
  while (std::getline(std::cin, line)) {
    std::string error;
    try {
      if (line.size() > 100000) throw std::invalid_argument("Command too large");
      std::istringstream in(line);
      std::string command; in >> command;
      if (command == "LOAD") {
        const int n = count(in, 8), m = count(in, 40);
        std::vector<TrafficAgent> agents;
        for (int i = 0; i < n; ++i) agents.push_back(agent(in));
        auto obs = obstacles(in, m); end(in);
        world.reset(std::move(agents), std::move(obs));
      } else if (command == "GOAL") {
        const int id = count(in, 7); const auto goal = pose(in); end(in); world.set_goal(id, goal);
      } else if (command == "ADD") {
        const auto a = agent(in); end(in); world.add_agent(a);
      } else if (command == "OBSTACLES") {
        const int n = count(in, 40); auto obs = obstacles(in, n); end(in);
        world.set_obstacles(std::move(obs));
      } else if (command == "STEP") { end(in); world.step(); }
      else if (command == "GET") end(in);
      else throw std::invalid_argument("Unknown command");
    } catch (const std::exception& e) { error = e.what(); }
    reply(world, error);
  }
}
