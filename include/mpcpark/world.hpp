#pragma once

#include "mpcpark/multi_agent.hpp"

namespace mpcpark {

struct LiveAgent {
  TrafficAgent spec;
  VecX state{};
  ReferenceTrajectory reference;
  std::vector<VecU> warm;
  std::vector<VecX> prediction;
  std::string status = "PLANNING";
  int cursor = 0;
  bool route_valid = false;
};

// Persistent, role-independent simulation. Edits preserve measured states,
// brake the fleet, and replan from rest. No agent has a parking/passing role.
class LiveWorld {
 public:
  explicit LiveWorld(double decision_budget_ms = 150.0);
  void reset(std::vector<TrafficAgent> agents, std::vector<Rect> obstacles);
  void set_goal(size_t agent, const VecX& goal);
  void add_agent(const TrafficAgent& agent);
  void set_obstacles(std::vector<Rect> obstacles);
  void step();

  const std::vector<LiveAgent>& agents() const { return agents_; }
  const std::vector<Rect>& obstacles() const { return obstacles_; }
  double time() const { return time_; }
  double decision_ms() const { return decision_ms_; }
  double planning_ms() const { return planning_ms_; }
  double clearance() const { return clearance_; }
  int revision() const { return revision_; }
  int replans() const { return replans_; }
  bool stopped() const { return safety_stop_; }
  const std::string& event() const { return event_; }
  static constexpr double dt = 0.15;
  static constexpr int horizon = 25;

 private:
  PassingScenario scene() const;
  void validate_edit(const std::vector<TrafficAgent>& agents,
                     const std::vector<Rect>& obstacles) const;
  void request_replan(const std::string& event);
  void plan();
  std::vector<LiveAgent> agents_;
  std::vector<Rect> obstacles_;
  double time_ = 0, decision_ms_ = 0, planning_ms_ = 0, clearance_ = 0;
  double decision_budget_ms_;
  int revision_ = 0, replans_ = 0, blocked_ticks_ = 0, recovery_attempts_ = 0;
  bool pending_plan_ = true, safety_stop_ = false;
  std::string event_ = "Place a car and give it a goal";
};
}  // namespace mpcpark
