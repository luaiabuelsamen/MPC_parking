#include <iostream>
#include <stdexcept>

#include "mpcpark/multi_agent.hpp"

using namespace mpcpark;
namespace {
void require(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}
void protocol() {
  auto scene = make_oncoming_traffic_scenario();
  std::vector<VecX> states;
  for (const auto& a : scene.agents) states.push_back(a.start);
  PassNegotiation p;
  p.update(scene, states, false);
  require(p.phase == PassPhase::Waiting && !p.parking_hold, "premature parking hold");
  p.update(scene, states, true);
  require(p.parking_hold && p.phase == PassPhase::Waiting, "ignored oncoming traffic");
  states[2](kPx) = -16.5;
  p.update(scene, states, true);
  require(p.phase == PassPhase::Waiting, "used axle instead of rear bumper clearance");
  states[2](kPx) = -19;
  states[0](kV) = 0.1;
  p.update(scene, states, true);
  require(p.phase == PassPhase::Waiting, "granted before parking car stopped");
  states[0](kV) = 0;
  p.update(scene, states, true);
  require(p.phase == PassPhase::Passing && p.parking_hold, "failed to grant clear pass");
  states[1](kPx) = -5; states[1](kPy) = 6.4;
  p.update(scene, states, true);
  require(p.parking_hold, "released parking while follower still alongside");
  states[1](kPx) = 0;
  p.update(scene, states, true);
  require(p.phase == PassPhase::Returning && !p.parking_hold, "failed to release parking");
  states[1](kPy) = 3.2;
  p.update(scene, states, true);
  require(p.phase == PassPhase::Complete && !p.parking_hold, "completion did not latch");
}

void closed_loop() {
  const auto scene = make_oncoming_traffic_scenario();
  const auto refs = plan_agent_references(scene);
  DistributedOptions options;
  // Functional test remains meaningful under sanitizers. Timing is measured
  // separately by the encounter executable with its normal 150 ms budget.
  options.deadline_ms = 10000;
  options.max_steps = 700;
  const auto result = simulate_distributed_ilqr(scene, refs, options);
  require(result.success && !result.collision && !result.safety_stop, "three-car encounter failed");
  bool waited = false, yielded = false, passed = false, restored = false, resumed = false;
  double stopped_parking_tick = -1;
  for (const auto& s : result.samples) {
    waited = waited || s.modes[1] == "WAIT_FOR_GAP";
    yielded = yielded || s.modes[0] == "YIELD_TO_PASS";
    passed = passed || s.states[1](kPy) > 5.6;
    restored = restored || s.modes[1] == "LANE_RESTORED";
    resumed = resumed || (yielded && s.modes[0] == "REVERSE_PARK");
    if (s.modes[1] == "WAIT_FOR_GAP") {
      for (const auto& p : footprint(scene.agents[1].vehicle, s.states[1]))
        require(p(1) < 4.8, "follower entered opposing lane before grant");
    }
    if (s.modes[0] == "YIELD_TO_PASS") {
      if (stopped_parking_tick < 0) stopped_parking_tick = s.time;
      if (s.time - stopped_parking_tick > 1)
        require(std::fabs(s.states[0](kV)) < 0.04, "parking yield did not stop");
    }
  }
  require(waited && yielded && passed && restored && resumed, "did not exercise negotiated interaction");
}
}
int main() {
  try { protocol(); closed_loop(); std::cout << "traffic tests passed\n"; return 0; }
  catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
