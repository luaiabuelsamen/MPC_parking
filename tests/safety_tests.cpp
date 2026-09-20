#include <chrono>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "mpcpark/multi_agent.hpp"

using namespace mpcpark;
namespace {
void require(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}

PassingScenario empty_scene() {
  PassingScenario scene;
  scene.agents.resize(2);
  scene.agents[0].start(kPx) = -10;
  scene.agents[1].start(kPx) = 10;
  return scene;
}

void test_swept_motion() {
  auto scene = empty_scene();
  // Tiny bodies cross completely between endpoints; endpoint-only testing
  // misses their contact in the middle of the tick.
  for (auto& a : scene.agents) {
    a.vehicle.length = 0.2;
    a.vehicle.width = 0.2;
    a.vehicle.rear_overhang = 0.1;
    a.vehicle.v_max = 10;
  }
  VecX a{}, b{};
  a(kPx) = -1; a(kV) = 2;
  b(kPx) = 1; b(kTheta) = M_PI; b(kV) = 2;
  const auto coarse = check_motion(scene, {a, b}, {VecU{}, VecU{}}, 1, 1);
  const auto fine = check_motion(scene, {a, b}, {VecU{}, VecU{}}, 1, 20);
  require(!coarse.collision && fine.collision, "missed collision between control ticks");
  b(kPx) = 3;
  require(check_motion(scene, {a, b}, {VecU{}, VecU{}}, 1, 20).collision,
          "missed collision at terminal state");
  scene.static_obstacles = {Rect::from_size(0, 0, 0.1, 0.4, 0)};
  b(kPy) = 3;
  require(check_motion(scene, {a, b}, {VecU{}, VecU{}}, 1, 20).collision,
          "missed static obstacle between ticks");
}

void test_braking() {
  VehicleParams vehicle;
  for (double speed : {-0.02, -1.0, 0.0, 0.02, 1.0}) {
    VecX x{}; x(kV) = speed; x(kDelta) = 0.3;
    const auto u = braking_control(vehicle, x, 0.15);
    const auto next = step_rk4(vehicle, x, u, 0.15);
    require(std::fabs(next(kV)) <= std::fabs(speed) + 1e-12, "braking accelerated");
    require(speed * next(kV) >= -1e-12, "braking crossed zero velocity");
    require(next(kDelta) == x(kDelta), "braking unexpectedly changed steering");
  }
}

void test_deadline() {
  Problem problem;
  problem.horizon = 10;
  ParkingSolver solver(problem);
  const auto solution = solver.solve(VecX{}, {}, std::chrono::steady_clock::now());
  require(solution.stats.timed_out && solution.stats.inner_iters == 0,
          "expired solver deadline was ignored");
  require(solution.xs.size() == 11 && solution.us.size() == 10,
          "deadline returned malformed trajectory");
}

void test_fallback_and_validation() {
  auto scene = empty_scene();
  std::vector<ReferenceTrajectory> refs(2);
  for (int i = 0; i < 2; ++i) {
    scene.agents[i].goal = scene.agents[i].start;
    scene.agents[i].goal(kPx) += 1;
    scene.agents[i].start(kV) = 0.2;
    refs[i].xs.assign(3, scene.agents[i].goal);
    refs[i].xs.front() = scene.agents[i].start;
    refs[i].us.resize(2);
  }
  DistributedOptions options;
  options.mpc_horizon = 3;
  options.max_steps = 2;
  options.fault_start_step = 0;
  options.fault_steps = 2;
  const auto run = simulate_distributed_ilqr(scene, refs, options);
  require(run.injected_faults == 2 && run.fallback_steps == 2 && !run.collision,
          "forced missing results did not trigger fallback");
  require(std::fabs(run.samples.back().states[0](kV)) < 1e-10,
          "fallback did not stop plant");
  options.fault_steps = 0;
  options.deadline_ms = 0.000001;
  const auto late = simulate_distributed_ilqr(scene, refs, options);
  require(late.deadline_misses == 2 && late.fallback_steps == 2 && !late.collision,
          "expired coordination deadline did not reject commands");
  options.deadline_ms = 150;
  options.fault_steps = 2;

  // An initially colliding experiment must not advertise safe termination.
  scene.agents[1].start = scene.agents[0].start;
  const auto blocked = simulate_distributed_ilqr(scene, refs, options);
  require(blocked.safety_stop && blocked.collision && !blocked.success,
          "initial collision was mislabeled as safe");

  refs[0].us.clear();
  bool threw = false;
  try { simulate_distributed_ilqr(scene, refs, options); }
  catch (const std::invalid_argument&) { threw = true; }
  require(threw, "empty reference accepted");
  VecX nan{}; nan(kPx) = std::numeric_limits<double>::quiet_NaN();
  require(!check_motion(scene, {nan, VecX{}}, {VecU{}, VecU{}}, 0.15, 10).safe(),
          "nonfinite state passed safety check");
}

void test_stage_constraints() {
  Problem p;
  p.horizon = 2;
  p.dynamic_obstacles.resize(3);
  p.dynamic_obstacles[1] = {Rect::from_size(1.25, 0, 4.5, 1.8, 0)};
  p.dynamic_obstacles[2] = {Rect::from_size(20, 0, 4.5, 1.8, 0)};
  ParkingSolver solver(p);
  std::vector<double> c;
  std::vector<VecX> dc;
  solver.constraints_at(0, VecX{}, c, dc);
  require(c[0] < 0, "missing moving obstacle must be inactive");
  solver.constraints_at(1, VecX{}, c, dc);
  require(c[0] > 0, "moving obstacle at current stage was missed");
  solver.constraints_at(2, VecX{}, c, dc);
  require(c[0] < 0, "future stage reused stale moving obstacle");
}
}  // namespace

int main() {
  try {
    test_swept_motion(); test_braking(); test_deadline(); test_fallback_and_validation();
    test_stage_constraints();
    std::cout << "safety tests passed\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
