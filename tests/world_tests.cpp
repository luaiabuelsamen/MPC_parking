#include <iostream>
#include <stdexcept>

#include "mpcpark/world.hpp"

using namespace mpcpark;
namespace {
void require(bool test, const char* message) { if (!test) throw std::runtime_error(message); }
VecX pose(double x, double y, double yaw = 0) {
  VecX p; p(kPx)=x; p(kPy)=y; p(kTheta)=yaw; return p;
}
TrafficAgent car(double x, double y, double gx, double gy) {
  TrafficAgent a; a.start = pose(x,y); a.goal = pose(gx,gy); a.cruise_speed = 1; return a;
}
bool complete(const LiveWorld& world) {
  for (const auto& a : world.agents()) if (a.status != "ARRIVED") return false;
  return true;
}
void finish(LiveWorld& world, int ticks = 450) {
  for (int k=0; k<ticks && !complete(world); ++k) {
    world.step();
    require(!world.stopped(), "live controller reached unsafe stopping condition");
    require(world.clearance() > 0, "live controller contacted an obstacle or peer");
  }
  require(complete(world), "live controller did not reach arbitrary goals");
}
void live_edits() {
  LiveWorld world(10000);  // Functional checks also run under sanitizers.
  const auto block = Rect::from_size(0,0,5,4,0);
  world.reset({car(-14,-5,12,5)}, {block});
  for (int k=0; k<35; ++k) world.step();
  const VecX before = world.agents()[0].state;
  require(before(kV) > 0.5, "edit test must interrupt a moving car");
  const double time = world.time();
  world.set_obstacles({block, Rect::from_size(0,-4,4,2,0)});
  require((world.agents()[0].state - before).max_abs() == 0 && world.time() == time,
          "map edit teleported the measured state");
  world.add_agent(car(-14,8,10,8));
  world.step();
  require(world.agents()[0].state(kV) < before(kV), "edit failed to brake before replanning");
  finish(world);
  require(world.replans() >= 2, "map edit reused a stale route");
  world.set_goal(0, pose(5,5));
  require(world.agents()[0].status == "BRAKING_TO_REPLAN", "goal change did not wake an arrived agent");
  finish(world);
}
void crossing() {
  LiveWorld world(10000);
  world.reset({car(-12,-6,12,6), car(-12,6,12,-6)}, {});
  finish(world);
}
void rejected_edit_is_atomic() {
  LiveWorld world(10000);
  world.reset({car(-14,-5,12,5)}, {});
  world.step();
  const auto before = world.agents()[0].state;
  const int revision = world.revision();
  bool rejected = false;
  try { world.set_obstacles({vehicle_rect(world.agents()[0].spec.vehicle, before)}); }
  catch (const std::invalid_argument&) { rejected = true; }
  require(rejected && world.obstacles().empty() && world.revision() == revision &&
          (world.agents()[0].state - before).max_abs() == 0, "unsafe edit partially changed world");
  rejected = false;
  try { world.set_goal(4, pose(1,1)); } catch (const std::invalid_argument&) { rejected = true; }
  require(rejected, "invalid agent id accepted");
  // A goal covered by an obstacle is reported as NO_ROUTE, never ARRIVED.
  world.reset({car(-14,-5,0,0)}, {Rect::from_size(0,0,6,6,0)});
  world.step();
  require(world.agents()[0].status == "NO_ROUTE", "blocked goal did not report planning failure");
}
void deadline_fallback() {
  LiveWorld world(0.000001);
  world.reset({car(-10,0,10,0)}, {});
  for (int k=0; k<5; ++k) world.step();
  require(!world.stopped() && world.agents()[0].state(kV) == 0 &&
          world.agents()[0].state(kPx) == -10, "expired solve did not preserve checked stop");
}
}
int main() {
  try { live_edits(); crossing(); rejected_edit_is_atomic(); deadline_fallback(); std::cout << "persistent world tests passed\n"; return 0; }
  catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
