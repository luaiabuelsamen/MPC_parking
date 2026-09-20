// Kinematic bicycle model of a car, with the reference point at the centre of
// the rear axle.
//
//   state   x = [px, py, theta, v, delta]
//   control u = [a, omega]        (longitudinal accel, steering rate)
//
// Steering angle is a *state* rather than a control so that the optimiser can
// be charged for how fast the wheel is turned, which is what keeps parking
// manoeuvres drivable.
#pragma once

#include <array>
#include <vector>

#include "mpcpark/linalg.hpp"

namespace mpcpark {

constexpr int kNx = 5;
constexpr int kNu = 2;

using VecX = Mat<kNx, 1>;
using VecU = Mat<kNu, 1>;
using MatXX = Mat<kNx, kNx>;
using MatXU = Mat<kNx, kNu>;
using MatUU = Mat<kNu, kNu>;
using MatUX = Mat<kNu, kNx>;
using Vec2 = Mat<2, 1>;
using Mat2X = Mat<2, kNx>;

enum StateIdx { kPx = 0, kPy = 1, kTheta = 2, kV = 3, kDelta = 4 };
enum ControlIdx { kAccel = 0, kSteerRate = 1 };

struct VehicleParams {
  double wheelbase = 2.7;       // m, front axle to rear axle
  double length = 4.5;          // m, bumper to bumper
  double width = 1.8;           // m
  double rear_overhang = 1.0;   // m, rear axle to rear bumper

  double v_max = 2.0;           // m/s forward
  double v_min = -2.0;          // m/s reverse
  double a_max = 1.5;           // m/s^2
  double delta_max = 0.58;      // rad, ~33 deg
  double steer_rate_max = 0.8;  // rad/s

  int n_discs = 3;              // circles covering the footprint

  double disc_radius() const;
  // Offsets of the covering-circle centres along the body x axis, measured
  // from the rear axle.
  std::vector<double> disc_offsets() const;
  double front_overhang() const { return length - rear_overhang; }
  // Tightest turn the car can make, at the rear axle.
  double min_turn_radius() const;
};

// Footprint corners in world frame, ordered rear-left, rear-right,
// front-right, front-left.
std::array<Vec2, 4> footprint(const VehicleParams& p, const VecX& x);

// Centre of covering circle i in world frame, and its Jacobian w.r.t. state.
Vec2 disc_center(const VehicleParams& p, const VecX& x, int i);
Mat2X disc_center_jacobian(const VehicleParams& p, const VecX& x, int i);

// Continuous-time dynamics xdot = f(x, u).
VecX dynamics(const VehicleParams& p, const VecX& x, const VecU& u);
void dynamics_jacobian(const VehicleParams& p, const VecX& x, const VecU& u,
                       MatXX& A, MatXU& B);

// One RK4 step of the continuous dynamics.
VecX step_rk4(const VehicleParams& p, const VecX& x, const VecU& u, double dt);
// Same step, with the discrete Jacobians dx_next/dx and dx_next/du obtained by
// differentiating through the four RK4 stages.
void step_rk4_jacobian(const VehicleParams& p, const VecX& x, const VecU& u,
                       double dt, VecX& x_next, MatXX& A, MatXU& B);

}  // namespace mpcpark
