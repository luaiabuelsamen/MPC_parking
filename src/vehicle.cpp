#include "mpcpark/vehicle.hpp"

namespace mpcpark {

double VehicleParams::disc_radius() const {
  const double half_seg = 0.5 * length / static_cast<double>(n_discs);
  return std::sqrt(half_seg * half_seg + 0.25 * width * width);
}

std::vector<double> VehicleParams::disc_offsets() const {
  std::vector<double> offs;
  offs.reserve(n_discs);
  const double seg = length / static_cast<double>(n_discs);
  for (int i = 0; i < n_discs; ++i) {
    offs.push_back(-rear_overhang + (static_cast<double>(i) + 0.5) * seg);
  }
  return offs;
}

double VehicleParams::min_turn_radius() const {
  return wheelbase / std::tan(delta_max);
}

std::array<Vec2, 4> footprint(const VehicleParams& p, const VecX& x) {
  const double c = std::cos(x(kTheta)), s = std::sin(x(kTheta));
  const double xb = -p.rear_overhang, xf = p.front_overhang();
  const double hw = 0.5 * p.width;
  const double local[4][2] = {{xb, hw}, {xb, -hw}, {xf, -hw}, {xf, hw}};
  std::array<Vec2, 4> out{};
  for (int i = 0; i < 4; ++i) {
    out[i](0) = x(kPx) + c * local[i][0] - s * local[i][1];
    out[i](1) = x(kPy) + s * local[i][0] + c * local[i][1];
  }
  return out;
}

Vec2 disc_center(const VehicleParams& p, const VecX& x, int i) {
  const double d = p.disc_offsets()[i];
  Vec2 c{};
  c(0) = x(kPx) + d * std::cos(x(kTheta));
  c(1) = x(kPy) + d * std::sin(x(kTheta));
  return c;
}

Mat2X disc_center_jacobian(const VehicleParams& p, const VecX& x, int i) {
  const double d = p.disc_offsets()[i];
  Mat2X J{};
  J(0, kPx) = 1.0;
  J(1, kPy) = 1.0;
  J(0, kTheta) = -d * std::sin(x(kTheta));
  J(1, kTheta) = d * std::cos(x(kTheta));
  return J;
}

VecX dynamics(const VehicleParams& p, const VecX& x, const VecU& u) {
  VecX dx{};
  dx(kPx) = x(kV) * std::cos(x(kTheta));
  dx(kPy) = x(kV) * std::sin(x(kTheta));
  dx(kTheta) = x(kV) * std::tan(x(kDelta)) / p.wheelbase;
  dx(kV) = u(kAccel);
  dx(kDelta) = u(kSteerRate);
  return dx;
}

void dynamics_jacobian(const VehicleParams& p, const VecX& x, const VecU& u,
                       MatXX& A, MatXU& B) {
  (void)u;
  const double c = std::cos(x(kTheta)), s = std::sin(x(kTheta));
  const double td = std::tan(x(kDelta));
  const double cd = std::cos(x(kDelta));
  A = MatXX::zero();
  A(kPx, kTheta) = -x(kV) * s;
  A(kPx, kV) = c;
  A(kPy, kTheta) = x(kV) * c;
  A(kPy, kV) = s;
  A(kTheta, kV) = td / p.wheelbase;
  A(kTheta, kDelta) = x(kV) / (p.wheelbase * cd * cd);
  B = MatXU::zero();
  B(kV, kAccel) = 1.0;
  B(kDelta, kSteerRate) = 1.0;
}

VecX step_rk4(const VehicleParams& p, const VecX& x, const VecU& u, double dt) {
  const VecX k1 = dynamics(p, x, u);
  const VecX k2 = dynamics(p, x + (0.5 * dt) * k1, u);
  const VecX k3 = dynamics(p, x + (0.5 * dt) * k2, u);
  const VecX k4 = dynamics(p, x + dt * k3, u);
  return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

void step_rk4_jacobian(const VehicleParams& p, const VecX& x, const VecU& u,
                       double dt, VecX& x_next, MatXX& A, MatXU& B) {
  const MatXX I = MatXX::identity();

  const VecX x1 = x;
  const VecX k1 = dynamics(p, x1, u);
  MatXX A1; MatXU B1;
  dynamics_jacobian(p, x1, u, A1, B1);
  // dk1/dx = A1, dk1/du = B1

  const VecX x2 = x + (0.5 * dt) * k1;
  const VecX k2 = dynamics(p, x2, u);
  MatXX A2; MatXU B2;
  dynamics_jacobian(p, x2, u, A2, B2);
  const MatXX dk2dx = A2 * (I + (0.5 * dt) * A1);
  const MatXU dk2du = A2 * ((0.5 * dt) * B1) + B2;

  const VecX x3 = x + (0.5 * dt) * k2;
  const VecX k3 = dynamics(p, x3, u);
  MatXX A3; MatXU B3;
  dynamics_jacobian(p, x3, u, A3, B3);
  const MatXX dk3dx = A3 * (I + (0.5 * dt) * dk2dx);
  const MatXU dk3du = A3 * ((0.5 * dt) * dk2du) + B3;

  const VecX x4 = x + dt * k3;
  const VecX k4 = dynamics(p, x4, u);
  MatXX A4; MatXU B4;
  dynamics_jacobian(p, x4, u, A4, B4);
  const MatXX dk4dx = A4 * (I + dt * dk3dx);
  const MatXU dk4du = A4 * (dt * dk3du) + B4;

  x_next = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
  A = I + (dt / 6.0) * (A1 + 2.0 * dk2dx + 2.0 * dk3dx + dk4dx);
  B = (dt / 6.0) * (B1 + 2.0 * dk2du + 2.0 * dk3du + dk4du);
}

}  // namespace mpcpark
