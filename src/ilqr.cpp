#include "mpcpark/ilqr.hpp"

#include <chrono>
#include <limits>

namespace mpcpark {

namespace {

// Solve H x = -g for the free block, with the free block at most 2x2.
bool solve_free(const MatUU& H, const VecU& g, const int* free_idx, int nfree,
                double* out) {
  if (nfree == 0) return true;
  if (nfree == 1) {
    const int i = free_idx[0];
    if (H(i, i) <= 1e-12) return false;
    out[0] = -g(i) / H(i, i);
    return true;
  }
  const int i = free_idx[0], j = free_idx[1];
  const double a = H(i, i), b = H(i, j), c = H(j, i), d = H(j, j);
  const double det = a * d - b * c;
  if (std::fabs(det) < 1e-12) return false;
  out[0] = (-g(i) * d + g(j) * b) / det;
  out[1] = (-g(j) * a + g(i) * c) / det;
  return true;
}

double quad_value(const MatUU& H, const VecU& g, const VecU& du) {
  return 0.5 * dot(du, H * du) + dot(g, du);
}

}  // namespace

VecU box_qp(const MatUU& H, const VecU& g, const VecU& lo, const VecU& hi,
            bool clamped[kNu]) {
  // With two controls the constrained minimiser can be found exactly by
  // enumerating the nine possible active sets and keeping the feasible
  // candidate with the lowest objective.
  VecU best{};
  bool best_clamped[kNu] = {true, true};
  double best_val = std::numeric_limits<double>::infinity();
  bool have_best = false;

  for (int mask = 0; mask < 9; ++mask) {
    int state[kNu];  // 0 = free, 1 = at lower bound, 2 = at upper bound
    state[0] = mask % 3;
    state[1] = (mask / 3) % 3;

    VecU du{};
    int free_idx[kNu];
    int nfree = 0;
    for (int i = 0; i < kNu; ++i) {
      if (state[i] == 0) {
        free_idx[nfree++] = i;
      } else {
        du(i) = state[i] == 1 ? lo(i) : hi(i);
      }
    }
    // Gradient contribution of the clamped components on the free ones.
    VecU g_eff = g + H * du;
    for (int i = 0; i < kNu; ++i) {
      if (state[i] != 0) g_eff(i) = 0.0;
    }
    double sol[kNu] = {0.0, 0.0};
    if (!solve_free(H, g_eff, free_idx, nfree, sol)) continue;
    bool feasible = true;
    for (int f = 0; f < nfree; ++f) {
      const int i = free_idx[f];
      du(i) = sol[f];
      if (du(i) < lo(i) - 1e-9 || du(i) > hi(i) + 1e-9) feasible = false;
    }
    if (!feasible) continue;

    const double val = quad_value(H, g, du);
    if (!have_best || val < best_val) {
      have_best = true;
      best_val = val;
      best = du;
      for (int i = 0; i < kNu; ++i) best_clamped[i] = state[i] != 0;
    }
  }

  if (!have_best) {  // degenerate Hessian: fall back to a projected step
    for (int i = 0; i < kNu; ++i) {
      best(i) = clampd(-g(i), lo(i), hi(i));
      best_clamped[i] = true;
    }
  }
  for (int i = 0; i < kNu; ++i) clamped[i] = best_clamped[i];
  return best;
}

ParkingSolver::ParkingSolver(Problem problem) : p_(std::move(problem)) {
  if (static_cast<int>(p_.xref.size()) != p_.horizon + 1) {
    p_.xref.resize(p_.horizon + 1, p_.xref.empty() ? VecX{} : p_.xref.back());
  }
  if (static_cast<int>(p_.dynamic_obstacles.size()) != p_.horizon + 1) {
    p_.dynamic_obstacles.resize(p_.horizon + 1);
  }
  for (const auto& stage : p_.dynamic_obstacles) {
    max_dynamic_obstacles_ =
        std::max(max_dynamic_obstacles_, static_cast<int>(stage.size()));
  }
  reset_duals();
}

void ParkingSolver::set_reference(std::vector<VecX> xref) {
  p_.xref = std::move(xref);
  if (static_cast<int>(p_.xref.size()) != p_.horizon + 1) {
    p_.xref.resize(p_.horizon + 1, p_.xref.empty() ? VecX{} : p_.xref.back());
  }
}

int ParkingSolver::num_constraints() const {
  return p_.vehicle.n_discs *
             (static_cast<int>(p_.obstacles.size()) + max_dynamic_obstacles_) +
         4;
}

void ParkingSolver::reset_duals() {
  mu_ = p_.options.mu_init;
  lambda_.assign(p_.horizon + 1,
                 std::vector<double>(num_constraints(), 0.0));
}

void ParkingSolver::constraints(const VecX& x, std::vector<double>& c,
                                std::vector<VecX>& dc) const {
  constraints_at(0, x, c, dc);
}

void ParkingSolver::constraints_at(int stage, const VecX& x,
                                   std::vector<double>& c,
                                   std::vector<VecX>& dc) const {
  const int nc = num_constraints();
  c.assign(nc, 0.0);
  dc.assign(nc, VecX{});

  const double r = p_.vehicle.disc_radius() + p_.options.safety_margin;
  const int bounded_stage = std::max(0, std::min(stage, p_.horizon));
  const auto& moving = p_.dynamic_obstacles[bounded_stage];
  int idx = 0;
  for (int d = 0; d < p_.vehicle.n_discs; ++d) {
    const Vec2 pc = disc_center(p_.vehicle, x, d);
    const Mat2X J = disc_center_jacobian(p_.vehicle, x, d);
    for (const Rect& o : p_.obstacles) {
      Vec2 g{};
      const double sd = rect_sdf_grad(o, pc, g);
      // Clearance constraint: r - sd <= 0.
      c[idx] = r - sd;
      dc[idx] = -1.0 * (J.transpose() * g);
      ++idx;
    }
    for (int obstacle = 0; obstacle < max_dynamic_obstacles_; ++obstacle) {
      if (obstacle >= static_cast<int>(moving.size())) {
        c[idx++] = -1e6;
        continue;
      }
      Vec2 g{};
      const double sd = rect_sdf_grad(moving[obstacle], pc, g);
      c[idx] = r - sd;
      dc[idx] = -1.0 * (J.transpose() * g);
      ++idx;
    }
  }
  // Speed and steering limits as state constraints.
  c[idx] = x(kV) - p_.vehicle.v_max;
  dc[idx](kV) = 1.0;
  ++idx;
  c[idx] = p_.vehicle.v_min - x(kV);
  dc[idx](kV) = -1.0;
  ++idx;
  c[idx] = x(kDelta) - p_.vehicle.delta_max;
  dc[idx](kDelta) = 1.0;
  ++idx;
  c[idx] = -p_.vehicle.delta_max - x(kDelta);
  dc[idx](kDelta) = -1.0;
}

namespace {

VecX state_error(const VecX& x, const VecX& ref) {
  VecX e = x - ref;
  e(kTheta) = wrap_pi(x(kTheta) - ref(kTheta));
  return e;
}

}  // namespace

double ParkingSolver::trajectory_cost(const std::vector<VecX>& xs,
                                      const std::vector<VecU>& us,
                                      double& max_violation) const {
  const Weights& w = p_.weights;
  const int N = p_.horizon;
  double cost = 0.0;
  max_violation = 0.0;

  std::vector<double> c;
  std::vector<VecX> dc;
  for (int k = 0; k <= N; ++k) {
    const VecX e = state_error(xs[k], p_.xref[k]);
    if (k < N) {
      cost += 0.5 * (w.pos * (e(kPx) * e(kPx) + e(kPy) * e(kPy)) +
                     w.yaw * e(kTheta) * e(kTheta) + w.v * e(kV) * e(kV) +
                     w.delta * e(kDelta) * e(kDelta));
      cost += 0.5 * (w.accel * us[k](kAccel) * us[k](kAccel) +
                     w.steer_rate * us[k](kSteerRate) * us[k](kSteerRate));
    } else {
      cost += 0.5 * (w.term_pos * (e(kPx) * e(kPx) + e(kPy) * e(kPy)) +
                     w.term_yaw * e(kTheta) * e(kTheta) +
                     w.term_v * e(kV) * e(kV) +
                     w.term_delta * e(kDelta) * e(kDelta));
    }
    if (k == 0) continue;  // the initial state is fixed; penalising it is moot
    constraints_at(k, xs[k], c, dc);
    for (size_t i = 0; i < c.size(); ++i) {
      const double lam = lambda_[k][i];
      const double t = lam + mu_ * c[i];
      if (t > 0.0) cost += (t * t - lam * lam) / (2.0 * mu_);
      max_violation = std::max(max_violation, c[i]);
    }
  }
  return cost;
}

bool ParkingSolver::backward_pass(const std::vector<VecX>& xs,
                                  const std::vector<VecU>& us, double reg,
                                  std::vector<VecU>& k_ff,
                                  std::vector<MatUX>& K, double dV[2]) const {
  const Weights& w = p_.weights;
  const int N = p_.horizon;
  const MatXX I = MatXX::identity();
  dV[0] = dV[1] = 0.0;

  VecX qx_diag{};  // stage state weights
  qx_diag(kPx) = w.pos; qx_diag(kPy) = w.pos; qx_diag(kTheta) = w.yaw;
  qx_diag(kV) = w.v; qx_diag(kDelta) = w.delta;
  VecX qf_diag{};
  qf_diag(kPx) = w.term_pos; qf_diag(kPy) = w.term_pos;
  qf_diag(kTheta) = w.term_yaw; qf_diag(kV) = w.term_v;
  qf_diag(kDelta) = w.term_delta;

  std::vector<double> c;
  std::vector<VecX> dc;

  // Terminal value function.
  const VecX eN = state_error(xs[N], p_.xref[N]);
  VecX Vx{};
  MatXX Vxx = MatXX::zero();
  for (int i = 0; i < kNx; ++i) {
    Vx(i) = qf_diag(i) * eN(i);
    Vxx(i, i) = qf_diag(i);
  }
  constraints_at(N, xs[N], c, dc);
  for (size_t i = 0; i < c.size(); ++i) {
    const double t = lambda_[N][i] + mu_ * c[i];
    if (t <= 0.0) continue;
    Vx += t * dc[i];
    Vxx += mu_ * outer(dc[i], dc[i]);
  }

  VecU lo{}, hi{};
  lo(kAccel) = -p_.vehicle.a_max;
  hi(kAccel) = p_.vehicle.a_max;
  lo(kSteerRate) = -p_.vehicle.steer_rate_max;
  hi(kSteerRate) = p_.vehicle.steer_rate_max;

  for (int k = N - 1; k >= 0; --k) {
    VecX x_next{};
    MatXX A;
    MatXU B;
    step_rk4_jacobian(p_.vehicle, xs[k], us[k], p_.dt, x_next, A, B);

    // Stage cost derivatives.
    const VecX e = state_error(xs[k], p_.xref[k]);
    VecX lx{};
    MatXX lxx = MatXX::zero();
    for (int i = 0; i < kNx; ++i) {
      lx(i) = qx_diag(i) * e(i);
      lxx(i, i) = qx_diag(i);
    }
    if (k > 0) {
      constraints_at(k, xs[k], c, dc);
      for (size_t i = 0; i < c.size(); ++i) {
        const double t = lambda_[k][i] + mu_ * c[i];
        if (t <= 0.0) continue;
        lx += t * dc[i];
        lxx += mu_ * outer(dc[i], dc[i]);
      }
    }
    VecU lu{};
    MatUU luu = MatUU::zero();
    lu(kAccel) = w.accel * us[k](kAccel);
    lu(kSteerRate) = w.steer_rate * us[k](kSteerRate);
    luu(kAccel, kAccel) = w.accel;
    luu(kSteerRate, kSteerRate) = w.steer_rate;

    const MatXX Bt_pre = MatXX::zero();
    (void)Bt_pre;
    const Mat<kNu, kNx> Bt = B.transpose();
    const Mat<kNx, kNx> Vxx_reg = Vxx + reg * I;

    const VecX Qx = lx + A.transpose() * Vx;
    const VecU Qu = lu + Bt * Vx;
    const MatXX Qxx = lxx + A.transpose() * Vxx * A;
    const MatUU Quu = luu + Bt * Vxx * B;
    const MatUX Qux = Bt * Vxx * A;
    const MatUU Quu_reg = luu + Bt * Vxx_reg * B;
    const MatUX Qux_reg = Bt * Vxx_reg * A;

    MatUU chol;
    if (!cholesky(symmetrize(Quu_reg), chol)) return false;

    bool clamped[kNu];
    const VecU kk = box_qp(symmetrize(Quu_reg), Qu, lo - us[k], hi - us[k],
                           clamped);

    // Feedback gains: zero rows for the clamped controls, so the rollout does
    // not fight the limits.
    MatUX Kk = MatUX::zero();
    {
      MatUU Hfree = symmetrize(Quu_reg);
      MatUX rhs = Qux_reg;
      for (int i = 0; i < kNu; ++i) {
        if (!clamped[i]) continue;
        for (int j = 0; j < kNu; ++j) {
          Hfree(i, j) = 0.0;
          Hfree(j, i) = 0.0;
        }
        Hfree(i, i) = 1.0;
        for (int j = 0; j < kNx; ++j) rhs(i, j) = 0.0;
      }
      MatUX sol;
      if (!solve_spd(Hfree, rhs, sol)) return false;
      Kk = -1.0 * sol;
      for (int i = 0; i < kNu; ++i) {
        if (!clamped[i]) continue;
        for (int j = 0; j < kNx; ++j) Kk(i, j) = 0.0;
      }
    }

    dV[0] += dot(kk, Qu);
    dV[1] += 0.5 * dot(kk, Quu * kk);

    const MatXU Kt_Quu = Kk.transpose() * Quu;
    Vx = Qx + Kt_Quu * kk + Kk.transpose() * Qu + Qux.transpose() * kk;
    Vxx = symmetrize(Qxx + Kt_Quu * Kk + Kk.transpose() * Qux +
                     Qux.transpose() * Kk);

    k_ff[k] = kk;
    K[k] = Kk;
  }
  return true;
}

void ParkingSolver::rollout(const VecX& x0, const std::vector<VecX>& xs_ref,
                            const std::vector<VecU>& us_ref,
                            const std::vector<VecU>& k_ff,
                            const std::vector<MatUX>& K, double alpha,
                            std::vector<VecX>& xs,
                            std::vector<VecU>& us) const {
  const int N = p_.horizon;
  xs[0] = x0;
  for (int k = 0; k < N; ++k) {
    const VecX dx = state_error(xs[k], xs_ref[k]);
    VecU u = us_ref[k] + alpha * k_ff[k] + K[k] * dx;
    u(kAccel) = clampd(u(kAccel), -p_.vehicle.a_max, p_.vehicle.a_max);
    u(kSteerRate) = clampd(u(kSteerRate), -p_.vehicle.steer_rate_max,
                           p_.vehicle.steer_rate_max);
    us[k] = u;
    xs[k + 1] = step_rk4(p_.vehicle, xs[k], u, p_.dt);
  }
}

Solution ParkingSolver::solve(const VecX& x0,
                              const std::vector<VecU>& us_init) {
  using clock = std::chrono::steady_clock;
  const auto t_start = clock::now();
  const int N = p_.horizon;
  const SolverOptions& opt = p_.options;

  Solution sol;
  sol.us = us_init;
  if (static_cast<int>(sol.us.size()) != N) sol.us.assign(N, VecU{});
  sol.xs.assign(N + 1, VecX{});

  // Initial rollout.
  sol.xs[0] = x0;
  for (int k = 0; k < N; ++k) {
    sol.us[k](kAccel) = clampd(sol.us[k](kAccel), -p_.vehicle.a_max,
                               p_.vehicle.a_max);
    sol.us[k](kSteerRate) = clampd(sol.us[k](kSteerRate),
                                   -p_.vehicle.steer_rate_max,
                                   p_.vehicle.steer_rate_max);
    sol.xs[k + 1] = step_rk4(p_.vehicle, sol.xs[k], sol.us[k], p_.dt);
  }

  std::vector<VecU> k_ff(N);
  std::vector<MatUX> K(N);
  std::vector<VecX> xs_new(N + 1);
  std::vector<VecU> us_new(N);
  std::vector<double> c;
  std::vector<VecX> dc;

  double reg = opt.reg_init;
  double max_violation = 0.0;
  double cost = trajectory_cost(sol.xs, sol.us, max_violation);
  int total_inner = 0;

  for (int outer = 0; outer < opt.max_outer; ++outer) {
    // --- inner loop: iLQR against the current penalty and multipliers ---
    for (int iter = 0; iter < opt.max_inner; ++iter) {
      ++total_inner;
      double dV[2];
      if (!backward_pass(sol.xs, sol.us, reg, k_ff, K, dV)) {
        reg = std::min(opt.reg_max, std::max(reg * 10.0, opt.reg_min * 10.0));
        if (reg >= opt.reg_max) break;
        continue;
      }

      double grad_norm = 0.0;
      for (int k = 0; k < N; ++k) grad_norm = std::max(grad_norm, k_ff[k].max_abs());

      bool accepted = false;
      double new_cost = cost;
      for (double alpha = 1.0; alpha > 1e-4; alpha *= 0.5) {
        rollout(x0, sol.xs, sol.us, k_ff, K, alpha, xs_new, us_new);
        double viol = 0.0;
        new_cost = trajectory_cost(xs_new, us_new, viol);
        const double expected = -(alpha * dV[0] + alpha * alpha * dV[1]);
        const double actual = cost - new_cost;
        if (actual > 0.0 && (expected <= 0.0 || actual / expected > 1e-4)) {
          sol.xs = xs_new;
          sol.us = us_new;
          max_violation = viol;
          accepted = true;
          break;
        }
      }

      if (accepted) {
        reg = std::max(opt.reg_min, reg / 2.0);
        const double rel = (cost - new_cost) / std::max(1.0, std::fabs(cost));
        cost = new_cost;
        if (rel < opt.cost_tol && grad_norm < 1e-2) break;
        if (grad_norm < opt.grad_tol) break;
      } else {
        reg = std::min(opt.reg_max, reg * 10.0);
        if (reg >= opt.reg_max) break;
      }
    }

    // --- outer loop: multiplier and penalty update ---
    max_violation = 0.0;
    for (int k = 1; k <= N; ++k) {
      constraints_at(k, sol.xs[k], c, dc);
      for (size_t i = 0; i < c.size(); ++i) {
        lambda_[k][i] = std::max(0.0, lambda_[k][i] + mu_ * c[i]);
        max_violation = std::max(max_violation, c[i]);
      }
    }
    sol.stats.outer_iters = outer + 1;
    if (max_violation < opt.constraint_tol) {
      sol.stats.converged = true;
      break;
    }
    mu_ = std::min(opt.mu_max, mu_ * opt.mu_scale);
    cost = trajectory_cost(sol.xs, sol.us, max_violation);
  }

  sol.stats.cost = cost;
  sol.stats.max_violation = max_violation;
  sol.stats.inner_iters = total_inner;
  sol.stats.solve_ms =
      std::chrono::duration<double, std::milli>(clock::now() - t_start).count();
  return sol;
}

}  // namespace mpcpark
