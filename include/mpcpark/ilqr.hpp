// Nonlinear trajectory optimiser for the parking OCP.
//
// Inner loop : iLQR (Gauss-Newton DDP) with Levenberg-Marquardt regularisation
//              and an exact box-QP in the backward pass, so accel and steering
//              rate limits are respected by the feedback gains and not just
//              clipped afterwards.
// Outer loop : augmented Lagrangian on the state constraints -- obstacle
//              clearance, speed limits, steering limits. Penalties start soft
//              and tighten, which is what lets the solver push through a
//              slot it does not initially fit in.
#pragma once

#include <vector>

#include "mpcpark/geometry.hpp"
#include "mpcpark/vehicle.hpp"

namespace mpcpark {

struct Weights {
  // Stage cost on the deviation from the reference trajectory.
  double pos = 1.0;
  double yaw = 1.0;
  double v = 0.2;
  double delta = 0.5;
  // Control effort.
  double accel = 0.5;
  double steer_rate = 0.5;
  // Terminal cost on the deviation from the goal pose.
  double term_pos = 2000.0;
  double term_yaw = 2000.0;
  double term_v = 200.0;
  double term_delta = 50.0;
};

struct SolverOptions {
  int max_outer = 8;          // augmented Lagrangian updates
  int max_inner = 60;         // iLQR iterations per outer loop
  double cost_tol = 1e-4;     // relative cost decrease that counts as converged
  double grad_tol = 1e-5;     // feedforward magnitude that counts as converged
  double constraint_tol = 1e-3;  // metres of violation we accept
  double mu_init = 12.0;      // initial penalty
  double mu_scale = 7.0;      // penalty growth per outer iteration
  double mu_max = 1e7;
  double reg_init = 1e-6;
  double reg_min = 1e-8;
  double reg_max = 1e10;
  double safety_margin = 0.05;  // m, added to the covering-circle radius
};

struct Problem {
  VehicleParams vehicle;
  std::vector<Rect> obstacles;
  // Predicted moving obstacles at each state stage. Each stage may contain
  // fewer entries than the maximum; missing entries are treated as inactive.
  std::vector<std::vector<Rect>> dynamic_obstacles;
  double dt = 0.1;
  int horizon = 40;             // N steps, so N+1 states
  std::vector<VecX> xref;       // size N+1; the last entry is the goal pose
  Weights weights;
  SolverOptions options;
};

struct SolveStats {
  double cost = 0.0;
  double max_violation = 0.0;  // largest constraint violation, metres
  int outer_iters = 0;
  int inner_iters = 0;
  bool converged = false;
  double solve_ms = 0.0;
};

struct Solution {
  std::vector<VecX> xs;  // N+1 states
  std::vector<VecU> us;  // N controls
  SolveStats stats;
};

class ParkingSolver {
 public:
  explicit ParkingSolver(Problem problem);

  // Solve from x0. `us_init` may hold a warm start of length N; if it is empty
  // or the wrong length it is replaced by zeros. Multipliers persist between
  // calls unless reset_duals() is called, which is what makes the receding
  // horizon cheap.
  Solution solve(const VecX& x0, const std::vector<VecU>& us_init);

  void set_reference(std::vector<VecX> xref);
  void reset_duals();
  const Problem& problem() const { return p_; }

  // Exposed for testing: constraint values and Jacobians at a state.
  void constraints(const VecX& x, std::vector<double>& c,
                   std::vector<VecX>& dc) const;
  void constraints_at(int stage, const VecX& x, std::vector<double>& c,
                      std::vector<VecX>& dc) const;
  int num_constraints() const;

 private:
  double trajectory_cost(const std::vector<VecX>& xs,
                         const std::vector<VecU>& us, double& max_violation)
      const;
  bool backward_pass(const std::vector<VecX>& xs, const std::vector<VecU>& us,
                     double reg, std::vector<VecU>& k, std::vector<MatUX>& K,
                     double dV[2]) const;
  void rollout(const VecX& x0, const std::vector<VecX>& xs_ref,
               const std::vector<VecU>& us_ref, const std::vector<VecU>& k,
               const std::vector<MatUX>& K, double alpha,
               std::vector<VecX>& xs, std::vector<VecU>& us) const;

  Problem p_;
  int max_dynamic_obstacles_ = 0;
  double mu_ = 0.0;
  std::vector<std::vector<double>> lambda_;  // [stage][constraint]
};

// Exact minimiser of 0.5 du' H du + g' du over the box [lo - u, hi - u], for
// the two-dimensional control. Returns the step and marks which components
// ended up clamped.
VecU box_qp(const MatUU& H, const VecU& g, const VecU& lo, const VecU& hi,
            bool clamped[kNu]);

}  // namespace mpcpark
