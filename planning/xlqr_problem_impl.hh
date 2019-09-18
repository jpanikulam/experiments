#pragma once

#include "planning/xlqr_problem.hh"

#include "logging/assert.hh"

#include "numerics/project_psd.hh"

#include <iostream>
#include <limits>

namespace planning {

template <typename _Prob>
typename XlqrProblem<_Prob>::Solution XlqrProblem<_Prob>::solve(
    const typename XlqrProblem<_Prob>::State& x0,
    const typename XlqrProblem<_Prob>::Solution& initialization,
    const typename XlqrProblem<_Prob>::Visitor& visitor) const {
  Solution traj0;
  if (initialization.x.empty()) {
    traj0.x.resize(prob_.horizon());
    traj0.x[0] = x0;
    traj0.u.resize(prob_.horizon() - 1, ControlVec::Zero());
  } else {
    JASSERT_EQ(initialization.x.size(),
               prob_.horizon(),
               "Horizon and initialization must match in size");
    JASSERT_EQ(initialization.u.size(),
               prob_.horizon() - 1,
               "There must be one fewer controls that states");
    traj0 = initialization;
  }

  Solution current_soln;
  shoot(traj0, {}, -1.0, &current_soln);

  constexpr int MAX_ITERS = 15;
  int iter = 0;
  for (; iter < MAX_ITERS; ++iter) {
    if (visitor) {
      visitor(current_soln, iter, false);
    }

    const auto lqr_soln = ricatti(current_soln);
    current_soln = line_search(current_soln, lqr_soln);
  }
  if (visitor) {
    visitor(current_soln, iter + 1, true);
  }

  return current_soln;
}

template <typename _Prob>
double XlqrProblem<_Prob>::shoot(const typename XlqrProblem<_Prob>::Solution& soln,
                                 const typename XlqrProblem<_Prob>::LqrSolution& lqr_soln,
                                 double alpha,
                                 Solution* out_soln) const {
  if (out_soln) {
    out_soln->x.clear();
    out_soln->u.clear();
    out_soln->x.reserve(soln.x.size());
    out_soln->u.reserve(soln.u.size());
  }

  State x = soln.x.front();

  double cost = 0.0;
  for (std::size_t t = 0; t < soln.u.size(); ++t) {
    const StateVec error = prob_.delta_vec(x, soln.x.at(t));

    // TEMP: HACK
    ControlVec optimized_u;
    if (alpha == -1.0) {
      optimized_u = ControlVec::Zero();
    } else {
      const ControlVec feedback_u = lqr_soln.at(t).K * error;
      const ControlVec feedforward_u = alpha * lqr_soln[t].k;
      optimized_u = feedback_u + feedforward_u;
    }
    const ControlVec executed_u = soln.u[t] + optimized_u;
    cost += prob_.cost(x, executed_u, t);

    if (out_soln) {
      out_soln->x.push_back(x);
      out_soln->u.push_back(executed_u);
    }

    x = prob_.dynamics(x, executed_u);
  }
  if (out_soln) {
    out_soln->x.push_back(x);
  }
  cost += prob_.cost(x, ControlVec::Zero(), prob_.horizon());

  return cost;
}

template <typename _Prob>
typename XlqrProblem<_Prob>::Solution XlqrProblem<_Prob>::line_search(
    const typename XlqrProblem<_Prob>::Solution& soln,
    const typename XlqrProblem<_Prob>::LqrSolution& lqr_soln) const {
  double best_cost = std::numeric_limits<double>::max();
  double best_alpha = -1.0;
  for (double alpha : {1e-5, 0.01, 0.1, 0.125, 0.25, 0.5, 1.0}) {
    const double cost = shoot(soln, lqr_soln, alpha);
    if (cost < best_cost) {
      best_cost = cost;
      best_alpha = alpha;
    }
  }

  JASSERT_GT(best_alpha, 0.0, "Line search must find a positive alpha");

  Solution out_soln;
  shoot(soln, lqr_soln, best_alpha, &out_soln);
  out_soln.cost = best_cost;
  return out_soln;
}

template <typename _Prob>
typename XlqrProblem<_Prob>::LqrSolution XlqrProblem<_Prob>::ricatti(
    const typename XlqrProblem<_Prob>::Solution& soln) const {
  LqrSolution lqr_soln;
  lqr_soln.resize(soln.u.size());

  using StateHess = MatNd<Prob::X_DIM, Prob::X_DIM>;
  using ControlHess = MatNd<Prob::U_DIM, Prob::U_DIM>;
  using StateControlHessBlock = MatNd<Prob::U_DIM, Prob::X_DIM>;

  const Derivatives final_derivatives =
      cost_diffs_(soln.x.back(), ControlVec::Zero(), prob_.horizon());
  StateHess Vxx = final_derivatives.Q;
  VecNd<Prob::X_DIM> Vx = final_derivatives.g_x;

  const int ctrl_size = static_cast<int>(soln.u.size());
  for (int k = ctrl_size - 1; k >= 0; --k) {
    const State& x = soln.x.at(k);
    const ControlVec& u = soln.u.at(k);
    const Derivatives diffs = cost_diffs_(x, u, k);

    const StateVec Qx = diffs.g_x + diffs.A.transpose() * Vx;
    const ControlVec Qu = diffs.g_u + diffs.B.transpose() * Vx;

    const StateHess Qxx = diffs.Q + diffs.A.transpose() * Vxx * diffs.A;

    constexpr double MU_0 = 1e-1;
    const StateHess levenberg_damping = MU_0 * StateHess::Identity();

    const ControlHess Quu =
        diffs.R + diffs.B.transpose() * (Vxx + levenberg_damping) * diffs.B;
    const StateControlHessBlock Qux =
        diffs.N + diffs.B.transpose() * (Vxx + levenberg_damping) * diffs.A;

    const ControlHess Quu_damped1 = numerics::project_psd(Quu, 0.1);
    // const ControlHess Quu_damped = numerics::deproject_psd(Quu_damped1, 10.0);
    const ControlHess Quu_damped = Quu_damped1;

    // Safe LLT computation
    const Eigen::LDLT<ControlHess> Quu_llt(Quu_damped);
    if (Quu_llt.info() != Eigen::Success) {
      const std::string err = "LLT solve was degenerate at state " + std::to_string(k);

      JASSERT_EQ(Quu_llt.info(), Eigen::Success, err.c_str());
    }

    // Control policy is trivial
    lqr_soln[k].K = -Quu_llt.solve(Qux);
    lqr_soln[k].k = -Quu_llt.solve(Qu);

    Vx = Qx + (lqr_soln[k].K.transpose() * Quu_damped * lqr_soln[k].k) +
         (lqr_soln[k].K.transpose() * Qu) + (Qux.transpose() * lqr_soln[k].k);

    const StateHess Vxx_tmp = Qxx + (lqr_soln[k].K.transpose() * Quu * lqr_soln[k].K) +
                              (lqr_soln[k].K.transpose() * Qux) +
                              (Qux.transpose() * lqr_soln[k].K);
    Vxx = ((Vxx_tmp + Vxx_tmp.transpose()) * 0.5).eval();

    // Ugh: Should have have just done this the whole time
    Vxx = numerics::project_psd(Vxx, 0.1);
  }
  return lqr_soln;
}

}  // namespace planning
