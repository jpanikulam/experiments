#pragma once

#include "planning/problem.hh"

#include <functional>

namespace planning {

struct XlqrConfig {
  int max_iterations = 15;
  double min_mu_state = 0.1;
  double min_mu_ctrl = 0.1;
  double qxx_min_eigenvalue = 0.2;
  double quu_min_eigenvalue = 0.2;
};

template <typename _Prob>
class XlqrProblem {
  using Prob = _Prob;
  using ControlVec = typename Prob::ControlVec;
  using StateVec = typename Prob::StateVec;
  using State = typename Prob::State;
  using Derivatives = typename Prob::Derivatives;

  struct Damping {
    double mu_state = 10.0;
    double mu_ctrl = 10.0;
  };

 public:
  XlqrProblem(const Prob& prob,
              const typename Prob::CostDiffs& cost_diffs,
              const XlqrConfig& cfg = {})
      : prob_(prob), cost_diffs_(cost_diffs), cfg_(cfg) {
  }

  struct Solution {
    std::vector<State> x;
    StdVector<ControlVec> u;
    double cost = std::numeric_limits<double>::max();
  };

  using Visitor = std::function<void(const Solution& soln, int iteration, bool final)>;
  Solution solve(const State& x0,
                 const Solution& initialization = {},
                 const Visitor& visitor = {}) const;

 private:
  struct LqrFeedback {
    // Feedback
    MatNd<Prob::U_DIM, Prob::X_DIM> K = MatNd<Prob::U_DIM, Prob::X_DIM>::Zero();

    // Feed-forward
    ControlVec k = ControlVec::Zero();
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  using LqrSolution = std::vector<LqrFeedback>;

  double shoot(const Solution& soln,
               const LqrSolution& lqr_soln,
               double alpha,
               Solution* out_soln = nullptr) const;

  struct LineSearchResult {
    Solution soln;
    double best_alpha = -1.0;
    double best_cost = -1.0;
  };

  LineSearchResult line_search(const Solution& soln, const LqrSolution& lqr_soln) const;

  LqrSolution ricatti(const Solution& soln, const Damping& damping = {}) const;

  Prob prob_;
  typename Prob::CostDiffs cost_diffs_;
  XlqrConfig cfg_;
};

}  // namespace planning
