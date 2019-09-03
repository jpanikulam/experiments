#pragma once

#include "planning/problem.hh"

#include <functional>

namespace planning {

template <typename _Prob>
class XlqrProblem {
  using Prob = _Prob;
  using ControlVec = typename Prob::ControlVec;
  using StateVec = typename Prob::StateVec;
  using State = typename Prob::State;
  using Derivatives = typename Prob::Derivatives;

 public:
  XlqrProblem(const Prob& prob, const typename Prob::CostDiffs& cost_diffs)
      : prob_(prob), cost_diffs_(cost_diffs) {
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

  Solution line_search(const Solution& soln, const LqrSolution& lqr_soln) const;

  LqrSolution ricatti(const Solution& soln) const;

  Prob prob_;
  typename Prob::CostDiffs cost_diffs_;
};

}  // namespace planning
