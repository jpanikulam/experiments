#include "numerics/optimize.hh"

#include <limits>

namespace numerics {
namespace {
OptimizationState compute_lagrange_update(const OptimizationState& current_state,
                                          const OptimizationProblem& problem) {
  return {};
}

struct LinesearchResult {
  bool valid = false;
  double cost = std::numeric_limits<double>::max();
  detail::Vecx best_x;
};

LinesearchResult line_search(const OptimizationState& current_state,
                             const detail::Vecx& direction,
                             const OptimizationProblem& problem) {
  detail::Vecx best_x = current_state.x;
  double best_cost_so_far = problem.objective(best_x, nullptr, nullptr);

  bool did_decrease = false;
  for (const double alpha : {0.001, 0.2, 0.5, 1.0, 5.0, 9.0, 25.0}) {
    const detail::Vecx evaluation_pt = current_state.x - (alpha * direction);
    const double cost_at_alpha = problem.objective(evaluation_pt, nullptr, nullptr);

    if (cost_at_alpha < best_cost_so_far) {
      best_cost_so_far = cost_at_alpha;
      best_x = evaluation_pt;
      did_decrease = true;
    }
  }
  const LinesearchResult result(
      {.valid = did_decrease, .cost = best_cost_so_far, .best_x = best_x});
  return result;
}

}  // namespace

template <>
OptimizationState optimize<ObjectiveMethod::kGradientDescent, ConstraintMethod::kAugLag>(
    const OptimizationState& initialization, const OptimizationProblem& problem) {
  OptimizationState iteration_state = initialization;

  constexpr int MAX_ITERS = 10000;
  for (int k = 0; k < MAX_ITERS; ++k) {
    detail::Vecx gradient;
    problem.objective(iteration_state.x, &gradient, nullptr);

    const auto lsr = line_search(iteration_state, gradient, problem);
    if (!lsr.valid) {
      // Could not minimize
      return iteration_state;
    }
    const double cost_ratio =
        (iteration_state.x - lsr.best_x).squaredNorm() / iteration_state.x.squaredNorm();

    iteration_state.x = lsr.best_x;
    constexpr double COST_CVG_RATIO = 1e-12;
    if (cost_ratio < COST_CVG_RATIO) {
      // Converged
      break;
    }
  }
  return iteration_state;
}
}  // namespace numerics
