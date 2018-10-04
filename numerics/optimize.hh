#pragma once

#include "eigen.hh"

#include "numerics/cost_function.hh"

#include <functional>
#include <vector>

namespace numerics {

namespace detail {
using Vecx = Eigen::VectorXd;
using Matx = Eigen::MatrixXd;
}  // namespace detail

struct BoxConstraint {
  // If max/min are equal to double precision, this will be treated
  // as an equality constraint, which is a bit more stable than two inequality
  // constraints
  int dimension = -1;
  double max = 0.0;
  double min = 0.0;
};

// Implemented as regularized lagrangian
struct Constraint {
  CostFunction g;
  bool mirrored = false;
};

struct OptimizationProblem {
  CostFunction objective;
  std::vector<Constraint> func_constraints;
  std::vector<BoxConstraint> box_constraints;
};

enum class ObjectiveMethod : uint8_t {
  kNewton = 0,
  kGaussNewton = 1,
  kGradientDescent = 2,
};

enum class ConstraintMethod : uint8_t {
  kAugLag = 0,
  kPenalty = 1,
};

struct OptimizationState {
  detail::Vecx x;
  detail::Vecx lambda;
};

template <ObjectiveMethod OBJECTIVE_METHOD, ConstraintMethod CONSTRAINT_METHOD>
OptimizationState optimize(const OptimizationState& initialization,
                           const OptimizationProblem& problem);
}  // namespace numerics
