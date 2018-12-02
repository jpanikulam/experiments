#pragma once

#include "eigen.hh"
#include "sophus.hh"

#include "numerics/optimize.hh"
#include "numerics/wrap_optimizer.hh"

namespace planning {

namespace {
using Vec3 = Eigen::Vector3d;
using VecX = Eigen::VectorXd;

}  // namespace

// Concept: Dynamics
// Dynamics must be a function, dynamics of the form
// State <- dynamics(State, Vector(U_DIM), dt);
//
template <typename State, int U_DIM>
class GenericPlanner {
 public:
  using Cost = std::function<double(const State&, const VecNd<U_DIM>& u, int t)>;
  using Dynamics =
      std::function<State(const State&, const VecNd<U_DIM>& u, const double dt)>;

  GenericPlanner(const Dynamics& dynamics, const Cost& cost)
      : dynamics_(dynamics), cost_(cost) {
  }

  static constexpr int HORIZON = 5;

  //
  // Form the problem
  //

  std::vector<State> plan(const State& x0) const {
    const auto opt_problem = build_optimization_problem(x0);
    const VecX soln = optimize(opt_problem);
    std::vector<State> full_soln;

    {  // TODO: Factor this out
      State xt = x0;
      constexpr double dt = 0.1;
      for (int t = 0; t < HORIZON; ++t) {
        const VecNd<U_DIM> ut = soln.segment(t, U_DIM);
        full_soln.push_back(xt);
        xt = dynamics_(xt, ut, dt);
      }
    }
    return full_soln;
  }

 private:
  numerics::OptimizationProblem build_optimization_problem(const State& x0) const {
    // Forward shooting
    const auto opt_cost = [this, x0](const VecX& u) {
      double total_cost = 0.0;
      State xt = x0;

      constexpr double dt = 0.1;
      for (int t = 0; t < HORIZON; ++t) {
        const VecNd<U_DIM> ut = u.segment(t, U_DIM);
        xt = dynamics_(xt, ut, dt);
        total_cost += cost_(xt, ut, t);
      }
      return total_cost;
    };

    const auto wrapped_cost = numerics::wrap_numerical_grad(opt_cost);

    const numerics::OptimizationProblem problem{wrapped_cost};
    return problem;
  }

  VecX optimize(const numerics::OptimizationProblem& problem) const {
    // TODO: Better initialization!
    const VecX u_init = VecX::Zero(HORIZON * U_DIM);
    const numerics::OptimizationState initialization{u_init};

    const auto result =
        numerics::optimize<numerics::ObjectiveMethod::kGradientDescent,
                           numerics::ConstraintMethod::kAugLag>(initialization, problem);
    return result.x;
  }

  const Dynamics dynamics_;
  const Cost cost_;
};

}  // namespace planning

// #include "planning/generic_planner.hh"
