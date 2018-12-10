#include "planning/jet/jet_planner.hh"

#include "numerics/optimize.hh"
#include "numerics/wrap_optimizer.hh"

#include "planning/differentiation.hh"
#include "planning/problem.hh"

#include "planning/xlqr_problem.hh"
#include "planning/xlqr_problem_impl.hh"

#include "planning/generic_planner.hh"

namespace planning {
namespace jet {
constexpr int X_DIM = 13;
constexpr int U_DIM = 4;

constexpr double DT = 0.1;
constexpr int HORIZON = 15;

double jet_cost(const State& state, const VecNd<U_DIM>& u, int t) {
  const auto control = from_vector(u);
  double cost = 0.0;

  {
    cost += control.q.squaredNorm();
    cost += control.throttle_dot * control.throttle_dot;
  }
  {
    const jcc::Vec3 attitude = state.R_world_from_body * jcc::Vec3::UnitZ();
    if (t >= 13) {
      const double sin_error = attitude.cross(jcc::Vec3::UnitZ()).squaredNorm();
      // cost += 30.0 * sin_error * sin_error;
      // cost += 10.0 * state.x.squaredNorm();
      cost += 100.0 * state.x.z() * state.x.z();
      // cost += 150.0 * state.v.squaredNorm();
    }
    cost += 25.0 * state.v.squaredNorm();
    cost += 10.0 * state.w.squaredNorm();
  }
  return cost;
}

State dynamics(const State& state, const VecNd<U_DIM>& u, const double dt) {
  Parameters params;
  params.mass = 100.0;
  params.unit_z = jcc::Vec3::UnitZ();
  params.external_force = -jcc::Vec3::UnitZ() * 1.0;

  return rk4_integrate(state, from_vector(u), params, dt);
}

using JetDim = Dimensions<X_DIM, U_DIM>;

GenericDerivatives<JetDim> generate_derivatives(const State& state,
                                                const VecNd<U_DIM>& u,
                                                int t) {
  const Problem<JetDim, State> prob(
      jet_cost, dynamics, delta_vec, apply_delta, HORIZON, DT);
  const Differentiator diff(prob);
  const auto A = diff.state_jacobian(state, u);
  const auto B = diff.control_jacobian(state, u);

  GenericDerivatives<JetDim> derivatives;
  derivatives.A = A;
  derivatives.B = B;

  derivatives.Q = diff.state_hessian(state, u, t);
  derivatives.R = diff.control_hessian(state, u, t);

  derivatives.g_x = diff.state_gradient(state, u, t);
  derivatives.g_u = diff.control_gradient(state, u, t);

  return derivatives;
}

std::vector<StateControl> plan_generic(const State& x0,
                                       const std::vector<Controls>& initialization) {
  const GenericPlanner<State, U_DIM> planner(dynamics, jet_cost);

  std::vector<VecNd<U_DIM>> initialization_vec;
  for (const auto& control : initialization) {
    initialization_vec.push_back(to_vector(control));
  }

  const auto pre_result = planner.plan(x0, initialization_vec);
  std::vector<StateControl> result;
  for (const auto& pre : pre_result) {
    result.push_back({pre.state, from_vector(pre.control)});
  }

  return result;
}

std::vector<StateControl> plan(const State& x0,
                               const std::vector<Controls>& initialization) {
  using JetProblem = Problem<JetDim, State>;
  const JetProblem prob(jet_cost, dynamics, delta_vec, apply_delta, HORIZON, DT);
  const XlqrProblem<JetProblem> planner(prob, generate_derivatives);

  // std::vector<VecNd<U_DIM>> initialization_vec;
  // for (const auto& control : initialization) {
  //   initialization_vec.push_back(to_vector(control));
  // }

  const auto pre = planner.solve(x0);
  std::vector<StateControl> result;
  // for (const auto& pre : pre_result) {
  for (int k = 0; k < HORIZON; ++k) {
    result.push_back({pre.x[k], from_vector(pre.u[k])});
  }

  return result;
}

}  // namespace jet
}  // namespace planning