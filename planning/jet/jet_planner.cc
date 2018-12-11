#include "planning/jet/jet_planner.hh"

#include "planning/differentiation.hh"
#include "planning/generic_planner.hh"
#include "planning/jet/jet_xlqr.hh"

namespace planning {
namespace jet {

double huber(double x, double k) {
  constexpr double half = 0.5;
  if (x < k) {
    return (x * x) * half;
  } else {
    return k * (std::abs(x) - (k * half));
  }
}

double jet_cost(const State& state, const VecNd<U_DIM>& u, int t) {
  const auto control = from_vector(u);
  double cost = 0.0;

  {
    cost += 25.0 * control.q.squaredNorm();
    cost += control.throttle_dot * control.throttle_dot;
  }
  {
    if (t >= 14) {
      // const jcc::Vec3 attitude = state.R_world_from_body * jcc::Vec3::UnitZ();
      // const double sin_error = attitude.cross(jcc::Vec3::UnitZ()).squaredNorm();
      // cost += 14.0 * sin_error * sin_error;

      const jcc::Vec3 target_pos(0.0, 0.0, 3.0);
      const jcc::Vec3 error = state.x - target_pos;

      // cost += 10.0 * error.squaredNorm();
      // cost += 100.0 * error.z() * error.z();

      cost += 10.0 * huber(error.norm(), 5.0);
      cost += 100.0 * huber(error.z(), 5.0);

      cost += 25.0 * state.v.squaredNorm();
      // cost += 150.0 * state.v.squaredNorm();
    }
    cost += 5.0 * state.v.squaredNorm();
    cost += 10.0 * state.w.squaredNorm();
    cost += state.throttle_pct * state.throttle_pct;
    cost += 25.0 * std::pow(std::max(state.throttle_pct, 10.0), 2);
    cost += 100.0 * std::pow(std::min(state.throttle_pct, 0.0), 2);
  }
  return cost;
}

State dynamics(const State& state, const VecNd<U_DIM>& u, const double dt) {
  Parameters params;
  params.mass = 100.0;
  params.unit_z = jcc::Vec3::UnitZ();
  params.external_force = -jcc::Vec3::UnitZ() * 5.0;

  return rk4_integrate(state, from_vector(u), params, dt);
}

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

std::vector<StateControl> plan(const State& x0,
                               const std::vector<Controls>& initialization) {
  const JetProblem prob(jet_cost, dynamics, delta_vec, apply_delta, HORIZON, DT);
  const JetXlqr planner(prob, generate_derivatives);

  const auto pre = planner.solve(x0);
  std::vector<StateControl> result;
  for (int k = 0; k < HORIZON; ++k) {
    result.push_back({pre.x[k], from_vector(pre.u[k])});
  }

  return result;
}

}  // namespace jet
}  // namespace planning