#include "planning/jet/jet_planner.hh"

#include "numerics/optimize.hh"
#include "numerics/wrap_optimizer.hh"

#include "planning/generic_planner.hh"

namespace planning {
namespace jet {
constexpr int U_DIM = 4;

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
      cost += 30.0 * sin_error * sin_error;
      // cost += 10.0 * state.x.squaredNorm();
      cost += 100.0 * state.x.z() * state.x.z();
      cost += 150.0 * state.v.squaredNorm();
    }
    cost += 25.0 * state.v.squaredNorm();
    cost += 1.0 * state.w.squaredNorm();
  }
  return cost;
}

State dynamics(const State& state, const VecNd<U_DIM>& u, const double dt) {
  Parameters params;
  params.mass = 100.0;
  params.unit_z = jcc::Vec3::UnitZ();
  // params.external_force = -jcc::Vec3::UnitZ() * 0.01;

  return rk4_integrate(state, from_vector(u), params, dt);
}

std::vector<StateControl> plan(const State& x0,
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

}  // namespace jet
}  // namespace planning