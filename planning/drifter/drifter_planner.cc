#include "planning/drifter/drifter_planner.hh"

#include "planning/costs/costs.hh"
#include "planning/differentiation.hh"
#include "planning/drifter/drifter_xlqr.hh"
#include "planning/generic_planner.hh"

#include "util/clamp.hh"

#include "logging/assert.hh"
//
// TODO
#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

namespace planning {
namespace drifter {

DrifterProblem::Cost make_drifter_cost(const BehaviorPrimitives& desires) {
  return [desires](const State& state, const VecNd<U_DIM>& u, int t) {
    const auto control = Controls::from_vector(u);
    double cost = 0.0;

    cost += 1.0 * square(control.a);
    cost += 1.0 * square(control.phidot);

    if (t >= HORIZON - 3) {
      const jcc::Vec3 error = state.x_world - desires.target;
      cost += 1.0 * huber(error.norm(), 1.0);
      cost += 10.0 * square(state.x_vel);
    }

    cost += 50.0 * quad_hinge(state.x_vel, 1.5);
    // cost += 50.0 * square(state.x_vel);
    // if(state.x_vel > 0.4) {
    // cost += 50.0 * square(state.x_vel - 0.4);
    // }

    cost += square(state.phi);

    for (const auto& avoid : desires.avoids) {
      const double error = (state.x_world - avoid.pos_world).norm();

      // normalized_error should be M_PI when we're beyond the radius of influence

      const double max_error = M_PI;
      const double refactored_error = M_PI * error / (avoid.radius * 1.0);
      const double normalized_error = jcc::clamp(refactored_error, 0.0, max_error);

      cost += 3.0 * (1.0 + std::cos((normalized_error)));
    }

    return cost;
  };
}

Parameters get_parameters() {
  Parameters params;
  return params;
}

State dynamics(const State& state, const VecNd<U_DIM>& u, const double dt) {
  const Parameters params = get_parameters();
  return rk4_integrate(state, Controls::from_vector(u), params, dt);
}

auto make_generate_derivatives(const Problem<DrifterDim, State>& problem) {
  return [problem](const State& state,
                   const VecNd<U_DIM>& u,
                   int t) -> GenericDerivatives<DrifterDim> {
    const Differentiator diff(problem);

    GenericDerivatives<DrifterDim> derivatives;
    derivatives.A = diff.state_jacobian(state, u);
    derivatives.B = diff.control_jacobian(state, u);

    derivatives.Q = diff.state_hessian(state, u, t);
    derivatives.R = diff.control_hessian(state, u, t);

    derivatives.g_x = diff.state_gradient(state, u, t);
    derivatives.g_u = diff.control_gradient(state, u, t);

    return derivatives;
  };
}

std::vector<StateControl> plan(const State& x0,
                               const BehaviorPrimitives& desires,
                               const std::vector<Controls>& initialization) {
  // if ((x0.x_world - desires.target).norm() < 0.01) {
  //   return {StateControl{x0, Controls{}}, StateControl{x0, Controls{}}};
  // }

  const DrifterProblem prob(make_drifter_cost(desires),
                            dynamics,
                            State::compute_delta,
                            State::apply_delta,
                            HORIZON,
                            DT);
  const DrifterXlqr planner(prob, make_generate_derivatives(prob));

  // const auto pre = planner.solve(x0, {});

  DrifterXlqr::Solution auto_init;
  auto x = x0;
  for (int k = 0; k < HORIZON - 1; ++k) {
    auto_init.x.push_back(x);

    Controls uu;
    uu.a = 0.0;
    const VecNd<Controls::DIM> u = Controls::to_vector(uu);
    auto_init.u.push_back(u);
    x = dynamics(x, u, DT);
  }
  auto_init.x.push_back(x);

  // TODO TODO TODO
  DrifterXlqr::Solution soln;
  try {
    const auto pre = planner.solve(x0, auto_init);
    soln = pre;
  } catch (const jcc::JccException& exc) {
    return {StateControl{x0, Controls{}}, StateControl{x0, Controls{}}};
  }

  std::vector<StateControl> result;
  for (int k = 0; k < HORIZON; ++k) {
    result.push_back({soln.x[k], Controls::from_vector(soln.u[k])});
  }

  return result;
}
}  // namespace drifter
}  // namespace planning
