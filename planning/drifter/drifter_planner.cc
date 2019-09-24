#include "planning/drifter/drifter_planner.hh"

#include "planning/costs/costs.hh"
#include "planning/costs/grid_cache.hh"
#include "planning/differentiation.hh"
#include "planning/drifter/drifter_xlqr.hh"
#include "planning/generic_planner.hh"

#include "util/clamp.hh"

#include "logging/assert.hh"
//
// TODO
#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

#include "geometry/spatial/bounding_box.hh"
#include "util/clamp.hh"
#include "util/lerp.hh"

namespace planning {
namespace drifter {

double soft_intersect(const double d1, const double d2, const double k) {
  const double h = jcc::clamp(0.5 - (0.5 * (d2 - d1) / k), 0.0, 1.0);
  return jcc::lerp(h, std::min(d2, d1), std::max(d2, d1)) + (k * h * (1.0 - h));
}

double sd_box(const jcc::Vec3& pt, const jcc::Vec3& box_pos, const jcc::Vec3& extents) {
  // constexpr double blend_factor = 5.0;

  double d_decided = 0.0;
  for (int k = 0; k < 2; ++k) {
    const jcc::Vec3 ehat_k = jcc::Vec3::Unit(k);
    // const double d_k_plus = pt.dot(ehat_k) - box_pos.dot(ehat_k) + extents[k];
    const double d_k_plus = (pt - box_pos).dot(ehat_k) + extents[k];
    // d_decided = soft_intersect(d_decided, d_k_plus, blend_factor);
    d_decided = std::max(d_decided, d_k_plus);
  }

  for (int k = 0; k < 2; ++k) {
    const jcc::Vec3 ehat_k = jcc::Vec3::Unit(k);
    const double d_k_minus = (pt - box_pos).dot(-ehat_k) - extents[k];
    // d_decided = soft_intersect(d_decided, d_k_minus, blend_factor);
    d_decided = std::max(d_decided, d_k_minus);
  }

  return d_decided;
}

DrifterProblem::Cost make_drifter_cost(const PlannerConfiguration& cfg,
                                       const BehaviorPrimitives& desires) {
  const std::vector<jcc::Vec2> points = {
      jcc::Vec2(-1000.0, 0.0),  //
      jcc::Vec2(0.0, 0.0),      //
      jcc::Vec2(1.0, 0.0),      //
      jcc::Vec2(2.0, 0.0),      //
      jcc::Vec2(1000.0, 0.0)    //
  };
  //

  // Lots of room for optimization here
  const auto fnc = [points](const jcc::Vec2& x) {
    double min_dist = 100.0;
    for (std::size_t k = 1; k < points.size(); ++k) {
      const jcc::Vec2 line_segment = points[k] - points[k - 1];
      const double length = line_segment.norm();
      const jcc::Vec2 nhat = line_segment / length;

      const double t = nhat.dot(x - points[k - 1]);
      const double t_clamped = jcc::clamp(t, 0.0, length);
      const double distance = (x - (points[k - 1] + (t_clamped * nhat))).norm();

      min_dist = std::min(min_dist, distance);
    }
    return min_dist;
  };

  struct Cache {
    GridCache2d curve_distance;
  };

  Cache caches;
  // TODO consider templating
  caches.curve_distance = GridCache2d(fnc);

  // caches.curve_distance.get(jcc::Vec2(1.2, 1.3));

  return [fnc, caches, cfg, desires](const State& state, const VecNd<U_DIM>& u, int t) {
    const auto control = Controls::from_vector(u);
    double cost = 0.0;

    //
    // Value Approximation
    //
    if (t >= HORIZON - 3) {
      const jcc::Vec3 error = state.x_world - desires.target;
      cost += cfg.goal.goal_weight * huber(error.norm(), 1.0);
      cost += cfg.goal.terminal_vel_weight * square(state.x_vel);
      cost += cfg.goal.terminal_vel_weight * square(state.phi);
    }

    //
    // Control
    //
    {
      const double a_weight = cfg.control.acceleration_weight;
      cost += a_weight * square(control.a);
      cost += a_weight * quad_hinge(std::abs(control.a), cfg.bounds.max_accel);
      cost += cfg.control.phidot_weight * square(control.phidot);
    }

    //
    // State Costs
    //
    {
      // Velocity bounds and cost
      cost += cfg.bounds.v_max_weight * quad_hinge(state.x_vel, desires.max_vel);
      cost += cfg.bounds.v_max_weight * quad_hinge(-state.x_vel, -cfg.bounds.min_speed);
      cost += cfg.bounds.v_max_weight * quad_hinge(state.x_vel, cfg.bounds.max_speed);

      // Phi bounds and cost
      cost += cfg.bounds.phi_weight * square(state.phi);
      cost += cfg.bounds.phi_max_bound_weight *
              quad_hinge(std::abs(state.phi), cfg.bounds.phi_max);
    }

    //
    // Pointing constraints
    //
    if (cfg.debug.enable_look_target) {
      const jcc::Vec3 world_sideways = state.R_world_from_body * jcc::Vec3::UnitY();
      const jcc::Vec3 world_look_dir = (desires.look_target - state.x_world).normalized();
      cost += cfg.goal.pointing_weight *
              quad_hinge((1.0 - world_sideways.dot(world_look_dir)), 0.0);
    }

    //
    // Obstacles & Friends
    //
    for (const auto& avoid : desires.avoids) {
      const double error = (state.x_world - avoid.pos_world).norm();
      // normalized_error should be M_PI when we're beyond the radius of influence
      const double max_error = M_PI;
      const double refactored_error = M_PI * error / (avoid.radius * 1.0);
      const double normalized_error = jcc::clamp(refactored_error, 0.0, max_error);
      cost += cfg.obstacles.avoid_weight * (1.0 + std::cos((normalized_error)));
    }

    if (cfg.goal.enable_path) {
      if (cfg.debug.convenience_flag) {
        cost += cfg.goal.path_weight *
                huber_hinge(fnc(state.x_world.head<2>()), cfg.goal.path_margin, 0.2);
      } else {
        cost += cfg.goal.path_weight *
                huber_hinge(caches.curve_distance(state.x_world.head<2>()),
                            cfg.goal.path_margin, 1.0);
      }
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
                               const PlannerConfiguration& cfg,
                               const std::vector<Controls>& initialization) {
  // if ((x0.x_world - desires.target).norm() < 0.01) {
  //   return {StateControl{x0, Controls{}}, StateControl{x0, Controls{}}};
  // }

  const DrifterProblem prob(make_drifter_cost(cfg, desires),
                            dynamics,
                            State::compute_delta,
                            State::apply_delta,
                            HORIZON,
                            DT);

  const XlqrConfig xlqr_cfg{.max_iterations = cfg.optimization.max_iterations,
                            .min_mu_state = cfg.optimization.min_state_damping,
                            .min_mu_ctrl = cfg.optimization.min_ctrl_damping};

  const DrifterXlqr planner(prob, make_generate_derivatives(prob), xlqr_cfg);

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
