#include "planning/joint_planner.hh"

#include "numerics/wrap_optimizer.hh"

namespace planning {

VecX JointPlanner::form_state() const {
  const int n_joints = body_.joints().size();
  VecX x = VecX::Zero(n_joints * X_SIZE);

  for (const auto& joint : body_.joints()) {
    x[angle_ind(joint.first)] = joint.second.angle;
    x[velocity_ind(joint.first)] = joint.second.velocity;
  }
  return x;
}

Body JointPlanner::form_body(const VecX& u,
                             const JointPlanner::Dynamics& dyn,
                             const int t_desired) const {
  Body body = body_;
  const int n_joints = body_.joints().size();

  const VecX x0 = form_state();
  VecX xt = x0;

  for (int t = 0; t < std::min(HORIZON, t_desired); ++t) {
    const VecX ut = u.segment(t, (U_SIZE * n_joints));
    xt = dyn(xt, ut);
  }

  for (const auto& joint : body_.joints()) {
    body.joint(joint.first).angle = xt[angle_ind(joint.first)];
    body.joint(joint.first).velocity = xt[velocity_ind(joint.first)];
  }
  return body;
}

numerics::OptimizationProblem JointPlanner::build_optimization_problem(
    const JointPlanner::PlanningProblem& planning_problem) const {
  // Forward shooting
  const auto opt_cost = [this, planning_problem](const VecX& u) {
    double total_cost = 0.0;

    const int n_joints = body_.joints().size();

    const VecX x0 = form_state();
    VecX xt = x0;

    for (int t = 0; t < HORIZON; ++t) {
      const VecX ut = u.segment(t, (U_SIZE * n_joints));
      xt = planning_problem.dynamics(xt, ut);
      total_cost += planning_problem.cost(xt, ut);
    }
    return total_cost;
  };

  const auto wrapped_cost = numerics::wrap_numerical_grad(opt_cost);

  const numerics::OptimizationProblem problem{wrapped_cost};
  return problem;
}

VecX JointPlanner::optimize(const numerics::OptimizationProblem& problem) const {
  const int n_joints = body_.joints().size();
  const VecX u_init = VecX::Zero(HORIZON * U_SIZE * n_joints);
  const numerics::OptimizationState initialization{u_init};

  const auto result =
      numerics::optimize<numerics::ObjectiveMethod::kGradientDescent,
                         numerics::ConstraintMethod::kAugLag>(initialization, problem);
  return result.x;
}

JointPlanner::PlanningProblem JointPlanner::generate_opt_funcs() const {
  const Vec3 target(1.0, 1.0, 1.0);
  const int target_joint = 2;

  constexpr double dt = 0.1;
  const auto dynamics = [this, dt](const VecX& xt, const VecX& ut) {
    VecX xtp1 = xt;
    for (const auto& joint : body_.joints()) {
      const int joint_id = joint.first;
      xtp1[angle_ind(joint_id)] += dt * xt[velocity_ind(joint_id)];
      xtp1[velocity_ind(joint_id)] += dt * ut[accel_ind(joint_id)];
    }
    return xtp1;
  };

  const auto per_state_cost = [this, target, target_joint](const VecX& xt,
                                                           const VecX& ut) {
    double cost = 0.0;
    for (const auto& joint : body_.joints()) {
      const int joint_id = joint.first;
      const double angle = xt[angle_ind(joint_id)];
      const double vel = xt[velocity_ind(joint_id)];
      const double accel = ut[accel_ind(joint_id)];

      const double angle_cost = (angle * angle);
      const double vel_cost = vel * vel;
      const double ctrl_cost = accel * accel;
      cost += ((angle_cost) + (0.01 * vel_cost) + (0.01 * ctrl_cost));
    }
    return cost;
  };

  return {per_state_cost, dynamics};
}

}  // namespace planning