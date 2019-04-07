#pragma once

#include "estimation/optimization/acausal_optimizer.hh"
#include "logging/assert.hh"
#include "numerics/group_diff.hh"

#include <algorithm>
#include <limits>
#include <memory>

namespace estimation {
namespace optimization {

template <typename Prob>
typename AcausalOptimizer<Prob>::DynamicsDifferentials AcausalOptimizer<
    Prob>::dynamics_diff(const State& x, const Parameters& p, const double dt) const {
  const auto& model = dynamics_;
  const auto f_of_x = [dt, &model, &p](const State& x) {
    // Hold p, x_1
    return model(x, p, dt);
  };

  const auto f_of_p = [dt, &model, &x](const Parameters& p) {
    // Hold x_0, x_1
    return model(x, p, dt);
  };

  const MatXd df_dx = numerics::group_jacobian<State, State>(x, f_of_x);
  const MatXd df_dp = numerics::group_jacobian<Parameters, State>(p, f_of_p);
  return {df_dx, df_dp};
}

template <typename Prob>
typename AcausalOptimizer<Prob>::StateForTime AcausalOptimizer<Prob>::get_state_for_time(
    const Solution& soln, const TimePoint& t) const {
  const auto cmp = [](const TimePoint target, const StateObservation& b) {
    return target < b.time_of_validity;
  };
  const auto upper_bound = std::upper_bound(soln.x.begin(), soln.x.end(), t, cmp);
  const auto state_iter = upper_bound - 1u;
  const int x_ind = static_cast<int>(std::distance(soln.x.begin(), state_iter));

  const auto state_obs = *state_iter;

  const double dt = to_seconds(t - state_obs.time_of_validity);
  JASSERT_GE(dt, 0.0, "Time must flow forward");

  return {x_ind, state_obs.x, dt};
}

template <typename Prob>
LinearSystem AcausalOptimizer<Prob>::populate(const Solution& soln) const {
  const auto measurements = heap_.to_sorted_vector();

  const double total_dt = to_seconds(measurements.back().time_of_validity -
                                     measurements.front().time_of_validity);
  const int n_params = 1;
  // const int n_states = static_cast<int>(std::ceil(total_dt / MAX_DT));
  const int n_states = soln.x.size();
  const int n_measurements = static_cast<int>(heap_.size());
  const int n_residuals = (n_states - 1) + n_measurements;

  BlockSparseMatrix J(n_residuals, n_states + n_params);
  BlockSparseMatrix R_inv(n_residuals, n_residuals);

  std::vector<VecXd> v;
  v.resize(n_residuals);

  // Immediately after the states
  const int p_ind = n_states;

  const auto& p = soln.p;
  for (int z_ind = 0; z_ind < static_cast<int>(measurements.size()); ++z_ind) {
    const auto& z_t = measurements.at(z_ind);

    const StateForTime x_for_t = get_state_for_time(soln, z_t.time_of_validity);
    const State x_t = dynamics_(x_for_t.x, p, x_for_t.dt);
    const DynamicsDifferentials dyn_diff = dynamics_diff(x_for_t.x, p, x_for_t.dt);
    const MeasurementDifferentials y_obs = add_observation_residual(x_t, z_t, p);
    v[z_ind] = y_obs.residual;

    /*
        std::cout << "\n" << std::endl;
        std::cout << "---- type:" << z_t.type << std::endl;
        std::cout << "dt: " << x_for_t.dt << std::endl;
        std::cout << "z_ind: " << z_ind << std::endl;
        std::cout << "x_ind: " << x_for_t.x_ind << std::endl;
        std::cout << "y_obs.dh_dx: " << y_obs.dh_dx.rows() << ", " << y_obs.dh_dx.cols()
                  << std::endl;
        std::cout << "dyn_diff.df_dx: " << dyn_diff.df_dx.rows() << ", "
                  << dyn_diff.df_dx.cols() << std::endl;
    */

    const MatXd dh_dx = y_obs.dh_dx * dyn_diff.df_dx;
    const MatXd dh_dp = y_obs.dh_dp + (y_obs.dh_dx * dyn_diff.df_dp);
    J.set(z_ind, x_for_t.x_ind, dh_dx);
    J.set(z_ind, p_ind, dh_dp);

    const MatXd& cov = covariances_.at(z_t.type);
    R_inv.set(z_ind, z_ind, cov.inverse());
  }

  JASSERT_GT(soln.x.size(), 2u, "Must have at least 2 elements in the proposed solution");
  for (int x_ind = 0; x_ind < static_cast<int>(soln.x.size() - 1); ++x_ind) {
    const StateObservation& state_obs = soln.x.at(x_ind);
    const StateObservation& next_state_obs = soln.x.at(x_ind + 1);

    const State& x_t = state_obs.x;
    const State& x_t1 = next_state_obs.x;
    const double dt =
        to_seconds(next_state_obs.time_of_validity - state_obs.time_of_validity);

    JASSERT_GT(dt, 0.0, "Time must be strictly positive");
    // Arbitrary constant that is surprising enough to indicate a bug
    JASSERT_LT(dt, MAX_DT, "Time must be less than max passed time");

    // This doesn't yield great fill-in
    const int z_obs_ind = x_ind + n_measurements;
    const VecXd y_dyn =
        add_dynamics_residual(x_t, x_t1, p, dt, x_ind, z_obs_ind, p_ind, out(J));
    v[z_obs_ind] = y_dyn;

    const double inv_dt = 1.0 / dt;
    R_inv.set(z_obs_ind, z_obs_ind, dyn_info_ * inv_dt);
  }

  LinearSystem system;
  system.v = v;
  system.J = J;
  system.R_inv = R_inv;
  return system;
}

template <typename Prob>
typename AcausalOptimizer<Prob>::MeasurementDifferentials AcausalOptimizer<
    Prob>::add_observation_residual(const State& x,
                                    const Measurement& z,
                                    const Parameters& p) const {
  // y = (z[t] - h(x[t]; p))
  // J[obs_ind,  state_ind] = dy/dx[t] = H
  // J[obs_ind, params_ind] = dy/dp    = C

  const auto& model = models_.at(z.type);
  const auto v_of_x = [&model, &z, &p](const State& x) {
    // Hold p
    return model.error(x, z.observation, p);
  };
  const auto v_of_p = [&model, &z, &x](const Parameters& p) {
    // Hold x
    return model.error(x, z.observation, p);
  };

  const MatXd dv_dx = numerics::dynamic_group_jacobian<State>(x, v_of_x);
  const MatXd dv_dp = numerics::dynamic_group_jacobian<Parameters>(p, v_of_p);
  // bsm->set(residual_ind, x_ind, dy_dx);
  // bsm->set(residual_ind, param_ind, dy_dp);

  return {model.error(x, z.observation, p), -dv_dx, -dv_dp};
}

template <typename Prob>
VecXd AcausalOptimizer<Prob>::add_dynamics_residual(const State& x_0,
                                                    const State& x_1,
                                                    const Parameters& p,
                                                    double dt,
                                                    int x_ind,
                                                    int residual_ind,
                                                    int param_ind,
                                                    Out<BlockSparseMatrix> bsm) const {
  // y = (x[t+1] - f(x[t]; p)
  // J[obs_ind,          t] = -dv/dx[t]   = A
  // J[obs_ind, params_ind] = -dv/dp      = G
  // J[obs_ind,        t+1] = -dv/dx[t+1] = -I

  const auto& model = dynamics_;
  const auto v_of_x = [dt, &model, &x_1, &p](const State& x) {
    // Hold p, x_1
    return State::compute_delta(x_1, model(x, p, dt));
  };

  const auto v_of_p = [dt, &model, &x_0, &x_1](const Parameters& p) {
    // Hold x_0, x_1
    return State::compute_delta(x_1, model(x_0, p, dt));
  };

  // TODO ELIMINATE
  const MatXd dv_dx = -numerics::dynamic_group_jacobian<State>(x_0, v_of_x);
  const MatXd dv_dp = -numerics::dynamic_group_jacobian<Parameters>(p, v_of_p);

  using StateToStateJac = MatNd<State::DIM, State::DIM>;
  const StateToStateJac dv_d_x1 = -StateToStateJac::Identity();

  bsm->set(residual_ind, x_ind, dv_dx);
  bsm->set(residual_ind, x_ind + 1, dv_d_x1);
  bsm->set(residual_ind, param_ind, dv_dp);

  return State::compute_delta(x_1, model(x_0, p, dt));
}

template <typename Prob>
typename AcausalOptimizer<Prob>::Solution AcausalOptimizer<Prob>::update_solution(
    const Solution& prev_soln, const VecXd& delta) const {
  constexpr int n_params = 1;
  const int n_states = static_cast<int>(prev_soln.x.size());
  Solution updated_soln;
  updated_soln.x.resize(n_states);

  JASSERT_EQ(delta.rows(),
             (n_states * State::DIM) + (n_params * Parameters::DIM),
             "Delta must have the correct dimension");
  for (int t = 0; t < static_cast<int>(prev_soln.x.size()); ++t) {
    const VecNd<State::DIM> sub_delta = delta.segment(t * State::DIM, State::DIM);
    updated_soln.x[t].x = State::apply_delta(prev_soln.x.at(t).x, sub_delta);
    updated_soln.x[t].time_of_validity = prev_soln.x.at(t).time_of_validity;
  }
  const VecNd<Parameters::DIM> p_delta =
      delta.segment(n_states * State::DIM, Parameters::DIM);

  updated_soln.p = Parameters::apply_delta(prev_soln.p, p_delta);
  return updated_soln;
}

template <typename Prob>
double AcausalOptimizer<Prob>::cost(const LinearSystem& system,
                                    const slam::RobustEstimator& rcost,
                                    std::vector<double>* weights) const {
  double cost = 0.0;
  if (weights) {
    weights->clear();
    weights->resize(system.v.size(), 1.0);
  }

  for (int k = 0; k < static_cast<int>(system.v.size()); ++k) {
    const VecXd& sub_v = system.v.at(k);
    const double error = sub_v.dot(system.R_inv.get(k, k) * sub_v);
    const auto cost_weight = rcost.cost_weight(error);
    cost += cost_weight.cost;
  }

  JASSERT_GT(cost, 0.0, "Cost must be positive");

  return cost;
}

namespace {
std::shared_ptr<slam::RobustEstimator> cost_schedule(int k) {
  return std::make_shared<slam::L2Cost>();
}
}  // namespace

template <typename Prob>
typename AcausalOptimizer<Prob>::Solution AcausalOptimizer<Prob>::solve(
    const Solution& initialization, const Visitor& visitor) const {
  JASSERT(numerics::is_pd(dyn_info_), "Dynamics information matrix must be PD");

  Solution soln = initialization;

  // Generate our jacobian

  constexpr double LAMBDA_UP_FACTOR = 10.0;
  constexpr double LAMBDA_DOWN_FACTOR = 0.5;
  constexpr double LAMBDA_DOWN_LITE_FACTOR = 0.75;
  constexpr double LAMBDA_INITIAL = 0.1;
  constexpr double DECREASE_RATIO_FOR_DAMPING_DOWN = 0.7;
  constexpr double MAX_LAMBDA = 1e6;
  constexpr double MIN_LAMBDA = 1e-9;

  LinearSystem current_system = populate(soln);
  double prev_cost = cost(current_system, *cost_schedule(0));
  double lambda = LAMBDA_INITIAL;

  for (int k = 0; k < 1000; ++k) {
    if (visitor) {
      visitor(soln);
    }
    if (lambda > MAX_LAMBDA) {
      std::cout << "\tConverging (Lambda)" << std::endl;
      break;
    }

    std::cout << "\n-------" << std::endl;
    current_system = populate(soln);

    const auto rcost = cost_schedule(k);
    std::cout << "Cost at [" << k << "]: " << cost(current_system, *rcost) << std::endl;
    // {
    //   std::vector<double> weights;
    //   std::cout << "Cost at [" << k << "]: " << cost(current_system, *rcost,
    //   &weights)
    //             << std::endl;
    //   for (int j = 0; j < current_system.R_inv.block_cols(); ++j) {
    //     const Eigen::MatrixXd old = current_system.R_inv.get(j, j);
    //     const double weight = weights.at(j);
    //     current_system.R_inv.set(j, j, old * weight);
    //   }
    // }

    const auto maybe_delta =
        current_system.J.solve_lst_sq(current_system.v, current_system.R_inv, lambda);

    if (!maybe_delta) {
      lambda *= LAMBDA_UP_FACTOR;
      std::cout << "Numerical issue: Increasing lambda to " << lambda << std::endl;
      continue;
    }

    const VecXd delta = *maybe_delta;
    const auto new_soln = update_solution(soln, delta);
    const auto new_system = populate(new_soln);
    const double new_cost = cost(new_system, *rcost);

    std::cout << "\tPossible cost: " << new_cost << std::endl;
    if (new_cost > prev_cost) {
      lambda *= LAMBDA_UP_FACTOR;
      std::cout << "\tIncreasing damping to " << lambda << std::endl;
      continue;
    } else if (new_cost < DECREASE_RATIO_FOR_DAMPING_DOWN * prev_cost) {
      lambda *= LAMBDA_DOWN_FACTOR;
      lambda = std::max(lambda, MIN_LAMBDA);
      std::cout << "\tDecreasing damping to " << lambda << std::endl;
    } else {
      lambda *= LAMBDA_DOWN_LITE_FACTOR;
      lambda = std::max(lambda, MIN_LAMBDA);
      std::cout << "\tDecreasing damping to " << lambda << std::endl;
    }

    prev_cost = new_cost;
    current_system = new_system;
    soln = new_soln;
  }
  {
    std::vector<VecXd> v_final;
    const LinearSystem system_final = populate(soln);
    std::cout << "Cost Final: " << cost(system_final, *cost_schedule(1500)) << std::endl;
    if (visitor) {
      visitor(soln);
    }
  }

  return soln;
}

}  // namespace optimization
}  // namespace estimation
