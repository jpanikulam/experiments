#pragma once

#include "estimation/optimization/acausal_optimizer.hh"
#include "numerics/group_diff.hh"

namespace estimation {
namespace optimization {

template <typename Prob>
VecXd AcausalOptimizer<Prob>::add_observation_residual(const State& x,
                                                       const Measurement& z,
                                                       const Parameters& p,
                                                       int x_ind,
                                                       int residual_ind,
                                                       int param_ind,
                                                       Out<BlockSparseMatrix> bsm) {
  // y = (z[t] - h(x[t]; p))
  // J[obs_ind,  state_ind] = dy/dx[t] = -H
  // J[obs_ind, params_ind] = dy/dp    = -C

  const auto& model = models_.at(z.type);
  const auto y_of_x = [&model, &z, &p](const State& x) {
    // Hold p
    return model(x, z.observation, p);
  };
  const auto y_of_p = [&model, &z, &x](const Parameters& p) {
    // Hold x
    return model(x, z.observation, p);
  };

  const MatXd dy_dx = numerics::dynamic_group_jacobian<State>(x, y_of_x);
  const MatXd dy_dp = numerics::dynamic_group_jacobian<Parameters>(p, y_of_p);
  bsm->set(residual_ind, x_ind, dy_dx);
  bsm->set(residual_ind, param_ind, dy_dp);

  return model(x, z.observation, p);
}

template <typename Prob>
VecXd AcausalOptimizer<Prob>::add_dynamics_residual(const State& x_0,
                                                    const State& x_1,
                                                    const Parameters& p,
                                                    double dt,
                                                    int x_ind,
                                                    int residual_ind,
                                                    int param_ind,
                                                    Out<BlockSparseMatrix> bsm) {
  // y = (x[t+1] - f(x[t]; p)
  // J[obs_ind,          t] = dy/dx[t]   = -A
  // J[obs_ind, params_ind] = dy/dp      = -G
  // J[obs_ind,        t+1] = dy/dx[t+1] = I

  const auto& model = dynamics_;
  const auto y_of_x = [dt, &model, &x_1, &p](const State& x) {
    // Hold p, x_1
    return compute_delta(x_1, model(x, p, dt));
  };

  const auto y_of_p = [dt, &model, &x_0, &x_1](const Parameters& p) {
    // Hold x_0, x_1
    return compute_delta(x_1, model(x_0, p, dt));
  };

  const MatXd dy_dx = numerics::dynamic_group_jacobian<State>(x_0, y_of_x);
  const MatXd dy_dp = numerics::dynamic_group_jacobian<Parameters>(p, y_of_p);

  using StateToStateJac = MatNd<State::DIM, State::DIM>;
  const StateToStateJac I = StateToStateJac::Identity();

  bsm->set(residual_ind, x_ind, dy_dx);
  bsm->set(residual_ind, x_ind + 1, I);
  bsm->set(residual_ind, param_ind, dy_dp);

  return compute_delta(x_1, model(x_0, p, dt));
}

template <typename Prob>
BlockSparseMatrix AcausalOptimizer<Prob>::populate(const Solution& soln,
                                                   Out<std::vector<VecXd>> v) {
  assert(soln.x.size() == heap_.size());

  constexpr int n_params = 1;
  const int n_measurements = static_cast<int>(heap_.size());
  const int n_states = static_cast<int>(soln.x.size());
  const int n_residuals = (n_states - 1) + n_measurements;

  BlockSparseMatrix J(n_residuals, n_states + n_params);

  v->clear();
  v->resize(n_residuals);

  // Immediately after the states
  const int p_ind = n_states;

  const auto& p = soln.p;
  const auto& measurements = heap_.backing();
  for (int t = 0; t < static_cast<int>(measurements.size()); ++t) {
    const auto& z_t = measurements[t];

    const double dt =
        to_seconds(measurements[t + 1].time_of_validity - z_t.time_of_validity);
    // TODO: Do something special if dt == 0 (Share a measurement on a state?)

    // TODO: Divide information by dt
    const int x_ind = t;
    const int z_ind = 2 * t;
    const State& x_t = soln.x.at(t);
    const VecXd y_obs =
        add_observation_residual(x_t, z_t, p, x_ind, z_ind, p_ind, out(J));
    (*v)[z_ind] = y_obs;

    if (t < static_cast<int>(soln.x.size()) - 1) {
      const State& x_t1 = soln.x.at(t + 1);
      const VecXd y_dyn =
          add_dynamics_residual(x_t, x_t1, p, dt, x_ind, z_ind + 1, p_ind, out(J));
      (*v)[z_ind + 1] = y_dyn;
    }
  }

  return J;
}

template <typename Prob>
typename AcausalOptimizer<Prob>::Solution AcausalOptimizer<Prob>::solve(
    const Solution& initialization) {
  assert(initialization.x.size() == heap_.size());
  Solution soln = initialization;

  ///////////////////////////////////////////
  constexpr int n_params = 1;
  const int n_measurements = static_cast<int>(heap_.size());
  const int n_states = static_cast<int>(soln.x.size());
  const int n_residuals = (n_states - 1) + n_measurements;
  ///////////////////////////////////////////

  // Generate our jacobian
  std::vector<VecXd> v;
  const BlockSparseMatrix J_bsm = populate(soln, out(v));
  const VecXd delta = J_bsm.solve_lst_sq(v);
  std::cout << delta.rows() << std::endl;
  std::cout << soln.x.size() << ": " << soln.x.size() * State::DIM << std::endl;

  std::cout << "----" << std::endl;
  for (int t = 0; t < static_cast<int>(soln.x.size()); ++t) {
    std::cout << t << " : " << t * State::DIM << " -> " << (t * State::DIM) + State::DIM
              << std::endl;
    const VecXd poopy = delta.segment(t * State::DIM, State::DIM);
    const VecNd<State::DIM> sub_delta = poopy;
    apply_delta(soln.x[t], sub_delta);
  }

  std::cout << "Final: " << (soln.x.size() * State::DIM) << " -> "
            << (soln.x.size() * State::DIM) + Parameters::DIM << std::endl;
  const VecNd<Parameters::DIM> p_delta =
      delta.segment(soln.x.size() * State::DIM, Parameters::DIM);
  apply_delta(soln.p, p_delta);

  return soln;
}

}  // namespace optimization
}  // namespace estimation