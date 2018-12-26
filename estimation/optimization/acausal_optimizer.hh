#pragma once

#include <map>

#include "out.hh"

namespace estimation {
namespace optimization {

template <typename _State, typename _Parameters>
struct ProblemType {
  using State = _State;
  using Parameters = _Parameters;
};

template <typename Prob>
struct Solution {
  std::vector<Prob::State> x;
  Prob::Parameters p;
};

template <typename Prob>
class AcausalOptimizer {
 public:
  struct Measurement {
    int type = -1;
    std::any observation;
    TimePoint time_of_validity;
  };

  using Dynamics = std::function<State(const State&, const Parameters&, double dt)>;

  template <typename Observation>
  using ErrorModel = std::function<VecNd<Observation::DIM>(
      const Prob::State&, const Observation&, const Prob::Parameters&)>;

  using TypelessErrorModel = std::function<VecXd(
      const Prob::State& x, const std::any& z, const Prob::Parameters& p)>;

  template <typename Observation>
  int add_error_model(const ErrorModel<Observation>& model) {
    int model_id = models_.size();
    const auto type_erased_model = [model](const Prob::State& x, const std::any& z,
                                           const Prob::Parameters& p) {
      const VecNd<Observation::DIM> error = model(x, std::any_cast<Observation>(z), p);
      return VecXd(error);
    };

    models_[model_id] = type_erased_model;
    return model_id;
  }

  template <typename Observation>
  void add_measurement(int block_type,
                       const Observation& obs,
                       const TimePoint& time_of_validity) {
    const std::any z = obs;
    heap_.push({block_type, z, time_of_validity});
  }

  void add_observation_residual(const State& x,
                                const Measurement& z,
                                const Parameters& p,
                                int x_ind,
                                int residual_ind,
                                int param_ind,
                                Out<BlockSparseMatrix> bsm) {
    // y = (z[t] - h(x[t]; p))
    // J[obs_ind,  state_ind] = dy/dx[t] = -H
    // J[obs_ind, params_ind] = dy/dp    = -C

    const auto model& = models_.at(z.type);
    const auto y_of_x = [&model, &z, &p](const Prob::State& x) {
      // Hold p
      return model(x, z, p);
    };
    const auto y_of_p = [&model, &z, &x](const Prob::Parameters& p) {
      // Hold x
      return model(x, z, p);
    };

    const MatXd dy_dx = dynamic_group_jacobian(x_0, y_of_x);
    const MatXd dy_dp = dynamic_group_jacobian(p, y_of_p);
    bsm->set(residual_ind, x_ind, dy_dx);
    bsm->set(residual_ind, param_ind, dy_dp);
  }

  void add_dynamics_residual(const State& x_0,
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

    const int block_type = 0;
    const auto model& = dynamics_;
    const auto y_of_p = [dt, &model, &x_0, &x_1](const Prob::Parameters& p) {
      // Hold p
      return compute_delta(x_1, model(x_0, p, dt));
    };

    const auto y_of_x = [dt, &model, &x_0, &x_1](const Prob::Parameters& p) {
      // Hold x1
      return compute_delta(x_1, model(x_0, p, dt));
    };

    const MatXd dy_dx = dynamic_group_jacobian(x_0, y_of_x);
    const MatXd dy_dp = dynamic_group_jacobian(p, y_of_p);

    using StateToStateJac = MatNd<State::DIM, State::DIM>;
    const StateToStateJac I = StateToStateJac::Identity();

    bsm->set(residual_ind, x_ind, dy_dx);
    bsm->set(residual_ind, x_ind + 1, I);
    bsm->set(residual_ind, param_ind, dy_dp);
  }

  Solution solve(const Solution& initialization) {
    assert(initialization.size() == heap_.size());
    Solution<Prob> soln = initialization;

    constexpr int n_params = 1;
    const int n_measurements = static_cast<int>(heap_.size());
    const int n_states = static_cast<int>(initialization.size());

    BlockSparseMatrix J(n_states + n_measurements, n_states + params);

    // Immediately after the states
    const int p_ind = n_states;

    const auto& measurements = heap_.backing();
    for (int t = 0; t < static_cast<int>(measurements.size()) - 1) {
      const auto& z_t = measurements[t];

      const double dt =
          to_seconds(measurements[t + 1].time_of_validity - z_t.time_of_validity);
      // TODO: Do something special if dt == 0 (Share a measurement on a state?)

      // TODO: Divide information by dt
      constexpr int z_ind = 2 * t;
      x_t.at(t);
      add_observation_residual(x_t, z_t, p, dt, x_ind, z_ind, p_ind, out(J));

      x_t1 = soln.at(t + 1);
      add_dynamics_residual(x_t, x_t1, p, dt, x_ind, z_ind + 1, p_ind, out(J));
    }

    // JtJ;

    // Jtv;
  }

 private:
  Dynamics dynamics_;
  std::map<int, TypelessErrorModel>;

  // Min-Heap
  Heap<Measurement> heap_([](const Measurement& a, const Measurement& b) {
    return a.time_of_validity > b.time_of_validity;
  };);
};

}  // namespace optimization
}  // namespace estimation