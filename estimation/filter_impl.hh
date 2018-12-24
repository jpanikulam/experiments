#pragma once

#include "estimation/filter.hh"
#include "estimation/time_point.hh"

#include "numerics/group_diff.hh"
#include "numerics/numdiff.hh"

namespace estimation {

// Returns the index of the added model
template <typename State>
int Ekf<State>::add_model_pr(const Ekf<State>::AnyObservationModel& model) {
  const int i = static_cast<int>(observation_models_.size());
  observation_models_[i] = model;
  return i;
}

template <typename State>
FilterState<State> Ekf<State>::update_state(const FilterState<State>& xp,
                                            const TimeDuration& dt) const {
  const MatNd<State::DIM, State::DIM> Q = MatNd<State::DIM, State::DIM>::Identity();

  const auto dynamics_fixed_dt = [this, dt](const State& x) -> State {
    return dynamics_(x, to_seconds(dt));
  };

  const MatNd<State::DIM, State::DIM> A =
      numerics::group_jacobian<State, State>(xp.x, dynamics_fixed_dt);

  const State x_new = dynamics_fixed_dt(xp.x);

  const MatNd<State::DIM, State::DIM> P_new =
      A.transpose() * xp.P * A + (Q * to_seconds(dt));
  return {x_new, P_new, xp.time_of_validity + dt};
}

template <typename State>
FilterState<State> Ekf<State>::dynamics_until(const FilterState<State>& x0,
                                              const TimePoint& t) const {
  constexpr TimeDuration MAX_DT = to_duration(0.1);

  FilterState<State> x = x0;
  TimePoint time_simulated = x0.time_of_validity;
  while (time_simulated < t) {
    const TimeDuration dt = std::min(MAX_DT, t - time_simulated);
    x = update_state(x, dt);
    time_simulated += dt;
  }
  return x;
}

template <typename State>
FilterState<State> Ekf<State>::service_all_measurements(
    const FilterState<State>& x_hat0) {
  FilterState<State> x_hat = x_hat0;
  while (!measurements_.empty()) {
    const Measurement meas = measurements_.top();
    x_hat = dynamics_until(x_hat, meas.time_of_validity);

    const int i = meas.type;
    const auto& observer = observation_models_.at(i);
    const FilterStateUpdate<State> update = observer(x_hat, meas.observation);
    x_hat.x = apply_delta(x_hat.x, update.dx);
    x_hat.P = update.P_new;

    std::cout << "x_hat.x.eps_dot: " << x_hat.x.eps_dot.transpose() << std::endl;
    std::cout << "x_hat.x.eps_ddot: " << x_hat.x.eps_ddot.transpose() << std::endl;

    measurements_.pop();
  }

  return x_hat;
}

}  // namespace estimation
