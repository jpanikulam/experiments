#pragma once

#include "logging/assert.hh"

#include "estimation/filter.hh"
#include "estimation/time_point.hh"

#include "numerics/group_diff.hh"
#include "numerics/is_pd.hh"
#include "numerics/numdiff.hh"
#include "numerics/symmetrize.hh"

namespace estimation {

template <typename State>
Ekf<State>::Ekf(const Dynamics& dynamics, const MatNd<State::DIM, State::DIM>& cov)
    : dynamics_(dynamics), Q_(cov) {
  JASSERT(numerics::is_pd(Q_), "Dynamics covariance must be PD");
}

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
  const auto dynamics_fixed_dt = [this, dt](const State& x) -> State {
    return dynamics_(x, to_seconds(dt));
  };

  const State x_new = dynamics_fixed_dt(xp.x);

  const MatNd<State::DIM, State::DIM> A =
      numerics::group_jacobian<State, State>(xp.x, dynamics_fixed_dt);
  const MatNd<State::DIM, State::DIM> P_new =
      (A * xp.P * A.transpose()) + (A * (Q_ * to_seconds(dt)) * A.transpose());

  return {x_new, P_new, xp.time_of_validity + dt};
}

template <typename State>
FilterState<State> Ekf<State>::dynamics_until(const FilterState<State>& x0,
                                              const TimePoint& t) const {
  constexpr TimeDuration MAX_DT = to_duration(0.01);

  JASSERT_LE(x0.time_of_validity, t, "Current time must be less than target time");

  FilterState<State> x = x0;
  TimePoint time_simulated = x0.time_of_validity;
  while (time_simulated < t) {
    const TimeDuration dt = std::min(MAX_DT, t - time_simulated);
    x = update_state(x, dt);
    time_simulated += dt;
  }
  JASSERT_EQ(time_simulated, t, "Failed to simulate the correct amount of time");
  JASSERT_EQ(x.time_of_validity, t, "Failed to simulate up to target time of validity");

  return x;
}

template <typename State>
FilterState<State> Ekf<State>::soft_service_all_measurements(
    const FilterState<State>& x_hat0) const {
  FilterState<State> x_hat = x_hat0;
  // Copy measurements
  Heap<Measurement> measurements{measurements_};
  while (!measurements.empty()) {
    const Measurement meas = measurements.top();
    x_hat = dynamics_until(x_hat, meas.time_of_validity);

    const int i = meas.type;
    const auto& observer = observation_models_.at(i);
    const FilterStateUpdate<State> update = observer(x_hat, meas.observation);

    x_hat.x = State::apply_delta(x_hat.x, update.dx);
    x_hat.P = numerics::symmetrize(update.P_new);

    measurements.pop();
  }
  return x_hat;
}

template <typename State>
FilterState<State> Ekf<State>::service_all_measurements(
    const FilterState<State>& x_hat0) {
  FilterState<State> x_hat = x_hat0;

  const auto x_hat_f = soft_service_all_measurements(x_hat);
  measurements_.clear();

  return x_hat_f;
}

template <typename State>
jcc::Optional<FilterState<State>> Ekf<State>::service_next_measurement(
    const FilterState<State>& x_hat0) {
  if (!measurements_.empty()) {
    const Measurement meas = measurements_.top();

    const auto dt = meas.time_of_validity - x_hat0.time_of_validity;
    // const double dt_sec = to_seconds(dt);

    JASSERT_GE(dt, to_duration(0.0), "Time moved backwards");
    JASSERT_LE(dt, to_duration(0.5),
               "A very long time has passed between measurements, "
               "the filter has almost certainly already diverged");

    const auto x_hat_t =
        (dt > to_duration(0.0)) ? dynamics_until(x_hat0, meas.time_of_validity) : x_hat0;
    JASSERT_EQ(x_hat_t.time_of_validity, meas.time_of_validity,
               "Must simulate up to time of measurement");

    const int i = meas.type;

    const auto& observer = observation_models_.at(i);
    const FilterStateUpdate<State> update = observer(x_hat_t, meas.observation);

    FilterState<State> x_est_t = x_hat_t;
    x_est_t.x = State::apply_delta(x_hat_t.x, update.dx);
    x_est_t.P = numerics::symmetrize(update.P_new);

    JASSERT_EQ(x_est_t.time_of_validity, meas.time_of_validity,
               "Failed to bring state tov to measurement tov");

    measurements_.pop();
    return {x_est_t};
  } else {
    return {};
  }
}

}  // namespace estimation
