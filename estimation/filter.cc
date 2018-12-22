#include <any>
#include <queue>

#include "estimation/filter.hh"
#include "estimation/time_point.hh"

namespace estimation {

template <State>
class FilterUpdater {
 public:
  FilterState<State> update_state(const FilterState<State>& xp,
                                  const TimeDuration& dt) const {
    const MatNd<State::DIM, State::DIM> A = diffs_.state_jacobian(xp.x);
    const State x_new = dynamics_(x, to_seconds(dt));

    const MatNd<State::DIM, State::DIM> P_new = A.transpose() * xp.P * A + (Q * dt);
    return {x_new, P_new, xp.time_of_validity + to_duration(dt)};
  }

 private:
  Dynamics dynamics_;
  Differentiator diffs_;

  MatNd<State::DIM, State::DIM> Q = MatNd<State::DIM, State::DIM>::Identity();
};

template <typename State>
class Ekf {
 private:
  struct Measurement {
    int type = -1;
    std::any observation;
    TimePoint time_of_validity;
  };
  struct Comparator {
    bool operator()(const Measurement& a, const Measurement& b) const {
      return a.time_of_validity > b.time_of_validity;
    }
  };

  using ObservationModel =
      std::function<StateVec(const FilterState&, const std::any& obs)>;

  // Returns the index of the added model
  int add_model(const ObservationModel& model) {
    const int i = static_cast<int>(observation_models_.size());
    observation_models_[i] = model;
    return i;
  }

 public:
  using MinQueue = std::priority_queue<Observation, std::vector, Comparator>;

  Ekf(const TimePoint& start_time, const FilterState& initial_state) {
    last_time_serviced_ = start_time;
    filter_state_ = initial_state;
  }

  template <typename MeasurementType>
  int add_model(const std::function<StateVec(const FilterState&,
                                             const MeasurementType& meas)>& fnc) {
    const auto type_erased_fnc = [](const FilterState& x, const std::any& obs) {
      return fnc(x, std::any_cast<MeasurementType>(obs));
    };
    return add_model(type_erased_fnc);
  }


  void measure(const Measurement& measurement) {
    measurements_.push(measurement);
  }

  FilterState dynamics_until(const FilterState& x0, const TimePoint& t) const {
    constexpr double MAX_DT = 0.1;

    FilterState x = x0;
    TimePoint time_simulated = x0.time_of_validity;
    while (time_simulated < t) {
      const TimeDuration dt = std::min(MAX_DT, t - time_simulated);
      x = updater_.update_state(x, dt);
      time_simulated += dt;
    }
    return x;
  }

  FilterState service_all_measurements() {
    FilterState x_hat = filter_state_;
    while (!measurements_.empty()) {
      const Measurement meas = measurements_.top();
      x_hat = dynamics_until(x_hat, meas.time_of_validity);
      measurements_.pop();
    }

    filter_state_ = x_hat;
    last_time_serviced_ = filter_state_.time_of_validity;

    return x_hat;
  }

 private:
  std::map<int, ObservationModel> observation_models_;

  MinQueue measurements_;

  FilterUpdater<State> updater_;
  FilterState filter_state_;
  TimePoint last_time_serviced_;
};

}  // namespace estimation
