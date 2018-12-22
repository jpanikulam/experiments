#pragma once

#include "estimation/filter_state.hh"

#include <any>
#include <map>
#include <queue>

namespace estimation {

template <typename State>
class Ekf {
 private:
  using StateVec = VecNd<State::DIM>;

  struct Measurement {
    int type = -1;
    std::any observation;
    TimePoint time_of_validity;
  };

  using ObservationModel =
      std::function<StateVec(const FilterState<State>&, const std::any& obs)>;

  // Returns the index of the added model
  int add_model(const ObservationModel& model);

 public:
  using Dynamics = std::function<State(const State& x, double dt)>;

  Ekf(const FilterState<State>& initial_state, const Dynamics& dynamics)
      : filter_state_(initial_state), dynamics_(dynamics) {
  }

  FilterState<State> update_state(const FilterState<State>& xp,
                                  const TimeDuration& dt) const;

  template <typename MeasurementType>
  int add_model(const std::function<StateVec(const FilterState<State>&,
                                             const MeasurementType& meas)>& fnc) {
    const auto type_erased_fnc = [fnc](const FilterState<State>& x, const std::any& obs) {
      return fnc(x, std::any_cast<MeasurementType>(obs));
    };
    return add_model(type_erased_fnc);
  }

  template <typename MeasurementType>
  void measure(const MeasurementType& observation,
               const TimePoint& time_of_validity,
               int measurement_id) {
    measurements_.push({measurement_id, std::any_cast(observation), time_of_validity});
  }

  FilterState<State> dynamics_until(const FilterState<State>& x0,
                                    const TimePoint& t) const;

  FilterState<State> service_all_measurements(const FilterState<State>& x_hat0) const;

 private:
  struct Comparator {
    bool operator()(const Measurement& a, const Measurement& b) const {
      return a.time_of_validity > b.time_of_validity;
    }
  };
  using MinQueue = std::priority_queue<Measurement, std::vector<Measurement>, Comparator>;
  MinQueue measurements_;

  std::map<int, ObservationModel> observation_models_;

  FilterState<State> filter_state_;

  const Dynamics dynamics_;
};

}  // namespace estimation
