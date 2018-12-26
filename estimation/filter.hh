#pragma once

#include "estimation/filter_state.hh"
#include "estimation/observation_model.hh"

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

  using AnyObservationModel = std::function<FilterStateUpdate<State>(
      const FilterState<State>&, const std::any& obs)>;

  // Returns the index of the added model
  int add_model_pr(const AnyObservationModel& model);

 public:
  using Dynamics = std::function<State(const State& x, double dt)>;

  Ekf(const Dynamics& dynamics) : dynamics_(dynamics) {
  }

  FilterState<State> update_state(const FilterState<State>& xp,
                                  const TimeDuration& dt) const;

  template <typename MeasurementType>
  using ErrorModel =
      std::function<VecNd<MeasurementType::DIM>(const State&, const MeasurementType&)>;

  template <typename MeasurementType>
  int add_model(const ErrorModel<MeasurementType>& fnc) {
    const ObservationModel<State, MeasurementType> model(fnc);
    const auto type_erased_fnc = [model](const FilterState<State>& x,
                                         const std::any& obs) {
      return model.generate_update(x, std::any_cast<MeasurementType>(obs));
    };
    return add_model_pr(type_erased_fnc);
  }

  template <typename MeasurementType>
  void measure(const MeasurementType& observation,
               const TimePoint& time_of_validity,
               int measurement_id) {
    const std::any z = observation;
    measurements_.push({measurement_id, z, time_of_validity});
  }

  FilterState<State> dynamics_until(const FilterState<State>& x0,
                                    const TimePoint& t) const;

  FilterState<State> service_all_measurements(const FilterState<State>& x_hat0);

 private:
  struct Comparator {
    bool operator()(const Measurement& a, const Measurement& b) const {
      return a.time_of_validity > b.time_of_validity;
    }
  };
  using MinQueue = std::priority_queue<Measurement, std::vector<Measurement>, Comparator>;
  MinQueue measurements_;

  std::map<int, AnyObservationModel> observation_models_;

  const Dynamics dynamics_;
};

}  // namespace estimation
