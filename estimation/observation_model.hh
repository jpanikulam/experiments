#pragma once

#include "eigen.hh"
#include "estimation/filter_state.hh"

#include "numerics/group_diff.hh"

// TODO
#include <iostream>

namespace estimation {

template <typename State, typename Observation>
class ObservationModel {
 public:
  using StateVec = VecNd<State::DIM>;
  using ObsVec = VecNd<Observation::DIM>;

  using ErrorModel = std::function<ObsVec(const State&, const Observation&)>;
  using LogLikelihood = std::function<double(const State&, const Observation& z)>;

  ObservationModel(const ErrorModel& error_model) : error_model_(error_model) {
  }

  // This return z [-] h(x)
  ObsVec error(const State& x, const Observation& z) const {
    return error_model_(x, z);
  }

  FilterStateUpdate<State> generate_update(const FilterState<State>& xp,
                                           const Observation& z) const {
    using ObservationInformation = MatNd<Observation::DIM, Observation::DIM>;

    const ObservationInformation R = ObservationInformation::Identity();
    const ObsVec innovation = error_model_(xp.x, z);

    const auto held_error_model = [this, &z](const State& x) -> ObsVec {
      const ObsVec y = error_model_(x, z);
      return y;
    };

    const MatNd<Observation::DIM, State::DIM> H =
        numerics::group_jacobian<Observation::DIM, State>(xp.x, held_error_model);
    // const MatNd<Observation::Dim, Observation::Dim> R = diff_.likelihood_cov(x);

    const ObservationInformation S = (H * xp.P * H.transpose()) + R;

    const Eigen::LLT<ObservationInformation> S_llt(S);
    if (S_llt.info() != Eigen::Success) {
      std::cout << "LLT solve was degenerate" << std::endl;
      assert(false);
    }

    const MatNd<State::DIM, Observation::DIM> PHt = xp.P * H.transpose();
    const StateVec update = PHt * S_llt.solve(innovation);
    using StateInfo = MatNd<State::DIM, State::DIM>;
    const StateInfo P_new = (StateInfo::Identity() - (PHt * S_llt.solve(H))) * xp.P;

    return FilterStateUpdate<State>{update, P_new};
  }

  FilterStateUpdate<State> operator()(const FilterState<State>& xp,
                                      const Observation& z) const {
    return generate_update(xp, z);
  }

 private:
  const ErrorModel error_model_;
  LogLikelihood log_likelihood_;
};

}  // namespace estimation