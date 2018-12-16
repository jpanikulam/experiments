#pragma once

//
// TODO
// - [] Bootstrap optimizer
//

namespace estimation {
struct FilterUpdate {
  StateVec dx;
  StateInfo P_new;
};

template <typename State, typename Observation>
class ObservationModel {
  using StateVec = VecNd<State::DIM>;
  using ObsVec = VecNd<Observation::DIM>;

  using ErrorModel = std::function<ObsVec(const State&, const Observation&)>;
  using LogLikelihood = std::function<double(const State&, const Observation& z)>;

  ObservationModel(const ErrorModel& error_model, const LogLikelihood& log_likelihood)
      : error_model_(error_model), log_likelihood_(log_likelihood) {
  }

  StateVec generate_update(const ObsVec& innovation) {
    using MatNd<Observation::DIM, Observation::DIM> ObservationInformation;
    const ObservationInformation R = ObservationInformation::Identity();

    const MatNd<Observation::Dim, State::Dim> H = diff_.obs_diff(x);
    const MatNd<Observation::Dim, Observation::Dim> R = diff_.likelihood_cov(x);

    const ObservationInformation S = (H.transpose() * P * H) + R;

    const Eigen::LLT<ObservationInformation> S_llt(S);
    if (llt.info() != Eigen::Success) {
      std::cout << "LLT solve was degenerate" << std::endl;
      assert(false);
    }

    const MatNd<State::DIM, Observation::DIM> PHt = P * H.transpose();
    const StateVec update = PHt * S_llt.solve(innovation);
    using MatNd<State::DIM, State::DIM> StateInfo;
    const StateInfo P_new = (StateInfo::Identity() - (PHt * S_llt.solve(H))) * P;

    return FilterUpdate{update, P_new};
  }

 private:
  // This return z [-] h(x)
  ObsVec error(const State& x, const Observation& z) {
    return error_model_(x, z);
  }

  ErrorModel error_model_;
  LogLikelihood log_likelihood_;
};

}  // namespace estimation