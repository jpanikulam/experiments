#pragma once

#include "estimation/observation_model.hh"

#include "logging/assert.hh"
#include "numerics/group_diff.hh"
#include "numerics/is_pd.hh"
#include "numerics/is_symmetric.hh"

#include "vision/robust_estimator.hh"
namespace estimation {
namespace {
template <int rows>
double compute_likelihood(const MatNd<rows, rows>& info, const VecNd<rows>& innov) {
  const double mahalanobis_sq = 0.5 * innov.dot(info * innov);
  // constexpr double K_LOG_2PI = rows * std::log(2.0 * M_PI);
  // const double log_normalizer =
  // -0.5 * (K_LOG_2PI + std::log(info.inverse().determinant()));
  // return -mahalanobis_sq + log_normalizer;

  return std::exp(-mahalanobis_sq) /
         std::sqrt(std::pow(2.0 * M_PI, rows) * (1.0 / info.determinant()));
}
}  // namespace

template <typename State, typename Observation>
ObservationModel<State, Observation>::ObservationModel(
    const typename ObservationModel<State, Observation>::ErrorModel& error_model,
    const MatNd<Observation::DIM, Observation::DIM>& cov)
    : error_model_(error_model), cov_(cov) {
  JASSERT(numerics::is_pd(cov_), "Observation covariance must be positive definite");
}

template <typename State, typename Observation>
FilterStateUpdate<State> ObservationModel<State, Observation>::generate_update(
    const FilterState<State>& xp, const Observation& z) const {
  using ObservationInfo = MatNd<Observation::DIM, Observation::DIM>;

  const ObsVec innovation = error_model_(xp.x, z);

  const auto held_error_model = [this, &z](const State& x) -> ObsVec {
    const ObsVec y = error_model_(x, z);
    return y;
  };

  const MatNd<Observation::DIM, State::DIM> H =
      -numerics::group_jacobian<Observation::DIM, State>(xp.x, held_error_model);

  const ObservationInfo S = (H * xp.P * H.transpose()) + cov_;

  JASSERT(numerics::is_symmetric(xp.P), "Current state covariance must be symmetric");

  const Eigen::LLT<ObservationInfo> S_llt(S);
  JASSERT_EQ(S_llt.info(), Eigen::Success, "LLT must not be degenerate");

  using StateInfo = MatNd<State::DIM, State::DIM>;

  const ObservationInfo S_inv = S.inverse();

  // const double log_likelihood = compute_likelihood(S_inv, innovation);

  // const slam::HuberCost hc(0.2);
  // const auto cw = hc((innovation.transpose() * S_inv).dot(innovation));
  // jcc::Debug() << "Weight: " << cw.weight << std::endl;

  const MatNd<State::DIM, Observation::DIM> K = xp.P * H.transpose() * S_inv;
  const StateVec update = K * innovation;
  const StateInfo P_new = (StateInfo::Identity() - (K * H)) * xp.P;

  //

  // const MatNd<State::DIM, Observation::DIM> PHt = xp.P * H.transpose();
  // const StateVec update = PHt * S_llt.solve(innovation);
  // const StateInfo P_new = (StateInfo::Identity() - (PHt * S_llt.solve(H))) * xp.P;

  return FilterStateUpdate<State>{update, P_new};
}

}  // namespace estimation
