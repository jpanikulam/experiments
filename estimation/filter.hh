#pragma once

namespace estimation {

template <typename State>
struct FilterState {
  // The actual state
  State x;

  // State-covariance
  MatNd<State::X_DIM, State::X_DIM> P;

  // When this state was the estimate for the true state
  TimePoint time_of_validity;
};
}  // namespace estimation