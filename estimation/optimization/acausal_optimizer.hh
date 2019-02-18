#pragma once

#include <any>
#include <map>
#include <vector>

#include "estimation/optimization/block_sparse_matrix.hh"
#include "estimation/time_point.hh"

#include "numerics/is_pd.hh"
#include "vision/robust_estimator.hh"

#include "out.hh"
#include "util/heap.hh"

namespace estimation {
namespace optimization {

template <typename _State, typename _Parameters>
struct ProblemType {
  using State = _State;
  using Parameters = _Parameters;

  struct StateObservation {
    State x;
    TimePoint time_of_validity;
  };
};

template <typename Prob>
struct OptimizerSolution {
  std::vector<typename Prob::StateObservation> x;
  typename Prob::Parameters p;
};

struct LinearSystem {
  BlockSparseMatrix J;
  BlockSparseMatrix R_inv;
  std::vector<VecXd> v;
};

template <typename Prob>
class AcausalOptimizer {
 public:
  using State = typename Prob::State;
  using Parameters = typename Prob::Parameters;
  using Solution = OptimizerSolution<Prob>;
  using Dynamics = std::function<State(const State&, const Parameters&, double dt)>;
  using StateObservation = typename Prob::StateObservation;

  template <typename Observation>
  using ErrorModel = std::function<VecNd<Observation::DIM>(
      const State&, const Observation&, const Parameters&)>;

  struct TypelessErrorModel {
    using TypeleErasedErrorModel =
        std::function<VecXd(const State& x, const std::any& z, const Parameters& p)>;

    TypeleErasedErrorModel error;
    int out_dim = -1;
  };

  struct Measurement {
    int type = -1;
    std::any observation;
    TimePoint time_of_validity;
  };

  // struct DebugInfo {
  //   std::vector<State> x_estimated;
  //   std::vector<Measurement> original_z;
  // };

  AcausalOptimizer(const Dynamics& dynamics) : dynamics_(dynamics) {
  }

  template <typename Observation>
  int add_error_model(const ErrorModel<Observation>& model,
                      const MatNd<Observation::DIM, Observation::DIM>& cov) {
    int model_id = models_.size();
    assert(numerics::is_pd(cov));
    const auto type_erased_model = [model](const State& x, const std::any& z,
                                           const Parameters& p) {
      const VecNd<Observation::DIM> error = model(x, std::any_cast<Observation>(z), p);
      return VecXd(error);
    };

    models_[model_id] = {type_erased_model, Observation::DIM};
    covariances_[model_id] = MatXd(cov);

    return model_id;
  }

  void set_dynamics_cov(const MatNd<State::DIM, State::DIM>& cov) {
    dyn_info_ = cov.inverse();
    assert(numerics::is_pd(dyn_info_));
  }

  template <typename Observation>
  void add_measurement(const Observation& obs,
                       const TimePoint& time_of_validity,
                       int block_type) {
    const std::any z = obs;
    heap_.push({block_type, z, time_of_validity});
  }

  using Visitor = std::function<void(const Solution&)>;
  Solution solve(const Solution& initialization, const Visitor& visitor = {}) const;

 private:
  struct DynamicsDifferentials {
    MatXd df_dx;
    MatXd df_dp;
  };

  struct MeasurementDifferentials {
    VecXd residual;
    MatXd dh_dx;
    MatXd dh_dp;
  };

  struct StateForTime {
    int x_ind;
    State x;
    double dt;
  };

  DynamicsDifferentials dynamics_diff(const State& x,
                                      const Parameters& p,
                                      const double dt) const;

  StateForTime get_state_for_time(const Solution& soln, const TimePoint& t) const;

  MeasurementDifferentials add_observation_residual(const State& x,
                                                    const Measurement& z,
                                                    const Parameters& p) const;

  VecXd add_dynamics_residual(const State& x_0,
                              const State& x_1,
                              const Parameters& p,
                              double dt,
                              int x_ind,
                              int residual_ind,
                              int param_ind,
                              Out<BlockSparseMatrix> bsm) const;

  Solution update_solution(const Solution& soln, const VecXd& delta) const;
  LinearSystem populate(const Solution& soln) const;

  double cost(const LinearSystem& system,
              const slam::RobustEstimator& rcost,
              std::vector<double>* weights = nullptr) const;

  Dynamics dynamics_;
  MatNd<State::DIM, State::DIM> dyn_info_;

  std::map<int, TypelessErrorModel> models_;
  std::map<int, MatXd> covariances_;

  // Min-Heap
  Heap<Measurement> heap_ =
      Heap<Measurement>([](const Measurement& a, const Measurement& b) {
        return a.time_of_validity > b.time_of_validity;
      });
  static constexpr double MAX_DT = 0.5;
};

}  // namespace optimization
}  // namespace estimation