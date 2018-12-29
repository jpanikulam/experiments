#pragma once

#include <any>
#include <map>
#include <vector>

#include "estimation/optimization/block_sparse_matrix.hh"
#include "estimation/time_point.hh"

#include "out.hh"
#include "util/heap.hh"

namespace estimation {
namespace optimization {

template <typename _State, typename _Parameters>
struct ProblemType {
  using State = _State;
  using Parameters = _Parameters;
};

template <typename Prob>
struct OptimizerSolution {
  std::vector<typename Prob::State> x;
  typename Prob::Parameters p;
};

template <typename Prob>
class AcausalOptimizer {
 public:
  using State = typename Prob::State;
  using Parameters = typename Prob::Parameters;
  using Solution = OptimizerSolution<Prob>;
  using Dynamics = std::function<State(const State&, const Parameters&, double dt)>;

  template <typename Observation>
  using ErrorModel = std::function<VecNd<Observation::DIM>(
      const State&, const Observation&, const Parameters&)>;

  using TypelessErrorModel =
      std::function<VecXd(const State& x, const std::any& z, const Parameters& p)>;

  struct Measurement {
    int type = -1;
    std::any observation;
    TimePoint time_of_validity;
  };

  AcausalOptimizer(const Dynamics& dynamics) : dynamics_(dynamics) {
  }

  template <typename Observation>
  int add_error_model(const ErrorModel<Observation>& model) {
    int model_id = models_.size();
    const auto type_erased_model = [model](const State& x, const std::any& z,
                                           const Parameters& p) {
      const VecNd<Observation::DIM> error = model(x, std::any_cast<Observation>(z), p);
      return VecXd(error);
    };

    models_[model_id] = type_erased_model;
    return model_id;
  }

  template <typename Observation>
  void add_measurement(const Observation& obs,
                       const TimePoint& time_of_validity,
                       int block_type) {
    const std::any z = obs;
    heap_.push({block_type, z, time_of_validity});
  }

  using Visitor = std::function<void(const Solution&)>;
  Solution solve(const Solution& initialization, const Visitor& visitor = {});

 private:
  VecXd add_observation_residual(const State& x,
                                 const Measurement& z,
                                 const Parameters& p,
                                 int x_ind,
                                 int residual_ind,
                                 int param_ind,
                                 Out<BlockSparseMatrix> bsm);

  VecXd add_dynamics_residual(const State& x_0,
                              const State& x_1,
                              const Parameters& p,
                              double dt,
                              int x_ind,
                              int residual_ind,
                              int param_ind,
                              Out<BlockSparseMatrix> bsm);

  Solution update_solution(const Solution& soln, const VecXd& delta);
  BlockSparseMatrix populate(const Solution& soln, Out<std::vector<VecXd>> v);

  double cost(const std::vector<VecXd>& residuals);

  Dynamics dynamics_;
  std::map<int, TypelessErrorModel> models_;

  // Min-Heap
  Heap<Measurement> heap_ =
      Heap<Measurement>([](const Measurement& a, const Measurement& b) {
        return a.time_of_validity > b.time_of_validity;
      });
};

}  // namespace optimization
}  // namespace estimation