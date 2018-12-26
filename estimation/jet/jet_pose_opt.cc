#include "estimation/jet/jet_pose_opt.hh"

#include "estimation/optimization/acausal_optimizer_impl.hh"

//%deps(block_sparse_matrix)
//%deps(numdiff)

namespace estimation {
namespace optimization {

template class AcausalOptimizer<jet_filter::JetOptimizerProblem>;

}  // namespace optimization
}  // namespace estimation
