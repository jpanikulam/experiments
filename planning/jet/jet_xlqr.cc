#include "planning/jet/jet_xlqr.hh"

#include "planning/xlqr_problem_impl.hh"

namespace planning {
template class Problem<jet::JetDim, jet::State>;
template class XlqrProblem<jet::JetProblem>;
}  // namespace planning
