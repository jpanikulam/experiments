#include "planning/drifter/drifter_xlqr.hh"

#include "planning/xlqr_problem_impl.hh"
#include "numerics/numdiff.hh"
#include "numerics/num_hessian.hh"

namespace planning {
template class Problem<drifter::DrifterDim, drifter::State>;
template class XlqrProblem<drifter::DrifterProblem>;
template class Differentiator<drifter::DrifterProblem>;
}  // namespace planning
