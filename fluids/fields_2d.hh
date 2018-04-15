#include "eigen.hh"

namespace jcc {
namespace fluids {
namespace plane {

std::array<Eigen::MatrixXd, 2> compute_gradient(const Eigen::MatrixXd& potential_field, double dx);

// Scalar field operations
Eigen::MatrixXd compute_divergence(const std::array<Eigen::MatrixXd, 2>& vector_field, double dx);

Eigen::MatrixXd compute_laplacian(const Eigen::MatrixXd& potential_field, double dx);

}  // namespace plane.
}  // namespace fluids.
}  // namespace jcc.