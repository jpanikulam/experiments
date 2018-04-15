#include "eigen.hh"

namespace jcc {
namespace fluids {
namespace plane {

// using VectorField = std::array<Eigen::MatrixXd, 2, Eigen::aligned_allocator<Eigen::Vector4f>>;

struct SimulationConfig {
  // Pressure constant
  const double rho = 1e3;

  // Viscosity
  const double nu = 0.9;

  const double dx = 1e-1;

  const double dt = 1e-3;
};

struct SimulationState {
  Eigen::MatrixXd                pressure_field;
  std::array<Eigen::MatrixXd, 2> velocity_field;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

std::array<Eigen::MatrixXd, 2> compute_advection(const std::array<Eigen::MatrixXd, 2> velocity_field,
                                                 const SimulationConfig&              config);

std::array<Eigen::MatrixXd, 2> compute_pressure(const Eigen::MatrixXd pressure_field, const SimulationConfig& cfg);

std::array<Eigen::MatrixXd, 2> compute_diffusion(const std::array<Eigen::MatrixXd, 2> velocity_field,
                                                 const SimulationConfig&              cfg);

SimulationState compute_projection(const std::array<Eigen::MatrixXd, 2> divergent_velocity,
                                   const SimulationConfig&              cfg);

}  // namespace plane
}  // namespace fluids
}  // namespace jcc