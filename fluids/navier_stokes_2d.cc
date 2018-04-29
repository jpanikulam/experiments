#include "navier_stokes_2d.hh"

#include "eigen.hh"

#include "fields_2d.hh"

// TODO
#include <iostream>

namespace jcc {
namespace fluids {
namespace plane {

Eigen::MatrixXd solve_poisson_jacobi_image(const Eigen::MatrixXd& x, const Eigen::MatrixXd& b, const JacobiConfig& config) {
  const int rows      = x.rows();
  using Out           = Eigen::MatrixXd;
  const int max_iters = config.max_iters == -1 ? 100 * rows : config.max_iters;

  Out x_soln = x;

  const double inv_beta = 1.0 / config.beta;

  int k = 0;
  for (; k < max_iters; ++k) {
    Out tmp = Out::Zero(rows, rows);
    // x_{i - 1, j}
    tmp.rightCols(rows - 1) += x_soln.leftCols(rows - 1);

    // x_{i + 1, j}
    tmp.leftCols(rows - 1) += x_soln.rightCols(rows - 1);

    // x_{i, j - 1}
    tmp.bottomRows(rows - 1) += x_soln.topRows(rows - 1);

    // x_{i, j + 1}
    tmp.topRows(rows - 1) += x_soln.bottomRows(rows - 1);

    tmp += config.alpha * b;
    tmp *= inv_beta;

    const double delta = (x_soln - tmp).norm() / x_soln.norm();
    x_soln = tmp;
    if (delta < config.min_rel_delta) {
      std::cout << "Done at : " << k << std::endl;
      break;
    }
  }

  std::cout << "Giving up at k : " << k << std::endl;

  return x_soln;
}

std::array<Eigen::MatrixXd, 2> compute_advection(const std::array<Eigen::MatrixXd, 2> velocity_field,
                                                 const SimulationConfig&              cfg) {
  // Divergence of velocity
  const Eigen::MatrixXd div_velocity = compute_divergence(velocity_field, cfg.dx);

  std::array<Eigen::MatrixXd, 2> result;

  result[0] = div_velocity.array() * velocity_field[0].array();
  result[1] = div_velocity.array() * velocity_field[1].array();
  return result;
}

std::array<Eigen::MatrixXd, 2> compute_pressure(const Eigen::MatrixXd pressure_field, const SimulationConfig& cfg) {
  // Divergence of velocity

  const double inv_rho = 1.0 / cfg.rho;

  auto grad = compute_gradient(pressure_field, cfg.dx);

  grad[0] *= inv_rho;
  grad[1] *= inv_rho;

  return grad;
}

// TODO: Use jacobi
std::array<Eigen::MatrixXd, 2> compute_diffusion(const std::array<Eigen::MatrixXd, 2> velocity_field,
                                                 const SimulationConfig&              cfg) {
  std::array<Eigen::MatrixXd, 2> result;

  constexpr int DEPTH = 2;
  for (int k = 0; k < DEPTH; ++k) {
    result[k] = cfg.nu * compute_laplacian(velocity_field[k], cfg.dx);
  }
  return result;
}

SimulationState compute_projection(const std::array<Eigen::MatrixXd, 2> divergent_velocity,
                                   const SimulationConfig&              cfg) {
  SimulationState       result;
  const Eigen::MatrixXd velocity_divergence = compute_divergence(divergent_velocity, cfg.dx);
  const int             rows                = velocity_divergence.rows();
  const int             cols                = velocity_divergence.cols();
  const Eigen::MatrixXd initialization      = Eigen::MatrixXd::Random(rows, cols);

  const JacobiConfig jacobi_cfg{.alpha = -(cfg.dx * cfg.dx), .beta = 4};
  result.pressure_field = solve_poisson_jacobi_image(velocity_divergence, velocity_divergence, jacobi_cfg);

  const std::array<Eigen::MatrixXd, 2> grad_p = compute_gradient(result.pressure_field, cfg.dx);

  constexpr int DEPTH   = 2;
  result.velocity_field = divergent_velocity;
  for (int k = 0; k < DEPTH; ++k) {
    result.velocity_field[k] -= grad_p[k];
  }

  return result;
}

}  // namespace plane.
}  // namespace fluids.
}  // namespace jcc.
