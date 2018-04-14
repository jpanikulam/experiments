#pragma once

#include "eigen.hh"

// TODO
#include <iostream>
#include <memory>
#include "viewer/primitives/frame.hh"
#include "viewer/primitives/image.hh"
#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"
#include "viewer/window_manager.hh"

namespace numerics {

struct JacobiConfig {
  double alpha = 1e-1;
  double beta  = 4.0;

  int max_iters = -1;
};

template <int     ROWS>
SquareMatNd<ROWS> jacobi_laplacian_image(const SquareMatNd<ROWS>& x,
                                         const SquareMatNd<ROWS>& b,
                                         const JacobiConfig&      config) {
  using Out           = SquareMatNd<ROWS>;
  const int max_iters = config.max_iters == -1 ? 2 * ROWS * ROWS : config.max_iters;

  Out x_soln = x;

  const double inv_beta = 1.0 / config.beta;

  constexpr bool DRAW_DEBUG_DATA = true;
  auto           viewer          = gl_viewer::get_window3d("ss");
  auto           image           = std::make_shared<gl_viewer::Image>(Eigen::MatrixXd(x));
  viewer->add_primitive(image);

  for (int k = 0; k < max_iters; ++k) {
    Out result = Out::Zero();
    // x_{i - 1, j}
    result.template rightCols<ROWS - 1>() += x_soln.template leftCols<ROWS - 1>();

    // x_{i + 1, j}
    result.template leftCols<ROWS - 1>() += x_soln.template rightCols<ROWS - 1>();

    // x_{i, j - 1}
    result.template bottomRows<ROWS - 1>() += x_soln.template topRows<ROWS - 1>();

    // x_{i, j + 1}
    result.template topRows<ROWS - 1>() += x_soln.template bottomRows<ROWS - 1>();

    result += config.alpha * b;

    x_soln = result * inv_beta;

    std::cout << "---" << std::endl;
    std::cout << x_soln << std::endl;

    if (DRAW_DEBUG_DATA) {
      // image->update_image(x_soln);
      // viewer->spin_until_step();
    }
  }

  std::cout << "\\\\\\\\\\\\" << std::endl;

  return x_soln;
}
}