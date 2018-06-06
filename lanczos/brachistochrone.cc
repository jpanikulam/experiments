#include "numerics/numdiff.hh"
#include "numerics/optimize.hh"

#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

namespace lanczos {
double cost_function(const Eigen::VectorXd &y,
                     Eigen::VectorXd *const gradient,
                     Eigen::MatrixXd *const hessian) {
  const double g = 1.0;
  const double dx = 0.1;
  const double alpha = static_cast<double>(y.rows()) * (dx + 0.1);

  const auto true_cost = [alpha, g, dx](const Eigen::VectorXd x) {
    int n_diff_elements = x.rows() - 2;
    Eigen::VectorXd dy_dx(x.rows());
    dy_dx.segment(1, n_diff_elements) =
        (x.segment(2, n_diff_elements) - x.segment(1, n_diff_elements)) / dx;
    dy_dx(0) = (x(0) - alpha) / dx;
    dy_dx.tail(1) = -x.tail(1) / dx;

    const Eigen::VectorXd cost_numerators = 1.0 + dy_dx.array().square();
    const Eigen::VectorXd cost_denominators = alpha - x.array();

    const Eigen::VectorXd times =
        (cost_numerators.array() / cost_denominators.array()).cwiseSqrt();
    const double total_time = (1.0 / std::sqrt(2.0 * g)) * times.sum() * dx;

    std::cout << "dydx--" << std::endl;
    std::cout << dy_dx.transpose() << std::endl;
    std::cout << "num--" << std::endl;
    std::cout << cost_numerators.transpose() << std::endl;
    std::cout << "den--" << std::endl;
    std::cout << cost_denominators.transpose() << std::endl;
    std::cout << "--" << std::endl;

    return total_time;
  };

  if (gradient) {
    *gradient = numerics::dynamic_numerical_gradient(y, true_cost);

    std::cout << gradient->transpose() << std::endl;

    {
      const auto view = gl_viewer::get_window3d("soup");
      const auto geo = view->add_primitive<gl_viewer::SimpleGeometry>();
      for (int k = 1; k < y.rows(); ++k) {
        const double x0 = (k - 1) * 0.1;
        const double x1 = k * 0.1;
        const Eigen::Vector3d start(x0, 0.0, y(k - 1));
        const Eigen::Vector3d end(x1, 0.0, y(k));

        geo->add_line({start, end});
      }

      view->spin_until_step();
      view->clear();
    }
  }

  return true_cost(y);
}

void solve() {
  const auto view = gl_viewer::get_window3d("soup");
  view->set_target_from_world(
      SE3(SO3::exp(Eigen::Vector3d(-3.1415 * 0.5, 0.0, 0.0)),
          Eigen::Vector3d::Zero()));

  numerics::OptimizationProblem problem;
  problem.objective = cost_function;

  numerics::OptimizationState state;
  state.x = Eigen::VectorXd::Ones(10, 1);
  for (int k = 0; k < state.x.rows(); ++k) {
    state.x(k) = 0.1 * (state.x.rows() - k);
  }

  const auto result =
      numerics::optimize<numerics::ObjectiveMethod::kGradientDescent,
                         numerics::ConstraintMethod::kAugLag>(state, problem);
  std::cout << result.x.transpose() << std::endl;
}

}  // namespace lanczos

int main() {
  lanczos::solve();
}