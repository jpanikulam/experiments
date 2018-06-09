#include "numerics/numdiff.hh"
#include "numerics/optimize.hh"

#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

namespace lanczos {

Eigen::VectorXd sigmoid_blend(const Eigen::VectorXd &x,
                              const double alpha,
                              const double beta) {
  constexpr double scale = 25.0;
  const Eigen::VectorXd x_exp = (scale * x).array().exp();
  const Eigen::VectorXd t = x_exp.array() / (x_exp.array() + 1.0);

  const Eigen::VectorXd convex_blend =
      ((1.0 - t.array()) * alpha) + (t.array() * beta);

  return convex_blend;
}

double cost_function(const Eigen::VectorXd &y,
                     Eigen::VectorXd *const gradient,
                     Eigen::MatrixXd *const hessian) {
  const double g = 9.0;
  const double dx = 0.01;
  const double alpha = 2.0;

  const auto true_cost = [alpha, g, dx](const Eigen::VectorXd &x,
                                        bool debug_output = false) {
    Eigen::VectorXd x_augmented(x.rows() + 2);
    x_augmented(0) = alpha;
    x_augmented.segment(1, x.rows()) = x;
    x_augmented(x.rows() + 1) = 0.0;

    Eigen::VectorXd arclength(x.rows() + 1);
    for (int k = 1; k < x_augmented.rows(); ++k) {
      const double height_diff = x_augmented(k) - x_augmented(k - 1);
      arclength(k - 1) = std::hypot(height_diff, dx);
    }

    Eigen::VectorXd velocity(x.rows() + 1);
    for (int k = 1; k < x_augmented.rows(); ++k) {
      const double h = 0.5 * (x_augmented(k) + x_augmented(k - 1));
      const double v2 = 2.0 * g * (alpha - h);
      velocity(k - 1) = std::sqrt(v2);
    }

    const Eigen::VectorXd times = (arclength.array() / velocity.array());
    const double total_time = times.sum();
    if (debug_output) {
      std::cout << "arclength:\n" << arclength.transpose() << std::endl;
      std::cout << "velocity:\n" << velocity.transpose() << std::endl;
      std::cout << "times:\n" << times.transpose() << std::endl;
      std::cout << "heights:\n" << x_augmented.transpose() << std::endl;
    }
    return total_time;
  };

  if (gradient) {
    *gradient = numerics::dynamic_numerical_gradient(y, true_cost);

    constexpr bool debug = false;
    true_cost(y, debug);
    if (debug) {
      std::cout << "Grad: " << std::endl;
      std::cout << gradient->transpose() << std::endl;
    }
    // Draw
    static int doobie = 0;
    ++doobie;
    if (doobie == 20) {
      doobie = 0;
      const auto view = gl_viewer::get_window3d("Curve Visualization");
      view->clear();
      const auto geo = view->add_primitive<gl_viewer::SimpleGeometry>();

      Eigen::VectorXd y_augmented(y.rows() + 2);
      y_augmented(0) = alpha;
      y_augmented.segment(1, y.rows()) = y;
      y_augmented(y.rows() + 1) = 0.0;

      for (int k = 1; k < y_augmented.rows(); ++k) {
        const double x0 = (k - 1) * dx;
        const double x1 = k * dx;
        const Eigen::Vector3d start(x0, 0.0, y_augmented(k - 1));
        const Eigen::Vector3d end(x1, 0.0, y_augmented(k));
        geo->add_line({start, end});

        // if (k < gradient->rows()) {
        //   const double grad = (*gradient)(k);
        //   const double normalized_grad = grad * 50.0;
        //   const Eigen::Vector3d grad_end(
        //       x1, 0.0, y_augmented(k) + normalized_grad);
        //   geo->add_line({end, grad_end, Eigen::Vector4d(1.0, 0.0, 0.5,
        //   0.5)});
        // }
      }

      view->spin_until_step();
    }
  }

  return true_cost(y);
}

void solve() {
  const auto view = gl_viewer::get_window3d("Curve Visualization");
  view->set_target_from_world(
      SE3(SO3::exp(Eigen::Vector3d(-3.1415 * 0.5, 0.0, 0.0)),
          Eigen::Vector3d::Zero()));

  numerics::OptimizationProblem problem;
  problem.objective = cost_function;

  numerics::OptimizationState state;
  state.x = 1.0 * Eigen::VectorXd::Random(1000).array() + 1.0;

  const auto result =
      numerics::optimize<numerics::ObjectiveMethod::kGradientDescent,
                         numerics::ConstraintMethod::kAugLag>(state, problem);
  std::cout << result.x.transpose() << std::endl;
}

}  // namespace lanczos

int main() {
  lanczos::solve();
}