#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <sophus/se2.hpp>
#include <sophus/so2.hpp>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector2d)

using Scalar = double;
using so2    = Sophus::SO2<Scalar>;
using se2    = Sophus::SE2<Scalar>;

namespace raytrace {

using Vec1 = Eigen::Matrix<double, 1, 1>;
using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Mat2 = Eigen::Matrix2d;

double cost(const Vec1 &eta, const std::vector<Vec2> &x, const std::vector<Vec2> &z) {
  const so2 T = so2::exp(eta(0));

  double error_sq = 0.0;
  for (std::size_t k = 0; k < x.size(); ++k) {
    const Vec2 error = (T * x[k]) - z[k];
    error_sq += error.squaredNorm();
  }

  return error_sq;
}

Vec1 dcost(const Vec1 &eta, const std::vector<Vec2> &x, const std::vector<Vec2> &z) {
  const so2 T  = so2::exp(eta(0));
  const so2 dT = so2::exp(eta(0) + (M_PI / 2.0));

  Vec1 derror_sq_total(0.0);
  for (std::size_t k = 0; k < x.size(); ++k) {
    const Vec2 error       = (T * x[k]) - z[k];
    const Vec2 derror_deta = dT * x[k];
    const Vec1 derror_sq   = (2 * error.transpose()) * derror_deta;

    derror_sq_total += derror_sq;
  }

  return derror_sq_total;
}

template <int cols, typename Callable1, typename Callable2>
Eigen::Matrix<double, cols, 1> gauss_newton(const Eigen::Matrix<double, cols, 1> &x0,
                                            const Callable1 &fcn,
                                            const Callable2 &jac,
                                            const double     feps = 1e-6) {
  using Vec      = Eigen::Matrix<double, cols, 1>;
  using gradient = Eigen::Matrix<double, cols, 1>;
  Vec x          = x0;

  for (int k = 0; k < 10; ++k) {
    const gradient J = jac(x);
    x          = x - (J.transpose() * J).inverse() * J.transpose() * fcn(x);
    // x = x - (J * J).inverse()

    std::cout << x.transpose() << " : " << fcn(x) << std::endl;
  }
  return x;
}
}