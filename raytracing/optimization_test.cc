#include "optimization.hh"

#include <gtest/gtest.h>

using Vec1 = Eigen::Matrix<double, 1, 1>;
using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
namespace raytrace {

// todo: move to numerics
template <int cols, typename Callable>
Eigen::Matrix<double, cols, 1> numerical_gradient(const Eigen::Matrix<double, cols, 1> &x,
                                                  const Callable &fcn,
                                                  const double    feps = 1e-6) {
  using OutVec = Eigen::Matrix<double, cols, 1>;

  OutVec jac = OutVec::Zero();

  for (int k = 0; k < cols; ++k) {
    OutVec zero = OutVec::Zero();
    zero(k)     = feps;

    jac(k) = (fcn(x + zero) - fcn(x - zero)) / (2 * feps);
  }
  return jac;
}

double gfunc(const Vec2 &z) {
  return z(0) * z(0) + 5 * z(1);
}
}

namespace rt = raytrace;

TEST(NumericalGradient, gfunc) {
  constexpr double EPS = 1e-6;

  //
  // Setup
  //

  const Vec2 x(1.0, 0.0);
  const Vec2 expected(2.0, 5.0);

  //
  // Action
  //

  const Vec2 J = rt::numerical_gradient<2>(x, rt::gfunc);

  //
  // Verification
  //

  EXPECT_LT((J - expected).lpNorm<Eigen::Infinity>(), EPS);
}

TEST(CostGradient, numerical) {
  //
  // Setup
  //

  const double                    theta = 0.2;
  const Eigen::Rotation2D<double> R(theta);

  std::vector<Vec2> x;
  std::vector<Vec2> z;

  for (int k = -1; k < 2; ++k) {
    const Vec2 v(k * 0.5, -0.5 * k);
    x.emplace_back(v);
    z.emplace_back(R * v);
  }

  const Vec1 eta(0.3);

  const auto fcn = std::bind(rt::cost, std::placeholders::_1, x, z);

  const Vec1 numerical_dcost = rt::numerical_gradient(eta, fcn);
  std::cout << "num: " << numerical_dcost << std::endl;
  std::cout << "ana: " << rt::dcost(eta, x, z) << std::endl;

  const auto jac = std::bind(rt::dcost, std::placeholders::_1, x, z);

  rt::gauss_newton<1>(eta, fcn, jac);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}