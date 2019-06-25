#include "numerics/functions/quadratic.hh"

#include "eigen.hh"
#include "logging/assert.hh"

namespace numerics {

std::array<std::complex<double>, 2u> QuadraticFunction1d::roots() const {
  if (a_ == 0.0) {
    JASSERT_NE(b_, 0.0, "This thing has no roots");
    const std::complex<double> r1(-c_ / b_, 0.0);
    return {r1, r1};
  }
  using Mat = MatNd<2, 2>;
  Mat companion = Mat::Zero();
  {
    companion(0, 1) = 1.0;
    companion.row(1) = jcc::Vec2(-c_ / a_, -b_ / a_);
  }

  // This massively increases compile times
  const Eigen::EigenSolver<Mat> eig(companion);
  JASSERT_EQ(eig.info(), Eigen::Success, "Eigenvalue computation failed");

  return {eig.eigenvalues()(0), eig.eigenvalues()(1)};
}

QuadraticFunction1d fit_quadratic1d(
    double x1, double y1, double x2, double y2, double x3, double y3) {
  // Set matrix entries such that Ax = [poly(x1)...poly(x2)...poly(x3)];
  MatNd<3, 3> A;
  {
    A.row(0) << x1 * x1, x1, 1.0;
    A.row(1) << x2 * x2, x2, 1.0;
    A.row(2) << x3 * x3, x3, 1.0;
  }

  const jcc::Vec3 b(y1, y2, y3);
  const jcc::Vec3 coeffs = A.colPivHouseholderQr().solve(b);

  return QuadraticFunction1d(coeffs.x(), coeffs.y(), coeffs.z());
}

}  // namespace numerics