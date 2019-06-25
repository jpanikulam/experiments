#pragma once

#include <array>
#include <complex>

namespace numerics {

// TODO: Decide if a default construtor makes sense.
class QuadraticFunction1d {
 public:
  QuadraticFunction1d(double a, double b, double c) : a_(a), b_(b), c_(c) {
  }

  double operator()(const double x) const {
    // Horner-evaluate
    return c_ + x * (b_ + (x * a_));
  }

  std::array<std::complex<double>, 2u> roots() const;

  // Note: This returns nonsense if a is zero
  double min_x() const {
    return -b_ / (2 * a_);
  }

  std::array<double, 3> coeffs() const {
    return {a_, b_, c_};
  }

 private:
  double a_;
  double b_;
  double c_;
};

QuadraticFunction1d fit_quadratic1d(
    double x1, double y1, double x2, double y2, double x3, double y3);

}  // namespace numerics