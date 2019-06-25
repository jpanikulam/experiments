#include "numerics/functions/quadratic.hh"

#include "testing/gtest.hh"

namespace numerics {

TEST(TestQuadraticFunction, quadratic_evaluates_appropriately) {
  const double a = 3.0;
  const double b = 6.0;
  const double c = 4.0;

  const auto quad_fnc = QuadraticFunction1d(a, b, c);

  EXPECT_DOUBLE_EQ(quad_fnc(0.0), c);
  EXPECT_DOUBLE_EQ(quad_fnc(1.0), a + b + c);
  EXPECT_DOUBLE_EQ(quad_fnc(-1.0), a + -b + c);
}

TEST(TestQuadraticFunction, fits_quadratic) {
  const double a = 3.0;
  const double b = 6.0;
  const double c = 4.0;

  const auto quad_fnc = QuadraticFunction1d(a, b, c);
  const auto fit_quad_fnc =
      fit_quadratic1d(0.0, quad_fnc(0.0), 1.0, quad_fnc(1.0), 5.0, quad_fnc(5.0));

  constexpr double EPS = 1e-6;
  EXPECT_NEAR(quad_fnc(0.0), fit_quad_fnc(0.0), EPS);
  EXPECT_NEAR(quad_fnc(1.0), fit_quad_fnc(1.0), EPS);
  EXPECT_NEAR(quad_fnc(-1.0), fit_quad_fnc(-1.0), EPS);
}

TEST(TestQuadraticFunction, fits_linear) {
  const double a = 0.0;
  const double b = 6.0;
  const double c = 4.0;

  const auto quad_fnc = QuadraticFunction1d(a, b, c);
  const auto fit_quad_fnc =
      fit_quadratic1d(0.0, quad_fnc(0.0), 1.0, quad_fnc(1.0), 5.0, quad_fnc(5.0));

  constexpr double EPS = 1e-6;
  EXPECT_NEAR(quad_fnc(0.0), fit_quad_fnc(0.0), EPS);
  EXPECT_NEAR(quad_fnc(1.0), fit_quad_fnc(1.0), EPS);
  EXPECT_NEAR(quad_fnc(-1.0), fit_quad_fnc(-1.0), EPS);

  EXPECT_NEAR(fit_quad_fnc.coeffs().at(0), 0.0, EPS);
}

TEST(TestQuadraticFunction, roots_quadratic_complex) {
  const double a = 2.0;
  const double b = 4.0;
  const double c = 6.0;

  const auto quad_fnc = QuadraticFunction1d(a, b, c);
  const auto roots = quad_fnc.roots();
  const auto root_0 = roots.at(0);
  const auto root_1 = roots.at(1);

  constexpr double EPS = 1e-6;
  EXPECT_NEAR(root_0.real(), -1.0, EPS);
  EXPECT_NEAR(root_1.real(), -1.0, EPS);

  EXPECT_NEAR(std::min(root_0.imag(), root_1.imag()), -std::sqrt(2.0), EPS);
  EXPECT_NEAR(std::max(root_0.imag(), root_1.imag()), std::sqrt(2.0), EPS);
}

TEST(TestQuadraticFunction, roots_quadratic_real) {
  const double a = 2.0;
  const double b = 6.0;
  const double c = 4.0;

  const auto quad_fnc = QuadraticFunction1d(a, b, c);
  const auto roots = quad_fnc.roots();
  const double root_0 = roots.at(0).real();
  const double root_1 = roots.at(1).real();

  constexpr double EPS = 1e-6;
  EXPECT_NEAR(quad_fnc(root_0), 0.0, EPS);
  EXPECT_NEAR(quad_fnc(root_1), 0.0, EPS);
}

TEST(TestQuadraticFunction, roots_linear) {
  const double a = 0.0;
  const double b = 6.0;
  const double c = 4.0;

  const auto quad_fnc = QuadraticFunction1d(a, b, c);
  const auto roots = quad_fnc.roots();

  EXPECT_EQ(roots.at(0), roots.at(1));
  constexpr double EPS = 1e-6;
  EXPECT_NEAR(roots.at(0).real(), -c / b, EPS);
}

TEST(TestQuadraticFunction, min) {
  const double a = 3.0;
  const double b = 6.0;
  const double c = 4.0;

  const auto quad_fnc = QuadraticFunction1d(a, b, c);
  const double min = quad_fnc.min_x();
  constexpr double EPS = 1e-8;
  EXPECT_NEAR(min, -1.0, EPS);
}

}  // namespace numerics
