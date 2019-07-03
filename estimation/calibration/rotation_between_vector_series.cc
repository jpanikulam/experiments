#include "estimation/calibration/rotation_between_vector_series.hh"

#include "vision/robust_estimator.hh"

#include "logging/assert.hh"
#include "numerics/numdiff.hh"

namespace estimation {

RotationBetweenVectorSeries rotation_between_vector_series(
    const std::vector<jcc::Vec3>& a,
    const std::vector<jcc::Vec3>& b,
    const OptimizationVisitor& visitor) {
  JASSERT_EQ(a.size(), b.size(), "A and B must be the same size");
  RotationBetweenVectorSeries series;
  series.weights.resize(a.size(), 0.0);
  series.success = false;

  const auto est = slam::HuberCost(0.4);

  series.b_from_a = SO3::exp(jcc::Vec3(M_PI, 0.1, -0.4));

  double prev_avg_error_sq = std::numeric_limits<double>::max();
  for (int iter = 0; iter < 500; ++iter) {
    if (visitor) {
      visitor(series);
    }
    MatNd<3, 1> Jtv = MatNd<3, 1>::Zero();
    MatNd<3, 3> JtJ = MatNd<3, 3>::Zero();

    series.average_error.setZero();
    for (std::size_t i = 0; i < a.size(); ++i) {
      const auto a_i = a[i];
      const auto b_i = b[i];

      const auto error_fcn = [&R_b_from_a = series.b_from_a, &a_i, &b_i](
                                 const jcc::Vec3& w) {
        const jcc::Vec3 v =
            b_i.normalized() - ((SO3::exp(w) * R_b_from_a) * a_i.normalized());
        return v;
      };
      const auto J_i =
          numerics::numerical_jacobian<3, 3>(jcc::Vec3::Zero(), error_fcn, 1e-6);

      const jcc::Vec3 v = error_fcn(jcc::Vec3::Zero());
      const auto cost_and_weight = est(v.transpose() * v);
      const double w = cost_and_weight.weight;
      series.weights[i] = w;

      series.average_error += v;
      Jtv += w * J_i * v;
      JtJ += w * J_i.transpose() * J_i;
    }

    series.average_error /= a.size();

    constexpr double MARQUARDT_LAMBDA = 1e1;
    const MatNd<3, 3> marquardt = (JtJ.diagonal() * MARQUARDT_LAMBDA).asDiagonal();
    const Eigen::LDLT<MatNd<3, 3>> ldlt(marquardt + JtJ);
    JASSERT_EQ(ldlt.info(), Eigen::Success, "LDLT must not fail");

    const jcc::Vec3 delta = ldlt.solve(Jtv);
    series.b_from_a = SO3::exp(delta) * series.b_from_a;

    const double average_error_norm_sq = series.average_error.dot(series.average_error);
    if (std::abs(average_error_norm_sq - prev_avg_error_sq) < 0.000001) {
      break;
    }

    prev_avg_error_sq = average_error_norm_sq;
  }

  series.success = true;
  if (visitor) {
    visitor(series);
  }
  return series;
}
}  // namespace estimation
