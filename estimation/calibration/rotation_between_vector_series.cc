#include "estimation/calibration/rotation_between_vector_series.hh"

#include "logging/assert.hh"
#include "numerics/numdiff.hh"

namespace estimation {

RotationBetweenVectorSeries rotation_between_vector_series(
    const std::vector<jcc::Vec3>& a, const std::vector<jcc::Vec3>& b) {
  JASSERT_EQ(a.size(), b.size(), "A and B must be the same size");

  SO3 R_b_from_a;
  jcc::Vec3 average_error;
  for (int iter = 0; iter < 10; ++iter) {
    MatNd<3, 1> Jtv = MatNd<3, 1>::Zero();
    MatNd<3, 3> JtJ = MatNd<3, 3>::Zero();

    average_error.setZero();
    for (std::size_t i = 0; i < a.size(); ++i) {
      const auto a_i = a[i];
      const auto b_i = b[i];

      const auto error_fcn = [&R_b_from_a, &a_i, &b_i](const jcc::Vec3& w) {
        const jcc::Vec3 v = (R_b_from_a * SO3::exp(w)) * a_i - b_i;
        return v;
      };
      const auto J_i = numerics::numerical_jacobian<3, 3>(jcc::Vec3::Zero(), error_fcn, 1e-6);

      const jcc::Vec3 v = error_fcn(jcc::Vec3::Zero());
      average_error += v;
      Jtv += J_i * v;
      JtJ += J_i.transpose() * J_i;
    }

    average_error /= a.size();
    const Eigen::LDLT<MatNd<3, 3>> ldlt(JtJ);
    const jcc::Vec3 delta = -ldlt.solve(Jtv);
    R_b_from_a = R_b_from_a * SO3::exp(delta);
  }

  RotationBetweenVectorSeries rot;
  rot.b_from_a = R_b_from_a;
  rot.success = true;
  rot.average_error = average_error;

  return rot;
}
}  // namespace estimation
