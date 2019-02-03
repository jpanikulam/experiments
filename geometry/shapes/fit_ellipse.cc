#include "geometry/shapes/fit_ellipse.hh"

#include "numerics/numdiff.hh"

namespace geometry {
namespace shapes {

namespace {

// A function that is monotonic in the squared distance to an ellipse
double monotonic_sqd_ellipse(const Eigen::Vector3d& point, const Ellipse& ellipse) {
  const jcc::Vec3 q = point - ellipse.p0;
  const jcc::Vec3 deformed_point = ellipse.cholesky_factor.transpose().inverse() * q;

  const double error_to_sphere = deformed_point.norm() - 1.0;
  return error_to_sphere;
}
}  // namespace

EllipseFit fit_ellipse(const std::vector<jcc::Vec3>& pts, const Visitor& visitor) {
  constexpr int OBS_DIM = 1;
  constexpr int PARAM_DIM = 9;
  using ParamVec = VecNd<PARAM_DIM>;
  const auto ellipse_from_params = [](const ParamVec& v) {
    MatNd<3, 3> L;

    // clang-format off
    L << v[0],  0.0,   0.0,
         v[3],  v[1],  0.0,
         v[4],  v[5],  v[2];
    // clang-format on

    const jcc::Vec3 p = (jcc::Vec3() << v[6], v[7], v[8]).finished();
    return Ellipse{L, p};
  };

  constexpr int MAX_ITERS = 20;
  // ParamVec params = ParamVec::Ones();
  ParamVec params;
  params[0] = 1.5;
  params[1] = 1.5;
  params[2] = 1.5;
  params[3] = 0.0;
  params[4] = 0.0;
  params[5] = 0.0;
  params[6] = 0.0;
  params[7] = 0.0;
  params[8] = 0.0;

  using JtJMat = MatNd<PARAM_DIM, PARAM_DIM>;

  for (int k = 0; k < MAX_ITERS; ++k) {
    JtJMat jtj;
    jtj.setZero();
    MatNd<PARAM_DIM, OBS_DIM> jtv;
    jtv.setZero();
    for (const auto& pt : pts) {
      const auto single_residual = [&ellipse_from_params, &pt](const ParamVec& v) {
        const Ellipse ellipse = ellipse_from_params(v);
        const auto residual = monotonic_sqd_ellipse(pt, ellipse);
        return (VecNd<OBS_DIM>() << residual).finished();
      };

      const auto J = numerics::numerical_jacobian<OBS_DIM>(params, single_residual);
      const auto v = single_residual(params);
      jtj += J.transpose() * J;
      jtv += J.transpose() * v;
    }

    // A modest levenberg coefficient for calming the optimization
    constexpr double LM_COEFF = 1.0;
    const JtJMat damped_jtj = (JtJMat::Identity() * LM_COEFF) + jtj;
    const Eigen::LLT<MatNd<PARAM_DIM, PARAM_DIM>> llt(damped_jtj);
    const ParamVec delta = llt.solve(jtv);

    params -= delta;
    if (visitor) {
      visitor({0.0, ellipse_from_params(params)});
    }
  }

  return {0.0, ellipse_from_params(params)};
}

}  // namespace shapes
}  // namespace geometry