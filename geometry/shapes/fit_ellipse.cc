#include "geometry/shapes/fit_ellipse.hh"

#include "numerics/numdiff.hh"

namespace geometry {
namespace shapes {

namespace {
constexpr int OBS_DIM = 1;
constexpr int PARAM_DIM = 9;
using ParamVec = VecNd<PARAM_DIM>;

// A function that is monotonic in the squared distance to an ellipse
double monotonic_sqd_ellipse(const Eigen::Vector3d& point, const Ellipse& ellipse) {
  const jcc::Vec3 q = point - ellipse.p0;
  const jcc::Vec3 deformed_point = ellipse.cholesky_factor.transpose().inverse() * q;

  const double error_to_sphere = deformed_point.norm() - 1.0;
  return error_to_sphere;
}

// Create a reasonable initialization
ParamVec create_reasonable_initialization() {
  ParamVec params;
  params[0] = 1.0;
  params[1] = 1.0;
  params[2] = 1.0;
  params[3] = 0.0;
  params[4] = 0.0;
  params[5] = 0.0;
  params[6] = 0.0;
  params[7] = 0.0;
  params[8] = 0.0;
  return params;
}

}  // namespace

EllipseFit fit_ellipse(const std::vector<jcc::Vec3>& pts, const Visitor& visitor) {
  //
  // Create function mapping parameter vector to an ellipse
  //

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

  //
  // Set up definitions for optimizing
  //

  constexpr int MAX_ITERS = 20;
  // A modest levenberg coefficient for calming the torrid seas of optimization
  constexpr double LM_COEFF = 1.0;

  using JtJMat = MatNd<PARAM_DIM, PARAM_DIM>;
  using JtvMat = MatNd<PARAM_DIM, OBS_DIM>;

  ParamVec params = create_reasonable_initialization();

  for (int k = 0; k < MAX_ITERS; ++k) {
    JtJMat jtj = JtJMat::Zero();
    JtvMat jtv = JtvMat::Zero();

    //
    // Compute innovations and accumlate JtJ and Jtv
    //

    for (const auto& pt : pts) {
      const auto single_residual = [&ellipse_from_params, &pt](const ParamVec& v) {
        const Ellipse ellipse = ellipse_from_params(v);
        const auto residual = monotonic_sqd_ellipse(pt, ellipse);
        return (VecNd<OBS_DIM>() << residual).finished();
      };

      const auto J = -numerics::numerical_jacobian<OBS_DIM>(params, single_residual);
      const auto v = single_residual(params);
      jtj += J.transpose() * J;
      jtv += J.transpose() * v;
    }

    //
    // Damp the optimization --
    // - The currently used distance measurement includes inverting a matrix,
    //   so avoiding singularity is important
    // - It's possible to do this optimization in terms of L_inv instead of L, which
    //   removes that sensitivity, and makes it possible to fit flat ellipses
    // - If you find this text by googling and have that use case, send me an email and
    //   I'll write it for you.
    //
    const JtJMat damped_jtj = (JtJMat::Identity() * LM_COEFF) + jtj;
    const Eigen::LLT<MatNd<PARAM_DIM, PARAM_DIM>> llt(damped_jtj);
    const ParamVec delta = llt.solve(jtv);

    params += delta;
    if (visitor) {
      visitor({0.0, ellipse_from_params(params)});
    }
  }

  return {0.0, ellipse_from_params(params)};
}

}  // namespace shapes
}  // namespace geometry