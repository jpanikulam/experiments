#include "estimation/demos/make_model.hh"

namespace estimation {

NonlinearCameraModel make_model() {
  estimation::ProjectionCoefficients proj_coeffs;
  proj_coeffs.fx = 279.76;
  proj_coeffs.fy = 280.034;
  proj_coeffs.cx = 235.992;
  proj_coeffs.cy = 141.951;
  proj_coeffs.k1 = 0.0669332;
  proj_coeffs.k2 = -0.224151;
  proj_coeffs.p1 = 0.00993584;
  proj_coeffs.p2 = 0.00848696;
  proj_coeffs.k3 = 0.10179;
  proj_coeffs.rows = 270;
  proj_coeffs.cols = 480;

  const NonlinearCameraModel model(proj_coeffs);
  return model;
}

}  // namespace estimation