#pragma once

#include <functional>

#include "eigen.hh"

namespace geometry {
namespace shapes {

using AffineMat3 = MatNd<3, 3>;
struct Ellipse {
  // Cholesky factor for the ellipse
  // Stored as a lower-triangular matrix
  AffineMat3 cholesky_factor;

  // Ellipse centroid
  jcc::Vec3 p0;

  // The ellipse is an implicit function, described as:
  // (x - p0).T * L * L.T * (x - p0) = 1.0
  //
  // Where L is the cholesky factor
};

struct EllipseFit {
  // Reserved but unused
  double total_error;

  // The ellipse that has been fit to the data
  Ellipse ellipse;
};

using Visitor = std::function<void(const EllipseFit& fit)>;

// Fit an ellipsoid to data
// pts: 3D points that belong (noisly) to the surface of an ellipsoid
// [visitor]: A function that will be called on the current state of
//            the optimization at each iteration
EllipseFit fit_ellipse(const std::vector<jcc::Vec3>& pts, const Visitor& visitor = {});

jcc::Vec3 deform_ellipse_to_unit_sphere(const jcc::Vec3& pt_on_ellipse,
                                        const Ellipse& ellipse);

jcc::Vec3 deform_unit_sphere_to_ellipse(const jcc::Vec3& pt_on_sphere,
                                        const Ellipse& ellipse);

}  // namespace shapes
}  // namespace geometry