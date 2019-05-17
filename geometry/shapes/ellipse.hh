#pragma once

#include "eigen.hh"

namespace geometry {
namespace shapes {

struct Ellipse {
  // Cholesky factor for the ellipse
  // Stored as a lower-triangular matrix
  MatNd<3, 3> cholesky_factor;

  // Ellipse centroid
  jcc::Vec3 p0;

  // The ellipse is an implicit function, described as:
  // (x - p0).T * L * L.T * (x - p0) = 1.0
  //
  // Where L is the cholesky factor
};

jcc::Vec3 deform_ellipse_to_unit_sphere(const jcc::Vec3& pt_on_ellipse,
                                        const Ellipse& ellipse);

jcc::Vec3 deform_unit_sphere_to_ellipse(const jcc::Vec3& pt_on_sphere,
                                        const Ellipse& ellipse);

}  // namespace shapes
}  // namespace geometry