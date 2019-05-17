#include "geometry/shapes/ellipse.hh"

namespace geometry {
namespace shapes {

jcc::Vec3 deform_ellipse_to_unit_sphere(const jcc::Vec3& pt_on_ellipse,
                                        const Ellipse& ellipse) {
  const jcc::Vec3 pt_on_sphere =
      ellipse.cholesky_factor.transpose().inverse() * (pt_on_ellipse - ellipse.p0);
  return pt_on_sphere;
}

jcc::Vec3 deform_unit_sphere_to_ellipse(const jcc::Vec3& pt_on_sphere,
                                        const Ellipse& ellipse) {
  const jcc::Vec3 pt_on_ellipse =
      (ellipse.cholesky_factor.transpose() * pt_on_sphere) + ellipse.p0;
  return pt_on_ellipse;
}

}  // namespace shapes
}  // namespace geometry