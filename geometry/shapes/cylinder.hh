#pragma once

#include "eigen.hh"
#include "geometry/types/unit_vector.hh"

namespace geometry {
namespace shapes {

struct Cylinder {
  // Center of the bottom circle
  Vec3 bottom_center;

  // 2-Dof orientation
  // Bottom-plane normal
  UnitVector3 normal;

  // Distance between top and bottom circles
  double height;

  // Radius of both circles
  double radius;
};

//____________
//      |
//     c.---.p
//      |  /
//      | /
//______./____
//      b
/*
inline double sd_cylinder(const Eigen::Vector3d& point, const Cylinder& cylinder) {
  const jcc::Vec3 bp = point - cylinder.bottom_center;

  const double length_cb = bp.dot(cylinder.normal.vector());
  const double length_cb_sqr = length_cb * length_cb;

  const double length_pc = std::sqrt(length_cb_sqr)
}
*/

}  // namespace shapes
}  // namespace geometry
