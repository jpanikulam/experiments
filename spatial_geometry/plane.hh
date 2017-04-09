#pragma once

#include <Eigen/Dense>
#include "out.hh"

#include "spatial_geometry/line.hh"

namespace geometry {

struct Plane {
  using Vec3 = Eigen::Vector3d;

  Vec3 origin;
  Vec3 normal;

  bool intersect(const Line& line, Out<Vec3> point) const;
  bool intersect(const Ray& ray, Out<Vec3> point) const;
};
}  // namespace geometry
