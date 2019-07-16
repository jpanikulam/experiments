#pragma once

#include "eigen.hh"
#include "out.hh"

#include "geometry/line.hh"
#include "geometry/shapes/ray.hh"
#include "geometry/types/unit_vector.hh"

namespace geometry {

struct Plane {
  using Vec3 = Eigen::Vector3d;

  Vec3 origin;
  Unit3 normal;

  bool intersect(const Line& line, Out<Vec3> point) const;
  bool intersect(const Ray& ray, Out<Vec3> point) const;
};

inline Plane operator*(const SE3& destination_from_source,
                       const Plane& plane_source_frame) {
  const Plane plane_destination_frame{
      .origin = destination_from_source * plane_source_frame.origin,
      .normal = destination_from_source * plane_source_frame.normal};
  return plane_destination_frame;
}
}  // namespace geometry
