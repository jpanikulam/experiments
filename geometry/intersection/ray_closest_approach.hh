#pragma once

// TODO: Move ray_ray_approx_intersect as well

#include "geometry/ray.hh"
#include "geometry/shapes/line_segment.hh"

#include "eigen.hh"

namespace geometry {

struct LineApproachParameters {
  double along_ray;
  double squared_distance;
  jcc::Vec3 on_line;
};

LineApproachParameters line_ray_closest_approach(const Ray& a,
                                                 const shapes::LineSegment& b);

}  // namespace geometry