#pragma once

// TODO: Move ray_ray_approx_intersect as well

#include "geometry/shapes/line_segment.hh"
#include "geometry/shapes/ray.hh"

#include "util/optional.hh"

#include "eigen.hh"

namespace geometry {

struct LineApproachParameters {
  double along_ray;
  double squared_distance;
  jcc::Vec3 on_line;
};

jcc::Optional<LineApproachParameters> line_ray_closest_approach(
    const Ray& a, const shapes::LineSegment& b);

}  // namespace geometry