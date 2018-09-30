#pragma once

#include "eigen.hh"

#include "geometry/shapes/circle.hh"
#include "geometry/shapes/halfspace.hh"
#include "geometry/shapes/sphere.hh"

#include "util/optional.hh"

namespace geometry {
namespace intersection {

jcc::Optional<shapes::Circle> sphere_plane_intersection(const shapes::Sphere& sphere,
                                                        const shapes::Plane& plane) {
  const double chord_height = std::abs(sd_halfspace(sphere.center, plane));
  if (chord_height > sphere.radius) {
    return {};
  }

  // Call upon Pythagoras!
  const double radius_sq = sphere.radius * sphere.radius;
  const double chord_height_sq = chord_height * chord_height;
  const double r = std::sqrt(radius_sq - chord_height_sq);
  const shapes::Circle circle{plane, r};
  return circle;
}

}  // namespace intersection
}  // namespace geometry