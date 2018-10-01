#pragma once

#include "eigen.hh"

#include "geometry/shapes/halfspace.hh"

namespace geometry {
namespace shapes {

struct Circle {
  Plane plane;
  double radius;
};

}  // namespace shapes
}  // namespace geometry
