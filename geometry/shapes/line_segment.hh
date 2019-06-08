#pragma once

#include "eigen.hh"

namespace geometry {
namespace shapes {

struct LineSegment {
  jcc::Vec3 point_a;
  jcc::Vec3 point_b;
};

}  // namespace shapes
}  // namespace geometry