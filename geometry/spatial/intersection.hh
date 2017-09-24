#pragma once

#include <cstdef>

namespace geometry {
namespace spatial {

struct Intersection {
  double distance;
  bool intersected = false;

  static Intersection no_intersection() {
    Intersection intersect;
    intersect.intersected = false;
    return intersect;
  }
};
}
}
