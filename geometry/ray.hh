#pragma once

#include <Eigen/Dense>

namespace geometry {

struct Ray {
  using Vec3 = Eigen::Vector3d;
  Vec3 origin;
  Vec3 direction;

  Vec3 operator()(const double t) const {
    return origin + (t * direction);
  }
};
}