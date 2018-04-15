#pragma once

#include <eigen3/Eigen/Dense>

namespace geometry {

struct Line {
  using Vec3 = Eigen::Vector3d;
  Vec3 point;
  Vec3 direction;
};

}  // namespace geometry
