#pragma once

#include "eigen.hh"
#include "eigen_helpers.hh"

#include <functional>
#include <vector>

namespace geometry {
namespace spatial {
using Vec3 = Eigen::Vector3d;

struct Simplex {
  constexpr int VERTS = 4;
  std::array<Vec3, VERTS> vertices;
  int dim = 0;
};

struct Shape {
  std::vector<Simplex> simplices;
};

void gjk(const Shape& a, const Vec3& b, const std::function<Simplex>& visitor);

}  // namespace spatial
}  // namespace geometry