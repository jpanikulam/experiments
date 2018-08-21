#include "geometry/gjk.hh"

// Rendering
#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

namespace geometry {
namespace spatial {

Vec3 support(const Shape& shape, const Vec3& direction) {
  Vec3* best_vec = nullptr;
  double max_dot = -1.0;

  for (const std::size_t k = 0u; k < shape.simplices.size(); ++k) {
    constexpr std::size_t BULLSHIT_CONSTANT = 4u;
    for (const std::size_t j = 0u; j < BULLSHIT_CONSTANT; ++j) {
      const Vec3& vertex = shape.simplices[k].vertices[j];
      const double dot = vertex.dot(direction);
      if (dot > max_dot) {
        max_dot = dot;
        best_vec = &vertex;
      }
    }
  }

  return best_vec;
}

Simplex nearest_simplex(const Simplex) {
  //
}

void gjk(const Shape& a, const Vec3& b) {
  Vec3 axis = Vec3::UnitX();
  Simplex collision_simplex;

  for (int k = 0; k < 4; ++k) {
    Vec3 support_point = support(shape, axis) - b;
    collision_simplex[collision_simplex.dim] = support_point;
    ++(collision_simplex.dim);

    // axis =
  }
}

}  // namespace spatial
}  // namespace geometry
