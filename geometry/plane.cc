#include "geometry/plane.hh"

namespace geometry {

bool Plane::intersect(const Line& line, Out<Vec3> point) const {
  constexpr double EPS = 1e-7;

  if (line.direction.cross(normal.vec()).norm() < EPS) {
    // Parallel
    return false;
  }

  const double d =
      (origin - line.point).dot(normal.vec()) / (line.direction.dot(normal.vec()));
  *point = line.point + (d * line.direction);
  return true;
}

bool Plane::intersect(const Ray& ray, Out<Vec3> point) const {
  constexpr double EPS = 1e-7;

  if (ray.direction.cross(normal.vec()).norm() < EPS) {
    // Parallel
    return false;
  }

  const double d =
      (origin - ray.origin).dot(normal.vec()) / (ray.direction.dot(normal.vec()));

  *point = ray.origin + (d * ray.direction);

  if (d < 0.0) {
    // Pointing in the wrong direction
    return false;
  } else {
    return true;
  }
}
}  // namespace geometry