#include "planning/simulation/interactable_geometry.hh"

#include <algorithm>

namespace jcc {
namespace simulation {

std::vector<RayIntersection> InteractableGeometry::all_intersections(
    const geometry::Ray& ray) {
  std::vector<RayIntersection> intersections;
  for (const auto& bbox : aabb_) {
    const auto intersection = bbox.second.bbox.intersect(ray);
    if (intersection.intersected) {
      intersections.push_back(
          {bbox.first, intersection.distance, ray(intersection.distance)});
    }
  }
  std::sort(intersections.begin(), intersections.end(),
            [](const RayIntersection& a, const RayIntersection& b) -> bool {
              return a.distance < b.distance;
            });

  return intersections;
}

std::vector<BoundingBoxIntersection> InteractableGeometry::all_intersections(
    const InteractableGeometry::BBox3& other) {
  std::vector<BoundingBoxIntersection> intersections;

  for (const auto& bbox : aabb_) {
    const auto intersection = bbox.second.bbox.intersect(other);

    if (intersection.contained) {
      intersections.push_back({bbox.first, intersection.contained});
    }
  }

  return intersections;
}

}  // namespace simulation
}  // namespace jcc