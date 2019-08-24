#include "planning/simulation/interactable_geometry.hh"

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
  return intersections;
}

std::vector<BoundingBoxIntersection> InteractableGeometry::all_intersections(
    const InteractableGeometry::BBox3& bbox) {
  return {};
}

}  // namespace simulation
}  // namespace jcc