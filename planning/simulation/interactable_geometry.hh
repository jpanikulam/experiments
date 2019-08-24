#pragma once

#include "eigen.hh"
#include "geometry/shapes/ray.hh"
#include "geometry/spatial/bounding_box.hh"

#include <map>
namespace jcc {
namespace simulation {

struct RayIntersection {
  int intersected_id = -1;
  double distance = -1.0;

  jcc::Vec3 intersection_point;
};

struct BoundingBoxIntersection {
  bool fully_contained = false;
  int id;
};

class InteractableGeometry {
 public:
  using BBox3 = geometry::spatial::BoundingBox<3>;
  std::vector<RayIntersection> all_intersections(const geometry::Ray& ray);

  std::vector<BoundingBoxIntersection> all_intersections(const BBox3& bbox);

  void add_intersectible(int id, const BBox3& bbox) {
    aabb_[id] = {bbox};
  }

  void clear(int id) {
    aabb_.erase(id);
  }

  void clear() {
    aabb_.clear();
  }

 private:
  // GeometryBuffer geometry_;

  struct ManagedBoundingBox {
    BBox3 bbox;
  };

  std::map<int, ManagedBoundingBox> aabb_;
};
}  // namespace simulation
}  // namespace jcc