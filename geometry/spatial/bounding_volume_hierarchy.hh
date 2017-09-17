#pragma once

#include "volume.hh"
#include "sphere_volume.hh"
#include "bounding_box.hh"

#include <functional>
#include <vector>

namespace geometry {
namespace spatial {

// Implemented as a binary spherical bvh
constexpr int DIM = 3;
class BoundingVolumeHierarchy final : public Volume {
public:
  using VisitorFunction = std::function<void(const BoundingBox<DIM> &box, int depth, bool leaf)>;

  // Build a bvh
  // @param[in] volumes The volumes around which to build the hierarchy
  // @param[in] visitor A function that will be called on every new node
  void build(const std::vector<Volume *> &volumes,
             const VisitorFunction &visitor = [](const BoundingBox<DIM> &, int, bool) {});

  Intersection intersect(const Ray &ray) const override;
  bool does_intersect(const Ray &ray) const override;
  BoundingBox<DIM> bounding_box() const override;

private:
  static constexpr int BRANCH_FACTOR = 2;
  struct ManagingVolume {
    BoundingBox<DIM> sphere;
    std::array<int, BRANCH_FACTOR> children;
  };

  int add_node_and_children(std::vector<BoundingBox<DIM>> &bounding_boxes,
                            size_t begin,
                            size_t end,
                            int depth,
                            const VisitorFunction &visitor);

  std::vector<Volume *> volumes_;
  std::vector<ManagingVolume> tree_;
};
} // namespace geometry
} // namespace spatial
