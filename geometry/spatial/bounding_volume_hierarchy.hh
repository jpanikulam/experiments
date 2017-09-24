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
  struct TreeElement {
    struct Node {
      int left_child_index;
    };

    struct Leaf {
      int start;
      int end;
    };

    union {
      Node node;
      Leaf leaf;
    };
    BoundingBox<DIM> bounding_box;
    bool is_leaf = false;
  };

  struct AABB {
    BoundingBox<DIM> bbox;
    int volume_index;
  };

  using NodeBuildVisitorFunction = std::function<void(const BoundingBox<DIM> &box, int depth, bool leaf)>;

  // Build a bvh
  // @param[in] volumes The volumes around which to build the hierarchy
  // @param[in] visitor A function that will be called on every new node
  void build(const std::vector<Volume *> &volumes,
             const NodeBuildVisitorFunction &visitor = [](const BoundingBox<DIM> &, int, bool) {});

  Intersection intersect(const Ray &ray) const override;
  bool does_intersect(const Ray &ray) const override;
  BoundingBox<DIM> bounding_box() const override;

  // Expose the whole tree
  const std::vector<TreeElement> &tree() const {
    return tree_;
  }

  const std::vector<AABB> &aabb() const {
    return aabb_;
  }

private:
  int add_node_and_children(std::vector<AABB> &bounding_boxes,
                            size_t node_index,
                            size_t begin,
                            size_t end,
                            int depth,
                            const NodeBuildVisitorFunction &visitor);

  std::vector<TreeElement> tree_;
  std::vector<AABB> aabb_;
};
} // namespace geometry
} // namespace spatial
