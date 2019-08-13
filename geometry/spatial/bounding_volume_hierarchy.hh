#pragma once

#include "geometry/spatial/bounding_box.hh"
#include "geometry/spatial/volume.hh"

#include "out.hh"

#include <functional>
#include <vector>

namespace geometry {
namespace spatial {

// Fast intersection of rays with a large number of intersectible volumes
//
// Selected Applications:
//  1. Simulating LIDAR by estimating intersection with millions of triangle meshes
//  2. Implementing ray-tracing
//  3. 3D mouse picking for 3D UI's
//
// Concept of Operation:
//  1. Store a list of `Volume`s which all expose a method for intersecting a ray, and
//     a method for computing a bounding box
//  2. Build a binary tree of successively smaller axis-aligned bounding boxes
//  3. Test ray-BVH intersection in log2(N) time
//
// NOTE:
// - Standard flat-storage for a binary tree is employed
//
// Bibliography:
// [1] Wikipedia Article: "Bounding Volume Hierarchy"
//     https://en.wikipedia.org/wiki/Bounding_volume_hierarchy
// [2] Spatial Splits in Bounding Volume Hierarchies
//     https://www.nvidia.com/docs/IO/77714/sbvh.pdf
// [3] Official US Federal Government Position of the Height of an Empty Tree
//     https://xlinux.nist.gov/dads/HTML/height.html
class BoundingVolumeHierarchy final : public Volume {
 public:
  // Dimension of the bounding volume hierarchy
  static constexpr int DIM = 3;

  // Public for serialization purposes
  // An element of the BVH tree is one of two things:
  //   1. A node: Which has two children
  //   2. A leaf: Which contains some number of intersectible `Volume`s
  //
  // Both leaves and nodes have a bounding box for testing intersection
  //
  // NOTE:
  // We use a `union` here, not for type erasure, but for memory efficiency
  //
  // NOTE:
  // Many Volumes are in the same leaf, because the constant factor for traversal time
  // overwhelms the constant factor for leaf exploration time for ~8-16 volumes
  //
  // This is because we can accelerate testing many volumes with SIMD instructions, and
  // fit all of those volumes in a few cache lines
  struct TreeElement {
    struct Node {
      // The index in the TreeElements list of the first child of this node
      // The *right* child is `1 + left_child_index`
      int left_child_index;
    };

    struct Leaf {
      // Index of first `Volume` in the leaf, referencing the TreeElement list
      int start;
      // Index of last `Volume` in the leaf, referencing the TreeElement list
      int end;
    };

    union {
      Node node;
      Leaf leaf;
    };

    // The bounding box for this node
    BoundingBox<DIM> bounding_box;

    // Tag for specifying the type of the element
    bool is_leaf = false;
  };

  // Public for serialization purposes
  //
  // An AABB here is an axis-aligned bounding box, with a pointer
  // (here, an index) to the volume it corresponds to.
  struct AABB {
    BoundingBox<DIM> bbox;
    int volume_index;
  };

  // These visitors are intended chiefly for debugging.
  using NodeBuildVisitorFunction =
      std::function<void(const BoundingBox<DIM> &box, int depth, bool leaf)>;
  using IntersectVisitorFunction =
      std::function<void(const TreeElement &tree_element, bool intersected)>;

  // Build a bvh
  // @param[in] volumes The volumes around which to build the hierarchy
  //                    The lifetime of the referenced volumes must exceed
  //                    the lifetime of the BVH
  // @param[in] visitor A function that will be called on the creation of
  //                    each new node
  void build(const std::vector<Volume *> &volumes,
             const NodeBuildVisitorFunction &visitor =
                 [](const BoundingBox<DIM> &, int, bool) {});

  // Intersect a ray with the bounding volume hierarchy
  // @param[in] ray The ray to intersect with the BVH
  // @param[in] visitor A function that will be called whenever an
  //                    unvisited tree node is visited. Even ones
  //                    that the ray does not intersect
  // @returns A report about the location of the possible intersection
  Intersection intersect(const Ray &ray, const IntersectVisitorFunction &visitor) const;

  // Intersect a ray with the bounding volume hierarchy
  // @param[in] ray The ray to intersect with the BVH
  // @returns A report about the location of the possible intersection
  Intersection intersect(const Ray &ray) const override {
    return intersect(ray, [](const TreeElement &tree_element, bool) {});
  }

  //
  // NOTE: This is implemented to support the full `Volume` interface
  //       This function
  //
  // Shortcut the traversal of the tree and
  // return early if an intersection is detected
  //
  // @param[in] ray The ray to test intersection against
  //
  // @returns True if the ray intersects the BVH
  //
  bool does_intersect(const Ray &ray) const override;

  // @returns a BoundingBox for the entire BVH
  BoundingBox<DIM> bounding_box() const override;

  // Exposed for serialization
  //
  // @returns The full BVH tree
  const std::vector<TreeElement> &tree() const {
    return tree_;
  }

  // Exposed for serialization
  //
  // @returns The axis aligned bounding boxes
  const std::vector<AABB> &aabb() const {
    return aabb_;
  }

 private:
  // Recursively construct a node and its children
  //
  // @param[in, out] node_index The index of the node to be created
  // @param[in] begin The index of the first descendant
  // @param[in] end The index of the last descendant
  // @param[in] depth The current depth inside the tree
  // @param[in] visitor An optional visitor to be called upon node
  //                    construction completion
  int add_node_and_children(std::vector<AABB> &bounding_boxes,
                            size_t node_index,
                            size_t begin,
                            size_t end,
                            int depth,
                            const NodeBuildVisitorFunction &visitor);

  // Visit, determine intersection and determine distance to a leaf node
  //
  // @param[in] leaf The leaf to inspect
  // @param[in] ray The ray to test intersection with
  // @param[out] closest The index of the nearest volume
  //                     along the ray inside the leaf
  //                     == -1 if no intersection is found. Test this.
  //
  // @returns The distance to the nearest volume inside the leaf
  //          Undefined if no intersection
  double traverse_leaf(const TreeElement::Leaf &leaf,
                       const Ray &ray,
                       Out<int> closest) const;

  // Flat-storage of the tree
  std::vector<TreeElement> tree_;
  // Flat storage of the associated axis-aligned bounding boxes
  std::vector<AABB> aabb_;
};
}  // namespace spatial
}  // namespace geometry
