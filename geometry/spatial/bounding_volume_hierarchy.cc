#include "geometry/spatial/bounding_volume_hierarchy.hh"

#include "util/heap.hh"

#include <limits>

namespace geometry {
namespace spatial {

namespace {
struct Partition {
  double split_cost;
  double split_value;
  int dim = -1;
  size_t index = 0;
};

// Compute a pivot for the tree
Partition compute_partition(std::vector<BoundingVolumeHierarchy::AABB> &boxes,
                            size_t begin,
                            size_t end,
                            int dim) {
  constexpr bool SMART_PARTITION = true;

  if (!SMART_PARTITION) {
    const auto begin_it = boxes.begin();
    std::sort(begin_it + begin, begin_it + end,
              [dim](const BoundingVolumeHierarchy::AABB &a,
                    const BoundingVolumeHierarchy::AABB &b) {
                return a.bbox.upper()(dim) < b.bbox.upper()(dim);
              });

    const std::size_t best_split_index = (begin + end) / 2;
    Partition partition;
    partition.dim = dim;
    partition.split_value = boxes[best_split_index].bbox.upper()(dim);
    partition.split_cost = -1.0;
    partition.index = best_split_index;

    return partition;

  } else {
    const auto begin_it = boxes.begin();
    std::sort(begin_it + begin, begin_it + end,
              [dim](const BoundingVolumeHierarchy::AABB &a,
                    const BoundingVolumeHierarchy::AABB &b) {
                return a.bbox.upper()(dim) < b.bbox.upper()(dim);
              });

    //
    // Partition the hierarchy based on [2] by testing different possibly box sizes
    //
    BoundingBox<BoundingVolumeHierarchy::DIM> lower_bbox;
    double best_split_cost = std::numeric_limits<double>::max();
    int best_split_index = -1;
    for (size_t k = begin; k < end; ++k) {
      const double lower_size = static_cast<double>(k - begin);
      const double upper_size = static_cast<double>(end - k);
      assert((lower_size + upper_size) == (end - begin));

      BoundingBox<BoundingVolumeHierarchy::DIM> upper_bbox;
      for (size_t u = k; u < end; ++u) {
        upper_bbox.expand(boxes[u].bbox);
      }

      lower_bbox.expand(boxes[k].bbox);

      const double lower_sa = lower_bbox.surface_area();
      const double upper_sa = upper_bbox.surface_area();

      const double split_cost = (lower_sa * lower_size) + (upper_sa * upper_size);

      if (split_cost < best_split_cost) {
        best_split_cost = split_cost;
        best_split_index = k;
      }
    }

    assert(best_split_index >= 0);

    Partition partition;
    partition.dim = dim;
    partition.split_value = boxes[best_split_index].bbox.upper()(dim);
    partition.split_cost = best_split_cost;
    partition.index = best_split_index;
    return partition;
  }
}
}  // namespace

int BoundingVolumeHierarchy::add_node_and_children(
    std::vector<AABB> &bounding_boxes,
    const size_t node_index,
    const size_t begin,
    const size_t end,
    const int depth,
    const NodeBuildVisitorFunction &visitor) {
  auto &node = tree_[node_index];
  for (size_t k = begin; k < end; ++k) {
    node.bounding_box.expand(bounding_boxes[k].bbox);
  }
  const bool is_leaf = (end - begin) < 16;
  // Visit!
  visitor(node.bounding_box, depth, is_leaf);
  node.is_leaf = is_leaf;
  if (is_leaf) {
    node.leaf.start = begin;
    node.leaf.end = end;
    return 1;
  }

  // TODO(jpanikulam): Implement dimension search

  //
  // Partition the tree -- this is a pivot operation so we can use flat storage
  //

  const Partition chosen_partition =
      compute_partition(bounding_boxes, begin, end, depth % 3);
  const int dim = chosen_partition.dim;
  const double value = chosen_partition.split_value;
  std::partition(bounding_boxes.begin() + begin, bounding_boxes.begin() + end,
                 [dim, value](const AABB &a) { return a.bbox.upper()(dim) < value; });

  // The index of the left child is 1 + the last index in the tree
  const int lci = tree_.size();
  node.node.left_child_index = lci;

  // These will be the left and right child
  // Recall that we are using *indices* instead of pointers, so we can modify
  // the size and capacity of the flat storage vector
  tree_.emplace_back();
  tree_.emplace_back();

  // Make the left child and all of its children
  int l_depth = add_node_and_children(bounding_boxes, lci, begin, chosen_partition.index,
                                      depth + 1, visitor);
  // Make the right child and all of its children
  int r_depth = add_node_and_children(bounding_boxes, lci + 1, chosen_partition.index,
                                      end, depth + 1, visitor);
  // Determine the depth of the tree
  int depth_beneath = std::max(l_depth, r_depth);

  return depth_beneath + 1;
}

void BoundingVolumeHierarchy::build(const std::vector<Volume *> &volumes,
                                    const NodeBuildVisitorFunction &visitor) {
  aabb_.resize(volumes.size());
  for (std::size_t k = 0; k < volumes.size(); ++k) {
    aabb_[k].bbox = volumes[k]->bounding_box();
    aabb_[k].volume_index = k;
  }
  // The height of an empty tree is undefined [3]
  tree_.resize(1);
  add_node_and_children(aabb_, 0, 0, aabb_.size(), 0, visitor);
}

double BoundingVolumeHierarchy::traverse_leaf(const TreeElement::Leaf &leaf,
                                              const Ray &ray,
                                              Out<int> closest) const {
  double min_distance = std::numeric_limits<double>::max();
  *closest = -1;

  // Visit all elements in the leaf
  for (int k = leaf.start; k < leaf.end; ++k) {
    const auto &aabb = aabb_[k];
    const Intersection intersection = aabb.bbox.intersect(ray);
    std::cout << "Checking" << std::endl;
    if (intersection.intersected) {
      std::cout << "Got intersection" << std::endl;
      // Note: min(a, b) can be done branchlessly on modern processors, so prefer this
      min_distance = std::min(min_distance, intersection.distance);
      *closest = k;
    }
  }
  return min_distance;
}

Intersection BoundingVolumeHierarchy::intersect(
    const Ray &ray, const IntersectVisitorFunction &visitor) const {
  struct IntersectionCandidate {
    int index;
    double min_distance;
  };

  // Use this heap as a priority queue, start with the root of the tree
  const IntersectionCandidate root{0, -1.0};
  Heap<IntersectionCandidate> nodes_to_intersect(
      [](const IntersectionCandidate &a, const IntersectionCandidate &b) {
        return a.min_distance > b.min_distance;
      });
  nodes_to_intersect.push(root);

  //
  // Depth-first search the tree by traversing the node priority queue
  //

  double min_distance = std::numeric_limits<double>::max();
  int best_leaf = -1;
  while (!nodes_to_intersect.empty()) {
    const IntersectionCandidate to_intersect = nodes_to_intersect.pop();

    // Do not explore if the lower bound on distance to the contents of a node is larger
    // than the closest thing we have seen so far
    if (to_intersect.min_distance > min_distance) {
      continue;
    }

    const auto &node = tree_[to_intersect.index];
    const auto parent_intersection = node.bounding_box.intersect(ray);

    visitor(node, parent_intersection.intersected);

    //
    // Inspect the tree element
    // - If it is a leaf: Test all of the leaf elements for intersection
    //                    and compute the minimum distance
    // - If it is a node: Add its children to the queue for inspection
    //
    if (parent_intersection.intersected) {
      if (node.is_leaf) {
        int nearest_index;
        const double distance = traverse_leaf(node.leaf, ray, out(nearest_index));
        if (nearest_index != -1 && distance < min_distance) {
          min_distance = distance;
          best_leaf = nearest_index;
        }

      } else {
        // Note: This `k : ...` scatter/gather operation is
        //       more cache-friendly than one might think
        for (const int k : {node.node.left_child_index, node.node.left_child_index + 1}) {
          const IntersectionCandidate new_candidate{k, parent_intersection.distance};
          nodes_to_intersect.push(new_candidate);
        }
      }
    }
  }

  Intersection result;
  result.intersected = best_leaf != -1;
  result.distance = min_distance;
  return result;
}

bool BoundingVolumeHierarchy::does_intersect(const Ray &ray) const {
  // TODO(jpanikulam): Shortcut the full intersection test if this function is used often
  return intersect(ray).intersected;
}

BoundingBox<BoundingVolumeHierarchy::DIM> BoundingVolumeHierarchy::bounding_box() const {
  // Return the root node
  return tree_[0].bounding_box;
}
}  // namespace spatial
}  // namespace geometry
