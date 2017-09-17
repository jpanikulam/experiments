#include "bounding_volume_hierarchy.hh"

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

// Partition compute_partition(std::vector<BoundingBox<DIM>> &boxes, size_t begin, size_t end, int dim) {
//   size_t half = ((end - begin) / 2) + begin;
//   const auto begin_it = boxes.begin();
//   std::nth_element(
//       begin_it + begin, begin_it + half, begin_it + end,
//       [dim](const BoundingBox<DIM> &a, const BoundingBox<DIM> &b) { return a.upper()(dim) < b.upper()(dim); });
//   Partition partition;
//   partition.dim = dim;
//   partition.split_value = boxes[half].upper()(dim);
//   partition.split_cost = (boxes[end].upper()(dim) - boxes[begin].lower()(dim));
//   partition.index = half;
//   return partition;
// }

Partition compute_partition(std::vector<BoundingBox<DIM>> &boxes, size_t begin, size_t end, int dim) {
  const auto begin_it = boxes.begin();
  // std::sort(begin_it + begin, begin_it + end,
  // [dim](const BoundingBox<DIM> &a, const BoundingBox<DIM> &b) { return a.upper()(dim) < b.upper()(dim); });
  std::sort(begin_it + begin, begin_it + end,
            [dim](const BoundingBox<DIM> &a, const BoundingBox<DIM> &b) { return a.center()(dim) < b.center()(dim); });

  BoundingBox<DIM> lower_bbox;
  double best_split_cost = std::numeric_limits<double>::max();
  int best_split_index = -1;
  for (size_t k = begin; k < end; ++k) {
    const int complement_k = (end - 1) - (k - begin);

    // const double lower_size = 1.0;
    const double lower_size = static_cast<double>(k - begin);
    // const double upper_size = 1.0;
    const double upper_size = static_cast<double>(end - k);
    assert((lower_size + upper_size) == (end - begin));

    BoundingBox<DIM> upper_bbox;
    for (size_t u = k; u < end; ++u) {
      upper_bbox.expand(boxes[u]);
    }

    lower_bbox.expand(boxes[k]);

    const double lower_sa = lower_bbox.surface_area();
    const double upper_sa = upper_bbox.surface_area();

    const double split_cost = (lower_sa * lower_size) + (upper_sa * upper_size);
    if (k == begin || k == (end - 1)) {
      std::cout << "sa: " << lower_sa << ", " << upper_sa << std::endl;
    }

    if (split_cost < best_split_cost) {
      best_split_cost = split_cost;
      best_split_index = k;
    }
  }

  assert(best_split_index >= 0);

  Partition partition;
  partition.dim = dim;
  partition.split_value = boxes[best_split_index].upper()(dim);
  partition.split_cost = best_split_cost;
  partition.index = best_split_index;
  return partition;
}
}

int BoundingVolumeHierarchy::add_node_and_children(std::vector<BoundingBox<DIM>> &bounding_boxes,
                                                   size_t begin,
                                                   size_t end,
                                                   int depth,
                                                   const VisitorFunction &visitor) {

  BoundingBox<DIM> node_bbox;
  for (size_t k = begin; k < end; ++k) {
    node_bbox.expand(bounding_boxes[k]);
  }

  const bool is_leaf = (end - begin) < 16;

  // Visit!
  visitor(node_bbox, depth, is_leaf);

  if (is_leaf) {
    return 1;
  }

  // Partition chosen_partition;
  // double best_cost = std::numeric_limits<double>::max();
  // for (int dim_candidate = 0; dim_candidate < DIM; ++dim_candidate) {
  //   const Partition partition = compute_partition(bounding_boxes, begin, end, dim_candidate);
  //   if (partition.split_cost < best_cost) {
  //     best_cost = partition.split_cost;
  //     chosen_partition = partition;
  //   }
  // }

  std::cout << "(" << begin << " , " << end << ")" << std::endl;
  const Partition chosen_partition = compute_partition(bounding_boxes, begin, end, depth % 3);

  const int dim = chosen_partition.dim;
  const double value = chosen_partition.split_value;
  // std::partition(bounding_boxes.begin() + begin, bounding_boxes.begin() + end,
  // [dim, value](const BoundingBox<DIM> &a) { return a.upper()(dim) < value; });

  int l_depth = add_node_and_children(bounding_boxes, begin, chosen_partition.index, depth + 1, visitor);
  int r_depth = add_node_and_children(bounding_boxes, chosen_partition.index, end, depth + 1, visitor);
  int depth_beneath = std::max(l_depth, r_depth);

  return depth_beneath + 1;
}

void BoundingVolumeHierarchy::build(const std::vector<Volume *> &volumes, const VisitorFunction &visitor) {
  std::vector<BoundingBox<DIM>> bounding_boxes(volumes.size());
  for (std::size_t k = 0; k < volumes.size(); ++k) {
    bounding_boxes[k] = volumes[k]->bounding_box();
  }
  add_node_and_children(bounding_boxes, 0, bounding_boxes.size(), 0, visitor);
}

Intersection BoundingVolumeHierarchy::intersect(const Ray &ray) const {
  return {};
}

bool BoundingVolumeHierarchy::does_intersect(const Ray &ray) const {
  return true;
}

BoundingBox<DIM> BoundingVolumeHierarchy::bounding_box() const {
  //
  return {};
}
} // namespace geometry
} // namespace spatial
