#include "geometry/visualization/put_bvh.hh"

#include "geometry/visualization/put_bounding_box.hh"

namespace geometry {

spatial::BoundingVolumeHierarchy::IntersectVisitorFunction make_visit(
    viewer::GeometryBuffer &buf) {
  return [&buf](const geometry::spatial::BoundingVolumeHierarchy::TreeElement &el,
                bool intersected) {
    const double alpha = el.is_leaf ? 0.4 : 0.2;
    const jcc::Vec4 color =
        intersected ? jcc::Vec4(0.1, 0.8, 0.1, alpha) : jcc::Vec4(0.1, 0.8, 0.8, alpha);

    if (el.is_leaf) {
      put_bounding_box(buf, el.bounding_box, color, 0.2);
    }
  };
}

}  // namespace geometry
