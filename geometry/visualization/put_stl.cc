#include "geometry/visualization/put_stl.hh"

#include "geometry/import/read_stl.hh"

#include "eigen.hh"

namespace geometry {
namespace visualization {

PutFunction create_put_stl(const std::string& path_to_stl) {
  const auto tri = geometry::import::read_stl(path_to_stl);
  const auto put = [tri](viewer::SimpleGeometry& geo, const SE3& body_from_world) {
    const jcc::Vec3 color(0.8, 0.8, 0.8);
    geo.add_triangle_mesh({tri, color, true, 3.0, true});
  };
  return put;
}

}  // namespace visualization
}  // namespace geometry
