#include "geometry/visualization/put_stl.hh"

#include "geometry/import/read_stl.hh"

#include "eigen.hh"

namespace geometry {
namespace visualization {

PutFunction create_put_stl(const std::string& path_to_stl) {
  const auto tri = geometry::import::read_stl(path_to_stl);
  const auto put = [tri](viewer::SimpleGeometry& geo, const SE3& body_from_world) {
    const jcc::Vec4 color(0.8, 0.8, 0.8, 0.4);
    for (size_t k = 0; k < tri.triangles.size(); ++k) {
      const auto& this_tri = tri.triangles[k];
      geo.add_line({this_tri.vertices[0], this_tri.vertices[1], color});
      geo.add_line({this_tri.vertices[1], this_tri.vertices[2], color});
      geo.add_line({this_tri.vertices[2], this_tri.vertices[0], color});
    }
  };
  return put;
}

}  // namespace visualization
}  // namespace geometry
