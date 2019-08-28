#include "planning/simulation/sim_viewer_asset.hh"

#include "geometry/spatial/fit_bounding_box.hh"
#include "logging/assert.hh"

namespace jcc {
namespace simulation {

Asset build_asset(const std::string& path) {
  const auto trimesh_optl = geometry::import::read_stl(path);

  const std::string error = "Could not load mesh at " + path;

  JASSERT(static_cast<bool>(trimesh_optl), error.c_str());
  const auto bbox = geometry::spatial::fit_bounding_box(*trimesh_optl);

  return Asset{*trimesh_optl, bbox};
}

}  // namespace simulation
}  // namespace jcc