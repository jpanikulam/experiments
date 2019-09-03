#include "planning/simulation/sim_viewer_asset.hh"

#include "geometry/spatial/fit_bounding_box.hh"
#include "geometry/spatial/triangle_volume.hh"

#include "logging/assert.hh"

namespace jcc {
namespace simulation {

struct ManagedBvh {
  std::vector<geometry::spatial::TriangleVolume> triangles;
  std::shared_ptr<geometry::spatial::BoundingVolumeHierarchy> bvh;
};

namespace {

std::shared_ptr<ManagedBvh> build_bvh(const geometry::TriMesh& tri_mesh) {
  auto mgd_bvh = std::make_shared<ManagedBvh>();
  mgd_bvh->bvh = std::make_shared<geometry::spatial::BoundingVolumeHierarchy>();

  // auto triangles = std::make_shared<std::vector<TriangleVolume>>();
  mgd_bvh->triangles.reserve(tri_mesh.triangles.size());

  std::vector<geometry::spatial::Volume*> tri_ptrs;
  tri_ptrs.reserve(tri_mesh.triangles.size());

  for (size_t k = 0; k < tri_mesh.triangles.size(); ++k) {
    mgd_bvh->triangles.emplace_back(tri_mesh.triangles[k].vertices);
    tri_ptrs.push_back(&mgd_bvh->triangles.back());
  }

  mgd_bvh->bvh->build(tri_ptrs);
  return mgd_bvh;
}
}  // namespace

std::shared_ptr<geometry::spatial::BoundingVolumeHierarchy> Asset::get_bvh() const {
  return bvh->bvh;
}

Asset build_asset(const std::string& path) {
  const auto trimesh_optl = geometry::import::read_stl(path);

  const std::string error = "Could not load mesh at " + path;

  JASSERT(static_cast<bool>(trimesh_optl), error.c_str());
  const auto bbox = geometry::spatial::fit_bounding_box(*trimesh_optl);

  return Asset{*trimesh_optl, bbox, build_bvh(*trimesh_optl)};
}

}  // namespace simulation
}  // namespace jcc