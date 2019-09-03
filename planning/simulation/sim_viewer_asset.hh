#pragma once

#include "geometry/import/read_stl.hh"
#include "geometry/spatial/bounding_box.hh"
#include "geometry/spatial/bounding_volume_hierarchy.hh"

#include <memory>
#include <string>

namespace jcc {
namespace simulation {

struct ManagedBvh;

struct Asset {
  geometry::TriMesh mesh;
  geometry::spatial::BoundingBox<3> bbox;

  std::shared_ptr<ManagedBvh> bvh;
  std::shared_ptr<geometry::spatial::BoundingVolumeHierarchy> get_bvh() const;
};

Asset build_asset(const std::string& path);

}  // namespace simulation
}  // namespace jcc