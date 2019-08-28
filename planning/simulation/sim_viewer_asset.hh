#pragma once

#include "geometry/import/read_stl.hh"
#include "geometry/spatial/bounding_box.hh"

#include <string>

namespace jcc {
namespace simulation {

struct Asset {
  geometry::TriMesh mesh;
  geometry::spatial::BoundingBox<3> bbox;
};

Asset build_asset(const std::string& path);

}  // namespace simulation
}  // namespace jcc