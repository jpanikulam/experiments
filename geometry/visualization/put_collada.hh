#pragma once

#include "viewer/primitives/simple_geometry.hh"

#include "geometry/import/read_collada.hh"

namespace geometry {
namespace visualization {

void put_collada(viewer::SimpleGeometry& geo, const geometry::import::ColladaModel& model);

}  // namespace visualization
}  // namespace geometry