#include "planning/jet/jet_model.hh"

#include "geometry/visualization/put_collada.hh"

namespace planning {
namespace jet {

void JetModel::put(viewer::SimpleGeometry& geo, const State& jet) const {
  const SE3 world_from_jet = SE3(jet.R_world_from_body, jet.x);
  geometry::visualization::put_collada(geo, model_, world_from_jet);
}

}  // namespace jet
}  // namespace planning