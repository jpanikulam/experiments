#include "planning/jet/jet_model.hh"

#include "geometry/visualization/put_collada.hh"

namespace planning {
namespace jet {

void JetModel::put(viewer::SimpleGeometry& geo) const {
  geometry::visualization::put_collada(geo, model_);
  model.put(*accum_geo);
}

}  // namespace jet
}  // namespace planning