#include "planning/jet/jet_model.hh"

#include "geometry/visualization/put_collada.hh"

namespace planning {
namespace jet {
// const std::string path = "/home/jacob/repos/experiments/data/Hover-Jet Vehicle.dae";

// JetModel::JetModel() {

  // model_ = geometry::import::ColladaModel model(path);
// }

void JetModel::put(viewer::SimpleGeometry& geo) {
  geometry::visualization::put_collada(geo, model_);
}

}  // namespace jet
}  // namespace planning