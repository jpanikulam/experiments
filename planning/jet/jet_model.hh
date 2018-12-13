#pragma once

#include "geometry/import/read_collada.hh"
#include "viewer/primitives/simple_geometry.hh"

namespace planning {
namespace jet {

const std::string path = "/home/jacob/repos/experiments/data/Hover-Jet Vehicle.dae";
class JetModel {
 public:
  JetModel() : model_(path) {
  }
  void put(viewer::SimpleGeometry& geo);

 private:
  geometry::import::ColladaModel model_;
};

}  // namespace jet
}  // namespace planning