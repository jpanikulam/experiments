#pragma once

#include "geometry/import/read_collada.hh"
#include "planning/jet/jet_dynamics.hh"
#include "viewer/primitives/simple_geometry.hh"

namespace planning {
namespace jet {

class JetModel {
 public:
  JetModel(const std::string& path =
               "/home/jacob/repos/experiments/data/Hover-Jet Vehicle.dae")
      : model_(path) {
  }
  void put(viewer::SimpleGeometry& geo, const State& jet) const;

 private:
  geometry::import::ColladaModel model_;
};

}  // namespace jet
}  // namespace planning