#pragma once

#include "geometry/import/read_collada.hh"
#include "planning/jet/jet_dynamics.hh"

#include "viewer/primitives/scene_tree.hh"

namespace planning {
namespace jet {

class JetModel {
 public:
  JetModel(const std::string& path =
               "/home/jacob/repos/experiments/data/Hover-Jet Vehicle.dae")
      : model_(path) {
  }

  void insert(viewer::SceneTree& tree) const;
  // const State& jet

 private:
  geometry::import::ColladaModel model_;
};

}  // namespace jet
}  // namespace planning