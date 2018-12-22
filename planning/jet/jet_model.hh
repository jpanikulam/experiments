#pragma once

#include "geometry/import/read_collada.hh"
#include "planning/jet/jet_dynamics.hh"

#include "viewer/primitives/scene_tree.hh"

#include "util/environment.hh"

namespace planning {
namespace jet {

class JetModel {
 public:
  JetModel() : model_(jcc::Environment::asset_path() + "Hover-Jet Vehicle.dae") {
  }

  void insert(viewer::SceneTree& tree) const;
  // const State& jet

 private:
  geometry::import::ColladaModel model_;
};

}  // namespace jet
}  // namespace planning