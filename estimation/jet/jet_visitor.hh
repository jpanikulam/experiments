#pragma once

#include "estimation/jet/jet_optimizer.hh"
#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

namespace estimation {
namespace jet_filter {

JetPoseOptimizer::Visitor make_visitor(
    const std::shared_ptr<viewer::Window3D>& view,
    const std::shared_ptr<viewer::SimpleGeometry>& geo);

class MeasurementVisualizer {
  //
  // Add measurements and corresponding times
  // Add observation models corresponding to a measurement type
  //
};

}  // namespace jet_filter
}  // namespace estimation
