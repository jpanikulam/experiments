#pragma once

#include "estimation/jet/jet_optimizer.hh"
#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

namespace estimation {
namespace jet_filter {

JetPoseOptimizer::Visitor make_visitor(
    const std::shared_ptr<viewer::Window3d>& view,
    const std::shared_ptr<viewer::SimpleGeometry>& geo);

}  // namespace jet_filter
}  // namespace estimation