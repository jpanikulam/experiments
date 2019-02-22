#pragma once

#include "estimation/jet/jet_optimizer.hh"
#include "viewer/primitives/simple_geometry.hh"

namespace estimation {
namespace jet_filter {

namespace {
void draw_states(viewer::SimpleGeometry& geo,
                 const std::vector<JetOptimizer::StateObservation>& states,
                 bool truth) {
  const int n_states = static_cast<int>(states.size());
  for (int k = 0; k < n_states; ++k) {
    auto& state = states.at(k).x;
    const SE3 T_world_from_body = get_world_from_body(state);
    if (truth) {
      geo.add_axes({T_world_from_body, 0.1});
    } else {
      geo.add_axes({T_world_from_body, 0.05, 2.0, true});

      if (k < n_states - 1) {
        const auto& next_state = states.at(k + 1).x;
        const SE3 T_world_from_body_next = get_world_from_body(next_state);
        geo.add_line(
            {T_world_from_body.translation(), T_world_from_body_next.translation()});
      }
    }
  }
}

}  // namespace

JetPoseOptimizer::Visitor make_visitor(
    const std::shared_ptr<viewer::Window3d>& view,
    const std::shared_ptr<viewer::SimpleGeometry>& geo) {
  const auto visitor = [view, geo](const JetPoseOptimizer::Solution& soln) {
    geo->clear();
    draw_states(*geo, soln.x, false);
    geo->flip();
    std::cout << "\tOptimized T_imu_from_vehicle: "
              << soln.p.T_imu_from_vehicle.translation().transpose() << "; "
              << soln.p.T_imu_from_vehicle.so3().log().transpose() << std::endl;
    view->spin_until_step();
  }
}

}  // namespace jet_filter
}  // namespace estimation