#include "planning/body.hh"

#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

#include <unordered_map>

// - Figure out why my computer is so loud
//  - I am 75% sure it's coil wine in the GPU

// x - UnitVector
// x - Intersectible cylinder
// x - Clickable/hoverable objects in viewer

// Text in Viewer
//  - Generate text and draw lines to it in a vertical box
//  - Simple 2D UI-maker interface
//  - jfc rename "SimpleGeometry" to Geometry"
//  ~ - Fix the color bug (Or...why do gl attributes jump around?)
//  x - Fix alpha on the textures
//  x - Working text drawing w/ no sugar
//  x - Freetype interface

// - Visualize torque and force at joints (Or, at any point on the body!)

namespace planning {
void go() {
  //
  // View setup
  //
  const auto view = viewer::get_window3d("Mr. Arm, arms");
  view->set_target_from_world(
      SE3(SO3::exp(Eigen::Vector3d(-3.1415 * 0.5, 0.0, 0.0)), Eigen::Vector3d::Zero()));
  const auto geo = view->add_primitive<viewer::SimpleGeometry>();
  const auto plan_geo = view->add_primitive<viewer::SimpleGeometry>();
  view->set_continue_time_ms(20);

  //
  // Render forces
  //
}
}  // namespace planning

int main() {
  planning::go();
}