#include "planning/simulation/sim_viewer.hh"

#include "viewer/primitives/simple_geometry.hh"

#include "logging/assert.hh"

namespace jcc {
void go() {
  auto view = create_sim_viewer("2D Robot Test");
  jcc::Success() << "Loaded assets..." << std::endl;

  const auto geo = view->add_primitive<viewer::SimpleGeometry>();

  view->spin_until_step();
}
}  // namespace jcc

int main() {
  jcc::go();
}
