#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

#include "geometry/visualization/put_stl.hh"

#include "planning/jet/jet_dynamics.hh"

#include "eigen.hh"
#include "sophus.hh"

namespace planning {
namespace jet {

void setup() {
  const auto view = viewer::get_window3d("Mr. Jet, jets");
  view->set_target_from_world(
      SE3(SO3::exp(Eigen::Vector3d(-3.1415 * 0.5, 0.0, 0.0)), Eigen::Vector3d::Zero()));
  view->set_continue_time_ms(250);

  const auto background = view->add_primitive<viewer::SimpleGeometry>();
  const geometry::shapes::Plane ground{jcc::Vec3::UnitZ(), 0.0};
  background->add_plane({ground});
  background->flip();
}

void go() {
  const auto view = viewer::get_window3d("Mr. Jet, jets");

  const std::string jet_path = "/home/jacob/repos/experiments/data/jetcat_p160.stl";
  const auto put_jet = geometry::visualization::create_put_stl(jet_path);
  const auto jet_geo = view->add_primitive<viewer::SimpleGeometry>();

  State jet;
  jet.R_world_from_body = SO3::exp(jcc::Vec3::UnitX());
  jet.x = jcc::Vec3(1.0, 1.0, 0.0);
  jet.v = jcc::Vec3(0.2, 0.0, 0.0);
  jet.w = jcc::Vec3::UnitX() * 0.1;

  for (int j = 0; j < 1000; ++j) {
    const SE3 world_from_jet = SE3(jet.R_world_from_body, jet.x);
    put_jet(*jet_geo, world_from_jet);

    const SO3 world_from_target_rot = SO3::exp(jcc::Vec3::UnitX() * -3.1415 * 0.5);
    const SE3 world_from_target(world_from_target_rot, world_from_jet.translation());
    view->set_target_from_world(world_from_target.inverse());
    jet_geo->add_line({jcc::Vec3::Zero(), world_from_jet.translation()});

    jet_geo->flip();
    view->spin_until_step();

    const double dt = 0.25;
    jet = rk4_integrate(jet, {}, {}, dt);
  }
}

}  // namespace jet
}  // namespace planning

int main() {
  planning::jet::go();
}
