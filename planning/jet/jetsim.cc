#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

#include "geometry/visualization/put_stl.hh"

#include "planning/jet/jet_dynamics.hh"
#include "planning/jet/jet_planner.hh"

#include "eigen.hh"
#include "sophus.hh"

namespace planning {
namespace jet {

namespace {
void setup() {
  const auto view = viewer::get_window3d("Mr. Jet, jets");
  view->set_target_from_world(
      SE3(SO3::exp(Eigen::Vector3d(-3.1415 * 0.5, 0.0, 0.0)), Eigen::Vector3d::Zero()));
  view->set_continue_time_ms(10);

  const auto background = view->add_primitive<viewer::SimpleGeometry>();
  const geometry::shapes::Plane ground{jcc::Vec3::UnitZ(), 0.0};
  background->add_plane({ground});
  background->flip();
}
}  // namespace

void go() {
  setup();
  const auto view = viewer::get_window3d("Mr. Jet, jets");

  const std::string jet_path = "/home/jacob/repos/experiments/data/jetcat_p160.stl";
  const auto put_jet = geometry::visualization::create_put_stl(jet_path);
  const auto jet_geo = view->add_primitive<viewer::SimpleGeometry>();

  const auto accum_geo = view->add_primitive<viewer::SimpleGeometry>();
  Parameters params;
  params.mass = 100.0;
  params.unit_z = jcc::Vec3::UnitZ();
  // params.external_force = jcc::Vec3::UnitZ() * -1.0;

  State jet;
  jet.x = jcc::Vec3(1.0, 1.0, -0.5);
  // jet.v = jcc::Vec3(0.0, 0.0, 0.1);

  jet.w = jcc::Vec3::UnitX() * 0.00;
  jet.throttle_pct = 0.2;

  std::vector<Controls> prev_controls;
  for (int j = 0; j < 1000; ++j) {
    const double dt = 0.1;
    const jcc::Vec3 prev = jet.x;

    const auto future_states = plan(jet, prev_controls);

    {
      prev_controls.clear();
      for (const auto& xu : future_states) {
        prev_controls.push_back(xu.control);
      }
    }
    for (std::size_t k = 0; k < future_states.size(); ++k) {
      const auto& state = future_states.at(k).state;
      const SE3 world_from_state = SE3(state.R_world_from_body, state.x);
      const double scale =
          static_cast<double>(k) / static_cast<double>(future_states.size());
      jet_geo->add_axes({world_from_state, scale});
    }

    std::cout << "Done optimizing" << std::endl;
    jet = future_states[1].state;
    std::cout << "x: " << jet.x.transpose() << std::endl;
    std::cout << "w: " << jet.w.transpose() << std::endl;
    std::cout << "v: " << jet.v.transpose() << std::endl;
    std::cout << "t: " << jet.throttle_pct << std::endl;

    // const State u = future_states.front();
    // jet = rk4_integrate(jet, {}, params, dt);

    accum_geo->add_line({prev, jet.x, jcc::Vec4(1.0, 0.7, 0.7, 0.7), 5.0});
    const SE3 world_from_jet = SE3(jet.R_world_from_body, jet.x);
    put_jet(*jet_geo, world_from_jet);
    jet_geo->add_line({world_from_jet.translation(),
                       world_from_jet.so3() * jcc::Vec3::UnitZ() * jet.throttle_pct,
                       jcc::Vec4(1.0, 0.3, 0.3, 0.8), 5.0});

    const SO3 world_from_target_rot = SO3::exp(jcc::Vec3::UnitX() * 3.1415 * 0.5);
    const SE3 world_from_target(world_from_target_rot, world_from_jet.translation());
    view->set_target_from_world(world_from_target.inverse());

    jet_geo->flip();
    accum_geo->flush();
    view->spin_until_step();
  }
}

}  // namespace jet
}  // namespace planning

int main() {
  planning::jet::go();
}
