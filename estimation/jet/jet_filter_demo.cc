#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

#include "estimation/jet/jet_rk4.hh"

#include "util/environment.hh"

#include "eigen.hh"
#include "sophus.hh"

namespace estimation {
namespace jet_filter {

State dynamics(const State& x, const double h) {
  const Parameters z = {};
  return rk4_integrate(x, z, h);
}

void setup() {
  const auto view = viewer::get_window3d("Mr. Filter, filters");
  view->set_target_from_world(
      SE3(SO3::exp(Eigen::Vector3d(-3.1415 * 0.5, 0.0, 0.0)), Eigen::Vector3d::Zero()));
  view->set_continue_time_ms(10);

  const auto background = view->add_primitive<viewer::SimpleGeometry>();
  const geometry::shapes::Plane ground{jcc::Vec3::UnitZ(), 0.0};
  background->add_plane({ground});
  background->flip();
}

void go() {
  setup();
  const auto view = viewer::get_window3d("Mr. Filter, filters");
  const auto geo = view->add_primitive<viewer::SimpleGeometry>();

  const jcc::Vec3 g(0.0, 0.0, 0.0);
  const SO3 r_vehicle_from_sensor;
  const jcc::Vec3 t_vehicle_from_sensor = jcc::Vec3(0.0, 0.0, 1.0);
  const SE3 vehicle_from_sensor = SE3(r_vehicle_from_sensor, t_vehicle_from_sensor);

  State x;
  // x.eps_dot = VecNd<6>::Ones();
  x.eps_dot[0] = 1.0;
  x.eps_dot[3] = 0.0;
  x.eps_dot[4] = 0.0;
  x.eps_dot[5] = 1.0;

  x.eps_ddot[2] = 0.0;
  x.eps_ddot[5] = 0.0;

  const auto res =
      observe_accel(x.T_world_from_body, x.eps_dot, x.eps_ddot, vehicle_from_sensor, g);
  std::cout << res.transpose() << std::endl;

  constexpr double dt = 0.01;
  for (int k = 0; k < 1000; ++k) {
    x = dynamics(x, dt);
    geo->add_axes({x.T_world_from_body.inverse()});
    geo->add_axes({(x.T_world_from_body * vehicle_from_sensor).inverse(), 0.25});

    geo->flip();

    view->spin_until_step();
  }
}

}  // namespace jet_filter
}  // namespace estimation

int main() {
  estimation::jet_filter::go();
}