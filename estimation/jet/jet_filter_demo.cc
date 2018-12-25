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
  const SO3 r_sensor_from_body;
  const jcc::Vec3 t_sensor_from_body = jcc::Vec3(0.0, 0.0, 1.0);
  const SE3 sensor_from_body = SE3(r_sensor_from_body, t_sensor_from_body);

  State x;
  x.T_body_from_world = SE3(SO3(), jcc::Vec3(5.0, 0.0, 0.0));

  x.eps_dot[0] = 0.0;
  x.eps_dot[1] = 0.0;
  x.eps_dot[2] = 0.0;
  x.eps_dot[3] = 0.0;
  x.eps_dot[4] = 0.0;
  x.eps_dot[5] = 0.0;

  x.eps_ddot[0] = 0.0;
  x.eps_ddot[1] = 1.0;
  x.eps_ddot[2] = 0.0;
  x.eps_ddot[3] = 0.0;
  x.eps_ddot[4] = 0.0;
  x.eps_ddot[5] = 1.0;

  const auto res =
      observe_accel(x.T_body_from_world, x.eps_dot, x.eps_ddot, sensor_from_body, g);

  constexpr double dt = 0.01;
  while (!view->should_close()) {
    x = dynamics(x, dt);

    geo->add_axes({x.T_body_from_world.inverse()});
    const SE3 world_from_sensor = (sensor_from_body * x.T_body_from_world).inverse();
    geo->add_axes({world_from_sensor, 0.25});
    const auto res =
        observe_accel(x.T_body_from_world, x.eps_dot, x.eps_ddot, sensor_from_body, g);

    geo->add_ray(
        {world_from_sensor.translation(), world_from_sensor.so3() * res, res.norm()});

    geo->flip();

    view->spin_until_step();
  }
}

}  // namespace jet_filter
}  // namespace estimation

int main() {
  estimation::jet_filter::go();
}