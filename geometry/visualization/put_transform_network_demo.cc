#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

#include "geometry/kinematics/transform_network.hh"
#include "geometry/visualization/put_transform_network.hh"

namespace geometry {

void draw() {
  const auto view = viewer::get_window3d("Beams Baby");
  const auto bgnd = view->add_primitive<viewer::SimpleGeometry>();
  const geometry::shapes::Plane ground{jcc::Vec3::UnitZ(), 0.0};
  bgnd->add_plane({ground, 1.0, jcc::Vec4(0.8, 0.8, 0.8, 0.4)});
  bgnd->flip();

  view->add_toggle_hotkey("phase", false, 'P');

  const auto geo = view->add_primitive<viewer::SimpleGeometry>();

  TransformNetwork tfn;
  tfn.add_edge("world", "fiducial", SE3(SO3(), jcc::Vec3(1.0, 0.0, 0.1)));

  tfn.add_edge("fiducial", "camera", SE3(SO3(), jcc::Vec3(1.0, 0.0, 1.1)));
  tfn.add_edge("jet", "camera",
               SE3(SO3::exp(jcc::Vec3(0.2, 0.2, -0.5)), jcc::Vec3(1.0, 0.0, -1.0)));

  tfn.add_edge("camera", "IMU-1",
               SE3(SO3::exp(jcc::Vec3(1.2, 0.2, -0.5)), jcc::Vec3(0.0, 1.0, -1.0)));
  tfn.add_edge("camera", "IMU-2",
               SE3(SO3::exp(jcc::Vec3(1.2, 0.2, -0.5)), jcc::Vec3(0.0, -1.0, -1.0)));

  const SE3 imu2_from_imu1 = tfn.source_from_destination("IMU-2", "IMU-1");
  tfn.add_edge("IMU-2", "IMU-1", imu2_from_imu1);

  tfn.add_edge("jet", "servos",
               SE3(SO3::exp(jcc::Vec3(-0.2, 0.2, -0.5)), jcc::Vec3(1.0, 1.0, -1.0)));

  const std::string root_node = "world";

  put_transform_network(*geo, tfn, root_node);
  geo->flip();
  view->spin_until_step();
}

}  // namespace geometry

int main() {
  geometry::draw();
}