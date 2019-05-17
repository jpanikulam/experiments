// #include "testing/gtest.hh"

#include "geometry/shapes/fit_ellipse.hh"
#include "sophus.hh"

#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

namespace geometry {
namespace shapes {
using Vec3 = Eigen::Vector3d;

void demo() {
  const auto view = viewer::get_window3d("Fit Ellipse");
  const auto geo = view->add_primitive<viewer::SimpleGeometry>();
  std::vector<jcc::Vec3> points;

  const jcc::Vec3 master_point = jcc::Vec3::UnitZ() * 12.0;
  const Eigen::Matrix3d U_1 = Eigen::Matrix3d::Random().triangularView<Eigen::Upper>();
  const Eigen::Matrix3d U = Eigen::Matrix3d::Identity() * 3.0  + U_1;

  viewer::Points view_points;
  for (double phi = 0.0; phi <= 100.0 * M_PI; phi += 0.1) {
    const SO3 R = SO3::exp(jcc::Vec3(
        std::sin(phi), -0.5 * std::cos(0.25 * phi) + std::sin(0.9 * (phi + 0.9)), 0.0));

    const jcc::Vec3 pt = U * (R * master_point) + jcc::Vec3::UnitX() * 5.2;
    points.push_back(pt + (jcc::Vec3::Random() * 0.9));
    // points.push_back((R * master_point));
  }
  geo->add_points({points});
  geo->flush();

  const auto ell_geo = view->add_primitive<viewer::SimpleGeometry>();
  const auto visitor = [&ell_geo, &view](const EllipseFit& fit) {
    ell_geo->add_ellipsoid({fit.ellipse, jcc::Vec4(0.4, 0.6, 0.4, 0.7), 2.0});
    ell_geo->flip();
    view->spin_until_step();
  };
  const auto result = fit_ellipse(points, visitor);

  ell_geo->add_ellipsoid({result.ellipse, jcc::Vec4(0.2, 1.0, 0.2, 1.0), 4.0});
  ell_geo->flip();
  view->spin_until_step();

  std::cout << "Done, avg error: " << result.average_error << std::endl;
}

}  // namespace shapes
}  // namespace geometry

int main() {
  geometry::shapes::demo();
}