#include "eigen.hh"

#include "geometry/intersection/sphere_plane.hh"
#include "geometry/rotation_to.hh"
#include "geometry/shapes/circle.hh"
#include "geometry/shapes/halfspace.hh"
#include "geometry/shapes/sphere.hh"

#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

namespace geometry {
namespace intersection {

using Vec3 = Eigen::Vector3d;

void put_circle(viewer::SimpleGeometry& geo, const geometry::shapes::Circle& circle) {
  constexpr double DELTA_RAD = 0.01;

  const auto rot_real_from_circle = geometry::rotation_to(Vec3::UnitZ(), circle.u_normal);
  const SE3 world_from_circle = SE3(rot_real_from_circle, circle.center);

  viewer::Polygon polygon;
  polygon.outline = true;
  for (double s = 0.0; s <= M_PI * 2.0; s += DELTA_RAD) {
    const Vec3 local_pt =
        Vec3(circle.radius * std::cos(s), circle.radius * std::sin(s), 0.0);

    polygon.points.push_back(world_from_circle * local_pt);
  }
  geo.add_polygon(polygon);
}

void go() {
  const auto view = viewer::get_window3d("Mr. Walks, walks");
  view->set_target_from_world(
      SE3(SO3::exp(Eigen::Vector3d(-3.1415 * 0.5, 0.0, 0.0)), Eigen::Vector3d::Zero()));
  const auto geo = view->add_primitive<viewer::SimpleGeometry>();
  view->set_continue_time_ms(20);

  constexpr double dt = 0.1;
  for (double t = 0.0; t < 100.0; t += dt) {
    const shapes::Plane ground{Vec3::UnitZ(), 0.0};
    geo->add_plane({ground});

    const shapes::Sphere sphere{Vec3(1.0, 1.0, std::cos(t)), 1.0};
    geo->add_billboard_circle({sphere.center, sphere.radius});

    const auto intersection = sphere_plane_intersection(sphere, ground);

    if (intersection) {
      put_circle(*geo, *intersection);
    }

    geo->flip();
    view->spin_until_step();
  }
}

}  // namespace intersection
}  // namespace geometry
int main() {
  geometry::go();
}