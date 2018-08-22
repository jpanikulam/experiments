#include "geometry/spatial/gjk.hh"

#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

namespace geometry {
namespace spatial {

namespace {
using Vec4 = Eigen::Vector4d;

void add_triangle(viewer::SimpleGeometry& geom,
                  const Vec3& a,
                  const Vec3& b,
                  const Vec3& c,
                  const Vec4& color) {
  geom.add_line({a, b, color});
  geom.add_line({b, c, color});
  geom.add_line({c, a, color});
}

void add_simplex(viewer::SimpleGeometry& geom, const Simplex& s, const Vec4& color) {
  if (s.vertices.size() == 1) {
    viewer::Points points;
    points.points.push_back(s.vertices[0]);
    geom.add_points(points);
  }

  if (s.vertices.size() == 2) {
    geom.add_line({s.vertices[0], s.vertices[1], color});
  }

  if (s.vertices.size() >= 3) {
    add_triangle(geom, s.vertices[0], s.vertices[1], s.vertices[2], color);
  }
  if (s.vertices.size() == 4) {
    add_triangle(geom, s.vertices[1], s.vertices[2], s.vertices[3], color);
    add_triangle(geom, s.vertices[2], s.vertices[0], s.vertices[3], color);
    add_triangle(geom, s.vertices[0], s.vertices[1], s.vertices[3], color);
  }
}

}  // namespace

void demo_intersection() {
  //

  Shape shape_1;

  const Vec3 v0 = Vec3::UnitX();
  const Vec3 v1 = Vec3::UnitY();
  const Vec3 v2 = Vec3::UnitZ();
  const Vec3 v3 = -(v0 + v1 + v2) / 3.0;

  Simplex s1;
  s1.vertices.push_back(v0);
  s1.vertices.push_back(v1);
  s1.vertices.push_back(v2);
  s1.vertices.push_back(v3);

  shape_1.simplices.push_back(s1);
  // shape_1.simplices.push_back(s2);

  auto win = viewer::get_window3d("GJK View");
  auto geom = win->add_primitive<viewer::SimpleGeometry>();
  auto mesh_a = win->add_primitive<viewer::SimpleGeometry>();

  //
  // Draw it all
  //
  {
    for (const auto& simplex : shape_1.simplices) {
      add_simplex(*mesh_a, simplex, Vec4(1.0, 1.0, 1.0, 0.7));
    }
    mesh_a->flush();
  }

  const auto visitor = [&win, &geom](const Simplex& s) {
    Vec4 color(1.0, 0.0, 0.2, 1.0);
    add_simplex(*geom, s, color);

    geom->flip();
    win->spin_until_step();
    geom->clear();
  };

  const Vec3 target(0.1, 0.1, 0.1);
  viewer::Points points;
  points.points.push_back(target);
  mesh_a->add_points(points);
  mesh_a->flush();

  gjk(shape_1, target, visitor);
}

}  // namespace spatial
}  //  namespace geometry

int main() {
  geometry::spatial::demo_intersection();
}