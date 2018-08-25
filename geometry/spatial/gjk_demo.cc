#include "geometry/spatial/gjk.hh"

#include "geometry/import/read_stl.hh"
#include "sophus.hh"
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
  if (s.vertices.size() == 1u) {
    viewer::Points points;
    points.points.push_back(s.vertices[0]);
    geom.add_points(points);
  } else if (s.vertices.size() == 2u) {
    geom.add_line({s.vertices[0], s.vertices[1], color});
  } else if (s.vertices.size() >= 3u) {
    add_triangle(geom, s.vertices[0], s.vertices[1], s.vertices[2], color);
  }
  if (s.vertices.size() == 4u) {
    add_triangle(geom, s.vertices[1], s.vertices[2], s.vertices[3], color);
    add_triangle(geom, s.vertices[2], s.vertices[0], s.vertices[3], color);
    add_triangle(geom, s.vertices[0], s.vertices[1], s.vertices[3], color);
  }
}
}  // namespace

Shape add_cube() {
  auto win = viewer::get_window3d("GJK View");
  auto geom = win->add_primitive<viewer::SimpleGeometry>();

  const std::string file_path = "/home/jacob/repos/experiments/data/cube_shape.stl";
  const auto tri = geometry::import::read_stl(file_path);

  const Vec4 color(1.0, 1.0, 1.0, 0.6);

  const SO3 world_from_tri_rot;
  const SE3 world_from_tri(world_from_tri_rot, Vec3(2.0, 2.0, 2.0));

  Shape shape;
  for (size_t k = 0; k < tri.triangles.size(); ++k) {
    Simplex simplex;
    simplex.vertices.push_back(world_from_tri * tri.triangles[k].vertices[0]);
    simplex.vertices.push_back(world_from_tri * tri.triangles[k].vertices[1]);
    simplex.vertices.push_back(world_from_tri * tri.triangles[k].vertices[2]);
    shape.simplices.push_back(simplex);

    add_simplex(*geom, simplex, color);
  }
  geom->flip();
  return shape;
}

Shape make_tetrahedron() {
  Shape shape_1;

  auto win = viewer::get_window3d("GJK View");

  auto mesh_a = win->add_primitive<viewer::SimpleGeometry>();

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

  //
  // Draw it all
  //
  {
    for (const auto& simplex : shape_1.simplices) {
      add_simplex(*mesh_a, simplex, Vec4(1.0, 1.0, 1.0, 0.7));
    }
  }
  mesh_a->flip();
  return shape_1;
}

void demo_intersection() {
  //
  const auto shape = add_cube();

  auto win = viewer::get_window3d("GJK View");
  auto geom = win->add_primitive<viewer::SimpleGeometry>();
  auto mesh_a = win->add_primitive<viewer::SimpleGeometry>();

  const auto visitor = [&win, &geom](const Simplex& s) {
    Vec4 color(1.0, 0.0, 0.2, 1.0);
    add_simplex(*geom, s, color);

    geom->flip();
    win->spin_until_step();
    geom->clear();
  };

  const Vec3 target(-2.0, -1.0, -1.0);
  viewer::Points points;
  points.points.push_back(target);
  mesh_a->add_points(points);
  mesh_a->flush();

  gjk(shape, target, visitor);
}

}  // namespace spatial
}  // namespace geometry

int main() {
  geometry::spatial::demo_intersection();
}