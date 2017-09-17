#include "testing/gtest.hh"

#include "viewer/primitives/box.hh"
#include "viewer/primitives/simple_geometry.hh"

#include "viewer/window_3d.hh"
#include "viewer/window_manager.hh"

#include "geometry/import/read_stl.hh"
#include "geometry/spatial/sphere_volume.hh"
#include "geometry/spatial/triangle_volume.hh"

#include "geometry/spatial/bounding_volume_hierarchy.hh"

#include "eigen_helpers.hh"

#include <map>

TEST(BoundingVolumeHierarchyTest, bounding_volumes) {
  auto win = gl_viewer::get_window3d("Window A");

  const std::string file_path = "/home/jacob/repos/experiments/test_stuff.stl";
  const auto tri = geometry::import::read_stl(file_path);

  gl_viewer::WindowManager::draw();
  auto scene_geometry = std::make_shared<gl_viewer::SimpleGeometry>();
  win->add_primitive(scene_geometry);
  auto visitor_geometry = std::make_shared<gl_viewer::SimpleGeometry>();
  win->add_primitive(visitor_geometry);

  std::vector<geometry::spatial::Volume *> tri_ptrs;
  tri_ptrs.reserve(tri.triangles.size());
  std::vector<geometry::spatial::TriangleVolume> triangles;
  triangles.reserve(tri.triangles.size());

  for (size_t k = 0; k < tri.triangles.size(); ++k) {
    scene_geometry->add_line({tri.triangles[k].vertices[0], tri.triangles[k].vertices[1]});
    scene_geometry->add_line({tri.triangles[k].vertices[1], tri.triangles[k].vertices[2]});
    scene_geometry->add_line({tri.triangles[k].vertices[2], tri.triangles[k].vertices[0]});

    triangles.emplace_back(tri.triangles[k].vertices);
    tri_ptrs.push_back(&triangles.back());
  }

  win->spin_until_step();

  std::map<int, Eigen::Vector4d> colors;
  for (int stop_depth = 0; stop_depth < 10; ++stop_depth) {
    const auto visitor = [&visitor_geometry, &win, &colors, stop_depth](const geometry::spatial::BoundingBox<3> &box,
                                                                        int depth, bool leaf) {
      gl_viewer::AxisAlignedBox aabb;
      aabb.lower = box.lower();
      aabb.upper = box.upper();

      if (colors.count(depth) == 0) {
        colors[depth] = Eigen::Vector4d::Random();
      }
      if (leaf) {
        aabb.color = Eigen::Vector4d(0.0, 1.0, 0.0, 0.8);
      } else {
        aabb.color = colors[depth];
      }

      if (depth != stop_depth) {
        return;
      }

      aabb.color[3] = 1.0;
      aabb.color[0] = 1.0;

      std::cout << box.upper().transpose() << std::endl;
      std::cout << box.lower().transpose() << std::endl;
      std::cout << "----" << std::endl;

      visitor_geometry->add_box(aabb);
      win->spin_until_step();

    };
    geometry::spatial::BoundingVolumeHierarchy bvh;
    bvh.build(tri_ptrs, visitor);
    win->spin_until_step();
    visitor_geometry->clear();
  }
}
