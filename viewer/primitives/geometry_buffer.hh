#pragma once

#include "viewer/primitives/simple_geometry_primitives.hh"
#include "viewer/colors/colors.hh"

#include <map>
#include <vector>

namespace viewer {

struct GeometryBuffer {
  std::vector<Axes> axes;
  std::vector<Line> lines;
  std::vector<Points> points;
  std::vector<Point> raw_points;
  std::vector<Points2d> points2d;
  std::vector<Sphere> spheres;
  std::vector<Ellipsoid> ellipsoids;
  std::vector<Plane> planes;
  std::vector<Polygon> polygons;
  std::vector<ColoredPoints> colored_points;
  std::vector<TriMesh> tri_meshes;

  void clear() {
    axes.clear();
    lines.clear();
    points.clear();
    raw_points.clear();
    points2d.clear();
    spheres.clear();
    ellipsoids.clear();
    planes.clear();
    polygons.clear();
    colored_points.clear();
    tri_meshes.clear();

    // mesh_displaylists.clear();
    displaylist_clear_queued = true;
  }

  // This is not great
  // TODO: Add a universal UUID -> displaylist map
  mutable std::map<std::size_t, int> mesh_displaylists;
  mutable bool displaylist_clear_queued = false;
};

void draw_geometry_buffer(const GeometryBuffer &buffer);

}  // namespace viewer