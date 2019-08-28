#include "viewer/primitives/geometry_buffer.hh"

#include <GL/glew.h>

#include "viewer/gl_aliases.hh"

namespace viewer {

void draw_geometry_buffer(const GeometryBuffer &buffer) {
  glMatrixMode(GL_MODELVIEW);

  for (const auto &axes : buffer.axes) {
    draw_axes(axes);
  }

  for (const auto &points : buffer.points) {
    draw_points(points);
  }

  for (const auto &point : buffer.raw_points) {
    draw_point(point);
  }

  for (const auto &points2d : buffer.points2d) {
    draw_points2d(points2d);
  }

  for (const auto &circle : buffer.spheres) {
    draw_sphere(circle);
  }

  for (const auto &ellipse : buffer.ellipsoids) {
    draw_ellipsoid(ellipse);
  }

  for (const auto &polygon : buffer.polygons) {
    draw_polygon(polygon);
  }

  for (const auto &colored_points : buffer.colored_points) {
    draw_colored_points(colored_points);
  }

  for (const auto &plane : buffer.planes) {
    draw_plane_grid(plane);
  }

  if (buffer.displaylist_clear_queued) {
    buffer.displaylist_clear_queued = false;
    for (auto &dl : buffer.mesh_displaylists) {
      glDeleteLists(dl.second, 1u);
    }
    buffer.mesh_displaylists.clear();
  }

  constexpr bool USE_CACHE = true;
  for (std::size_t i = 0; i < buffer.tri_meshes.size(); ++i) {
    if (USE_CACHE) {
      if (buffer.mesh_displaylists.count(i) == 0) {
        auto tri_mesh2 = buffer.tri_meshes[i];
        tri_mesh2.world_from_mesh = SE3();
        const GLuint index = glGenLists(1);
        assert(index > 0);
        buffer.mesh_displaylists[i] = index;
        glNewList(index, GL_COMPILE);
        draw_trimesh(tri_mesh2);
        glEndList();
      }
      glPushMatrix();
      glTransform(buffer.tri_meshes[i].world_from_mesh);
      glCallList(buffer.mesh_displaylists.at(i));
      glPopMatrix();
    } else {
      const auto &tri_mesh = buffer.tri_meshes[i];
      draw_trimesh(tri_mesh);
    }
  }

  draw_lines(buffer.lines);
}
}  // namespace viewer
