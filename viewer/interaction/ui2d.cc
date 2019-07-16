#include "viewer/interaction/ui2d.hh"

#include "logging/assert.hh"
#include "viewer/projection.hh"

#include <GL/glew.h>
#include "viewer/gl_aliases.hh"

namespace viewer {

namespace {

constexpr double Z_DIST = -0.1;

void draw_pointer_targets(const std::vector<PointerTarget> &pointer_targets,
                          const Projection &proj,
                          const CharacterLibrary &char_lib) {
  glPushMatrix();
  glPointSize(6.0);
  glColor3d(1.0, 1.0, 1.0);
  for (const auto &target : pointer_targets) {
    const auto screen_pt = proj.project(target.world_pos);

    glBegin(GL_POINTS);
    { glVertex3d(screen_pt.point.x(), screen_pt.point.y(), Z_DIST); }
    glEnd();

    glLineWidth(0.3);
    glBegin(GL_LINE_STRIP);
    {
      glVertex3d(screen_pt.point.x(), screen_pt.point.y(), Z_DIST);
      glVertex3d(target.location.point.x(), target.location.point.y(), Z_DIST);
    }
    glEnd();

    glTranslated(target.location.point.x(), target.location.point.y(), Z_DIST);
    write_string(target.text, char_lib);
  }
  glPopMatrix();
}

PlotRange compute_plot_range(const LinePlot2d &line_plot) {
  PlotRange range;
  for (const auto &subplot_pair : line_plot.subplots) {
    const auto &subplot = subplot_pair.second;

    for (const auto &pt : subplot.points) {
      range.y_max = std::max(range.y_max, pt.y());
      range.y_min = std::min(range.y_min, pt.y());
      range.x_max = std::max(range.x_max, pt.x());
      range.x_min = std::min(range.x_min, pt.x());
    }
  }

  if (line_plot.plot_range.x_max != line_plot.plot_range.x_min) {
    range.x_max = line_plot.plot_range.x_max;
    range.x_min = line_plot.plot_range.x_min;
  }

  if (line_plot.plot_range.y_max != line_plot.plot_range.y_min) {
    range.y_max = line_plot.plot_range.y_max;
    range.y_min = line_plot.plot_range.y_min;
  }

  return range;
}

void draw_lineplot(const LinePlot2d &line_plot,
                   const Projection &proj,
                   const CharacterLibrary &char_lib) {
  glPushMatrix();
  glPushAttrib(GL_CURRENT_BIT | GL_LINE_BIT);

  constexpr double X_OFFSET = 0.2;
  glTranslated(X_OFFSET, 0.2, 0.0);

  const double aspect =
      static_cast<double>(proj.viewport_size().width) / proj.viewport_size().height;

  glColor4d(0.6, 0.6, 0.6, 0.6);
  glBegin(GL_QUADS);
  const double field_y_max = 0.5;
  const double field_y_min = -0.5;
  const double field_x_min = 0.0;
  const double field_x_max = (aspect - X_OFFSET);

  glVertex3d(field_x_min, field_y_min, -0.9);
  glVertex3d(field_x_max, field_y_min, -0.9);
  glVertex3d(field_x_max, field_y_max, -0.9);
  glVertex3d(field_x_min, field_y_max, -0.9);
  glEnd();

  const PlotRange range = compute_plot_range(line_plot);
  const double x_range = range.x_max - range.x_min;
  const double abs_y_max = std::max(std::abs(range.y_min), std::abs(range.y_max));

  glEnable(GL_LINE_STIPPLE);
  for (const auto &subplot_pair : line_plot.subplots) {
    const auto &subplot = subplot_pair.second;
    glColor(subplot.color);

    if (subplot.dotted) {
      glLineStipple(1.0, 0x00FF);
    } else {
      glLineStipple(1.0, 0xFFFF);
    }
    glLineWidth(subplot.line_width);
    glBegin(GL_LINE_STRIP);

    for (const auto &pt : subplot.points) {
      const double x_val = field_x_max * (pt.x() - range.x_min) / x_range;
      const double y_val = field_y_max * pt.y() / abs_y_max;

      glVertex(jcc::Vec2(x_val, y_val));
    }

    glEnd();
  }

  glLineStipple(1, 0x00FF);
  glLineWidth(1.0);
  glBegin(GL_LINES);
  {
    glColor4d(1.0, 0.0, 0.0, 0.8);
    glVertex3d(field_x_min, 0.0, 0.5);
    glVertex3d(field_x_max, 0.0, 0.5);

    glColor4d(0.0, 1.0, 0.0, 0.8);
    const double x_origin = (0.0 - range.x_min) / x_range;
    glVertex3d(x_origin, field_y_min, 0.5);
    glVertex3d(x_origin, field_y_max, 0.5);
  }
  glEnd();

  glTranslated(0.0, field_y_max + 0.05, 0.0);
  const std::string max_txt =
      line_plot.plot_title + "\nMax: " + std::to_string(abs_y_max);
  write_string(max_txt, char_lib, 0.5);

  glPopAttrib();
  glPopMatrix();
}

void draw_image(const Image &image,
                const Projection &proj,
                const CharacterLibrary &char_lib) {
  glPushMatrix();
  glEnable(GL_TEXTURE_2D);
  if (!image.texture.ready()) {
    //
    // We want the *height* of the texture to be 1.0
    //
    const double width_per_height =
        image.image.cols / static_cast<double>(image.image.rows);

    const jcc::Vec2 size(width_per_height, 1.0);

    image.texture = SmartTexture(size);
    image.texture.tex_image_2d(GL_TEXTURE_2D, 0, GL_RGB, image.image.cols,
                               image.image.rows, 0, GL_BGR, GL_UNSIGNED_BYTE,
                               image.image.data);
  }
  glTranslated(0.0, 0.0, -0.8);
  image.texture.draw();

  glDisable(GL_TEXTURE_2D);
  glPopMatrix();
}

void draw_points(const std::vector<Point2d> points,
                 const Projection &proj,
                 const CharacterLibrary &char_lib) {
  for (const auto &pt : points) {
    glPointSize(pt.size);
    glBegin(GL_POINTS);
    glColor(pt.color);
    glVertex3d(pt.point.x(), 1.0 - pt.point.y(), 0.1);
    glEnd();
  }
}

void draw_lines(const std::vector<Line2d> &lines,
                const Projection &proj,
                const CharacterLibrary &char_lib) {
  for (const auto &line : lines) {
    glLineWidth(line.line_width);
    glColor(line.color);
    glBegin(GL_LINES);
    glVertex3d(line.start.x(), 1.0 - line.start.y(), -0.2);
    glVertex3d(line.end.x(), 1.0 - line.end.y(), -0.2);
    glEnd();
  }
}

void draw_grid_mesh(const GridMesh &mesh) {
  const auto &pts = mesh.vertices;
  const int width = mesh.width;
  constexpr double Z_DIST = -0.1;
  constexpr double Z_DIST_OUTLINE = 0.1;

  const int cols_per_row = mesh.vertices.size() / width;
  const int rows = width;

  glColor(mesh.color);
  glBegin(GL_QUADS);
  {
    for (int r = 0; r < rows - 1; ++r) {
      for (int c = 0; c < cols_per_row - 1; ++c) {
        const int tl = c + (r * cols_per_row);
        const int tr = c + 1 + (r * cols_per_row);
        const int br = c + 1 + ((r + 1) * cols_per_row);
        const int bl = c + ((r + 1) * cols_per_row);

        glVertex3d(pts[tl].x(), 1.0 - pts[tl].y(), Z_DIST);
        glVertex3d(pts[tr].x(), 1.0 - pts[tr].y(), Z_DIST);
        glVertex3d(pts[br].x(), 1.0 - pts[br].y(), Z_DIST);
        glVertex3d(pts[bl].x(), 1.0 - pts[bl].y(), Z_DIST);
      }
    }
    glEnd();
  }

  {
    glColor4d(0.1, 0.8, 0.1, 0.8);
    glLineWidth(1.0);

    for (int r = 0; r < rows - 1; ++r) {
      for (int c = 0; c < cols_per_row - 1; ++c) {
        const int tl = c + (r * cols_per_row);
        const int tr = c + 1 + (r * cols_per_row);
        const int br = c + 1 + ((r + 1) * cols_per_row);
        const int bl = c + ((r + 1) * cols_per_row);

        glBegin(GL_LINE_LOOP);
        glVertex3d(pts[tl].x(), 1.0 - pts[tl].y(), Z_DIST_OUTLINE);
        glVertex3d(pts[tr].x(), 1.0 - pts[tr].y(), Z_DIST_OUTLINE);
        glVertex3d(pts[br].x(), 1.0 - pts[br].y(), Z_DIST_OUTLINE);
        glVertex3d(pts[bl].x(), 1.0 - pts[bl].y(), Z_DIST_OUTLINE);
        glEnd();
      }
    }
  }
}

}  // namespace

void Ui2d::add_pointer_target(const PointerTarget &pointer_target) {
  const std::lock_guard<std::mutex> lk(draw_mutex_);
  back_buffer_.pointer_targets.push_back(pointer_target);
}

void Ui2d::add_lineplot(const LinePlot2d &line_plot) {
  const std::lock_guard<std::mutex> lk(draw_mutex_);
  back_buffer_.line_plots.push_back(line_plot);
}

void Ui2d::add_grid_mesh(const GridMesh &grid_mesh) {
  const std::lock_guard<std::mutex> lk(draw_mutex_);
  JASSERT_GT(grid_mesh.width, 1, "Grid width must be greater 1");
  JASSERT_FALSE(grid_mesh.vertices.empty(), "Grid cannot be empty.");
  JASSERT_EQ(grid_mesh.vertices.size() % grid_mesh.width, 0u,
             "Grid size must be divisible by width");
  back_buffer_.grid_meshes.push_back(grid_mesh);
}

void Ui2d::add_image(const cv::Mat &image, double width_m) {
  const std::lock_guard<std::mutex> lk(draw_mutex_);
  cv::Mat image_rgb;
  if (image.channels() == 1) {
    cv::cvtColor(image, image_rgb, cv::COLOR_GRAY2BGR);
  } else {
    image_rgb = image;
  }

  back_buffer_.images.push_back({image_rgb, width_m});
}

void Ui2d::clear() {
  const std::lock_guard<std::mutex> lk(draw_mutex_);
  back_buffer_.clear();
  front_buffer_.clear();
}

void Ui2d::flip() {
  const std::lock_guard<std::mutex> lk(draw_mutex_);
  front_buffer_ = std::move(back_buffer_);
}

void Ui2d::flush() {
  const std::lock_guard<std::mutex> lk(draw_mutex_);
  const auto insert = [](auto &into, const auto &from) {
    into.insert(into.begin(), from.begin(), from.end());
  };

  insert(front_buffer_.pointer_targets, back_buffer_.pointer_targets);
  insert(front_buffer_.line_plots, back_buffer_.line_plots);
  insert(front_buffer_.images, back_buffer_.images);
  insert(front_buffer_.points, back_buffer_.points);
  insert(front_buffer_.grid_meshes, back_buffer_.grid_meshes);
  insert(front_buffer_.lines, back_buffer_.lines);
}

void Ui2d::draw() const {
  const std::lock_guard<std::mutex> lk(draw_mutex_);

  // Create the character library when it's renderin' time
  if (char_lib_.size() == 0u) {
    char_lib_ = create_text_library();
  }

  const auto proj = Projection::get_from_current();

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();

  const double width = static_cast<double>(proj.viewport_size().width);
  const double height = static_cast<double>(proj.viewport_size().height);
  const double aspect_ratio = width / height;
  glOrtho(-aspect_ratio, aspect_ratio, -1.0, 1.0, -1.0, 1.0);

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();

  draw_pointer_targets(front_buffer_.pointer_targets, proj, char_lib_);
  for (const auto &line_plot : front_buffer_.line_plots) {
    draw_lineplot(line_plot, proj, char_lib_);
  }

  draw_points(front_buffer_.points, proj, char_lib_);
  draw_lines(front_buffer_.lines, proj, char_lib_);

  for (const auto &mesh : front_buffer_.grid_meshes) {
    draw_grid_mesh(mesh);
  }

  for (const auto &image : front_buffer_.images) {
    draw_image(image, proj, char_lib_);
  }

  glPopMatrix();

  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
}

}  // namespace viewer
