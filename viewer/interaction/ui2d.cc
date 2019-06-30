#include "viewer/interaction/ui2d.hh"

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

void draw_lineplot(const LinePlot2d &line_plot, const CharacterLibrary &char_lib) {
  glPushAttrib(GL_CURRENT_BIT | GL_LINE_BIT);

  glTranslated(0.2, 0.2, 0.0);

  glColor4d(0.6, 0.6, 0.6, 0.6);
  glBegin(GL_QUADS);
  const double field_y_max = 0.5;
  const double field_y_min = -0.5;
  const double field_x_min = 0.0;
  const double field_x_max = 1.0;

  glVertex3d(field_x_min, field_y_min, 0.9);
  glVertex3d(field_x_max, field_y_min, 0.9);
  glVertex3d(field_x_max, field_y_max, 0.9);
  glVertex3d(field_x_min, field_y_max, 0.9);
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
      const double x_val = (pt.x() - range.x_min) / x_range;
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

void Ui2d::clear() {
  const std::lock_guard<std::mutex> lk(draw_mutex_);
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

  // Square: Fine
  // Full-Screen: Text is the right size, point is in the wrong place
  // glOrtho(-1.0, 1.0, -1.0 * 0.5 * aspect_ratio, 1.0 * 0.5 * aspect_ratio, -1.0, 1.0);

  // This seems to be fine....wtf?? This is impossible
  // Square: Fine
  // glOrtho(-1.0, 1.0, aspect_ratio, 1.0 * aspect_ratio, -1.0, 1.0);

  // glOrtho(-aspect_ratio, aspect_ratio, -1.0, 1.0, -1.0, 1.0);

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();

  draw_pointer_targets(front_buffer_.pointer_targets, proj, char_lib_);
  for (const auto &line_plot : front_buffer_.line_plots) {
    draw_lineplot(line_plot, char_lib_);
  }

  // glMatrixMode(GL_MODELVIEW);
  glPopMatrix();

  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
}

}  // namespace viewer