#include "viewer/interaction/ui2d.hh"

#include "viewer/projection.hh"

#include <GL/glew.h>

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
    glScaled(0.0005, -0.0005, 0.0005);
    write_string(target.text, char_lib);
  }
  glPopMatrix();
}

}  // namespace

void Ui2d::add_pointer_target(const PointerTarget &pointer_target) {
  const std::lock_guard<std::mutex> lk(draw_mutex_);
  back_buffer_.pointer_targets.push_back(pointer_target);
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
  constexpr double NEAR_CLIP = 0.001;
  constexpr double FAR_CLIP = 1000.0;

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

  // glMatrixMode(GL_MODELVIEW);
  glPopMatrix();

  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
}

}  // namespace viewer
