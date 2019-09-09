#include <GL/glew.h>

#include "viewer/gl_aliases.hh"
#include "viewer/rendering/pre_render.hh"

namespace viewer {

void apply_view(const OrbitCamera &view) {
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glTransform(view.camera_from_anchor());
  glTransform(view.anchor_from_world());
}

void prepare_to_render() {
  //
  // Flag soup
  //

  glShadeModel(GL_SMOOTH);

  // Check depth when rendering
  glEnable(GL_DEPTH_TEST);

  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_BLEND);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glClearColor(0.1, 0.1, 0.1, 1.0f);

  glEnable(GL_SAMPLE_ALPHA_TO_COVERAGE);
  glEnable(GL_LINE_SMOOTH);
  glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

  glEnable(GL_MULTISAMPLE);
}

void set_perspective(const GlSize &gl_size, bool ortho) {
  glViewport(0, 0, gl_size.width, gl_size.height);
  glMatrixMode(GL_PROJECTION);

  glLoadIdentity();

  constexpr double NEAR_CLIP = 0.001;
  constexpr double FAR_CLIP = 1000.0;
  const double aspect_ratio =
      (static_cast<double>(gl_size.width) / static_cast<double>(gl_size.height));
  if (ortho) {
    glOrtho(-aspect_ratio, aspect_ratio, -1.0, 1.0, NEAR_CLIP, FAR_CLIP);
  } else {
    constexpr double FOV = 60.0;
    gluPerspective(FOV, aspect_ratio, NEAR_CLIP, FAR_CLIP);
  }
}

}  // namespace viewer