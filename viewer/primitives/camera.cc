#include "viewer/primitives/camera.hh"

#include <GL/glew.h>

#include "viewer/gl_aliases.hh"

namespace viewer {

cv::Mat Camera::capture_framebuffer() const {
  constexpr int ORIGIN_X = 0;
  constexpr int ORIGIN_Y = 0;
  constexpr int FORMAT = GL_RED;

  constexpr int WIDTH = 640;
  constexpr int HEIGHT = 640;

  constexpr int TYPE = GL_UNSIGNED_BYTE;

  uint8_t* out_data = new uint8_t[WIDTH * HEIGHT];
  glReadPixels(ORIGIN_X, ORIGIN_Y, WIDTH, HEIGHT, FORMAT, TYPE, out_data);

  // glGetTexImage(1, 0, FORMAT, TYPE, out_data);

  // void glGetTexImage( GLenum target,
  // GLint level,
  // GLenum format,
  // GLenum type,
  // GLvoid * pixels);

  constexpr int IM_CV_TYPE = CV_8UC1;
  return cv::Mat(HEIGHT, WIDTH, IM_CV_TYPE, out_data);
}

void Camera::prepare_view() const {
  // back_buffer_.bind();
  glViewport(0, 0, size_.width, size_.height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  const double aspect_ratio =
      (static_cast<double>(size_.width) / static_cast<double>(size_.height));
  constexpr double NEAR_CLIP = 0.001;
  constexpr double FAR_CLIP = 1000.0;
  constexpr double FOV = 60.0;
  gluPerspective(FOV, aspect_ratio, NEAR_CLIP, FAR_CLIP);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glTransform(world_from_camera_);
}

void Camera::draw() {
  const std::lock_guard<std::mutex> lk(draw_mutex_);
  image_ = capture_framebuffer();
  have_image_ = true;
}

}  // namespace viewer