#include "image.hh"

#include <GL/glew.h>

namespace gl_viewer {

Image::Image(const cv::Mat& image, const double scale, double alpha) {
  to_update_ = true;
  alpha_     = alpha;
  update_image(image, scale);
}

Image::Image(const Eigen::MatrixXd& image, double scale, double alpha) {
  to_update_ = true;
  alpha_     = alpha;
  update_image(image, scale);
}

void Image::update_image(const Eigen::MatrixXd& image, double scale) {
  const std::lock_guard<std::mutex> lk(draw_mutex_);

  scale_ = scale;

  const Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> image_as_uchar = (image * 255.0).cast<uint8_t>();
  cv::Mat                                                      new_image;
  cv::eigen2cv(image_as_uchar, new_image);
  cv::cvtColor(new_image, image_, cv::COLOR_GRAY2BGR);
  to_update_ = true;
}

void Image::update_image(const cv::Mat& image, double scale) {
  const std::lock_guard<std::mutex> lk(draw_mutex_);

  scale_ = scale;
  if (image.channels() == 1) {
    cv::cvtColor(image, image_, cv::COLOR_GRAY2BGR);
  } else {
    image.copyTo(image_);
  }
  to_update_ = true;
}

void Image::update_gl() const {
  if (!allocated_texture_) {
    glGenTextures(1, &texture_id_);
    allocated_texture_ = true;
  }

  glBindTexture(GL_TEXTURE_2D, texture_id_);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image_.cols, image_.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, image_.data);
}

void Image::draw() const {
  const std::lock_guard<std::mutex> lk(draw_mutex_);

  if (to_update_) {
    update_gl();
    to_update_ = false;
  }

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, texture_id_);

  glColor4d(1.0, 1.0, 1.0, alpha_);
  glBegin(GL_QUADS);

  /*  glTexCoord2d(0, 0);
    glVertex3d(0, 0, 0);

    glTexCoord2d(0, 1);
    glVertex3d(0, image_.rows * scale_, 0);

    glTexCoord2d(1, 1);
    glVertex3d(image_.cols * scale_, image_.rows * scale_, 0);

    glTexCoord2d(1, 0);
    glVertex3d(image_.cols * scale_, 0, 0);
  */

  glTexCoord2d(1, 0);
  glVertex3d(image_.cols * scale_, 0, 0);
  glTexCoord2d(1, 1);
  glVertex3d(image_.cols * scale_, image_.rows * scale_, 0);
  glTexCoord2d(0, 1);
  glVertex3d(0, image_.rows * scale_, 0);
  glTexCoord2d(0, 0);
  glVertex3d(0, 0, 0);

  glEnd();

  glDisable(GL_TEXTURE_2D);
}
}  // namespace gl_viewer
