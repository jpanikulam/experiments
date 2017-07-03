#include "image.hh"

#include <GL/glew.h>

namespace gl_viewer {

Image::Image(const cv::Mat& image, const double scale, double alpha) {
  update_image(image, scale);
  alpha_ = alpha;
}

void Image::update_image(const cv::Mat& image, const double scale) {
  image.copyTo(image_);
  scale_ = scale;

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
}