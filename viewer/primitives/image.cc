#include "viewer/primitives/image.hh"

#include "logging/assert.hh"

#include <GL/glew.h>

namespace viewer {

Image::Image(const cv::Mat& image, const double scale, double alpha) {
  to_update_ = true;
  alpha_ = alpha;
  width_m_ = scale;
  update_image(image);
}

Image::Image(const Eigen::MatrixXd& image, double scale, double alpha) {
  to_update_ = true;
  alpha_ = alpha;
  width_m_ = scale;
  update_image(image);
}

void Image::update_image(const Eigen::MatrixXd& image) {
  const std::lock_guard<std::mutex> lk(draw_mutex_);

  const Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> image_as_uchar =
      (image * 255.0).cast<uint8_t>();
  cv::Mat new_image;
  cv::eigen2cv(image_as_uchar, new_image);
  cv::cvtColor(new_image, image_, cv::COLOR_GRAY2BGR);
  to_update_ = true;
}

void Image::update_image(const cv::Mat& image) {
  const std::lock_guard<std::mutex> lk(draw_mutex_);
  if (image.channels() == 1) {
    cv::Mat not_yet_flipped;
    cv::cvtColor(image, not_yet_flipped, cv::COLOR_GRAY2BGR);
    cv::flip(not_yet_flipped, image_, 0);
  } else if (image.channels() == 3) {
    cv::flip(image, image_, 0);
  } else {
    JASSERT(false, "This type of image is unsupported");
  }
  to_update_ = true;
}

void Image::update_gl() const {
  if (!tex_.ready()) {
    const double width_per_height = image_.cols / static_cast<double>(image_.rows);
    const jcc::Vec2 size(width_per_height, 1.0);
    tex_ = SmartTexture(size);
  }
  tex_.tex_image_2d(GL_TEXTURE_2D, 0, GL_RGB, image_.cols, image_.rows, 0, GL_BGR,
                    GL_UNSIGNED_BYTE, image_.data);
}

void Image::draw() const {
  const std::lock_guard<std::mutex> lk(draw_mutex_);

  if (to_update_) {
    update_gl();
    to_update_ = false;
  }

  glEnable(GL_TEXTURE_2D);
  tex_.draw();
  glDisable(GL_TEXTURE_2D);
}
}  // namespace viewer
