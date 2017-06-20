#pragma once

#include "primitive.hh"

#include <opencv2/opencv.hpp>

namespace gl_viewer {

class Image final : public Primitive {
 public:
  Image(const cv::Mat& image, double scale = 1.0, double alpha = 0.7);

  void update_image(const cv::Mat& image, double scale = 1.0);

  void draw() const override;

 private:
  cv::Mat      image_;
  unsigned int texture_id_;
  double       scale_ = 1.0;
  double       alpha_ = 0.7;

  bool allocated_texture_ = false;
};
}
