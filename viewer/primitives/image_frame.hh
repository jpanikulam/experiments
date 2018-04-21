#pragma once

#include "frame.hh"

namespace gl_viewer {

class ImageFrame final : public Frame {
 public:
  ImageFrame(const SE3& frame_from_parent, int rows, int cols, double scale);

  void draw() const override;

 private:
  int    rows_  = 1;
  int    cols_  = 1;
  double scale_ = 1.0;
};

}  // namespace gl_viewer
