#pragma once

#include "primitive.hh"

#include <Eigen/Dense>

#include <vector>

namespace gl_viewer {

struct Surface {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  Eigen::MatrixXd surface;
  double          scale;
};

class Plot final : public Primitive {
 public:
  void draw() const override;

  void add_surface(const Surface& surface) {
    surfaces_.push_back(surface);
  }

 private:
  // Will alignment fuck us?
  std::vector<Surface> surfaces_;
};
}
