#pragma once

#include <GL/glew.h>

#include "eigen.hh"

namespace jcc {

class ElementBuffer {
 public:
  using Vec3ui32 = Eigen::Matrix<uint32_t, 3, 1>;

  ElementBuffer();
  ElementBuffer(const std::vector<Vec3ui32>& indices);

  void draw_elements() const;

  ~ElementBuffer();

  void bind() const;
  void destroy();

  GLuint get_id() const {
    return buffer_id_;
  }

 private:
  GLuint buffer_id_ = 0;
  int n_faces_ = -1;
};

}  // namespace jcc
