#pragma once

#include "primitive.hh"

namespace gl_viewer {

class Box final : public Primitive {
 public:
  void draw() const override;
};
}