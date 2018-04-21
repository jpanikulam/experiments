#pragma once

namespace gl_viewer {
class Primitive {
 public:
  virtual ~Primitive() = default;
  virtual void draw() const = 0;

  void lockdraw();
};
}