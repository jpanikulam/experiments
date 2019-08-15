#pragma once

namespace viewer {
class Primitive {
 public:
  virtual ~Primitive() = default;
  virtual void flush() {
  }
  virtual void flip() {
  }

  virtual void state_update(){};
  virtual void draw() const = 0;
};
}  // namespace viewer