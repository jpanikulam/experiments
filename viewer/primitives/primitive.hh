#pragma once

namespace viewer {
class Primitive {
 public:
  virtual ~Primitive() = default;
  virtual void draw() const = 0;
};
}