#pragma once

#include <cstddef>

namespace jcc {
struct AttributeDescription {
  int location;
  int size;
  int type;
};

struct UniformDescription {
  int location;
  int size;
  int type;
};
}  // namespace jcc