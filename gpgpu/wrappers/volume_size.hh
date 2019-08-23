#pragma once

#include <cstddef>

namespace jcc {

struct VolumeSize {
  std::size_t cols;
  std::size_t rows;
  std::size_t slices;
};

}  // namespace jcc
