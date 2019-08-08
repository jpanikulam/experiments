#pragma once

#include <CL/cl.hpp>

namespace jcc {

struct ClInfo {
  cl::Context context;
  cl::Device device;
};

}  // namespace jcc
