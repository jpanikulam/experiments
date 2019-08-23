#pragma once

#include "gpgpu/opencl.hh"

namespace jcc {

struct ClInfo {
  cl::Context context;
  cl::Device device;
};

}  // namespace jcc
