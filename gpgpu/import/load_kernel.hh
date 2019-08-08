#pragma once

// %deps(${OpenCL_LIBRARY})

#include "gpgpu/wrappers/cl_info.hh"

#include <CL/cl.hpp>

namespace jcc {

cl::Program read_kernel(const ClInfo& cl_info, const std::string& path);

}  // namespace jcc
