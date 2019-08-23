#pragma once

#include "gpgpu/wrappers/cl_info.hh"

#include "gpgpu/opencl.hh"

#include <map>

namespace jcc {

std::map<std::string, cl::Kernel> read_kernels(const ClInfo& cl_info,
                                               const std::string& path,
                                               const std::string& include_path = "");

}  // namespace jcc
