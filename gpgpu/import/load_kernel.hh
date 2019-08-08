#pragma once

#include "gpgpu/wrappers/cl_info.hh"

#include <CL/cl.hpp>

#include <map>

namespace jcc {

std::map<std::string, cl::Kernel> read_kernels(const ClInfo& cl_info,
                                               const std::string& path);

}  // namespace jcc
