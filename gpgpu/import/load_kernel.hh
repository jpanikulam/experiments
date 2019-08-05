#pragma once

// %deps(${OpenCL_LIBRARY})

#include <CL/cl.hpp>

namespace jcc {

cl::Program read_kernel(const cl::Context& context,
                        const cl::Device& device,
                        const std::string& path);

}  // namespace jcc
