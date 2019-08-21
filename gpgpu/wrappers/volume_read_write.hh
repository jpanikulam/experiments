#pragma once

#include "gpgpu/wrappers/volume_size.hh"

#include "eigen.hh"

#include <CL/cl.hpp>

namespace jcc {
void fill_volume_zeros(const cl::CommandQueue cmd_queue,
                       const cl::Image3D& volume,
                       const jcc::VolumeSize& vol_size);

void fill_volume_section(const cl::CommandQueue cmd_queue,
                         const cl::Image3D& volume,
                         const jcc::Vec3i& region_start,
                         const jcc::VolumeSize& vol_size,
                         const jcc::Vec4 value);
}  // namespace jcc
