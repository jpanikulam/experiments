#include "gpgpu/wrappers/volume_read_write.hh"

// TODO
#include <iostream>

namespace jcc {

void fill_volume_zeros(const cl::CommandQueue& cmd_queue,
                       const cl::Image3D& volume,
                       const jcc::VolumeSize& vol_size) {
  fill_volume(cmd_queue, volume, vol_size, jcc::Vec4::Zero());
}

void fill_volume(const cl::CommandQueue& cmd_queue,
                 const cl::Image3D& volume,
                 const jcc::VolumeSize& vol_size,
                 const jcc::Vec4& value) {
  fill_volume_section(cmd_queue, volume, jcc::Vec3i::Zero(), vol_size, value);
}

void fill_volume_section(const cl::CommandQueue& cmd_queue,
                         const cl::Image3D& volume,
                         const jcc::Vec3i& region_start,
                         const jcc::VolumeSize& vol_size,
                         const jcc::Vec4& value) {
  const Eigen::Vector4f valuef = value.cast<float>();
  const cl_float4 fill_color = {valuef.x(), valuef.y(), valuef.z(), valuef.w()};

  cl::size_t<3> origin;
  {
    origin[0] = region_start.x();
    origin[1] = region_start.y();
    origin[2] = region_start.z();
  }

  cl::size_t<3> region;
  {
    region[0] = vol_size.cols;
    region[1] = vol_size.rows;
    region[2] = vol_size.slices;
  }

  cmd_queue.enqueueFillImage(volume, fill_color, origin, region);
}

void copy_volume(const cl::CommandQueue& cmd_queue,
                 const cl::Image3D& src,
                 const cl::Image3D& dst,
                 const jcc::VolumeSize& vol_size) {
  cl::size_t<3> origin;
  {
    origin[0] = 0;
    origin[1] = 0;
    origin[2] = 0;
  }

  cl::size_t<3> region;
  {
    region[0] = vol_size.cols;
    region[1] = vol_size.rows;
    region[2] = vol_size.slices;
  }

  cmd_queue.enqueueCopyImage(src, dst, origin, origin, region);
}

}  // namespace jcc
