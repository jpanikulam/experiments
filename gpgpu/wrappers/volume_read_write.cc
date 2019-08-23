#include "gpgpu/wrappers/volume_read_write.hh"

namespace jcc {
void fill_volume_zeros(const cl::CommandQueue cmd_queue,
                       const cl::Image3D& volume,
                       const jcc::VolumeSize& vol_size) {
  const cl_float4 fill_color = {0.0f, 0.0f, 0.0f, 0.0f};

  std::array<std::size_t, 3> origin;
  {
    origin[0] = 0;
    origin[1] = 0;
    origin[2] = 0;
  }

  std::array<std::size_t, 3> region;
  {
    region[0] = vol_size.cols;
    region[1] = vol_size.rows;
    region[2] = vol_size.slices;
  }

  cmd_queue.enqueueFillImage(volume, fill_color, origin, region);
}

void fill_volume_section(const cl::CommandQueue cmd_queue,
                         const cl::Image3D& volume,
                         const jcc::Vec3i& region_start,
                         const jcc::VolumeSize& vol_size,
                         const jcc::Vec4 value) {
  const Eigen::Vector4f valuef = value.cast<float>();
  const cl_float4 fill_color = {valuef.x(), valuef.y(), valuef.z(), valuef.w()};

  std::array<std::size_t, 3> origin;
  {
    origin[0] = region_start.x();
    origin[1] = region_start.y();
    origin[2] = region_start.z();
  }

  std::array<std::size_t, 3> region;
  {
    region[0] = vol_size.cols;
    region[1] = vol_size.rows;
    region[2] = vol_size.slices;
  }

  cmd_queue.enqueueFillImage(volume, fill_color, origin, region);
}

void copy_volume(const cl::CommandQueue cmd_queue,
                 const cl::Image3D& src,
                 const cl::Image3D& dst,
                 const jcc::VolumeSize& vol_size) {
  std::array<std::size_t, 3> origin;
  {
    origin[0] = 0;
    origin[1] = 0;
    origin[2] = 0;
  }

  std::array<std::size_t, 3> region;
  {
    region[0] = vol_size.cols;
    region[1] = vol_size.rows;
    region[2] = vol_size.slices;
  }

  cmd_queue.enqueueCopyImage(src, dst, origin, origin, region);
}

}  // namespace jcc
