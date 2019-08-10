#include "gpgpu/wrappers/image_read_write.hh"

#include "gpgpu/wrappers/errors.hh"

namespace jcc {

void send_image_to_device(const cl::CommandQueue &cmd_queue,
                          const cl::Image2D &dv_ptr,
                          const cv::Mat &mat) {
  constexpr cl_bool FLG_NO_BLOCK = CL_FALSE;

  cl::size_t<3> origin;
  {
    origin[0] = 0;
    origin[1] = 0;
    origin[2] = 0;
  }

  cl::size_t<3> region;
  {
    region[0] = mat.cols;
    region[1] = mat.rows;
    region[2] = 1;
  }

  JCHECK_STATUS(
      cmd_queue.enqueueWriteImage(dv_ptr, FLG_NO_BLOCK, origin, region, 0, 0, mat.data));
}

void read_image_from_device(const cl::CommandQueue &cmd_queue,
                            const cl::Image2D &dv_ptr,
                            Out<cv::Mat> out_mat) {
  cl::size_t<3> origin;
  {
    origin[0] = 0;
    origin[1] = 0;
    origin[2] = 0;
  }

  cl::size_t<3> region;
  {
    region[0] = out_mat->cols;
    region[1] = out_mat->rows;
    region[2] = 1;
  }

  constexpr cl_bool FLG_BLOCK = CL_TRUE;
  JCHECK_STATUS(
      cmd_queue.enqueueReadImage(dv_ptr, FLG_BLOCK, origin, region, 0, 0, out_mat->data));
}
}  // namespace jcc