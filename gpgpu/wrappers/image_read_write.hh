#pragma once

#include "out.hh"

#include "gpgpu/opencl.hh"
#include <opencv2/opencv.hpp>

namespace jcc {

void send_image_to_device(const cl::CommandQueue &cmd_queue,
                          const cl::Image2D &dv_ptr,
                          const cv::Mat &mat);
void read_image_from_device(const cl::CommandQueue &cmd_queue,
                            const cl::Image2D &dv_ptr,
                            Out<cv::Mat> out_mat);

}  // namespace jcc