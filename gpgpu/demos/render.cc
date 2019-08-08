
#include "eigen.hh"
#include "gpgpu/import/load_kernel.hh"
#include "gpgpu/wrappers/create_context.hh"
#include "gpgpu/wrappers/errors.hh"

#include "viewer/interaction/ui2d.hh"
#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

#include <CL/cl.hpp>

#include <iostream>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

inline cv::Ptr<cv::aruco::GridBoard> get_aruco_board() {
  const auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
  constexpr float FIDUCIAL_WIDTH_METERS = 99.0 / 1000;
  constexpr float FIDUCIAL_GAP_WIDTH_METERS = 50.0 / 1000;
  return cv::aruco::GridBoard::create(4, 4, FIDUCIAL_WIDTH_METERS,
                                      FIDUCIAL_GAP_WIDTH_METERS, dict);
}

int main() {
  cl_int status = 0;
  const auto cl_info = jcc::create_context();
  const auto kernels =
      read_kernels(cl_info, "/home/jacob/repos/experiments/gpgpu/demos/render.cl");

  auto kernel = kernels.at("render");

  constexpr auto FLG_NO_NORMALIZED_COORDS = CL_FALSE;
  const cl::Sampler sampler(cl_info.context, FLG_NO_NORMALIZED_COORDS,
                            CL_ADDRESS_CLAMP, CL_FILTER_LINEAR);

  cv::Mat board_image;
  get_aruco_board()->draw(cv::Size(1000, 1000), board_image, 5, 1);
  board_image.convertTo(board_image, CV_32FC1);

  const cl::ImageFormat format(CL_R, CL_FLOAT);
  cl::Image2D dv_input_image(cl_info.context, CL_MEM_READ_ONLY, format, board_image.cols,
                             board_image.rows);
  cl::Image2D dv_output_image(cl_info.context, CL_MEM_WRITE_ONLY, format,
                              board_image.cols, board_image.rows);
  kernel.setArg(0, dv_input_image);
  kernel.setArg(1, dv_output_image);
  kernel.setArg(2, sampler);

  cl::CommandQueue cmd_queue(cl_info.context);

  cl::size_t<3> origin;
  origin[0] = 0;
  origin[1] = 0;
  origin[2] = 0;

  cl::size_t<3> region;
  region[0] = board_image.cols;
  region[1] = board_image.rows;
  region[2] = 1;

  status = cmd_queue.enqueueWriteImage(dv_input_image, CL_FALSE, origin, region, 0, 0,
                                       board_image.data);
  jcc::check_status(status);

  const auto view = viewer::get_window3d("Gravity Visualization");
  const auto ui2d = view->add_primitive<viewer::Ui2d>();

  for (float theta = 0.0f; theta < 3.0f; theta += 0.05f) {
    kernel.setArg(3, theta);
    status = cmd_queue.enqueueNDRangeKernel(kernel, {0},
                                            {board_image.cols, board_image.rows}, {});
    jcc::check_status(status);

    cv::Mat out_img(cv::Size(board_image.rows, board_image.cols), CV_32FC1,
                    cv::Scalar(0.0));
    status = cmd_queue.enqueueReadImage(dv_output_image, CL_TRUE, origin, region, 0, 0,
                                        out_img.data);
    jcc::check_status(status);

    out_img.convertTo(out_img, CV_8UC1);
    ui2d->add_image(out_img, 1.0);
    ui2d->flip();
    view->spin_until_step();
  }
}