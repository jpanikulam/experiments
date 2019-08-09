
#include "eigen.hh"
#include "gpgpu/import/load_kernel.hh"
#include "gpgpu/wrappers/create_context.hh"
#include "gpgpu/wrappers/errors.hh"

#include "estimation/calibration/nonlinear_camera_model.hh"

#include "viewer/interaction/ui2d.hh"
#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

#include <CL/cl.hpp>

#include <iostream>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

struct __attribute__((packed)) ProjectionCoefficientsf {
  ProjectionCoefficientsf() = default;
  ProjectionCoefficientsf(const estimation::ProjectionCoefficients &pc) {
    fx = static_cast<float>(pc.fx);
    fy = static_cast<float>(pc.fy);
    cx = static_cast<float>(pc.cx);
    cy = static_cast<float>(pc.cy);
    p1 = static_cast<float>(pc.p1);
    p2 = static_cast<float>(pc.p2);
    k1 = static_cast<float>(pc.k1);
    k2 = static_cast<float>(pc.k2);
    k3 = static_cast<float>(pc.k3);
    rows = static_cast<float>(pc.rows);
    cols = static_cast<float>(pc.cols);
  }

  float fx;
  float fy;
  float cx;
  float cy;

  float p1;
  float p2;

  float k1;
  float k2;
  float k3;

  cl_int rows;
  cl_int cols;
};

struct __attribute__((packed)) PackedSE3 {
  PackedSE3(const SE3 &se3) {
    const MatNd<3, 3> mat = se3.so3().matrix();
    r0[0] = mat(0, 0);
    r0[1] = mat(0, 1);
    r0[2] = mat(0, 2);
    r1[0] = mat(1, 0);
    r1[1] = mat(1, 1);
    r1[2] = mat(1, 2);
    r2[0] = mat(2, 0);
    r2[1] = mat(2, 1);
    r2[2] = mat(2, 2);

    t[0] = se3.translation().x;
    t[1] = se3.translation().y;
    t[2] = se3.translation().z;
  }
  float r0[4];
  float r1[4];
  float r2[4];
  float t[4];
};

cv::Ptr<cv::aruco::GridBoard> get_aruco_board() {
  const auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
  constexpr float FIDUCIAL_WIDTH_METERS = 99.0 / 1000;
  constexpr float FIDUCIAL_GAP_WIDTH_METERS = 50.0 / 1000;
  return cv::aruco::GridBoard::create(4, 4, FIDUCIAL_WIDTH_METERS,
                                      FIDUCIAL_GAP_WIDTH_METERS, dict);
}

estimation::NonlinearCameraModel make_model() {
  estimation::ProjectionCoefficients proj_coeffs;
  proj_coeffs.fx = 279.76;
  proj_coeffs.fy = 280.034;
  proj_coeffs.cx = 235.992;
  proj_coeffs.cy = 141.951;

  proj_coeffs.k1 = 0.0669332;
  proj_coeffs.k2 = -0.224151;
  proj_coeffs.p1 = 0.00993584;
  proj_coeffs.p2 = 0.00848696;
  proj_coeffs.k3 = 0.10179;

  // proj_coeffs.k1 = 0.0;
  // proj_coeffs.k2 = -0.0;
  // proj_coeffs.p1 = 0.0;
  // proj_coeffs.p2 = 0.0;
  // proj_coeffs.k3 = 0.0;

  proj_coeffs.rows = 480;
  proj_coeffs.cols = 480;

  const estimation::NonlinearCameraModel model(proj_coeffs);
  return model;
}

cv::Mat create_ray_lut(const estimation::NonlinearCameraModel &model,
                       const int cols,
                       const int rows) {
  cv::Mat deprojection_lut(cv::Size(cols, rows), CV_32FC4);
  for (int u = 0; u < cols; ++u) {
    for (int v = 0; v < rows; ++v) {
      const auto optl_ray = model.unproject(jcc::Vec2(u + 0.5, v + 0.5));
      const Eigen::Vector3f dir_f = optl_ray->direction.cast<float>();
      // deprojection_lut.at<cv::Vec3f>(u, v) = cv::Vec3f(0.0, 0.0, 1.0);
      deprojection_lut.at<cv::Vec4f>(u, v) =
          cv::Vec4f(dir_f.x(), dir_f.y(), dir_f.z(), 1.0);
    }
  }
  return deprojection_lut;
}

int main() {
  cl_int status = 0;
  const auto cl_info = jcc::create_context();
  const auto kernels =
      read_kernels(cl_info, "/home/jacob/repos/experiments/gpgpu/demos/render.cl");

  auto kernel = kernels.at("render");

  constexpr auto FLG_NO_NORMALIZED_COORDS = CL_FALSE;
  const cl::Sampler sampler(cl_info.context, FLG_NO_NORMALIZED_COORDS, CL_ADDRESS_CLAMP,
                            CL_FILTER_LINEAR);

  cv::Mat board_image;
  get_aruco_board()->draw(cv::Size(100, 100), board_image, 5, 1);
  board_image.convertTo(board_image, CV_32FC1);

  const int out_cols = 480;
  const int out_rows = 480;

  const cl::ImageFormat format(CL_R, CL_FLOAT);
  const cl::Image2D dv_fiducial_image(cl_info.context, CL_MEM_READ_ONLY, format,
                                      board_image.cols, board_image.rows, 0, nullptr,
                                      &status);
  JCHECK_STATUS(status);
  const cl::Image2D dv_rendered_image(cl_info.context, CL_MEM_WRITE_ONLY, format,
                                      out_cols, out_rows, 0, nullptr, &status);

  JCHECK_STATUS(status);
  const cl::ImageFormat ray_lut_fmt(CL_RGBA, CL_FLOAT);
  const cl::Image2D dv_ray_lut(cl_info.context, CL_MEM_READ_ONLY, ray_lut_fmt, out_cols,
                               out_rows, 0, nullptr, &status);
  JCHECK_STATUS(status);

  const auto cam_model = make_model();
  {
    JCHECK_STATUS(kernel.setArg(0, dv_fiducial_image));
    JCHECK_STATUS(kernel.setArg(1, dv_ray_lut));
    JCHECK_STATUS(kernel.setArg(2, dv_rendered_image));
    JCHECK_STATUS(kernel.setArg(3, sampler));
  }

  cl::CommandQueue cmd_queue(cl_info.context);

  cl::size_t<3> origin;
  {
    origin[0] = 0;
    origin[1] = 0;
    origin[2] = 0;
  }

  cl::size_t<3> fiducial_region;
  {
    fiducial_region[0] = board_image.cols;
    fiducial_region[1] = board_image.rows;
    fiducial_region[2] = 1;
  }

  cl::size_t<3> out_image_region;
  {
    out_image_region[0] = out_cols;
    out_image_region[1] = out_rows;
    out_image_region[2] = 1;
  }
  JCHECK_STATUS(cmd_queue.enqueueWriteImage(dv_fiducial_image, CL_FALSE, origin,
                                            fiducial_region, 0, 0, board_image.data));

  const cv::Mat ray_lut = create_ray_lut(cam_model, out_cols, out_rows).clone();
  JCHECK_STATUS(cmd_queue.enqueueWriteImage(dv_ray_lut, CL_FALSE, origin,
                                            out_image_region, 0, 0, ray_lut.data));

  const auto view = viewer::get_window3d("Gravity Visualization");
  const auto ui2d = view->add_primitive<viewer::Ui2d>();

  for (float theta = 0.0f; theta < 6.14f; theta += 0.05f) {
    JCHECK_STATUS(kernel.setArg(4, theta));
    cv::Mat out_img(cv::Size(out_cols, out_rows), CV_32FC1, cv::Scalar(0.0));
    JCHECK_STATUS(cmd_queue.enqueueNDRangeKernel(
        kernel, {0},
        {static_cast<std::size_t>(out_img.cols), static_cast<std::size_t>(out_img.rows)},
        {}));
    JCHECK_STATUS(cmd_queue.enqueueReadImage(dv_rendered_image, CL_TRUE, origin,
                                             out_image_region, 0, 0, out_img.data));
    out_img.convertTo(out_img, CV_8UC1);
    ui2d->add_image(out_img, 1.0);
    ui2d->flip();
    view->spin_until_step();
  }
}