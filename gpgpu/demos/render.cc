// Limitations:
//  - Assumes many things about work group sizes and image sizes
//    - I would not be surprised if one got a CL_INVALID_WORK_GROUP_SIZE
//

#include "gpgpu/import/load_kernel.hh"
#include "gpgpu/wrappers/create_context.hh"
#include "gpgpu/wrappers/errors.hh"
#include "gpgpu/wrappers/image_read_write.hh"

#include "estimation/calibration/nonlinear_camera_model.hh"
#include "util/timing.hh"

#include "viewer/interaction/ui2d.hh"
#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

#include "eigen.hh"
#include "out.hh"
#include "util/waves.hh"

#include <CL/cl.hpp>

#include <opencv2/aruco.hpp>

#include <iostream>
#include <vector>

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
    rows = static_cast<cl_int>(pc.rows);
    cols = static_cast<cl_int>(pc.cols);
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

    t[0] = se3.translation().x();
    t[1] = se3.translation().y();
    t[2] = se3.translation().z();
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

  proj_coeffs.rows = 270;
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
      deprojection_lut.at<cv::Vec4f>(v, u) =
          cv::Vec4f(dir_f.x(), dir_f.y(), dir_f.z(), 1.0);
    }
  }
  return deprojection_lut;
}

class Renderer {
 public:
  Renderer(const jcc::ClInfo &cl_info,
           const cl::CommandQueue cmd_queue,
           const estimation::NonlinearCameraModel &cam_model,
           const cv::Mat &source_image,
           const int cols,
           const int rows,
           const cl::Kernel &render_kernel) {
    cl_int status;
    out_cols_ = cols;
    out_rows_ = rows;
    render_kernel_ = render_kernel;

    //
    // Store and send the reference image
    //
    const cl::ImageFormat source_image_fmt(CL_R, CL_FLOAT);
    dv_source_image_ =
        cl::Image2D(cl_info.context, CL_MEM_READ_ONLY, source_image_fmt,
                    source_image.cols, source_image.rows, 0, nullptr, &status);
    JCHECK_STATUS(status);
    jcc::send_image_to_device(cmd_queue, dv_source_image_, source_image);

    //
    // Set up ray LUT texture
    //

    const cl::ImageFormat ray_lut_fmt(CL_RGBA, CL_FLOAT);
    dv_ray_lut_ = cl::Image2D(cl_info.context, CL_MEM_READ_ONLY, ray_lut_fmt, out_cols_,
                              out_rows_, 0, nullptr, &status);
    JCHECK_STATUS(status);
    const cv::Mat ray_lut = create_ray_lut(cam_model, out_cols_, out_rows_).clone();
    jcc::send_image_to_device(cmd_queue, dv_ray_lut_, ray_lut);

    //
    // Set up sampler
    //
    constexpr auto FLG_NO_NORMALIZED_COORDS = CL_FALSE;
    sampler_ = cl::Sampler(cl_info.context, FLG_NO_NORMALIZED_COORDS, CL_ADDRESS_CLAMP,
                           CL_FILTER_LINEAR);
  }

  void render(const cl::CommandQueue &cmd_queue,
              const SE3 &camera_from_plane,
              const cl::Image2D &dv_rendered_image,
              cv::Mat *out_img = nullptr) {
    const PackedSE3 cl_se3(camera_from_plane);
    {
      JCHECK_STATUS(render_kernel_.setArg(0, dv_source_image_));
      JCHECK_STATUS(render_kernel_.setArg(1, dv_ray_lut_));
      JCHECK_STATUS(render_kernel_.setArg(2, dv_rendered_image));
      JCHECK_STATUS(render_kernel_.setArg(3, sampler_));
      JCHECK_STATUS(render_kernel_.setArg(4, cl_se3));
    }

    const cl::NDRange work_group_size{static_cast<std::size_t>(out_cols_),
                                      static_cast<std::size_t>(out_rows_)};
    JCHECK_STATUS(
        cmd_queue.enqueueNDRangeKernel(render_kernel_, {0}, work_group_size, {480, 1}));

    if (out_img) {
      *out_img = cv::Mat(cv::Size(out_cols_, out_rows_), CV_32FC1, cv::Scalar(0.0));
      jcc::read_image_from_device(cmd_queue, dv_rendered_image, out(*out_img));
    }
  }

 private:
  cl::Sampler sampler_;
  cv::Mat ray_lut_;
  cl::Image2D dv_ray_lut_;
  cl::Image2D dv_source_image_;
  cl::Kernel render_kernel_;

  int out_cols_ = -1;
  int out_rows_ = -1;
};

int main() {
  cl_int status = 0;
  const auto cl_info = jcc::create_context();
  const auto kernels =
      read_kernels(cl_info, "/home/jacob/repos/experiments/gpgpu/demos/render.cl");

  auto render_kernel = kernels.at("render");
  auto ssd_kernel = kernels.at("sum_squared_diff");

  cv::Mat board_image;
  get_aruco_board()->draw(cv::Size(100, 100), board_image, 5, 1);
  board_image.convertTo(board_image, CV_32FC1);
  const auto cam_model = make_model();

  const int out_cols = 480;
  const int out_rows = 270;
  const cl::ImageFormat write_image_fmt(CL_R, CL_FLOAT);
  const cl::Image2D dv_rendered_image(cl_info.context, CL_MEM_READ_WRITE, write_image_fmt,
                                      out_cols, out_rows, 0, nullptr, &status);
  JCHECK_STATUS(status);
  const cl::Image2D dv_reference_image(cl_info.context, CL_MEM_READ_WRITE,
                                       write_image_fmt, out_cols, out_rows, 0, nullptr,
                                       &status);
  JCHECK_STATUS(status);

  cl::CommandQueue cmd_queue(cl_info.context);

  Renderer renderer(cl_info, cmd_queue, cam_model, board_image, out_cols, out_rows,
                    render_kernel);

  const double ref_theta = 0.0;
  const jcc::Vec3 ref_axis(12.0 * std::cos(ref_theta) * std::sin(ref_theta),
                           std::cos(ref_theta), 0.0);
  const SE3 ref_camera_from_plane(SO3::exp(ref_axis), jcc::Vec3(-0.5, -0.5, 2.0));
  renderer.render(cmd_queue, ref_camera_from_plane, dv_reference_image);

  const auto view = viewer::get_window3d("Render Visualization");
  const auto ui2d = view->add_primitive<viewer::Ui2d>();
  const auto geo = view->add_primitive<viewer::SimpleGeometry>();

  const int n_partial_sums = out_cols * out_rows / 480;
  const int n_partial_sum_bytes = sizeof(cl_float) * n_partial_sums;
  cl::Buffer dv_partial_sums(cl_info.context, CL_MEM_WRITE_ONLY, n_partial_sum_bytes);
  cl::Buffer dv_full_sum(cl_info.context, CL_MEM_WRITE_ONLY, sizeof(cl_float));

  JCHECK_STATUS(ssd_kernel.setArg(0, dv_reference_image));
  JCHECK_STATUS(ssd_kernel.setArg(1, dv_rendered_image));
  JCHECK_STATUS(ssd_kernel.setArg(2, dv_partial_sums));
  JCHECK_STATUS(ssd_kernel.setArg(3, dv_full_sum));
  {
    jcc::ScopedTimer tt;
    for (float theta = 0.0f; theta < 100.14f; theta += 0.01f) {
      const jcc::Vec3 axis(12.0 * std::cos(theta) * std::sin(theta), std::cos(theta),
                           0.0);
      const SE3 camera_from_plane(SO3::exp(axis), jcc::Vec3(-0.5, -0.5, 2.0));
      cv::Mat out_img;
      renderer.render(cmd_queue, camera_from_plane, dv_rendered_image, &out_img);

      JCHECK_STATUS(
          cmd_queue.enqueueNDRangeKernel(ssd_kernel, {0},
                                         cl::NDRange(static_cast<std::size_t>(out_cols),
                                                     static_cast<std::size_t>(out_rows)),
                                         {480, 1}));

      std::vector<float> partial_sums(n_partial_sums, 0.0f);
      constexpr bool BLOCK_READ = true;
      JCHECK_STATUS(cmd_queue.enqueueReadBuffer(
          dv_partial_sums, BLOCK_READ, 0, n_partial_sum_bytes, partial_sums.data()));

      float blk_total;
      JCHECK_STATUS(cmd_queue.enqueueReadBuffer(dv_full_sum, BLOCK_READ, 0,
                                                sizeof(cl_float), &blk_total));

      geo->add_point({axis, jcc::Vec4((40.0 - blk_total) / 40.0, 0.5, 0.5, 1.0)});
      geo->flush();

      out_img.convertTo(out_img, CV_8UC1);
      // ui2d->add_image(out_img, 1.0);
      // ui2d->flip();
    }
  }
  view->spin_until_step();
}

/*
Here are some benchmarks that I've spent now some time carefully verifying.

I'm able to perform the following 3 operations consistently at 2440 Hz.
  - Render a 480 x 270 `float` image via ray-plane intersection
  - Compute sum-squared difference via *twice* work-group parallel-reduce-add, and a bunch
of global memory reads
  - Transfer the *whole image* and the difference scalar

This means we are re transferring 1.3GB/s
This means we can perform the entire up/down transmit cycle with the GPU for a 500kb image
once every *FOUR MICROSECONDS*

If we don't transmit the image back, the rate is quadrupled. So for an optimization task
where we don't need to inspect the rendered image on the CPU, we can generate an error
every microsecond

Can tensorflow do this???

Contributing reasons:
- We do everything including the serial parts on the GPU
- We use 2D-caching texture memory on the GPU, instead of buffer memory -- buffer memory
is simple and therefore very tempting

NOTE: This isn't on jet hardware, but anyone familiar with GPU latency "myths" should be
surprised by these numbers.
*/