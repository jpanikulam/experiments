#include "gpgpu/import/load_kernel.hh"
#include "gpgpu/wrappers/create_context.hh"
#include "gpgpu/wrappers/errors.hh"
#include "gpgpu/wrappers/image_read_write.hh"

#include "gpgpu/demos/ray_lut.hh"
#include "util/timing.hh"

#include "viewer/interaction/ui2d.hh"
#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

#include "util/environment.hh"

#include "eigen.hh"
#include "out.hh"
#include "util/waves.hh"

#include <CL/cl.hpp>

#include <iostream>
#include <vector>

#include "gpgpu/demos/signed_distance_shapes.hh"

struct ImageSize {
  std::size_t cols = -1;
  std::size_t rows = -1;
};

estimation::NonlinearCameraModel nice_model(int cols, int rows) {
  estimation::ProjectionCoefficients proj_coeffs;
  proj_coeffs.fx = rows;
  proj_coeffs.fy = rows;
  proj_coeffs.cx = cols / 2;
  proj_coeffs.cy = rows / 2;

  proj_coeffs.k1 = 0.0;
  proj_coeffs.k2 = -0.0;
  proj_coeffs.p1 = 0.0;
  proj_coeffs.p2 = 0.0;
  proj_coeffs.k3 = 0.0;

  proj_coeffs.rows = rows;
  proj_coeffs.cols = cols;

  const estimation::NonlinearCameraModel model(proj_coeffs);
  return model;
}

void draw(viewer::Window3D& view,
          viewer::Ui2d& ui2d,
          cl::CommandQueue& cmd_queue,
          cl::Kernel& sdf_kernel,
          cl::Image2D& dv_rendered_image,
          const RenderConfig& cc_cfg,
          const ImageSize& out_im_size,
          float t) {
  const jcc::PrintingScopedTimer tt("Frame Time");
  const clRenderConfig cfg = cc_cfg.convert();
  JCHECK_STATUS(sdf_kernel.setArg(2, cfg));

  cl_float cl_t = t;
  JCHECK_STATUS(sdf_kernel.setArg(3, cl_t));

  const cl::NDRange work_group_size{static_cast<std::size_t>(out_im_size.cols),
                                    static_cast<std::size_t>(out_im_size.rows)};
  JCHECK_STATUS(
      cmd_queue.enqueueNDRangeKernel(sdf_kernel, {0}, work_group_size, {480, 1u}));

  cmd_queue.flush();

  cv::Mat out_img(cv::Size(out_im_size.cols, out_im_size.rows), CV_32FC4,
                  cv::Scalar(0.0, 0.0, 0.0, 0.0));
  jcc::read_image_from_device(cmd_queue, dv_rendered_image, out(out_img));

  out_img.convertTo(out_img, CV_8UC4);
  cv::cvtColor(out_img, out_img, CV_BGRA2BGR);

  ui2d.add_image(out_img, 1.0);
  ui2d.flip();
  out_img.release();
}

int main() {
  const auto view = viewer::get_window3d("Render Visualization");
  const auto ui2d = view->add_primitive<viewer::Ui2d>();
  const auto geo = view->add_primitive<viewer::SimpleGeometry>();

  cl_int status = 0;
  const auto cl_info = jcc::create_context();
  const auto kernels = read_kernels(
      cl_info, jcc::Environment::repo_path() + "gpgpu/demos/signed_distance.cl");

  auto sdf_kernel = kernels.at("compute_sdf");
  cl::CommandQueue cmd_queue(cl_info.context);

  ImageSize out_im_size;
  out_im_size.cols = 2 * 480;
  out_im_size.rows = 2 * 480;

  const auto model = nice_model(out_im_size.cols, out_im_size.rows);

  //
  // Send the ray LUT
  //

  const cl::ImageFormat ray_lut_fmt(CL_RGBA, CL_FLOAT);
  cl::Image2D dv_ray_lut(cl_info.context, CL_MEM_READ_ONLY, ray_lut_fmt, out_im_size.cols,
                         out_im_size.rows, 0, nullptr, &status);
  {
    JCHECK_STATUS(status);

    const cv::Mat ray_lut =
        jcc::create_ray_lut(model, out_im_size.cols, out_im_size.rows).clone();
    jcc::send_image_to_device(cmd_queue, dv_ray_lut, ray_lut);
  }

  //
  // Prepare a rendered image
  //

  const cl::ImageFormat output_image_fmt(CL_RGBA, CL_FLOAT);
  cl::Image2D dv_rendered_image(cl_info.context, CL_MEM_WRITE_ONLY, output_image_fmt,
                                out_im_size.cols, out_im_size.rows, 0, nullptr, &status);

  JCHECK_STATUS(sdf_kernel.setArg(0, dv_ray_lut));
  JCHECK_STATUS(sdf_kernel.setArg(1, dv_rendered_image));

  view->add_menu_hotkey("debug_mode", 0, 'P', 5);
  view->add_menu_hotkey("terminal_iteration", 10, 'I', 24);
  view->add_menu_hotkey("test_feature", 0, 'F', 2);

  // t += 0.1;

  float t = 0.0;
  RenderConfig cc_cfg;

  view->add_toggle_callback("terminal_iteration", [&](int iter_ct) {
    std::cout << "\tIteration Count --> " << iter_ct * 10 << std::endl;
    cc_cfg.terminal_iteration = iter_ct * 10;
    draw(*view, *ui2d, cmd_queue, sdf_kernel, dv_rendered_image, cc_cfg, out_im_size, t);
  });

  view->add_toggle_callback("test_feature", [&](int feature) {
    std::cout << "\tTest Feature --> " << feature << std::endl;
    cc_cfg.test_feature = feature;
    draw(*view, *ui2d, cmd_queue, sdf_kernel, dv_rendered_image, cc_cfg, out_im_size, t);
  });

  view->add_toggle_callback("debug_mode", [&](int dbg_mode) {
    if (dbg_mode == 1) {
      std::cout << "\tSetting Debug Mode: Iterations" << std::endl;
    } else if (dbg_mode == 2) {
      std::cout << "\tSetting Debug Mode: Cone Distance" << std::endl;
    } else if (dbg_mode == 3) {
      std::cout << "\tSetting Debug Mode: Ray Length" << std::endl;
    } else if (dbg_mode == 4) {
      std::cout << "\tSetting Debug Mode: Normals" << std::endl;
    } else {
      std::cout << "\tDisabling Debug Mode" << std::endl;
    }
    cc_cfg.debug_mode = dbg_mode;

    draw(*view, *ui2d, cmd_queue, sdf_kernel, dv_rendered_image, cc_cfg, out_im_size, t);
  });

  view->run_menu_callbacks();
  while (true) {
    t += 0.1;
    // cc_cfg.terminal_iteration = view->get_menu("terminal_iteration") * 10;

    draw(*view, *ui2d, cmd_queue, sdf_kernel, dv_rendered_image, cc_cfg, out_im_size, t);
    view->spin_until_step();
  }
}
