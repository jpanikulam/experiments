#include "gpgpu/demos/ray_lut.hh"
#include "gpgpu/import/load_kernel.hh"
#include "gpgpu/interplatform/cl_liegroups.hh"
#include "gpgpu/wrappers/create_context.hh"
#include "gpgpu/wrappers/errors.hh"
#include "gpgpu/wrappers/image_read_write.hh"
#include "gpgpu/wrappers/volume_size.hh"

#include "util/timing.hh"

#include "viewer/interaction/ui2d.hh"
#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

#include "util/environment.hh"

#include "eigen.hh"
#include "out.hh"
#include "util/waves.hh"

#include "gpgpu/opencl.hh"

#include <iostream>
#include <vector>

struct RenderConfig {
  int terminal_iteration;
  int test_feature;
  int debug_mode;
};

void draw(viewer::Window3D& view,
          viewer::Ui2d& ui2d,
          cl::CommandQueue& cmd_queue,
          cl::Kernel& vol_draw_kernel,
          cl::Image2D& dv_rendered_image,
          const RenderConfig& cc_cfg,
          const jcc::ImageSize& out_im_size,
          float t) {
  const jcc::PrintingScopedTimer tt("Frame Time");
  const SE3 world_from_camera = view.standard_camera_from_world().inverse();

  const jcc::clSE3 cl_world_from_camera(world_from_camera);

  JCHECK_STATUS(vol_draw_kernel.setArg(3, cl_world_from_camera));

  cl_float cl_t = t;
  JCHECK_STATUS(vol_draw_kernel.setArg(4, cl_t));

  const cl::NDRange work_group_size{static_cast<std::size_t>(out_im_size.cols),
                                    static_cast<std::size_t>(out_im_size.rows)};

  const cl::NDRange local_size({32u, 32u});
  JCHECK_STATUS(
      cmd_queue.enqueueNDRangeKernel(vol_draw_kernel, {0}, work_group_size, local_size));

  cmd_queue.flush();

  cv::Mat out_img(cv::Size(out_im_size.cols, out_im_size.rows), CV_32FC4,
                  cv::Scalar(0.0, 0.0, 0.0, 0.0));
  jcc::read_image_from_device(cmd_queue, dv_rendered_image, out(out_img));

  out_img.convertTo(out_img, CV_8UC4);
  cv::cvtColor(out_img, out_img, CV_BGRA2BGR);

  ui2d.add_image(out_img, 1.0);
  ui2d.flip();
}

void populate_test_volume(cl::CommandQueue& cmd_queue,
                          cl::Kernel& test_populate_kernel,
                          const cl::Image3D& volume,
                          const jcc::VolumeSize& vol_size) {
  test_populate_kernel.setArg(0, volume);

  const cl_float cl_t = 0.0f;

  test_populate_kernel.setArg(1, cl_t);
  const cl::NDRange work_group_size{vol_size.cols, vol_size.rows, vol_size.slices};

  const cl::NDRange local_size({10u, 10u, 1u});
  JCHECK_STATUS(cmd_queue.enqueueNDRangeKernel(test_populate_kernel, {0},
  work_group_size, local_size));
}

int main() {
  cl_int status = 0;
  const auto cl_info = jcc::create_context();
  const auto kernels = read_kernels(
      cl_info, jcc::Environment::repo_path() + "gpgpu/kernels/render_volume.cl");

  cl::CommandQueue cmd_queue(cl_info.context);

  auto test_populate_kernel = kernels.at("populate_test_volume");
  auto vol_draw_kernel = kernels.at("render_volume");

  const auto view = viewer::get_window3d("Render Visualization");
  view->set_view_preset(viewer::Window3D::ViewSetting::CAMERA);
  view->set_azimuth(0.0);
  view->set_elevation(0.0);

  const auto background = view->add_primitive<viewer::SimpleGeometry>();
  const geometry::shapes::Plane ground{jcc::Vec3(0.0, 1.0, 0.0).normalized(), 1.0};
  background->add_plane({ground});
  background->flip();

  const auto ui2d = view->add_primitive<viewer::Ui2d>();
  const auto geo = view->add_primitive<viewer::SimpleGeometry>();

  jcc::ImageSize out_im_size;
  out_im_size.cols = 2 * 480;
  out_im_size.rows = 2 * 480;

  const auto model = jcc::nice_model(out_im_size);

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

  const jcc::VolumeSize vol_size{100, 100, 100};
  const cl::ImageFormat vol_image_fmt(CL_RGBA, CL_FLOAT);
  cl::Image3D dv_volume(cl_info.context, CL_MEM_READ_WRITE, vol_image_fmt, vol_size.cols,
                        vol_size.rows, vol_size.slices, 0, 0, nullptr, &status);

  populate_test_volume(cmd_queue, test_populate_kernel, dv_volume, vol_size);
  //
  // Prepare a rendered image
  //

  const cl::ImageFormat output_image_fmt(CL_RGBA, CL_FLOAT);
  cl::Image2D dv_rendered_image(cl_info.context, CL_MEM_WRITE_ONLY, output_image_fmt,
                                out_im_size.cols, out_im_size.rows, 0, nullptr, &status);

  JCHECK_STATUS(vol_draw_kernel.setArg(0, dv_ray_lut));
  JCHECK_STATUS(vol_draw_kernel.setArg(1, dv_volume));
  JCHECK_STATUS(vol_draw_kernel.setArg(2, dv_rendered_image));

  view->add_menu_hotkey("debug_mode", 0, 'P', 5);
  view->add_menu_hotkey("terminal_iteration", 6, 'I', 24);
  view->add_menu_hotkey("test_feature", 0, 'F', 2);

  float t = 0.0;
  RenderConfig cc_cfg;

  view->add_toggle_callback("terminal_iteration", [&](int iter_ct) {
    std::cout << "\tIteration Count --> " << iter_ct * 10 << std::endl;
    cc_cfg.terminal_iteration = iter_ct * 10;

    draw(*view, *ui2d, cmd_queue, vol_draw_kernel, dv_rendered_image, cc_cfg, out_im_size,
         t);
  });

  view->add_toggle_callback("test_feature", [&](int feature) {
    std::cout << "\tTest Feature --> " << feature << std::endl;
    cc_cfg.test_feature = feature;
    draw(*view, *ui2d, cmd_queue, vol_draw_kernel, dv_rendered_image, cc_cfg, out_im_size,
         t);
  });

  view->add_toggle_callback("debug_mode", [&](int dbg_mode) {
    if (dbg_mode == 1) {
      std::cout << "\tSetting Debug Mode: " << std::endl;
    } else if (dbg_mode == 2) {
      std::cout << "\tSetting Debug Mode:  " << std::endl;
    } else if (dbg_mode == 3) {
      std::cout << "\tSetting Debug Mode:  " << std::endl;
    } else if (dbg_mode == 4) {
      std::cout << "\tSetting Debug Mode: " << std::endl;
    } else {
      std::cout << "\tDisabling Debug " << std::endl;
    }
    cc_cfg.debug_mode = dbg_mode;

    draw(*view, *ui2d, cmd_queue, vol_draw_kernel, dv_rendered_image, cc_cfg, out_im_size,
         t);
  });

  view->run_menu_callbacks();
  view->trigger_continue();

  while (true) {
    t += 0.1;
    geo->add_sphere({jcc::Vec3(0.0, 0.0, 3.0), 0.5});
    geo->add_sphere({jcc::Vec3(0.2, 0.0, 3.5), 1.0 + std::cos(0.05 * t)});
    geo->add_sphere({jcc::Vec3(0.0, 0.0, 0.0), 0.1});
    geo->flip();

    draw(*view, *ui2d, cmd_queue, vol_draw_kernel, dv_rendered_image, cc_cfg, out_im_size,
         t);
    view->spin_until_step();
  }
}
