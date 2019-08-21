#include "gpgpu/demos/ray_lut.hh"
#include "gpgpu/import/load_kernel.hh"
#include "gpgpu/interplatform/cl_liegroups.hh"
#include "gpgpu/wrappers/create_context.hh"
#include "gpgpu/wrappers/errors.hh"
#include "gpgpu/wrappers/image_read_write.hh"
#include "gpgpu/wrappers/volume_read_write.hh"

#include "gpgpu/kernels/fluid_defs.hh"

#include "viewer/interaction/ui2d.hh"
#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

#include "util/environment.hh"
#include "util/timing.hh"

#include "eigen.hh"
#include "out.hh"
#include "util/waves.hh"

#include <CL/cl.hpp>

#include <iostream>
#include <vector>

struct PhysicsContext {
  // RGBA
  cl::Image3D dv_u0;

  // RGBA
  cl::Image3D dv_u1;

  cl::Image3D dv_utemp;

  // RGBA
  cl::Image3D dv_pressure;

  // RGBA
  cl::Image3D dv_ptemp;

  jcc::VolumeSize vol_size;
};

struct RenderContext {
  PhysicsContext physics_ctx;

  cl::Image2D dv_rendered_image;
  cl::Image2D dv_ray_lut;
  jcc::ImageSize im_size;

  std::map<std::string, cl::Kernel> fluid_kernels;
  std::map<std::string, cl::Kernel> render_kernels;
};

//
// Send the ray LUT
//
cl::Image2D create_and_send_ray_lut(const jcc::ClInfo& cl_info,
                                    const cl::CommandQueue cmd_queue,
                                    const jcc::ImageSize& im_size) {
  cl_int status;
  const auto model = jcc::nice_model(im_size);
  const cl::ImageFormat ray_lut_fmt(CL_RGBA, CL_FLOAT);
  cl::Image2D dv_ray_lut(cl_info.context, CL_MEM_READ_ONLY, ray_lut_fmt, im_size.cols,
                         im_size.rows, 0, nullptr, &status);

  JCHECK_STATUS(status);

  const cv::Mat ray_lut = jcc::create_ray_lut(model, im_size.cols, im_size.rows).clone();
  jcc::send_image_to_device(cmd_queue, dv_ray_lut, ray_lut);
  return dv_ray_lut;
}

RenderContext generate_render_context(const jcc::ClInfo& cl_info,
                                      const cl::CommandQueue cmd_queue,
                                      const jcc::VolumeSize& vol_size,
                                      const jcc::ImageSize& im_size) {
  cl_int status;
  RenderContext rctx;

  const cl::ImageFormat vol_image_fmt(CL_RGBA, CL_FLOAT);
  {
    rctx.physics_ctx.dv_u0 =
        cl::Image3D(cl_info.context, CL_MEM_READ_WRITE, vol_image_fmt, vol_size.cols,
                    vol_size.rows, vol_size.slices, 0, 0, nullptr, &status);
  }
  JCHECK_STATUS(status);

  {
    rctx.physics_ctx.dv_u1 =
        cl::Image3D(cl_info.context, CL_MEM_READ_WRITE, vol_image_fmt, vol_size.cols,
                    vol_size.rows, vol_size.slices, 0, 0, nullptr, &status);
  }
  JCHECK_STATUS(status);

  {
    rctx.physics_ctx.dv_utemp =
        cl::Image3D(cl_info.context, CL_MEM_READ_WRITE, vol_image_fmt, vol_size.cols,
                    vol_size.rows, vol_size.slices, 0, 0, nullptr, &status);
  }
  JCHECK_STATUS(status);

  {
    rctx.physics_ctx.dv_pressure =
        cl::Image3D(cl_info.context, CL_MEM_READ_WRITE, vol_image_fmt, vol_size.cols,
                    vol_size.rows, vol_size.slices, 0, 0, nullptr, &status);
  }
  JCHECK_STATUS(status);

  {
    rctx.physics_ctx.dv_ptemp =
        cl::Image3D(cl_info.context, CL_MEM_READ_WRITE, vol_image_fmt, vol_size.cols,
                    vol_size.rows, vol_size.slices, 0, 0, nullptr, &status);
  }
  JCHECK_STATUS(status);

  rctx.physics_ctx.vol_size = vol_size;

  jcc::fill_volume_zeros(cmd_queue, rctx.physics_ctx.dv_u0, vol_size);
  jcc::fill_volume_zeros(cmd_queue, rctx.physics_ctx.dv_u1, vol_size);

  jcc::fill_volume_zeros(cmd_queue, rctx.physics_ctx.dv_pressure, vol_size);
  jcc::fill_volume_zeros(cmd_queue, rctx.physics_ctx.dv_ptemp, vol_size);

  /*  const jcc::Vec3i block_start(0, 0, 0);
    const jcc::Vec4 pressure(5.0, 0.0, 0.0, 0.0);

    jcc::fill_volume_section(cmd_queue, rctx.physics_ctx.dv_pressure, block_start,
    vol_size, pressure); jcc::fill_volume_section(cmd_queue, rctx.physics_ctx.dv_ptemp,
    block_start, vol_size, pressure);*/

  rctx.fluid_kernels =
      read_kernels(cl_info, jcc::Environment::repo_path() + "gpgpu/kernels/sim_fluid.cl");

  rctx.render_kernels = read_kernels(
      cl_info, jcc::Environment::repo_path() + "gpgpu/kernels/render_volume.cl");

  //
  // Prepare a rendered image
  //

  const cl::ImageFormat output_image_fmt(CL_RGBA, CL_FLOAT);
  rctx.dv_rendered_image =
      cl::Image2D(cl_info.context, CL_MEM_WRITE_ONLY, output_image_fmt, im_size.cols,
                  im_size.rows, 0, nullptr, &status);
  JCHECK_STATUS(status);

  rctx.im_size = im_size;

  //
  // Prepare a ray LUT
  //

  rctx.dv_ray_lut = create_and_send_ray_lut(cl_info, cmd_queue, im_size);
  return rctx;
}

void render_volume(viewer::Ui2d& ui2d,
                   cl::CommandQueue& cmd_queue,
                   RenderContext& rctx,
                   const SE3& world_from_camera,
                   const cl::Image3D& dv_volume,
                   const FluidSimConfig& cc_cfg) {
  auto vol_draw_kernel = rctx.render_kernels["render_volume"];

  JCHECK_STATUS(vol_draw_kernel.setArg(0, rctx.dv_ray_lut));
  JCHECK_STATUS(vol_draw_kernel.setArg(1, dv_volume));
  JCHECK_STATUS(vol_draw_kernel.setArg(2, rctx.dv_rendered_image));

  const jcc::clSE3 cl_world_from_camera(world_from_camera);
  JCHECK_STATUS(vol_draw_kernel.setArg(3, cl_world_from_camera));

  cl_float cl_scaling = 0.01f;
  JCHECK_STATUS(vol_draw_kernel.setArg(4, cl_scaling));

  const cl::NDRange work_group_size{static_cast<std::size_t>(rctx.im_size.cols),
                                    static_cast<std::size_t>(rctx.im_size.rows)};

  const cl::NDRange local_size({32u, 32u});
  JCHECK_STATUS(
      cmd_queue.enqueueNDRangeKernel(vol_draw_kernel, {0}, work_group_size, local_size));

  cmd_queue.flush();

  cv::Mat out_img(cv::Size(rctx.im_size.cols, rctx.im_size.rows), CV_32FC4,
                  cv::Scalar(0.0, 0.0, 0.0, 0.0));
  jcc::read_image_from_device(cmd_queue, rctx.dv_rendered_image, out(out_img));

  out_img.convertTo(out_img, CV_8UC4);
  cv::cvtColor(out_img, out_img, CV_BGRA2BGR);

  ui2d.add_image(out_img, 1.0);
  ui2d.flip();
}

void draw(viewer::Ui2d& ui2d,
          cl::CommandQueue& cmd_queue,
          RenderContext& rctx,
          const SE3 world_from_camera,
          const FluidSimConfig& cc_cfg,
          float t) {
  // const jcc::PrintingScopedTimer tt("Frame Time");

  //
  // Advect
  //

  static bool once = true;
  if (once) {
    once = false;
    const jcc::Vec4 vel0(0.9, 0.09, 0.9, 0.0);
    const jcc::Vec4 vel1(0.0, 0.0, 0.0, 0.0);

    const jcc::VolumeSize block_size{15u, 15u, 15u};
    const jcc::Vec3i block_start1(50, 50, 50);
    const jcc::Vec3i block_start2(50, 66, 50);

    jcc::fill_volume_section(cmd_queue, rctx.physics_ctx.dv_u0, jcc::Vec3i::Zero(),
                             rctx.physics_ctx.vol_size, jcc::Vec4(0.0, 0.0, 0.0, 0.0));

    // jcc::fill_volume_section(cmd_queue, rctx.physics_ctx.dv_u0, block_start1,
    // block_size,
    //                          vel0);

    // jcc::fill_volume_section(cmd_queue, rctx.physics_ctx.dv_u0, block_start2,
    // block_size,
    //                          vel1);
  }
  const auto cl_cfg = cc_cfg.convert();

  const auto vol_size = rctx.physics_ctx.vol_size;
  const cl::NDRange wg_offset{1u, 1u, 1u};
  const cl::NDRange work_group_size{vol_size.cols - 2, vol_size.rows - 2,
                                    vol_size.slices - 2};
  // const cl::NDRange local_size({20u, 20u, 1u});
  const cl::NDRange local_size{};

  //
  // Advect
  //

  constexpr bool ADVECT = true;
  constexpr bool DIFFUSE = true;
  if (ADVECT) {
    auto apply_velocity_bdry_cond_kernel =
        rctx.fluid_kernels.at("apply_velocity_bdry_cond");
    apply_velocity_bdry_cond_kernel.setArg(0, rctx.physics_ctx.dv_u0);
    apply_velocity_bdry_cond_kernel.setArg(1, rctx.physics_ctx.dv_u1);
    JCHECK_STATUS(cmd_queue.enqueueNDRangeKernel(apply_velocity_bdry_cond_kernel,
                                                 wg_offset, work_group_size, local_size));
    cmd_queue.flush();

    if (cc_cfg.debug_mode == 3) {
      render_volume(ui2d, cmd_queue, rctx, world_from_camera, rctx.physics_ctx.dv_u1,
                    cc_cfg);
    }

    std::swap(rctx.physics_ctx.dv_u0, rctx.physics_ctx.dv_u1);

    auto advect_kernel = rctx.fluid_kernels.at("advect_velocity");
    advect_kernel.setArg(0, rctx.physics_ctx.dv_u0);
    advect_kernel.setArg(1, rctx.physics_ctx.dv_u1);

    advect_kernel.setArg(2, cl_cfg);
    JCHECK_STATUS(cmd_queue.enqueueNDRangeKernel(advect_kernel, wg_offset,
                                                 work_group_size, local_size));

    cmd_queue.flush();
    std::swap(rctx.physics_ctx.dv_u0, rctx.physics_ctx.dv_u1);
  }
  //
  // Diffuse
  //
  constexpr int N_JACOBI_ITERS = 100;
  if (DIFFUSE) {
    auto diffuse_kernel = rctx.fluid_kernels.at("diffuse");
    for (int j = 0; j < N_JACOBI_ITERS; ++j) {
      diffuse_kernel.setArg(0, rctx.physics_ctx.dv_u0);
      diffuse_kernel.setArg(1, rctx.physics_ctx.dv_u1);
      diffuse_kernel.setArg(2, rctx.physics_ctx.dv_utemp);
      diffuse_kernel.setArg(3, cl_cfg);

      JCHECK_STATUS(cmd_queue.enqueueNDRangeKernel(diffuse_kernel, wg_offset,
                                                   work_group_size, local_size));
      cmd_queue.flush();
      std::swap(rctx.physics_ctx.dv_u1, rctx.physics_ctx.dv_utemp);
    }

    std::swap(rctx.physics_ctx.dv_u0, rctx.physics_ctx.dv_u1);
  }
  //
  // Compute divergence of `u`
  //

  {
    auto divergence_kernel = rctx.fluid_kernels.at("compute_divergence");
    // This is currently `w`
    divergence_kernel.setArg(0, rctx.physics_ctx.dv_u0);

    // This is now `div w`
    divergence_kernel.setArg(1, rctx.physics_ctx.dv_utemp);
    divergence_kernel.setArg(2, cl_cfg);

    JCHECK_STATUS(cmd_queue.enqueueNDRangeKernel(divergence_kernel, wg_offset,
                                                 work_group_size, local_size));
  }
  cmd_queue.flush();
  // if (cc_cfg.debug_mode == 2) {
  //   render_volume(ui2d, cmd_queue, rctx, world_from_camera, rctx.physics_ctx.dv_utemp,
  //                 cc_cfg);
  // }

  //
  // Compute pressure
  //

  constexpr bool PRESSURE = true;
  constexpr bool CHIRON = true;

  if (PRESSURE) {
    auto pressure_kernel = rctx.fluid_kernels.at("compute_pressure");

    for (int j = 0; j < N_JACOBI_ITERS; ++j) {
      auto apply_pressure_bdry_cond = rctx.fluid_kernels.at("apply_pressure_bdry_cond");

      // std::swap(rctx.physics_ctx.dv_u0, rctx.physics_ctx.dv_u1);

      pressure_kernel.setArg(0, rctx.physics_ctx.dv_utemp);
      pressure_kernel.setArg(1, rctx.physics_ctx.dv_pressure);
      pressure_kernel.setArg(2, rctx.physics_ctx.dv_ptemp);
      pressure_kernel.setArg(3, cl_cfg);

      JCHECK_STATUS(cmd_queue.enqueueNDRangeKernel(pressure_kernel, wg_offset,
                                                   work_group_size, local_size));
      cmd_queue.flush();
      // if ((j == ((cc_cfg.max_iteration - 1) / 10)) && cc_cfg.debug_mode == 1) {
      //   render_volume(ui2d, cmd_queue, rctx, world_from_camera,
      //                 rctx.physics_ctx.dv_pressure, cc_cfg);
      // }
      // std::swap(rctx.physics_ctx.dv_pressure, rctx.physics_ctx.dv_ptemp);

      apply_pressure_bdry_cond.setArg(0, rctx.physics_ctx.dv_ptemp);
      apply_pressure_bdry_cond.setArg(1, rctx.physics_ctx.dv_pressure);
      JCHECK_STATUS(cmd_queue.enqueueNDRangeKernel(apply_pressure_bdry_cond, wg_offset,
                                                   work_group_size, local_size));
      cmd_queue.flush();
    }

    if (cc_cfg.debug_mode == 1) {
      render_volume(ui2d, cmd_queue, rctx, world_from_camera,
                    rctx.physics_ctx.dv_pressure, cc_cfg);
    }

    // std::swap(rctx.physics_ctx.dv_u0, rctx.physics_ctx.dv_u1);
  }

  //
  // Apply Chiron Projection
  //

  if (CHIRON) {
    auto chiron_kernel = rctx.fluid_kernels.at("chiron_projection");
    // This is now `div w`
    chiron_kernel.setArg(0, rctx.physics_ctx.dv_u0);

    chiron_kernel.setArg(1, rctx.physics_ctx.dv_pressure);
    chiron_kernel.setArg(2, rctx.physics_ctx.dv_u1);
    chiron_kernel.setArg(3, cl_cfg);

    JCHECK_STATUS(cmd_queue.enqueueNDRangeKernel(chiron_kernel, wg_offset,
                                                 work_group_size, local_size));
    cmd_queue.flush();

    std::swap(rctx.physics_ctx.dv_u0, rctx.physics_ctx.dv_u1);
  }

  if (cc_cfg.debug_mode == 0) {
    render_volume(ui2d, cmd_queue, rctx, world_from_camera, rctx.physics_ctx.dv_u0,
                  cc_cfg);
  }

  {
    auto divergence_kernel = rctx.fluid_kernels.at("compute_divergence");
    divergence_kernel.setArg(0, rctx.physics_ctx.dv_u0);

    // This is now `div w`
    divergence_kernel.setArg(1, rctx.physics_ctx.dv_utemp);
    divergence_kernel.setArg(2, cl_cfg);

    JCHECK_STATUS(cmd_queue.enqueueNDRangeKernel(divergence_kernel, wg_offset,
                                                 work_group_size, local_size));
  }
  cmd_queue.flush();
  if (cc_cfg.debug_mode == 2) {
    render_volume(ui2d, cmd_queue, rctx, world_from_camera, rctx.physics_ctx.dv_utemp,
                  cc_cfg);
  }
}

auto setup_window() {
  const auto view = viewer::get_window3d("Render Visualization");
  view->set_view_preset(viewer::Window3D::ViewSetting::CAMERA);
  view->set_azimuth(0.0);
  view->set_elevation(0.0);

  const auto background = view->add_primitive<viewer::SimpleGeometry>();
  const geometry::shapes::Plane ground{jcc::Vec3(0.0, 1.0, 0.0).normalized(), 1.0};
  background->add_plane({ground});
  background->flip();

  return view;
}

int main() {
  const auto cl_info = jcc::create_context();

  cl::CommandQueue cmd_queue(cl_info.context);

  const jcc::ImageSize im_size = {2 * 480, 2 * 480};
  const jcc::VolumeSize vol_size = {100u, 100u, 100u};

  auto rctx = generate_render_context(cl_info, cmd_queue, vol_size, im_size);

  const auto view = setup_window();
  const auto ui2d = view->add_primitive<viewer::Ui2d>();
  const auto geo = view->add_primitive<viewer::SimpleGeometry>();

  view->add_menu_hotkey("debug_mode", 1, 'P', 4);
  view->add_menu_hotkey("max_iteration", 0, 'I', 12);
  view->add_menu_hotkey("test_feature", 0, 'F', 2);

  float t = 0.0;

  FluidSimConfig cc_cfg;
  // cc_cfg.debug_mode = 0;
  cc_cfg.nu = 0.1;
  cc_cfg.dt_sec = 0.01;
  cc_cfg.dx_m = 0.1;

  const auto basic_draw = [&]() {
    const SE3 world_from_camera = view->standard_camera_from_world().inverse();
    draw(*ui2d, cmd_queue, rctx, world_from_camera, cc_cfg, t);
  };

  view->add_toggle_callback("max_iteration", [&](int iter_ct) {
    std::cout << "\tIteration Count --> " << iter_ct * 10 << std::endl;
    cc_cfg.max_iteration = iter_ct * 10;

    // basic_draw();
  });

  view->add_toggle_callback("test_feature", [&](int feature) {
    std::cout << "\tTest Feature --> " << feature << std::endl;
    cc_cfg.test_feature = feature;
    // basic_draw();
  });

  view->add_toggle_callback("debug_mode", [&](int dbg_mode) {
    if (dbg_mode == 1) {
      std::cout << "\tSetting Debug Mode: Pressure" << std::endl;
    } else if (dbg_mode == 2) {
      std::cout << "\tSetting Debug Mode: Divergence" << std::endl;
    } else if (dbg_mode == 3) {
      std::cout << "\tSetting Debug Mode: Velocity Boundary Conditions" << std::endl;
    } else if (dbg_mode == 4) {
      std::cout << "\tSetting Debug Mode: " << std::endl;
    } else {
      std::cout << "\tDisabling Debug " << std::endl;
    }
    cc_cfg.debug_mode = dbg_mode;

    // basic_draw();
  });

  view->run_menu_callbacks();
  view->trigger_continue();

  while (true) {
    t += 0.1;
    basic_draw();
    view->spin_until_step();
  }
}
