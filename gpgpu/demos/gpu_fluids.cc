#include "gpgpu/demos/ray_lut.hh"
#include "gpgpu/import/load_kernel.hh"
#include "gpgpu/interplatform/cl_liegroups.hh"
#include "gpgpu/wrappers/create_context.hh"
#include "gpgpu/wrappers/errors.hh"
#include "gpgpu/wrappers/image_read_write.hh"
#include "gpgpu/wrappers/kernel_runner.hh"
#include "gpgpu/wrappers/volume_read_write.hh"

#include "gpgpu/kernels/fluid_defs.hh"

#include "viewer/interaction/ui2d.hh"
#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

#include "util/time_point.hh"

#include "util/environment.hh"
#include "util/timing.hh"

#include "eigen.hh"
#include "out.hh"
#include "util/waves.hh"

#include "gpgpu/opencl.hh"

#include <iostream>
#include <vector>

struct PhysicsContext {
  // RGBA
  cl::Image3D dv_u0;

  cl::Image3D dv_u_intermediate;

  cl::Image3D dv_u1;
  cl::Image3D dv_utemp;

  // RGBA
  cl::Image3D dv_pressure;
  cl::Image3D dv_ptemp;

  // RGBA
  cl::Image3D dv_ink;
  cl::Image3D dv_ink_tmp;
  cl::Image3D dv_ink_swap1;
  cl::Image3D dv_ink_swap2;

  jcc::VolumeSize vol_size;
};

struct RenderContext {
  PhysicsContext physics;

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
    rctx.physics.dv_u0 =
        cl::Image3D(cl_info.context, CL_MEM_READ_WRITE, vol_image_fmt, vol_size.cols,
                    vol_size.rows, vol_size.slices, 0, 0, nullptr, &status);
    jcc::fill_volume_zeros(cmd_queue, rctx.physics.dv_u0, vol_size);
  }
  JCHECK_STATUS(status);

  {
    rctx.physics.dv_u1 =
        cl::Image3D(cl_info.context, CL_MEM_READ_WRITE, vol_image_fmt, vol_size.cols,
                    vol_size.rows, vol_size.slices, 0, 0, nullptr, &status);
    jcc::fill_volume_zeros(cmd_queue, rctx.physics.dv_u1, vol_size);
  }
  JCHECK_STATUS(status);

  {
    rctx.physics.dv_utemp =
        cl::Image3D(cl_info.context, CL_MEM_READ_WRITE, vol_image_fmt, vol_size.cols,
                    vol_size.rows, vol_size.slices, 0, 0, nullptr, &status);
    jcc::fill_volume_zeros(cmd_queue, rctx.physics.dv_utemp, vol_size);
  }
  JCHECK_STATUS(status);

  {
    rctx.physics.dv_pressure =
        cl::Image3D(cl_info.context, CL_MEM_READ_WRITE, vol_image_fmt, vol_size.cols,
                    vol_size.rows, vol_size.slices, 0, 0, nullptr, &status);
    jcc::fill_volume_zeros(cmd_queue, rctx.physics.dv_pressure, vol_size);
  }
  JCHECK_STATUS(status);

  {
    rctx.physics.dv_ptemp =
        cl::Image3D(cl_info.context, CL_MEM_READ_WRITE, vol_image_fmt, vol_size.cols,
                    vol_size.rows, vol_size.slices, 0, 0, nullptr, &status);
    jcc::fill_volume_zeros(cmd_queue, rctx.physics.dv_ptemp, vol_size);
  }
  JCHECK_STATUS(status);

  {
    rctx.physics.dv_u_intermediate =
        cl::Image3D(cl_info.context, CL_MEM_READ_WRITE, vol_image_fmt, vol_size.cols,
                    vol_size.rows, vol_size.slices, 0, 0, nullptr, &status);
    jcc::fill_volume_zeros(cmd_queue, rctx.physics.dv_u_intermediate, vol_size);
  }
  JCHECK_STATUS(status);

  {
    rctx.physics.dv_ink =
        cl::Image3D(cl_info.context, CL_MEM_READ_WRITE, vol_image_fmt, vol_size.cols,
                    vol_size.rows, vol_size.slices, 0, 0, nullptr, &status);
    jcc::fill_volume_zeros(cmd_queue, rctx.physics.dv_ink, vol_size);
  }
  {
    rctx.physics.dv_ink_tmp =
        cl::Image3D(cl_info.context, CL_MEM_READ_WRITE, vol_image_fmt, vol_size.cols,
                    vol_size.rows, vol_size.slices, 0, 0, nullptr, &status);
    jcc::fill_volume_zeros(cmd_queue, rctx.physics.dv_ink_tmp, vol_size);
  }

  {
    rctx.physics.dv_ink_swap1 =
        cl::Image3D(cl_info.context, CL_MEM_READ_WRITE, vol_image_fmt, vol_size.cols,
                    vol_size.rows, vol_size.slices, 0, 0, nullptr, &status);
    jcc::fill_volume_zeros(cmd_queue, rctx.physics.dv_ink_swap1, vol_size);
  }

  {
    rctx.physics.dv_ink_swap2 =
        cl::Image3D(cl_info.context, CL_MEM_READ_WRITE, vol_image_fmt, vol_size.cols,
                    vol_size.rows, vol_size.slices, 0, 0, nullptr, &status);
    jcc::fill_volume_zeros(cmd_queue, rctx.physics.dv_ink_swap2, vol_size);
  }

  JCHECK_STATUS(status);

  rctx.physics.vol_size = vol_size;

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
                   const FluidSimConfig& cc_cfg,
                   const double scaling = 10.0,
                   const double t = 0.5) {
  // const jcc::PrintingScopedTimer tt("  Render Time");
  auto vol_draw_kernel = rctx.render_kernels["render_volume"];

  JCHECK_STATUS(vol_draw_kernel.setArg(0, rctx.dv_ray_lut));
  JCHECK_STATUS(vol_draw_kernel.setArg(1, dv_volume));
  JCHECK_STATUS(vol_draw_kernel.setArg(2, rctx.dv_rendered_image));

  const jcc::clSE3 cl_world_from_camera(world_from_camera);
  JCHECK_STATUS(vol_draw_kernel.setArg(3, cl_world_from_camera));

  cl_float cl_scaling = static_cast<cl_float>(scaling);
  JCHECK_STATUS(vol_draw_kernel.setArg(4, cl_scaling));

  cl_float cl_t = static_cast<cl_float>(-0.5f);
  JCHECK_STATUS(vol_draw_kernel.setArg(5, cl_t));

  const cl::NDRange work_group_size{static_cast<std::size_t>(rctx.im_size.cols),
                                    static_cast<std::size_t>(rctx.im_size.rows)};

  const cl::NDRange local_size({16u, 16u});
  JCHECK_STATUS(
      cmd_queue.enqueueNDRangeKernel(vol_draw_kernel, {0}, work_group_size, local_size));

  JCHECK_STATUS(cmd_queue.finish());

  cv::Mat out_img(cv::Size(rctx.im_size.cols, rctx.im_size.rows), CV_32FC4,
                  cv::Scalar(0.0, 0.0, 0.0, 0.0));
  jcc::read_image_from_device(cmd_queue, rctx.dv_rendered_image, out(out_img));

  out_img.convertTo(out_img, CV_8UC4);
  cv::cvtColor(out_img, out_img, CV_BGRA2BGR);

  ui2d.add_image(out_img, 1.0);
  ui2d.flip();
}

void draw(viewer::Ui2d& ui2d,
          jcc::KernelRunner& kernel_runner,
          RenderContext& rctx,
          const SE3 world_from_camera,
          const FluidSimConfig& cc_cfg,
          float t,
          bool paused) {
  const jcc::PrintingScopedTimer t0("Frame Time");

  //
  // Advect
  //

  const auto cl_cfg = cc_cfg.convert();

  const jcc::Vec3i ink_origin(50, 50, 50);
  const jcc::Vec4 ink_color(1.5, 1.0 + std::cos(t * 0.3), 0.2, 0.0);
  const jcc::VolumeSize ink_size{.cols = 7, .rows = 7, .slices = 7};
  jcc::fill_volume_section(kernel_runner.queue(), rctx.physics.dv_ink, ink_origin,
                           ink_size, ink_color);
  //
  // Advect
  //
  constexpr bool ADVECT = true;
  constexpr bool DIFFUSE = true;
  constexpr bool PRESSURE = true;
  constexpr bool CHIRON = true;
  constexpr double SCALING_VELOCITY_VIEW = 50.0;

  if (ADVECT) {
    // const jcc::PrintingScopedTimer tt("  Advection Time");
    auto apply_velocity_bdry_cond_kernel =
        rctx.fluid_kernels.at("apply_velocity_bdry_cond");
    apply_velocity_bdry_cond_kernel.setArg(0, rctx.physics.dv_u0);
    apply_velocity_bdry_cond_kernel.setArg(1, rctx.physics.dv_u1);
    kernel_runner.run(apply_velocity_bdry_cond_kernel, true);
    std::swap(rctx.physics.dv_u_intermediate, rctx.physics.dv_u1);

    auto advect_kernel = rctx.fluid_kernels.at("advect_velocity");
    advect_kernel.setArg(0, rctx.physics.dv_u_intermediate);
    advect_kernel.setArg(1, rctx.physics.dv_u1);
    advect_kernel.setArg(2, cl_cfg);
    kernel_runner.run(advect_kernel, true);

    apply_velocity_bdry_cond_kernel.setArg(0, rctx.physics.dv_u1);
    apply_velocity_bdry_cond_kernel.setArg(1, rctx.physics.dv_u_intermediate);
    kernel_runner.run(apply_velocity_bdry_cond_kernel, true);
  }

  {
    auto advect_vector = rctx.fluid_kernels.at("advect_vector");
    advect_vector.setArg(0, rctx.physics.dv_u_intermediate);
    advect_vector.setArg(1, rctx.physics.dv_ink);
    advect_vector.setArg(2, rctx.physics.dv_ink_tmp);
    advect_vector.setArg(3, cl_cfg);
    kernel_runner.run(advect_vector);

    if (cc_cfg.debug_mode == 3) {
      render_volume(ui2d, kernel_runner.queue(), rctx, world_from_camera,
                    rctx.physics.dv_ink_tmp, cc_cfg, SCALING_VELOCITY_VIEW * 5.0, t);
    }
  }

  constexpr int N_JACOBI_ITERS = 70;

  //
  // Diffuse ink
  //
  if (DIFFUSE) {
    // const jcc::PrintingScopedTimer tt("  Diffusion Time");
    auto diffuse_kernel = rctx.fluid_kernels.at("diffuse");
    for (int j = 0; j < 20; ++j) {
      // "Give me u1 s.t. laplacian(u1) == u0"
      diffuse_kernel.setArg(0, rctx.physics.dv_ink_tmp);
      diffuse_kernel.setArg(1, rctx.physics.dv_ink_swap1);
      diffuse_kernel.setArg(2, rctx.physics.dv_ink_swap2);
      diffuse_kernel.setArg(3, cl_cfg);

      kernel_runner.run(diffuse_kernel, false);

      diffuse_kernel.setArg(0, rctx.physics.dv_ink_tmp);
      diffuse_kernel.setArg(1, rctx.physics.dv_ink_swap2);
      diffuse_kernel.setArg(2, rctx.physics.dv_ink_swap1);
      diffuse_kernel.setArg(3, cl_cfg);

      kernel_runner.run(diffuse_kernel, false);
    }

    kernel_runner.queue().finish();
    std::swap(rctx.physics.dv_ink_tmp, rctx.physics.dv_ink_swap1);
  }

  //
  // Diffuse
  //
  if (DIFFUSE) {
    // const jcc::PrintingScopedTimer tt("  Diffusion Time");
    auto diffuse_kernel = rctx.fluid_kernels.at("diffuse");
    for (int j = 0; j < N_JACOBI_ITERS; ++j) {
      // "Give me u1 s.t. laplacian(u1) == u0"
      diffuse_kernel.setArg(0, rctx.physics.dv_u_intermediate);
      diffuse_kernel.setArg(1, rctx.physics.dv_u1);
      diffuse_kernel.setArg(2, rctx.physics.dv_utemp);
      diffuse_kernel.setArg(3, cl_cfg);

      kernel_runner.run(diffuse_kernel, false);

      diffuse_kernel.setArg(0, rctx.physics.dv_u_intermediate);
      diffuse_kernel.setArg(1, rctx.physics.dv_utemp);
      diffuse_kernel.setArg(2, rctx.physics.dv_u1);
      diffuse_kernel.setArg(3, cl_cfg);

      kernel_runner.run(diffuse_kernel, false);

      // std::swap(rctx.physics.dv_u1, rctx.physics.dv_utemp);
    }

    kernel_runner.queue().finish();
    std::swap(rctx.physics.dv_u_intermediate, rctx.physics.dv_u1);
  }

  //
  // Compute divergence of `u`
  //

  {
    // const jcc::PrintingScopedTimer tt("  Divergence Time");
    auto divergence_kernel = rctx.fluid_kernels.at("compute_divergence");
    // This is currently `w`
    divergence_kernel.setArg(0, rctx.physics.dv_u_intermediate);

    // This is now `div w`
    divergence_kernel.setArg(1, rctx.physics.dv_utemp);
    divergence_kernel.setArg(2, cl_cfg);

    kernel_runner.run(divergence_kernel, true);
  }

  if (cc_cfg.debug_mode == 2) {
    render_volume(ui2d, kernel_runner.queue(), rctx, world_from_camera,
                  rctx.physics.dv_utemp, cc_cfg, t);
  }

  //
  // Compute pressure
  //

  if (PRESSURE) {
    // const jcc::PrintingScopedTimer tt("  Pressure Time");
    auto pressure_kernel = rctx.fluid_kernels.at("compute_pressure");
    auto apply_pressure_bdry_cond = rctx.fluid_kernels.at("apply_pressure_bdry_cond");
    // div w
    pressure_kernel.setArg(0, rctx.physics.dv_utemp);
    // p ("best guess" right now)
    pressure_kernel.setArg(1, rctx.physics.dv_pressure);
    // Output p -- at convergence these are the same
    pressure_kernel.setArg(2, rctx.physics.dv_ptemp);
    pressure_kernel.setArg(3, cl_cfg);

    apply_pressure_bdry_cond.setArg(0, rctx.physics.dv_ptemp);
    apply_pressure_bdry_cond.setArg(1, rctx.physics.dv_pressure);

    for (int j = 0; j < N_JACOBI_ITERS; ++j) {
      kernel_runner.run(pressure_kernel, false);
      kernel_runner.run(apply_pressure_bdry_cond, false);
    }

    kernel_runner.queue().finish();
  }
  if (cc_cfg.debug_mode == 1) {
    render_volume(ui2d, kernel_runner.queue(), rctx, world_from_camera,
                  rctx.physics.dv_pressure, cc_cfg, 50.0, t);
  }

  //
  // Apply Chiron Projection
  //

  if (CHIRON) {
    auto chiron_kernel = rctx.fluid_kernels.at("chiron_projection");
    // This is currently `w`
    chiron_kernel.setArg(0, rctx.physics.dv_u_intermediate);

    // We're subtracting the gradient of pressure
    chiron_kernel.setArg(1, rctx.physics.dv_pressure);
    chiron_kernel.setArg(2, rctx.physics.dv_u1);
    chiron_kernel.setArg(3, cl_cfg);
    kernel_runner.run(chiron_kernel, true);

    std::swap(rctx.physics.dv_u_intermediate, rctx.physics.dv_u1);
  }

  if (cc_cfg.debug_mode == 0) {
    render_volume(ui2d, kernel_runner.queue(), rctx, world_from_camera,
                  rctx.physics.dv_u_intermediate, cc_cfg, SCALING_VELOCITY_VIEW, t);
  }

  if (!paused) {
    std::swap(rctx.physics.dv_u0, rctx.physics.dv_u_intermediate);
    std::swap(rctx.physics.dv_ink, rctx.physics.dv_ink_tmp);
  }
}

auto setup_window() {
  const auto view = viewer::get_window3d("Render Visualization");
  view->set_view_preset(viewer::Window3D::ViewSetting::CAMERA);
  view->set_azimuth(0.0);
  view->set_elevation(0.7);

  const auto background = view->add_primitive<viewer::SimpleGeometry>();
  const geometry::shapes::Plane ground{jcc::Vec3(0.0, 1.0, 0.0).normalized(), 1.0};
  background->add_plane({ground});
  background->flip();

  return view;
}

int main() {
  const auto cl_info = jcc::create_context();

  cl_int status;
  cl::CommandQueue cmd_queue(cl_info.context, CL_QUEUE_PROFILING_ENABLE, &status);
  JCHECK_STATUS(status);

  const jcc::ImageSize im_size = {2 * 480, 2 * 480};
  const std::size_t dimension = 96;
  const jcc::VolumeSize vol_size = {dimension + 2, dimension + 2, dimension + 2};

  const jcc::WorkGroupConfig wg_cfg{
      .work_group_size = {vol_size.cols - 2, vol_size.rows - 2, vol_size.slices - 2},
      .work_group_offset = {1u, 1u, 1u},
      .local_size = {16u, 16u, 1u}};
  jcc::KernelRunner kernel_runner(cmd_queue, wg_cfg);

  auto rctx = generate_render_context(cl_info, cmd_queue, vol_size, im_size);

  const auto view = setup_window();
  const auto ui2d = view->add_primitive<viewer::Ui2d>();
  const auto geo = view->add_primitive<viewer::SimpleGeometry>();

  view->add_menu_hotkey("paused", 1, 'P', 2);

  view->add_menu_hotkey("debug_mode", 3, 'V', 4);
  view->add_menu_hotkey("max_iteration", 0, 'I', 50);
  view->add_menu_hotkey("render_mode", 0, 'F', 2);

  float t = 0.0;
  bool paused = view->get_menu("paused") == 0;

  FluidSimConfig cc_cfg;
  cc_cfg.debug_mode = 3;
  cc_cfg.nu = 0.001;
  cc_cfg.dt_sec = 0.05;
  cc_cfg.dx_m = 0.1;

  const auto basic_draw = [&]() {
    const SE3 world_from_camera = view->standard_camera_from_world().inverse();
    draw(*ui2d, kernel_runner, rctx, world_from_camera, cc_cfg, t, paused);
  };

  view->add_toggle_callback("paused", [&](int paused_state) {
    if (paused_state == 1) {
      std::cout << "Pausing" << std::endl;
      paused = true;
    } else {
      std::cout << "Unpausing" << std::endl;
      paused = false;
    }
  });

  view->add_toggle_callback("max_iteration", [&](int iter_ct) {
    std::cout << "\tIteration Count --> " << iter_ct * 10 << std::endl;
    cc_cfg.max_iteration = iter_ct * 10;
  });

  view->add_toggle_callback("render_mode", [&](int feature) {
    std::cout << "\tTest Feature --> " << feature << std::endl;
    cc_cfg.render_mode = feature;
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
  });

  view->run_menu_callbacks();
  view->trigger_continue();

  // while (true) {
  for (; t < 100.0;) {
    t += 0.1;
    basic_draw();
    view->spin_until_step();
  }
  kernel_runner.print_average_execution_times();
}
