// %deps(${OpenCL_LIBRARY})

#include "eigen.hh"
#include "gpgpu/import/load_kernel.hh"
#include "gpgpu/wrappers/create_context.hh"
#include "gpgpu/wrappers/errors.hh"

#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

#include <CL/cl.hpp>

#include <iostream>
#include <vector>

namespace {
struct __attribute__((packed)) Star {
  cl_float mass = 1.1f;
  cl_float p_x = 0.0f;
  cl_float p_y = 0.0f;
  cl_float p_z = 0.0f;
  cl_float p_dummy = 0.0f;

  cl_float v_x = 0.0f;
  cl_float v_y = 0.0f;
  cl_float v_z = 0.0f;
  cl_float v_dummy = 0.0f;

  cl_float f_x = 0.0f;
  cl_float f_y = 0.0f;
  cl_float f_z = 0.0f;
  cl_float f_dummy = 0.0f;
};

std::vector<Star> make_stars(int n_stars) {
  std::vector<Star> host_stars(n_stars);
  for (int k = 0; k < n_stars; ++k) {
    constexpr double DIST = 10.0;
    jcc::Vec3 ran = DIST * (jcc::Vec3::Random());
    while (ran.norm() > DIST) {
      ran = DIST * (jcc::Vec3::Random());
    }
    const jcc::Vec1 mass = jcc::Vec1::Random();

    host_stars[k].mass = 50.0 * (mass[0] + 1.0);

    host_stars[k].p_x = ran.x();
    host_stars[k].p_y = ran.y();
    host_stars[k].p_z = ran.z();

    // const jcc::Vec3 v = 0.02 * ran.cross(jcc::Vec3::UnitZ()) / (ran.norm());
    const jcc::Vec3 v = 0.02 * jcc::Vec3::Random();

    host_stars[k].v_x = v.x();
    host_stars[k].v_y = v.y();
    host_stars[k].v_z = v.z();
  }
  return host_stars;
}

void put_stars(viewer::SimpleGeometry& geo, const std::vector<Star>& stars) {
  viewer::Points points;
  std::vector<double> intensities;
  std::vector<double> sizes;
  for (std::size_t k = 0; k < stars.size(); ++k) {
    const jcc::Vec3 pt(stars[k].p_x, stars[k].p_y, stars[k].p_z);
    const jcc::Vec3 vel(stars[k].v_x, stars[k].v_y, stars[k].v_z);
    const jcc::Vec3 force(stars[k].f_x, stars[k].f_y, stars[k].f_z);

    points.points.push_back(pt);
    sizes.push_back(stars[k].mass);
    intensities.push_back(std::max(vel.norm(), 0.01));
  }

  geo.add_colored_points(points, intensities);
}
}  // namespace

void run() {
  const auto cl_info = jcc::create_context();

  const std::size_t n_stars = 10000;
  const std::size_t n_bytes = n_stars * sizeof(Star);

  cl::Buffer dv_in_masses(cl_info.context, CL_MEM_READ_ONLY, n_bytes);
  cl::Buffer dv_out_masses(cl_info.context, CL_MEM_WRITE_ONLY, n_bytes);

  const auto kernels =
      read_kernels(cl_info, "/home/jacob/repos/experiments/gpgpu/demos/gravity.cl");

  auto kernel = kernels.at("simulate_gravity");

  cl::CommandQueue cmd_queue(cl_info.context);

  kernel.setArg(0, dv_in_masses);
  kernel.setArg(1, dv_out_masses);
  kernel.setArg(2, static_cast<cl_int>(n_stars));

  const auto view = viewer::get_window3d("Gravity Visualization");
  const auto geo = view->add_primitive<viewer::SimpleGeometry>();

  auto host_stars = make_stars(n_stars);
  while (!view->should_close()) {
    constexpr bool BLOCK_WRITE = false;
    constexpr std::size_t offset = 0;

    cl_int status = 0;
    status = cmd_queue.enqueueWriteBuffer(dv_in_masses, BLOCK_WRITE, offset, n_bytes,
                                          host_stars.data());
    jcc::check_status(status);

    status = cmd_queue.enqueueNDRangeKernel(kernel, {0}, {n_stars}, {1000});
    jcc::check_status(status);
    constexpr bool BLOCK_READ = true;
    status = cmd_queue.enqueueReadBuffer(dv_out_masses, BLOCK_READ, offset, n_bytes,
                                         host_stars.data());
    jcc::check_status(status);

    put_stars(*geo, host_stars);
    geo->flip();
    view->spin_until_step();
  }
}

int main() {
  run();
}