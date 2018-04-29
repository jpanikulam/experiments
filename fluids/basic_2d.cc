#include "eigen.hh"

#include "navier_stokes_2d.hh"

// Rendering
#include "viewer/primitives/frame.hh"
#include "viewer/primitives/image.hh"
#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"
#include "viewer/window_manager.hh"

namespace jcc {
namespace fluids {

plane::SimulationState enforce_boundary_conditions(const plane::SimulationState&  state,
                                                   const plane::SimulationConfig& sim_config) {
  auto new_state = state;
  // new_state.pressure_field

  // Top/Bottom boundaries
  // new_state.velocity_field[0]. =
  return new_state;
}

std::array<Eigen::MatrixXd, 2> mul(const std::array<Eigen::MatrixXd, 2>& a, const double b) {
  std::array<Eigen::MatrixXd, 2> c;
  constexpr int                  N = 2;
  for (int k = 0; k < N; ++k) {
    c[k] = a[k] * b;
  }
  return c;
}

std::array<Eigen::MatrixXd, 2> add(const std::array<Eigen::MatrixXd, 2>& a, const std::array<Eigen::MatrixXd, 2>& b) {
  std::array<Eigen::MatrixXd, 2> c;
  constexpr int                  N = 2;
  for (int k = 0; k < N; ++k) {
    c[k] = a[k] + b[k];
  }
  return c;
}

void simulate() {
  constexpr int          SIZE = 200;
  plane::SimulationState state;
  state.pressure_field = Eigen::MatrixXd::Zero(SIZE, SIZE);
  // state.pressure_field.block(40, 40, 20, 20).array() = 0.001;

  state.velocity_field[0]             = Eigen::MatrixXd::Zero(SIZE, SIZE);
  state.velocity_field[1]             = Eigen::MatrixXd::Zero(SIZE, SIZE);
  Eigen::MatrixXd force_x             = Eigen::MatrixXd::Zero(SIZE, SIZE);
  force_x.block(3, 20, 1, 30).array() = 0.1;

  auto viewer = gl_viewer::get_window3d("America's Favorite Visualization Tool");
  auto image  = std::make_shared<gl_viewer::Image>(state.pressure_field, 100.0);
  viewer->add_primitive(image);

  const plane::SimulationConfig cfg;
  for (int k = 0; k < 100; ++k) {
    std::cout << "k: " << k << std::endl;
    std::cout << "velocity_fieldx " << state.velocity_field[0].minCoeff() << " , " << state.velocity_field[0].maxCoeff()
              << std::endl;
    std::cout << "velocity_fieldy " << state.velocity_field[1].minCoeff() << " , " << state.velocity_field[1].maxCoeff()
              << std::endl;
    std::cout << "pressure_field " << state.pressure_field.minCoeff() << ", " << state.pressure_field.maxCoeff()
              << std::endl;

    // image->update_image(state.pressure_field);
    image->update_image((state.pressure_field / state.pressure_field.maxCoeff()));
    const auto advection = plane::compute_advection(state.velocity_field, cfg);
    const auto pressure  = plane::compute_pressure(state.pressure_field, cfg);
    const auto diffusion = plane::compute_diffusion(state.velocity_field, cfg);

    auto divergent_dvel = add(add(advection, pressure), diffusion);
    divergent_dvel[0] += force_x;

    const auto divergent_vel = add(mul(divergent_dvel, cfg.dt), state.velocity_field);

    state = plane::compute_projection(divergent_vel, cfg);
    state = enforce_boundary_conditions(state, cfg);
  }
}

}  // namespace fluids.
}  // namespace jcc.

int main() {
  //
  //
  //
  jcc::fluids::simulate();
}