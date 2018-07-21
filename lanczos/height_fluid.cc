#include "eigen.hh"

#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

#include "fluids/fields_2d.hh"

namespace fluids = jcc::fluids::plane;

namespace lanczos {
using Mat = Eigen::MatrixXd;
using Vec3 = Eigen::Vector3d;

Mat generate_terrain(int size) {
  Mat terrain_height = Mat::Zero(size, size);
  return terrain_height;
}

class FluidSystem {
 public:
  FluidSystem(const int size, const double scale) {
    depth_field_ = Mat::Ones(size, size) * 25.0;
    terrain_field_ = Mat::Ones(size, size) * 1.0;
    velocity_field_[0] = Mat::Zero(size, size);
    velocity_field_[1] = Mat::Zero(size, size);

    scale_mppx_ = scale;

    for (int x = 0; x < size; ++x) {
      for (int y = 0; y < size; ++y) {
        terrain_field_(x, y) = 1.0;
        depth_field_(x, y) = 23.0;
      }
    }

    for (int x = 190; x < 200; ++x) {
      for (int y = 190; y < 200; ++y) {
        // depth_field_(x, y) = 27.0;
        velocity_field_[0](x, y) += 0.6;
        velocity_field_[1](x, y) += 0.6;
      }
    }

    for (int x = 210; x < 250; ++x) {
      for (int y = 210; y < 250; ++y) {
        terrain_field_(x, y) = 28.0;
        depth_field_(x, y) = 0.0;
      }
    }
  }

  const Mat height_field() const {
    return depth_field_ + terrain_field_;
  }

  const Mat& terrain_field() const {
    return terrain_field_;
  }

  const Mat& v_x() const {
    return velocity_field_[0];
  }

  const Mat& v_y() const {
    return velocity_field_[1];
  }

  // Update depth
  void update_depth_field(double dt_sec) {
    const int size = depth_field_.cols();
    const double inv_scale = 1.0 / scale_mppx_;

    const auto& u = velocity_field_;
    const auto& d = depth_field_;

    const Mat dvx = (d.topRows(size - 1).array() * u[0].topRows(size - 1).array()) -
                    (d.bottomRows(size - 1).array() * u[0].bottomRows(size - 1).array());

    const Mat dvy = (d.leftCols(size - 1).array() * u[1].leftCols(size - 1).array()) -
                    (d.rightCols(size - 1).array() * u[1].rightCols(size - 1).array());

    const Mat delta_depth_field =
        (dvx.block(0, 1, size - 1, size - 1) + dvy.block(1, 0, size - 1, size - 1)) *
        inv_scale;

    depth_field_.block(1, 1, size - 1, size - 1) += -delta_depth_field * dt_sec;
    depth_field_ = depth_field_.array().max(0.0);
    depth_field_ = depth_field_.array().min(50.0);
  }

  // Advect velocity
  void update_velocity_field(double dt_sec) {
    const int size = depth_field_.cols();
    const double inv_scale = 1.0 / scale_mppx_;

    const auto h = depth_field_ + terrain_field_;
    const double g_inv_dx = -1.0 * inv_scale;

    velocity_field_[0].block(0, 0, size - 1, size) +=
        g_inv_dx * (h.topRows(size - 1) - h.bottomRows(size - 1)) * dt_sec;

    velocity_field_[1].block(0, 0, size, size - 1) +=
        g_inv_dx * (h.leftCols(size - 1) - h.rightCols(size - 1)) * dt_sec;

    const double damping = 1.0;
    velocity_field_ = fluids::mul(velocity_field_, damping);
  }

  // Brute-enforce boundary conditions
  // (Is applying the constraint via MUM more stable? Accurate?)
  void enforce_boundary_conditions() {
    for (int k = 0; k < 2; ++k) {
      velocity_field_[k].topRows(1) *= 0.0;
      velocity_field_[k].bottomRows(1) *= 0.0;
      velocity_field_[k].leftCols(1) *= 0.0;
      velocity_field_[k].rightCols(1) *= 0.0;
    }

    for (int x = 209; x < 251; ++x) {
      for (int y = 209; y < 251; ++y) {
        // terrain_field_(x, y) = 28.0;
        // depth_field_(x, y) = 0.0;
        velocity_field_[0](x, y) *= 0.1;
        velocity_field_[1](x, y) *= 0.1;
      }
    }
  }

  void simulate(double dt_sec) {
    //
    // Update the depth field
    //

    update_depth_field(dt_sec);

    //
    // Update the velocity field
    //

    update_velocity_field(dt_sec);

    //
    // Enforce boundary conditions
    //

    enforce_boundary_conditions();
  }

 private:
  Mat terrain_field_;
  Mat depth_field_;

  std::array<Mat, 2> velocity_field_;
  double scale_mppx_ = 10.0;
};

void visualize() {
  const auto view = viewer::get_window3d("Fluid Visualization");
  view->set_target_from_world(
      SE3(SO3::exp(Eigen::Vector3d(-3.1415 * 0.5, 0.0, 0.0)), Eigen::Vector3d::Zero()));
  view->clear();
  view->set_continue_time_ms(5);

  const double scale = 1.0;
  const int size = 500;

  FluidSystem system(size, scale);
  viewer::Points ground_points;

  // const Mat& terrain = system.terrain_field();
  // for (int x = 0; x < terrain.rows(); ++x) {
  //   for (int y = 0; y < terrain.cols(); ++y) {
  //     const double h = terrain(x, y);
  //     const Vec3 terrain_pt(x * scale, y * scale, h);
  //     ground_points.points.push_back(terrain_pt);
  //   }
  // }

  const auto fluid_geo = view->add_primitive<viewer::SimpleGeometry>();

  for (int k = 0; k < 10000; ++k) {
    const Mat heights = system.height_field();

    const size_t count = size * size;
    viewer::Points points;
    points.points.reserve(count);
    std::vector<double> intensities;
    intensities.reserve(count);
    points.size = scale * 5.0;

    for (int x = 0; x < heights.rows(); ++x) {
      for (int y = 0; y < heights.cols(); ++y) {
        const double h = heights(x, y);
        const Vec3 position(x * scale, y * scale, h);
        points.points.push_back(position);

        const double v_x = system.v_x()(x, y);
        const double v_y = system.v_y()(x, y);

        const double norm_v = std::hypot(v_x, v_y);

        // intensities.push_back(norm_v);
        intensities.push_back(h - 23.0);
      }
    }

    fluid_geo->add_colored_points(points, intensities);
    fluid_geo->flip();

    // const auto ground_geo = view->add_primitive<viewer::SimpleGeometry>();
    // ground_geo->add_points(ground_points);

    for (int iter = 0; iter < 10; ++iter) {
      system.simulate(0.001);
    }
    view->spin_until_step();
  }
}
}  // namespace lanczos

int main() {
  lanczos::visualize();
}