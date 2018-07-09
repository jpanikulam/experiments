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

    for (int x = 25; x < 35; ++x) {
      for (int y = 25; y < 35; ++y) {
        depth_field_(x, y) = 31.0;
      }
    }

    for (int x = 70; x < 80; ++x) {
      for (int y = 25; y < 35; ++y) {
        terrain_field_(x, y) = 1.0;
        depth_field_(x, y) = 20.0;
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

  void simulate(double dt_sec) {
    const int size = depth_field_.cols();
    const double inv_scale = 1.0 / scale_mppx_;

    const auto& u = velocity_field_;
    const auto& d = depth_field_;
    {
      const Mat dvx =
          (d.topRows(size - 1).array() * u[0].topRows(size - 1).array()) -
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

    const auto h = depth_field_ + terrain_field_;
    {
      const double g_inv_dx = -1.0 * inv_scale;
      {
        velocity_field_[0].block(0, 0, size - 1, size) +=
            g_inv_dx * (h.topRows(size - 1) - h.bottomRows(size - 1)) * dt_sec;
      }
      {
        velocity_field_[1].block(0, 0, size, size - 1) +=
            g_inv_dx * (h.leftCols(size - 1) - h.rightCols(size - 1)) * dt_sec;
      }
      const double damping = 0.999;
      velocity_field_ = fluids::mul(velocity_field_, damping);
    }
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
  const int size = 150;

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

        intensities.push_back(norm_v);
      }
    }

    view->clear();
    const auto fluid_geo = view->add_primitive<viewer::SimpleGeometry>();
    fluid_geo->add_colored_points(points, intensities);

    // const auto ground_geo = view->add_primitive<viewer::SimpleGeometry>();
    // ground_geo->add_points(ground_points);

    system.simulate(0.01);
    view->spin_until_step();
  }
}
}  // namespace lanczos

int main() {
  lanczos::visualize();
}