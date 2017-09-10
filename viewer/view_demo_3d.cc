
#include "primitives/box.hh"
#include "primitives/image.hh"
#include "primitives/plot.hh"
#include "primitives/simple_geometry.hh"

#include "window_3d.hh"
#include "window_manager.hh"

#include "geometry/import/read_stl.hh"

#include "sophus.hh"

#include <iostream>
#include <memory>
#include <thread>
#include <utility>

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;

namespace gl_viewer {

Eigen::MatrixXd make_quadratic_thing() {
  using Vec2 = Eigen::Vector2d;

  constexpr double SCALE = 0.01;
  Eigen::MatrixXd values(500, 500);
  Eigen::Matrix2d hessian = Eigen::Matrix2d::Identity();
  hessian(0, 0) = 2.0;
  hessian(1, 0) = -2.0;
  const Vec2 origin = Vec2(values.rows() / 2, values.cols() / 2) * SCALE;

  for (int row = 0; row < values.rows(); ++row) {
    for (int col = 0; col < values.cols(); ++col) {
      const Vec2 point = Vec2(row * SCALE, col * SCALE) - origin;
      values(row, col) = point.transpose() * (hessian * point);
    }
  }
  return values;
}

Eigen::MatrixXd mat_from_cv(const cv::Mat &image) {
  constexpr double INV_255 = 1.0 / 255.0;
  Eigen::MatrixXd mat(image.rows, image.cols);
  for (int row = 0; row < image.rows; ++row) {
    for (int col = 0; col < image.cols; ++col) {
      mat(row, col) = static_cast<double>(image.at<uint8_t>(row, col, 0)) * INV_255;
    }
  }
  return mat;
}

void run() {
  auto win = get_window3d("Window A");

  const std::string file_path = "/home/jacob/repos/experiments/cube_shape.stl";
  const auto tri = geometry::import::read_stl(file_path);

  const std::string godzilla_image_filename = "/home/jacob/repos/slam/data/calibration/godzilla.jpg";
  const cv::Mat godzilla_image_color = cv::imread(godzilla_image_filename);
  const std::string calibration_image_filename =
      "/home/jacob/repos/slam/data/calibration/domestic_goat_kid_in_capeweed.jpg";
  const cv::Mat calibration_image_color = cv::imread(calibration_image_filename);

  WindowManager::draw();

  auto geometry = std::make_shared<SimpleGeometry>();
  win->add_primitive(geometry);
  geometry->add_axes({SE3()});
  geometry->add_ray({Vec3::Zero(), Vec3(1.0, 1.0, 1.0).normalized(), 10.0});
  geometry->add_ray({Vec3(3.0, 0.0, 0.0), Vec3(1.0, 1.0, 1.0).normalized(), 10.0, Vec4(1.0, 0.4, 0.2, 1.0), 5.0});

  for (size_t k = 0; k < tri.triangles.size(); ++k) {
    geometry->add_line({tri.triangles[k].vertices[0], tri.triangles[k].vertices[1]});
    geometry->add_line({tri.triangles[k].vertices[1], tri.triangles[k].vertices[2]});
    geometry->add_line({tri.triangles[k].vertices[2], tri.triangles[k].vertices[0]});
  }

  geometry->add_billboard_circle({Vec3(0.0, 1.0, 1.0), 3.0});
  auto image = std::make_shared<Image>(calibration_image_color, 0.01);
  win->add_primitive(image);

  WindowManager::spin();
  std::cout << "Done" << std::endl;

  glfwTerminate();
  exit(EXIT_SUCCESS);
}
}

int main(void) {
  gl_viewer::run();
}
